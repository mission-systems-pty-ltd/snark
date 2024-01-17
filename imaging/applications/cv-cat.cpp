// Copyright (c) 2011 The University of Sydney

#ifdef WIN32
#include <winsock2.h>
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#endif
#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/program_options.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <comma/application/signal_flag.h>
#include <comma/name_value/parser.h>
#include <comma/application/verbose.h>
#include <comma/csv/binary.h>
#include <comma/string/split.h>
#include "../cv_mat/pipeline.h"
#include "../cv_mat/filters/help.h"

typedef std::pair< snark::cv_mat::serialization::header::buffer_t, cv::Mat > pair;
typedef snark::cv_mat::serialization serialization;

class rate_limit /// timer class, sleeping if faster than the specified fps
{
    public:
        rate_limit( double fps ) { if( fps > 1e-5 ) { period_ = boost::posix_time::microseconds( static_cast< unsigned int >( 1e6 / fps ) ); } }

        void wait()
        {
            if( period_.is_not_a_date_time() ) { return; }
            boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
            if( !last_.is_not_a_date_time() && ( now < last_ + period_ ) ) { boost::this_thread::sleep( last_ + period_ ); }
            last_ = now;
        }

    private:
        boost::posix_time::time_duration period_;
        boost::posix_time::ptime last_;
};

static comma::signal_flag is_shutdown( comma::signal_flag::hard );

static pair capture( cv::VideoCapture& capture, rate_limit& rate )
{
    cv::Mat image;
    capture >> image;
    rate.wait();
    static comma::csv::binary< snark::cv_mat::serialization::header > default_binary( "t,3ui", "t,rows,cols,type" );
    static snark::cv_mat::serialization::header::buffer_t buffer( default_binary.format().size() );
    snark::cv_mat::serialization::header h(image);
    h.timestamp = boost::posix_time::microsec_clock::universal_time();
    default_binary.put( h, &buffer[0] );
    return std::make_pair( buffer , image );
}

static pair read_image_files( const std::string& images, rate_limit& rate, bool timestamped, snark::cv_mat::serialization& input ) // todo? refactor that it can mix of images and videos, too?
{
    static bool done = false;
    static std::ifstream ifs( images ); // todo? use csv stream or at least io::stream?
    if( done ) { return pair(); }
    if( !ifs.is_open() ) { std::cerr << "cv-cat: failed to open '" << images << "'" << std::endl; exit( 1 ); }
    rate.wait();
    std::string line;
    while( true )
    {
        if( !ifs.good() || ifs.eof() ) { done = true; return pair(); }
        std::getline( ifs, line );
        line = comma::strip( line );
        if( !line.empty() && line[0] != '#' ) { break; }
    }
    if( !boost::filesystem::exists( line ) ) { std::cerr << "cv-cat: file not found '" << line << "'" << std::endl; exit( 1 ); }
    std::pair< boost::posix_time::ptime, cv::Mat > p;
    std::string extension = comma::split( line, '.' ).back();
    if( extension == "bin" || extension == "gz" )
    {
        std::ifstream i( line );
        if( !i.is_open() ) { std::cerr << "cv-cat: failed to open '" << line << "'" << std::endl; exit( 1 ); }
        if( extension == "bin" )
        {
            p = input.read< boost::posix_time::ptime >( i );
        }
        else
        {
            boost::iostreams::filtering_streambuf< boost::iostreams::input > zin;
            zin.push( boost::iostreams::gzip_decompressor() );
            zin.push( i );
            std::ostringstream oss;
            boost::iostreams::copy( zin, oss );
            std::istringstream iss( oss.str() ); // quick and dirty, watch performance
            p = input.read< boost::posix_time::ptime >( iss );
        }
    }
    else
    {
        p.second = cv::imread( line, cv::IMREAD_UNCHANGED );
    }
    if( !p.second.data ) { std::cerr << "cv-cat: failed to read image '" << line << "'" << std::endl; exit( 1 ); }
    if( timestamped )
    {
        std::vector< std::string > time_strings = comma::split( comma::split( line, '/' ).back(), '.' ); // quick and dirty, use boost::filesystem
        if( time_strings.size() == 2 ) { p.first = boost::posix_time::from_iso_string( time_strings[0] ); }
        else if ( time_strings.size() > 2 ) { p.first = boost::posix_time::from_iso_string( time_strings[0] + '.' + time_strings[1] ); }
    }
    static comma::csv::binary< snark::cv_mat::serialization::header > default_binary( "t,3ui", "t,rows,cols,type" );
    static snark::cv_mat::serialization::header::buffer_t buffer( default_binary.format().size() );
    snark::cv_mat::serialization::header h( p.second );
    h.timestamp = p.first.is_not_a_date_time() ? boost::posix_time::microsec_clock::universal_time() : p.first;
    default_binary.put( h, &buffer[0] );
    return std::make_pair( buffer, p.second );
}

static pair output_single_image( const std::pair< boost::posix_time::ptime, cv::Mat >& p )
{
    static bool done = false;
    if( done ) { return pair(); }
    done = true;
    static comma::csv::binary< snark::cv_mat::serialization::header > default_binary( "t,3ui", "t,rows,cols,type" );
    static snark::cv_mat::serialization::header::buffer_t buffer( default_binary.format().size() );
    snark::cv_mat::serialization::header h( p.second );
    h.timestamp = p.first.is_not_a_date_time() ? boost::posix_time::microsec_clock::universal_time() : p.first;
    default_binary.put( h, &buffer[0] );
    return std::make_pair( buffer, p.second );
}

static pair read( snark::cv_mat::serialization& input, rate_limit& rate )
{
    if( is_shutdown || std::cin.eof() || std::cin.bad() || !std::cin.good() ) { return pair(); }
    rate.wait();
    return input.read< snark::cv_mat::serialization::header::buffer_t >( std::cin );
}

void skip( unsigned int number_of_frames_to_skip, cv::VideoCapture& video_capture, rate_limit& rate ) { for( unsigned int i=0; i<number_of_frames_to_skip; i++ ) { capture( video_capture, rate ); } }
void skip( unsigned int number_of_frames_to_skip, snark::cv_mat::serialization& input, rate_limit& rate ) { for( unsigned int i=0; i<number_of_frames_to_skip; i++ ) { read( input, rate ); } }

static boost::posix_time::ptime get_timestamp_from_header( const snark::cv_mat::serialization::header::buffer_t& h, const comma::csv::binary< snark::cv_mat::serialization::header >* pbinary )
{
    if( h.empty() ) { return boost::posix_time::not_a_date_time; }
    // a use case could be: generate-our-smart-images-in-realtime | cv-cat ... will timestamp images with system time
    if( pbinary == NULL ) { return boost::posix_time::microsec_clock::universal_time(); }
    snark::cv_mat::serialization::header d;
    return pbinary->get( d, &h[0] ).timestamp;
}

static bool has_custom_fields( const std::string& fields )
{
    std::vector< std::string > v = comma::split(fields, ',');
    if( v.size() > snark::cv_mat::serialization::header::fields_num ) { return true; }  // If it has more fields than the default
    for( unsigned int i = 0; i < v.size(); ++i ) { if( v[i] != "t" && v[i] != "rows" && v[i] != "cols" && v[i] != "type" ) { return true; } }
    return false;
}

int main( int argc, char** argv )
{
    try
    {
        #ifdef WIN32
        _setmode( _fileno( stdin ), _O_BINARY );
        _setmode( _fileno( stdout ), _O_BINARY );
        #endif

        std::string name;
        std::string files;
        int device;
        unsigned int discard;
        double fps;
        std::string input_options_string;
        std::string output_options_string;
        unsigned int capacity = 16;
        unsigned int number_of_threads = 0;
        unsigned int number_of_frames_to_skip = 0;
        comma::uint32 image_type;
        std::string image_format;
        boost::program_options::options_description description( "options" );
        std::string help_command;
        description.add_options()
            ( "help,h", boost::program_options::value< std::string >( &help_command )->implicit_value( "" ), "display help message; if '--help command' is specified, focus on the 'command'-specific help" )
            ( "verbose,v", "more output; --help --verbose: more help" )
            ( "image-format", boost::program_options::value< comma::uint32 >(& image_type ), "get image format from type enumeration" )
            ( "image-type", boost::program_options::value< std::string >(& image_format ), "get image type enumeration from format or name" )
            ( "image-types", "get all image type enumerations" )
            ( "discard,d", "discard frames, if cannot keep up; same as --buffer=1" )
            ( "camera", "use first available opencv-supported camera" )
            ( "file", boost::program_options::value< std::string >( &name ), "image or video file name" )
            ( "files", boost::program_options::value< std::string >( &files ), "file with list of image filenames (videos not supported, yet)" )
            ( "id", boost::program_options::value< int >( &device ), "specify specific device by id ( OpenCV-supported camera )" )
            ( "buffer", boost::program_options::value< unsigned int >( &discard )->default_value( 0 ), "maximum buffer size before discarding frames, default: unlimited" )
            ( "fps", boost::program_options::value< double >( &fps )->default_value( 0 ), "specify max fps ( useful for files, may block if used with cameras ) " )
            ( "input", boost::program_options::value< std::string >( &input_options_string ), "input options, when reading from stdin (see --help --verbose)" )
            ( "output", boost::program_options::value< std::string >( &output_options_string ), "output options (see --help --verbose); default: same as --input" )
            ( "capacity", boost::program_options::value< unsigned int >( &capacity )->default_value( 16 ), "maximum input queue size before the reader thread blocks" )
            ( "threads", boost::program_options::value< unsigned int >( &number_of_threads )->default_value( 0 ), "number of threads; default: 0 (auto)" )
            ( "skip", boost::program_options::value< unsigned int >( &number_of_frames_to_skip )->default_value( 0 ), "number of initial frames to skip; default: 0" )
            ( "stay", "do not close at end of stream" )
            ( "timestamped", "if --file present, use file name for timestamp, e.g. --file=images/20170101T012345.jpg, or 20170101T012345.123.jpg, or 20170101T012345.123.0.jpg where 0 is index (see 'index' property in 'files' filter for explanation)" )
            ( "video", "has effect in opencv versions 2.12(?) and above; explicitly specify that filename given by --file refers to a video; e.g. --file ABC_0001.jpg will read a single image, --file ABC_0001.jpg will read images ABC_0001.jpg, ABC_0002.jpg, etc, if present" );
        boost::program_options::variables_map vm;
        boost::program_options::store( boost::program_options::parse_command_line( argc, argv, description), vm );
        boost::program_options::parsed_options parsed = boost::program_options::command_line_parser(argc, argv).options( description ).allow_unregistered().run();
        boost::program_options::notify( vm );
        comma::verbose.init(vm.count( "verbose" ), argv[0]);
        if( vm.count( "image-format" ) ) { std::cout << snark::cv_mat::format_from_type( image_type ) << std::endl; return 0; }
        if( vm.count( "image-type" ) ) { std::cout << snark::cv_mat::type_from_string( image_format ) << std::endl; return 0; }
        if( vm.count( "image-types" ) ) { std::cout << snark::cv_mat::all_image_types(); return 0; }
        if ( vm.count( "help" ) )
        {
            std::string command = vm[ "help" ].as< std::string >();
            if ( ! command.empty() ) { std::cerr << snark::cv_mat::command_specific_help( "cv-cat", command ) << std::endl; return 0; }
            std::cerr << "\n";
            std::cerr << "read images from stdin or a camera supported by opencv, apply filters and output to stdout\n";
            if( !vm.count( "verbose" ) ) { std::cerr << "see --help --verbose for filters usage\n"; }
            std::cerr << "\n";
            std::cerr << "usage: cv-cat [options] [<filters>]\n";
            std::cerr << "\n";
            std::cerr << "using opencv version " << CV_VERSION << "\n";
            std::cerr << "some functionality may not be available depending on the version of your installed opencv\n";
            std::cerr << "\n";
            std::cerr << "image header\n";
            std::cerr << "    default header fields: t,rows,cols,type\n";
            std::cerr << "    default header format: t,3ui\n";
            std::cerr << "    note: only the following scenarios are currently supported:\n";
            std::cerr << "          - input has no header (no-header option), output has default header fields (fields=t,rows,cols,type)\n";
            std::cerr << "          - input has no header (no-header option), output has no header (no-header option)\n";
            std::cerr << "          - input has arbitrary fields, input header fields are the same as output header fields\n";
            std::cerr << "          - input has arbitrary fields, output has no header (no-header option)\n";
            std::cerr << "          anything more sophisticated than that can be easily achieved e.g. by piping cv-cat to csv-shuffle\n";
            std::cerr << "\n";
            std::cerr << description;
            std::cerr << "\n";
            std::cerr << "examples:\n";
            std::cerr << "    take bayer-encoded images with 1000 rows and 500 columns, no header\n";
            std::cerr << "    do bayer conversion, transpose, and output without header to the file converted.bin\n";
            std::cerr << "\n";
            std::cerr << "        cat images.bin | cv-cat --input=\"rows=1000;cols=500;no-header;type=ub\" \\\n";
            std::cerr << "                                        \"bayer=1;transpose\" --output=no-header > converted.bin\n";
            std::cerr << "\n";
            std::cerr << "    view the result of the previous example\n";
            std::cerr << "\n";
            std::cerr << "        cat converted.bin | cv-cat --input=\"rows=500;cols=1000;no-header;type=3ub\" \"view\" > /dev/null\n";
            std::cerr << "\n";
            std::cerr << "    take output of the first found gige camera, resize, view as you go, and save in the file\n";
            std::cerr << "\n";
            std::cerr << "        gige-cat | cv-cat \"resize=640,380;view\" > gige-output.bin\n";
            std::cerr << "\n";
            std::cerr << "    play back and view gige-output.bin from the previous example\n";
            std::cerr << "    header format (by default): t,3ui (timestamp, cols, rows, type)\n";
            std::cerr << "    image size will be 640*380*3=729600\n";
            std::cerr << "\n";
            std::cerr << "        cat gige-output.bin | csv-play --binary=t,3ui,729600ub | cv-cat view > /dev/null\n";
            std::cerr << "\n";
            std::cerr << "    print image header (e.g. to figure out the image size or type)\n";
            std::cerr << "\n";
            std::cerr << "        gige-cat --output=\"header-only;fields=rows,cols,size,type\" | csv-from-bin 4ui | head\n";
            std::cerr << "\n";
            std::cerr << "    create a video with ffmpeg; -b: bitrate, -r: input/output framerate:\n";
            std::cerr << "        gige-cat | cv-cat \"encode=ppm\" --output=no-header \\\n";
            std::cerr << "            | ffmpeg -y -f image2pipe -vcodec ppm -r 25 -i pipe: -vcodec libx264  -threads 0 -b 2000k -r 25 video.mkv\n";
            std::cerr << "\n";
            std::cerr << "        gige-cat encode=png --output=no-header \\\n";
            std::cerr << "            | ffmpeg -y -f image2pipe -r 5 -c:v png -i pipe: -c:v libx264 -threads 0 -b:v 2000k \\\n";
            std::cerr << "                     -r 5 -preset slow -crf 22 video.avi\n";
            std::cerr << "\n";
            std::cerr << "    overlay a ruler on input stream to show scale (the overlay image can be created in a script using csv-to-svg)\n";
            std::cerr << "\n";
            std::cerr << "        create_ruler_svg > tmp/r.svg\n";
            std::cerr << "        convert -background transparent tmp/r.svg tmp/r.png\n";
            std::cerr << "        ...  | cv-cat \"overlay=tmp/r.png,10,10;view;null\" \n";
            if( vm.count( "verbose" ) )
            {
                std::cerr << "\n";
                std::cerr << snark::cv_mat::serialization::options::usage();
                std::cerr << "\n";
                std::cerr << snark::cv_mat::impl::filters<>::usage();
            }
            std::cerr << std::endl;
            return 0;
        }
        if( vm.count( "file" ) + vm.count( "camera" ) + vm.count( "id" ) > 1 ) { std::cerr << "cv-cat: --file, --camera, and --id are mutually exclusive" << std::endl; return 1; }
        if( vm.count( "discard" ) ) { discard = 1; }
        snark::cv_mat::serialization::options input_options = comma::name_value::parser( ';', '=' ).get< snark::cv_mat::serialization::options >( input_options_string );
        snark::cv_mat::serialization::options output_options = output_options_string.empty() ? input_options : comma::name_value::parser( ';', '=' ).get< snark::cv_mat::serialization::options >( output_options_string );
        if( input_options.no_header && !output_options.fields.empty() && input_options.fields != output_options.fields )
        {
            if( output_options.fields != snark::cv_mat::serialization::header::default_fields() )
            {
                std::cerr << "cv-cat: when --input has no-header option, --output fields can only be fields=" << snark::cv_mat::serialization::header::default_fields() << ", got: " << output_options.fields << std::endl; return 1;
            }
        }
        else { if( !output_options.fields.empty() && input_options.fields != output_options.fields ) { std::cerr << "cv-cat: customised output header fields not supported (todo); got: input fields: \"" << input_options.fields << "\" output fields: \"" << output_options.fields << "\"" << std::endl; return 1; } }
        // output fields and format will be empty when the user specifies only --output no-header or --output header-only
        if( output_options.fields.empty() ) { output_options.fields = input_options.fields; }
        if( !output_options.format.elements().empty() && input_options.format.string() != output_options.format.string() ) { std::cerr << "cv-cat: customised output header format not supported (todo); got: input format: \"" << input_options.format.string() << "\" output format: \"" << output_options.format.string() << "\"" << std::endl; return 1; }
        if( output_options.format.elements().empty() ) { output_options.format = input_options.format; };
        // This is needed because if binary is not set, serialization assumes standard fields and guess every field to be ui, very confusing for the user
        if( !input_options.fields.empty() && has_custom_fields( input_options.fields ) && input_options.format.elements().empty() )
        {
            std::cerr << "cv-cat: non default field detected in --input, please specify binary format for fields: " << input_options.fields << std::endl;
            return 1;
        }
        const std::vector< std::string >& filter_strings = boost::program_options::collect_unrecognized( parsed.options, boost::program_options::include_positional );
        std::string filters_string;
        if( filter_strings.size() == 1 ) { filters_string = filter_strings[0]; }
        if( filter_strings.size() > 1 ) { std::cerr << "expected filters as a single name-value string; got: " << comma::join( filter_strings, ' ' ) << std::endl; return 1; }
        if( filters_string.find( "encode" ) != filters_string.npos && !output_options.no_header ) { std::cerr << "cv-cat: warning: encoding image and not using no-header, are you sure?" << std::endl; }
        if( vm.count( "camera" ) ) { device = 0; }
        rate_limit rate( fps );
        cv::VideoCapture video_capture;
        snark::cv_mat::serialization input( input_options );
        snark::cv_mat::serialization output( output_options );
        boost::scoped_ptr< snark::tbb::bursty_reader< pair > > reader;
        std::pair< boost::posix_time::ptime, cv::Mat > p;
        typedef snark::imaging::applications::pipeline_with_header pipeline_with_header;
        typedef snark::cv_mat::filters_with_header filters_with_header;
        const unsigned int default_delay = vm.count( "file" ) == 0 ? 1 : 200; // HACK to make view work on single files
        const auto& filters = filters_with_header::make( filters_string, boost::bind( &get_timestamp_from_header, boost::placeholders::_1, input.header_binary() ), default_delay );
        if( vm.count( "file" ) )
        {
            if( !vm.count( "video" ) )
            {
                if( !boost::filesystem::exists( name ) ) { std::cerr << "cv-cat: file not found '" << name << "'" << std::endl; exit( 1 ); }
                auto extension = comma::split( name, '.' ).back();
                if( extension == "bin" ) // quick and dirty, to keep --file semantics uniform
                {
                    std::ifstream i( name );
                    if( !i.is_open() ) { std::cerr << "cv-cat: failed to open '" << name << "'" << std::endl; exit( 1 ); }
                    p = input.read< boost::posix_time::ptime >( i );
                }
                else if( extension == "gz" )
                {
                    std::ifstream i( name );
                    if( !i.is_open() ) { std::cerr << "cv-cat: failed to open '" << name << "'" << std::endl; exit( 1 ); }
                    boost::iostreams::filtering_streambuf< boost::iostreams::input > zin;
                    zin.push( boost::iostreams::gzip_decompressor() );
                    zin.push( i );
                    std::ostringstream oss;
                    boost::iostreams::copy( zin, oss );
                    std::istringstream iss( oss.str() ); // quick and dirty, watch performance
                    p = input.read< boost::posix_time::ptime >( iss );
                }
                else
                {
                    p.second = cv::imread( name, cv::IMREAD_UNCHANGED );
                }
            }
            if( p.second.data )
            {
                if( vm.count( "timestamped" ) )
                {
                    std::vector<std::string> time_strings = comma::split( comma::split( name, '/' ).back(), '.' ); // quick and dirty, use boost::filesystem
                    if ( time_strings.size() == 2 ){ p.first = boost::posix_time::from_iso_string( time_strings[0] ); }
                    else if ( time_strings.size() > 2 ){ p.first = boost::posix_time::from_iso_string( time_strings[0] + '.' + time_strings[1] ); }
                }
                reader.reset( new snark::tbb::bursty_reader< pair >( boost::bind( &output_single_image, boost::cref( p ) ), discard, capacity ) );
            }
            else
            {
                video_capture.open( name );
                skip( number_of_frames_to_skip, video_capture, rate );
                reader.reset( new snark::tbb::bursty_reader< pair >( boost::bind( &capture, boost::ref( video_capture ), boost::ref( rate ) ), discard, capacity ) );
            }
        }
        else if( vm.count( "files" ) )
        {
            reader.reset( new snark::tbb::bursty_reader< pair >( boost::bind( &read_image_files, boost::cref( files ), boost::ref( rate ), vm.count( "timestamped" ) > 0, boost::ref( input ) ), discard, capacity ) );
        }
        else if( vm.count( "camera" ) || vm.count( "id" ) )
        {
            video_capture.open( device );
            skip( number_of_frames_to_skip, video_capture, rate );
            reader.reset( new snark::tbb::bursty_reader< pair >( boost::bind( &capture, boost::ref( video_capture ), boost::ref( rate ) ), discard ) );
        }
        else
        {
            skip( number_of_frames_to_skip, input, rate );
            reader.reset( new snark::tbb::bursty_reader< pair >( boost::bind( &read, boost::ref( input ), boost::ref( rate ) ), discard, capacity ) );
        }
        pipeline_with_header pipeline( output, filters, *reader, number_of_threads );
        pipeline.run();
        if( vm.count( "stay" ) ) { while( !is_shutdown && cv::waitKey( 1000 ) == 255 ); }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "cv-cat: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "cv-cat : unknown exception" << std::endl; }
    return 1;
}
