// Copyright (c) 2024 Mission Systems

#include <atomic>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/none.h>
#include <comma/csv/names.h>
#include <comma/csv/split.h>
#include <comma/csv/stream.h>
#include <comma/io/stream.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include "../../tbb/bursty_reader.h"
#include "../../tbb/types.h"
#include "../video.h"

void usage( bool verbose )
{
    std::cerr << R"(
usage: video-cat <path> <output> <options>
    <path>; video device path, e.g. "/dev/video0"
)"
    << ( verbose
       ?   comma::io::ostream::usage( 4, true )
         + comma::csv::splitting::usage( 8, true )
         + "        default              : stdout"
       : "    <output>: default: stdout; run --help --verbose for details" )
    << R"(
options
    --height=<rows>
    --pixel-format=<type>; default: 'rggb'; choices: 'rggb', 'y16', todo: more types, just ask
    --size,--number-of-buffers=<n>; default=32
    --width=<bytes>
output options
    --discard; discard buffers when the output handler cannot keep up (due to
               an either intermittently or permanently slow consumer)
    --fields=[<fields>]; header output fields: t,width,height,type,count
                         default: no header
                         t: buffer timestamp
                         width,cols : output image width: --width divided by pixel size
                         height,rows: output image height: same as --height
                         type: image type as in opencv, see e.g. cv-cat -h -v for details
                             rggb  : 24 : CV_8UC4  or 4ub
                             y16   : 2  : CV_16UC1 or uw
                             support for more types: todo
    --flush; flush stdout after each output (remember when using --output-header-only)
    --image=<rows>,<cols>,<type>; convenience option to use instead of
                                  --width, --height, and --pixel-format
                                  and --fields=t,height,width,type
        <rows>: height in pixels
        <cols> : width in pixels
        <type>  : cv-cat-style image type, just ask if you need more types
                      4ub: same as rggb
                      uw : same as y16
    --latest; if --discard, always output the latest available video buffer and discard the rest
    --log-index; if logging, log index as binary with header <fields>
    --log-index-file=<filename>; default=index.bin
    --output-header-fields,--output-fields
    --output-header-format,--output-format
    --output-no-header,--no-header; do not output header
    --output-header-only,--header-only; output header only, e.g. for debugging
examples
    acquire and display video stream from a FLIR 16-bit greyscale camera
        porcelain
            video-cat /dev/video0 --image=512,640,uw \
                | cv-cat 'convert-to=f,1,-22400;convert-to=f,0.0008;convert-to=ub,255;color-map=jet;timestamp;view;null'
        bolts and nuts
            video-cat /dev/video0 --height 512 --width $(( 640 * 2 )) --pixel-format y16 \
                | cv-cat --input 'rows=512;cols=640;type=uw;no-header' \
                                 'convert-to=f,1,-22400;convert-to=f,0.0008;convert-to=ub,255;color-map=jet;timestamp;view;null'
)";
    exit( 0 );
}

namespace snark { namespace io { namespace video {

struct header // quick and dirty, to avoid dependency on imaging
{
    boost::posix_time::ptime t;
    std::uint32_t width{0};
    std::uint32_t height{0};
    std::uint32_t type{0}; // quick and dirty, to avoid dependency on imaging
    std::uint32_t count{0};
};

} } } // namespace snark { namespace io { namespace video {

namespace comma { namespace visiting {

template <> struct traits< snark::io::video::header >
{
    template < typename K, typename V > static void visit( const K&, snark::io::video::header& h, V& v )
    {
        v.apply( "t", h.t );
        v.apply( "width", h.width );
        v.apply( "height", h.height );
        v.apply( "type", h.type );
        v.apply( "count", h.count );
    }
    template < typename K, typename V > static void visit( const K&, const snark::io::video::header& h, V& v )
    {
        v.apply( "t", h.t );
        v.apply( "width", h.width );
        v.apply( "height", h.height );
        v.apply( "type", h.type );
        v.apply( "count", h.count );
    }
};

} } // namespace comma { namespace visiting {

namespace comma { namespace csv { namespace splitting {

template <> struct type_traits< snark::io::video::header >
{
    static boost::posix_time::ptime time( const snark::io::video::header& t ) { return t.t; }
    static unsigned int block( const snark::io::video::header& t ) { return 0; }
    static unsigned int id( const snark::io::video::header& t ) { return 0; }
};

} } } // namespace comma { namespace csv { namespace splitting {

namespace snark { namespace tbb {

template <> struct bursty_reader_traits< snark::io::video::stream::record >
{
    static bool valid( const snark::io::video::stream::record& r ) { return bool( r ); }
};

} } // namespace snark { namespace tbb {

namespace snark { namespace io { namespace video {

class index
{
    public:
        index( const std::string& filename, const comma::csv::options& csv )
            : _filename( filename )
            , _ofs( filename )
            , _ostream( _ofs, csv )
        {
            COMMA_ASSERT_BRIEF( _ofs.is_open(), "failed to create '" << filename << "'" );
        }

        void write( const video::header& h ) { _ostream.write( h ); }

        void flush() { _ofs.flush(); }

    private:
        std::string _filename;
        std::ofstream _ofs;
        comma::csv::output_stream< video::header > _ostream;
};

} } } // namespace snark { namespace io { namespace video {

int main( int ac, char** av )
{
    try
    {
        // {
        //     int count{0};
        //     const ::tbb::concurrent_bounded_queue< int >* queue{nullptr};
        //     snark::tbb::bursty_reader< int > p( [&]()->int
        //                                         {
        //                                             ::usleep( 250000 );
        //                                             std::cerr << "p: " << count << " queue.size(): " << queue->size() << std::endl;
        //                                             return count++;
        //                                         }, 4, 0, true );
        //     snark::tbb::filter< int, void >::type c( snark::tbb::filter_mode::serial_in_order, [&]( int v ) { ::usleep( 1000000 ); std::cerr << "c: received: " << v << " current: " << count << std::endl; } );
        //     queue = &p.queue();
        //     ::tbb::parallel_pipeline( 1, p.filter() & c );
        //     return 0;
        // }
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--output-header-fields,--output-fields" ) ) { std::cout << comma::join( comma::csv::names< snark::io::video::header >(), ',' ) << std::endl; return 0; }
        if( options.exists( "--output-header-format,--output-format" ) ) { std::cout << comma::csv::format::value< snark::io::video::header >() << std::endl; return 0; }
        const auto& unnamed = options.unnamed( "--discard,--latest,--output-header-fields,--output-fields,--output-header-format,--output-format,--output-header-only,--header-only,--output-no-header,--no-header,--log-index", "-.*" );
        COMMA_ASSERT_BRIEF( !unnamed.empty(), "please specify video device" );
        COMMA_ASSERT_BRIEF( unnamed.size() <= 2, "expected one video device; got'" << comma::join( unnamed, ' ' ) << "'" );
        auto name = unnamed[0];
        comma::csv::options csv( options, { { "rows", "height" }, { "cols", "width" } } ); // quick and dirty, to preserve compatibility with imaging::cv_cat::serialization (too fiddly to change)
        options.assert_mutually_exclusive( "--width,--height,--pixel-format,--fields", "--image" );
        if( options.exists( "--image" ) ) { csv.fields = "t,height,width,type"; }
        if( options.exists( "--output-no-header,--no-header" ) ) { csv.fields = ""; }
        csv.format( comma::csv::format::value< snark::io::video::header >( csv.fields, true ) );
        auto output_options = unnamed.size() < 2 ? "-" : unnamed[1];
        boost::optional< snark::io::video::index > index;
        typedef comma::csv::split< snark::io::video::header > log_t;
        typedef comma::csv::output_stream< snark::io::video::header > csv_stream_t;
        std::unique_ptr< log_t > log;
        std::unique_ptr< comma::io::ostream > os;
        std::unique_ptr< csv_stream_t > ostream;
        if( output_options.substr( 0, 4 ) == "log:" )
        {
            COMMA_ASSERT_BRIEF( !csv.fields.empty(), "only logging with header is supported, please specify at least one field in --fields" );
            log.reset( log_t::make( output_options, csv ) );
            if( options.exists( "--log-index" ) ) { index.emplace( log->how().address() + "/" + options.value< std::string >( "--log-index-file", "index.bin" ), csv ); }
        }
        else
        {
            COMMA_ASSERT_BRIEF( !options.exists( "--log-index" ), "--log-index makes sense only for 'log:...' outputs; got: '" << output_options << "'" );
            os = std::make_unique< comma::io::ostream >( output_options );
            ostream = std::make_unique< csv_stream_t >( *( *os ), csv );
        }
        snark::io::video::header header;
        unsigned int number_of_buffers = options.value< unsigned int >( "--size,--number-of-buffers", 32 );
        std::string image_options = options.value< std::string >( "--image", "" );
        unsigned int width{0}, height{0}, pixel_size{0};
        int pixel_format{0};
        if( image_options.empty() ) // todo: get rid of repetitiveness in this if-clause
        {
            width = options.value< unsigned int >( "--width" );
            height = options.value< unsigned int >( "--height" );
            std::string pixel_type_name = options.value< std::string >( "--pixel-format", "rggb" );
            if( pixel_type_name == "rggb" )
            {
                pixel_size = 4;
                header.type = 24;
                pixel_format = V4L2_PIX_FMT_SRGGB8;
            }
            else if( pixel_type_name == "y16" )
            {
                pixel_size = 2;
                header.type = 2;
                pixel_format = V4L2_PIX_FMT_Y16;
            }
            header.width = width / pixel_size;
            header.height = height;
        }
        else
        {
            const auto& v = comma::split( image_options, ',' );
            COMMA_ASSERT_BRIEF( v.size() == 3, "expected --image=<rows>,<cols>,<type>; got: '" << image_options << "'" );
            if( v[2] == "4ub" ) // todo: quick and dirty, generalise
            {
                pixel_size = 4;
                header.type = 24;
                pixel_format = V4L2_PIX_FMT_SRGGB8;
            }
            else if( v[2] == "uw" ) // todo: quick and dirty, generalise
            {
                pixel_size = 2;
                header.type = 2;
                pixel_format = V4L2_PIX_FMT_Y16;
            }
            header.height = boost::lexical_cast< unsigned int >( v[0] );
            header.width = boost::lexical_cast< unsigned int >( v[1] );
            width = header.width * pixel_size;
            height = header.height;
        }
        comma::saymore() << name << ": video stream: creating..." << std::endl;
        snark::io::video::stream video( name, width, height, number_of_buffers, pixel_format );
        comma::saymore() << name << ": video stream: created" << std::endl;
        comma::signal_flag is_shutdown;
        typedef snark::io::video::stream::record record_t;
        bool discard = options.exists( "--discard" );
        bool latest = options.exists( "--latest" );
        COMMA_ASSERT_BRIEF( !latest || discard, "if --latest, please specify --discard" );
        bool header_only = options.exists( "--output-header-only,--header-only" );
        std::atomic_uint count{0};
        auto read_once = [&]()->record_t { if( is_shutdown ) { video.stop(); return record_t(); } else { ++count; return video.read(); } };
        snark::tbb::filter< record_t, void >::type write_filter( snark::tbb::filter_mode::serial_in_order
                                                               , [&]( const record_t& record )
                                                                 {
                                                                     if( !record ) { return; }
                                                                     unsigned int c{count};
                                                                     int d = c - record.count;
                                                                     if( latest ) // todo? don't skip, just jump straight to the latest record
                                                                     {
                                                                         if( d > 1 ) { comma::saymore() << "thus record " << record.count << " discarded since asked to output latest record (" << c << "); " << std::endl; return; }
                                                                     }
                                                                     else
                                                                     {
                                                                        if( d >= int( number_of_buffers ) )
                                                                        {
                                                                            COMMA_ASSERT_BRIEF( discard, "asked to output record " << record.count << " but already have read record " << c << ", i.e. output is too slow and buffers get overwritten (number of buffers: " << number_of_buffers << ")" );
                                                                            comma::saymore() << "asked to output record " << record.count << " but already have read record " << c << "; discarded since output is too slow and buffers get overwritten (number of buffers: " << number_of_buffers << ")" << std::endl;
                                                                            return;
                                                                        }
                                                                     }
                                                                     header.t = record.buffer.t;
                                                                     header.count = record.count;
                                                                     static unsigned int size = width * height;
                                                                     const char* data = reinterpret_cast< const char* >( record.buffer.data );
                                                                     if( log )
                                                                     {
                                                                        log->write( header, data, size, csv.flush );
                                                                        if( index ) { index->write( header ); index->flush(); }
                                                                        return;
                                                                     }
                                                                     if( !csv.fields.empty() ) { ostream->write( header ); }
                                                                     if( !header_only ) { ( *os )->write( data, size ); }
                                                                     if( csv.flush ) { ( *os )->flush(); }
                                                                 } );
        
        // todo! handle exceptions in read_once() or make it no-throw
        // todo! handle errno eintr
        // todo! expose pixel type (V4L2_PIX_FMT_SRGGB8 etc)

        comma::saymore() << name << ": video stream: starting..." << std::endl;
        video.start();
        comma::saymore() << name << ": video stream: started" << std::endl;
        comma::saymore() << name << ": readers: creating..." << std::endl;
        snark::tbb::bursty_reader< record_t > bursty_reader( read_once, discard ? video.buffers().size() : 0, discard ? 0 : video.buffers().size(), true );
        comma::saymore() << name << ": readers: created" << std::endl;
        comma::saymore() << name << ": processing pipeline: running..." << std::endl;
        ::tbb::parallel_pipeline( 1, bursty_reader.filter() & write_filter );
        comma::saymore() << name << ": processing pipeline: stopped" << std::endl;
        video.stop();
        comma::saymore() << name << ": video stream: stopped" << std::endl;
        comma::saymore() << name << ": preparing to exit..." << std::endl;
        bursty_reader.join();
        comma::saymore() << name << ": done" << std::endl;
        return 0;
    }
    catch( const std::exception& ex ) { comma::say() << ex.what() << std::endl; }
    catch( ... ) { comma::say() << "unknown exception" << std::endl; }
    return 1;
}