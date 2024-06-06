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
#include <comma/timing/timestamped.h>
#include <comma/timing/traits.h>
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
    --pixel-type=<type>; default=rggb; todo...
    --size,--number-of-buffers=<n>; default=32
    --width=<bytes>
output options
    --discard; discard buffers when the output handler cannot keep up (due to
               an either intermittently or permanently slow consumer)
    --fields=[<fields>]; header output fields: t,width,height,type,count
                         default: no header
                         t: buffer timestamp
                         width: output image width: --width divided by pixel size
                         height: output image height: same as--height
                         type: image type as in opencv, see e.g. cv-cat -h -v for details
                             rggb: 24 (CV_8UC4 or 4ub)
                             support for more types: todo
    --latest; if --discard, always output the latest available video buffer and discard the rest
    --log-index; if logging, log index as binary <timestamp>,<fields>, where
                             <timestamp> is as in <timestamp>.bin log file name
    --log-index-file=<filename>; default=index.bin
    --output-header-fields,--output-fields
    --output-header-format,--output-format
    --output-header-only,--header-only; output header only, e.g. for debugging

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
            , _ostream( _ofs, _make_csv( csv ) )
        {
            COMMA_ASSERT_BRIEF( _ofs.is_open(), "failed to create '" << filename << "'" );
        }

        void write( const comma::timestamped< video::header >& h ) { _ostream.write( h ); }

    private:
        std::string _filename;
        std::ofstream _ofs;
        comma::csv::output_stream< comma::timestamped< video::header > > _ostream;
        comma::csv::options _make_csv( const comma::csv::options& csv )
        {
            comma::csv::options index_csv = csv;
            index_csv.format( "t," + csv.format().string() ); // quick and dirty
            index_csv.fields = "t";
            for( const auto& f: comma::split( csv.fields, ',' ) ) { index_csv.fields += f.empty() ? "," : ",data/" + f; } // quick and dirty
            return index_csv;
        }
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
        const auto& unnamed = options.unnamed( "--discard,--latest,--output-header-fields,--output-fields,--output-header-format,--output-format,--output-header-only,--header-only,--log-index", "-.*" );
        COMMA_ASSERT_BRIEF( !unnamed.empty(), "please specify video device" );
        COMMA_ASSERT_BRIEF( unnamed.size() <= 2, "expected one video device; got'" << comma::join( unnamed, ' ' ) << "'" );
        auto name = unnamed[0];
        comma::csv::options csv( options );
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
        snark::timestamped< snark::io::video::header > header;
        unsigned int width = options.value< unsigned int >( "--width" );
        unsigned int height = options.value< unsigned int >( "--height" );
        unsigned int number_of_buffers = options.value< unsigned int >( "--size,--number-of-buffers", 32 );
        unsigned int pixel_size = 4;
        header.data.width = width / pixel_size;
        header.data.height = height;
        header.data.type = 24; // todo! --type,--pixel-type
        comma::saymore() << name << ": video stream: creating..." << std::endl;
        snark::io::video::stream video( name, width, height, number_of_buffers );
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
                                                                     header.data.t = record.buffer.t;
                                                                     header.data.count = record.count;
                                                                     static unsigned int size = width * height;
                                                                     const char* data = reinterpret_cast< const char* >( record.buffer.data );
                                                                     if( log )
                                                                     {
                                                                        log->write( header.data, data, size, csv.flush );
                                                                        if( index ) { header.t = log->how().time(); index->write( header ); }
                                                                        return;
                                                                     }
                                                                     if( !csv.fields.empty() ) { ostream->write( header.data ); }
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