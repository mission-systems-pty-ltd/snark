// Copyright (c) 2024 Mission Systems

#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
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
        // --threads,-number-of-threads=<n>; default: run with --output-number-of-threads-default
        //                               if n > 1, then n
        //                               if 0 < n < 1, then default number of threads multiplied by n
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
    --output-header-fields,--output-fields
    --output-header-format,--output-format
    --output-header-only,--header-only; output header only, e.g. for debugging
    --output-number-of-threads-default

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

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--output-header-fields,--output-fields" ) ) { std::cout << comma::join( comma::csv::names< snark::io::video::header >(), ',' ) << std::endl; return 0; }
        if( options.exists( "--output-header-format,--output-format" ) ) { std::cout << comma::csv::format::value< snark::io::video::header >() << std::endl; return 0; }
        if( options.exists( "--output-number-of-threads-default" ) ) { std::cout << snark::tbb::default_concurrency() << std::endl; return 0; }
        const auto& unnamed = options.unnamed( "--discard,--output-header-fields,--output-fields,--output-header-format,--output-format,--output-header-only,--header-only,--output-number-of-threads-default", "-.*" );
        COMMA_ASSERT_BRIEF( !unnamed.empty(), "please specify video device" );
        COMMA_ASSERT_BRIEF( unnamed.size() <= 2, "expected one video device; got'" << comma::join( unnamed, ' ' ) << "'" );
        auto name = unnamed[0];
        comma::csv::options csv( options );
        csv.format( comma::csv::format::value< snark::io::video::header >( csv.fields, true ) );
        auto output_options = unnamed.size() < 2 ? "-" : unnamed[1];
        typedef comma::csv::split< snark::io::video::header > log_t;
        typedef comma::csv::output_stream< snark::io::video::header > csv_stream_t;
        std::unique_ptr< log_t > log;
        std::unique_ptr< comma::io::ostream > os;
        std::unique_ptr< csv_stream_t > ostream;
        if( output_options.substr( 0, 4 ) == "log:" )
        {
            COMMA_ASSERT_BRIEF( !csv.fields.empty(), "only logging with header is supported, please specify at least one field in --fields" );
            log.reset( log_t::make( output_options, csv ) );
        }
        else
        {
            os = std::make_unique< comma::io::ostream >( output_options );
            ostream = std::make_unique< csv_stream_t >( *( *os ), csv );
        }
        snark::io::video::header header;
        unsigned int width = options.value< unsigned int >( "--width" );
        unsigned int height = options.value< unsigned int >( "--height" );
        //double n = options.value< double >( "--threads,--number-of-threads", snark::tbb::default_concurrency() );
        //unsigned int number_of_threads = ( n >= 1 ? n : snark::tbb::default_concurrency() * n ) + 0.5; // quick and dirty
        //COMMA_ASSERT( number_of_threads > 1, "expected number of threads greater than 1; got: " << number_of_threads << " for --number-of-threads=" << n );
        unsigned int number_of_buffers = options.value< unsigned int >( "--size,--number-of-buffers", 32 );
        unsigned int pixel_size = 4;
        header.width = width / pixel_size;
        header.height = height;
        header.type = 24; // todo! --type,--pixel-type
        comma::saymore() << name << ": video stream: creating..." << std::endl;
        snark::io::video::stream video( name, width, height, number_of_buffers );
        comma::saymore() << name << ": video stream: created" << std::endl;
        comma::signal_flag is_shutdown;
        typedef snark::io::video::stream::record record_t;
        bool discard = options.exists( "--discard" );
        bool header_only = options.exists( "--output-header-only,--header-only" );
        auto read_once = [&]()->record_t { if( is_shutdown ) { video.stop(); return record_t(); } else { return video.read(); } };
        snark::tbb::filter< record_t, void >::type write_filter( snark::tbb::filter_mode::serial_in_order
                                                               , [&]( const record_t& record )
                                                                 {
                                                                     if( !record ) { return; }
                                                                     // todo: check whether we keep up with reader
                                                                     header.t = record.buffer.t;
                                                                     header.count = record.count;
                                                                     static unsigned int size = width * height;
                                                                     auto data = reinterpret_cast< const char* >( record.buffer.data );
                                                                     if( log ) { log->write( header, data, size ); return; }
                                                                     if( !csv.fields.empty() )
                                                                     {
                                                                         header.t = record.buffer.t;
                                                                         header.count = record.count;
                                                                         ostream->write( header );
                                                                     }
                                                                     if( !header_only ) { ( *os )->write( data, size ); }
                                                                     ( *os )->flush();
                                                                 } );
        
        // todo! handle exceptions in read_once() or make it no-throw
        // todo! handle errno eintr
        // todo! expose pixel type (V4L2_PIX_FMT_SRGGB8 etc)

        comma::saymore() << name << ": readers: creating..." << std::endl;
        snark::tbb::bursty_reader< record_t > bursty_reader( read_once, discard ? video.buffers().size() : 0, discard ? 0 : video.buffers().size(), true );
        snark::tbb::filter< void, void >::type filters = bursty_reader.filter() & write_filter;
        comma::saymore() << name << ": readers: created" << std::endl;
        comma::saymore() << name << ": video stream: starting..." << std::endl;
        video.start();
        comma::saymore() << name << ": video stream: started" << std::endl;
        //comma::saymore() << name << ": processing pipeline: running with maximum number of active tokens " << number_of_threads << "..." << std::endl;
        //::tbb::parallel_pipeline( number_of_threads, filters ); // ::tbb::parallel_pipeline( video.buffers().size() + 1, filters ); // while( bursty_reader->wait() ) { ::tbb::parallel_pipeline( 3, filters ); }
        comma::saymore() << name << ": processing pipeline: running with maximum number of active tokens " << number_of_buffers << "..." << std::endl;
        ::tbb::parallel_pipeline( number_of_buffers, filters ); // ::tbb::parallel_pipeline( video.buffers().size() + 1, filters ); // while( bursty_reader->wait() ) { ::tbb::parallel_pipeline( 3, filters ); }
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