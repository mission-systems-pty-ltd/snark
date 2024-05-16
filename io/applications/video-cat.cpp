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
        const auto& unnamed = options.unnamed( "--discard,--output-header-fields,--output-fields,--output-header-format,--output-format,--output-header-only,--header-only", "-.*" );
        COMMA_ASSERT_BRIEF( !unnamed.empty(), "please specify video device" );
        COMMA_ASSERT_BRIEF( unnamed.size() <= 2, "expected one video device; got'" << comma::join( unnamed, ' ' ) << "'" );
        auto name = unnamed[0];
        auto output_options = unnamed.size() < 2 ? "-" : unnamed[1];
        //std::unique_ptr< log = comma::csv::split< snark::io::video::header >::make( output_options );

        // todo: io::stream vs split::stream
        // todo: log: plug in

        comma::csv::options csv( options );
        csv.format( comma::csv::format::value< snark::io::video::header >( csv.fields, true ) );
        comma::csv::output_stream< snark::io::video::header > ostream( std::cout, csv );
        snark::io::video::header header;
        unsigned int width = options.value< unsigned int >( "--width" );
        unsigned int height = options.value< unsigned int >( "--height" );
        unsigned int pixel_size = 4;
        header.width = width / pixel_size;
        header.height = height;
        header.type = 24; // todo! --type,--pixel-type
        snark::io::video::stream video( name, width, height, options.value< unsigned int >( "--size,--number-of-buffers", 32 ) );
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
                                                                     static unsigned int size = width * height;
                                                                     if( !csv.fields.empty() )
                                                                     {
                                                                         header.t = record.buffer.t;
                                                                         header.count = record.count;
                                                                         ostream.write( header );
                                                                     }
                                                                     if( !header_only ) { std::cout.write( reinterpret_cast< const char* >( record.buffer.data ), size ); }
                                                                     std::cout.flush();
                                                                 } );
        video.start();

        // todo! handle exceptions in read_once() or make it no-throw
        // todo! handle errno eintr
        // todo! expose pixel type (V4L2_PIX_FMT_SRGGB8 etc)

        snark::tbb::bursty_reader< record_t > bursty_reader( read_once, discard ? video.buffers().size() : 0, video.buffers().size() );
        snark::tbb::filter< void, void >::type filters = bursty_reader.filter() & write_filter;
        ::tbb::parallel_pipeline( video.buffers().size() + 1, filters ); // while( bursty_reader->wait() ) { ::tbb::parallel_pipeline( 3, filters ); }
        video.stop();
        bursty_reader.join();
        return 0;
    }
    catch( const std::exception& ex ) { comma::say() << ex.what() << std::endl; }
    catch( ... ) { comma::say() << "unknown exception" << std::endl; }
    return 1;
}