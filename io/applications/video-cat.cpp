// Copyright (c) 2024 Mission Systems

#include <functional>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/names.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include "../../tbb/bursty_reader.h"
#include "../../tbb/types.h"
#include "../video.h"

void usage( bool verbose )
{
    std::cerr << R"(
usage: video-cat <path> <options>

options
    <path>; video device path, e.g. "/dev/video0"
    --height=<rows>
    --width=<bytes>
    --size,--number-of-buffers=<n>; default=32
output options
    --fields=[<fields>]; header output fields: t,width,height,type,count
                         default: no header
    --output-header-fields,--output-fields
    --output-header-format,--output-format

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

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--output-header-fields,--output-fields" ) ) { std::cout << comma::join( comma::csv::names< snark::io::video::header >(), ',' ) << std::endl; return 0; }
        if( options.exists( "--output-header-format,--output-format" ) ) { std::cout << comma::csv::format::value< snark::io::video::header >() << std::endl; return 0; }
        const auto& unnamed = options.unnamed( "--discard,--output-header-fields,--output-fields,--output-header-format,--output-format", "-.*" );
        COMMA_ASSERT_BRIEF( !unnamed.empty(), "please specify video device" );
        COMMA_ASSERT_BRIEF( unnamed.size() == 1, "expected one video device; got'" << comma::join( unnamed, ' ' ) << "'" );
        auto name = unnamed[0];
        comma::csv::options csv( options );
        csv.format( comma::csv::format::value< snark::io::video::header >( csv.fields, true ) );
        comma::csv::output_stream< snark::io::video::header > ostream( std::cout, csv );
        snark::io::video::header header;
        header.width = options.value< unsigned int >( "--width" );
        header.height = options.value< unsigned int >( "--height" );
        snark::io::video::stream video( name, header.width, header.height, options.value< unsigned int >( "--size,--number-of-buffers", 32 ) );
        comma::signal_flag is_shutdown;
        typedef std::pair< unsigned int, const snark::timestamped< void* > > input_t;
        bool discard = options.exists( "--discard" );


        // todo! ordering filter!!! or queue!!! or bursty non-discarding reader!!!
        snark::tbb::filter< void, input_t >::type read_filter( snark::tbb::filter_mode::serial_in_order
                                                             , [&]( ::tbb::flow_control& flow )->input_t
                                                               {
                                                                   if( !is_shutdown ) { return video.read(); }
                                                                   flow.stop();
                                                                   return { 0, snark::timestamped< void* >( nullptr ) };
                                                               } );
        snark::tbb::filter< input_t, void >::type write_filter( snark::tbb::filter_mode::serial_in_order
                                                              , [&]( input_t input )
                                                                {
                                                                    // todo: check whether we keep up with reader
                                                                    if( !csv.fields.empty() )
                                                                    {
                                                                        header.t = input.second.t;
                                                                        header.count = input.first;
                                                                        ostream.write( header );
                                                                    }
                                                                } );
        video.start();
        if( discard )
        {
            COMMA_THROW( comma::exception, "discard: todo" );
        //     snark::tbb::bursty_reader< input_t > bursty_reader( std::bind( read_video, std::placeholders::_1 ) );
        //     bursty_reader.reset( new snark::tbb::bursty_reader< block_t* >( &read_block_bursty_ ) );
        //     snark::tbb::filter< void, void >::type filters = bursty_reader->filter() & partition_filter & write_filter;
        //     ::tbb::parallel_pipeline( 3, filters ); // while( bursty_reader->wait() ) { ::tbb::parallel_pipeline( 3, filters ); }
        //     bursty_reader->join();
        }
        else
        {
            snark::tbb::filter< void, void >::type filters = read_filter & write_filter;
            ::tbb::parallel_pipeline( video.buffers().size() + 1, filters );
        }
        video.stop();
        return 0;
    }
    catch( const std::exception& ex ) { comma::say() << ex.what() << std::endl; }
    catch( ... ) { comma::say() << "unknown exception" << std::endl; }
    return 1;
}
