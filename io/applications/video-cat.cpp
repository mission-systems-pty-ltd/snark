// Copyright (c) 2024 Mission Systems

#include <functional>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/string/string.h>
#include "../../tbb/bursty_reader.h"
#include "../../tbb/types.h"
#include "../video.h"

void usage( bool )
{
    std::cerr << R"(
usage: video-cat <path> <options>

options
    <path>; video device path, e.g. "/dev/video0"
    --height=<rows>
    --width=<bytes>
    --size,--number-of-buffers=<n>; default=32

todo...

)";
    exit( 0 );
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        const auto& unnamed = options.unnamed( "", "-.*" );
        COMMA_ASSERT_BRIEF( !unnamed.empty(), "please specify video device" );
        COMMA_ASSERT_BRIEF( unnamed.size() == 1, "expected one video device; got'" << comma::join( unnamed, ' ' ) << "'" );
        auto name = unnamed[0];
        snark::io::video::stream video( name
                                      , options.value< unsigned int >( "--width" )
                                      , options.value< unsigned int >( "--height" )
                                      , options.value< unsigned int >( "--size,--number-of-buffers", 32 ) );
        comma::signal_flag is_shutdown;
        typedef std::pair< unsigned int, const snark::timestamped< void* > > input_t;
        snark::tbb::filter< void, input_t >::type read_filter( snark::tbb::filter_mode::serial_in_order
                                                             , [&]( ::tbb::flow_control& flow )->input_t
                                                               {
                                                                   if( !is_shutdown ) { return video.read(); }
                                                                   flow.stop();
                                                                   return { 0, snark::timestamped< void* >( nullptr ) };
                                                               } );
        video.start();

        // todo!
        // snark::tbb::filter< void, void >::type filters = bursty_reader->filter() & partition_filter & write_filter;
        // ::tbb::parallel_pipeline( 3, filters ); // while( bursty_reader->wait() ) { ::tbb::parallel_pipeline( 3, filters ); }


        // todo
        // if( discard )
        // {
        //snark::tbb::bursty_reader< input_t > bursty_reader( std::bind( read_video, std::placeholders::_1 ) );
        //     bursty_reader.reset( new snark::tbb::bursty_reader< block_t* >( &read_block_bursty_ ) );
        //     snark::tbb::filter< void, void >::type filters = bursty_reader->filter() & partition_filter & write_filter;
        //     ::tbb::parallel_pipeline( 3, filters ); // while( bursty_reader->wait() ) { ::tbb::parallel_pipeline( 3, filters ); }
        //     bursty_reader->join();
        // }
        // else
        // {
        //     snark::tbb::filter< void, block_t* >::type read_filter( snark::tbb::filter_mode::serial_in_order, &read_block_ );
        //     snark::tbb::filter< void, void >::type filters = read_filter & partition_filter & write_filter;
        //     ::tbb::parallel_pipeline( 3, filters );
        // }

        //bursty_reader.join();
        video.stop();
        return 0;
    }
    catch( const std::exception& ex ) { comma::say() << ex.what() << std::endl; }
    catch( ... ) { comma::say() << "unknown exception" << std::endl; }
    return 1;
}