// Copyright (c) 2021,2022,2025 Mission Systems Pty Ltd

#include <comma/application/command_line_options.h>
#include "app.h"
#include "traits.h"

// Data
// ----------------
//
// We register a callback method to accept a frame whenever it's ready.
//
// The SDK allocates memory for the frame but in the v1 library we can free it, so the logic is:
// callback: add pointer to data to queue and return, indicating that we will free the memory
// processing thread: watch for items on the queue, pop them and output data to stdout, free memory
//
// In the v3 library there is no option to manage the memory so everything is done in the callback:
// callback: receive pointer to data, output to stdout, return


// Alarms and logs
// ----------------
//
// There are two kinds of run-time message:
//
// * the SDK will output messages to a file descriptor set by inno_lidar_set_logs - we use stderr;
//   the log level is set by inno_lidar_set_log_level()
//
// * the device will send messages via a callback set by inno_lidar_set_callbacks()

static void bash_completion( unsigned int const ac, char const* const* av )
{
    for( const auto& o : snark::innovusion::app<void>::available_options() ) { std::cout << " " << o; }
    std::cout << std::endl;
    exit( 0 );
}

static void usage( bool verbose = false )
{
    std::cerr << "\nStream data from an Innovusion lidar";
    std::cerr << "\n";
    std::cerr << "\nUsage: " << comma::verbose.app_name() << " [<options>]";
    std::cerr << "\n";
    std::cerr << "\nOptions:";
    for( const auto& o : snark::innovusion::app<void>::option_descriptions() ) { std::cerr << "\n    " << o; }
    std::cerr << "\n";
    std::cerr << "\nOutput types:";
    std::cerr << "\n    none:   no output, useful for benchmarking the underlying SDK";
    std::cerr << "\n    raw:    raw inno_frame data from SDK";
    std::cerr << "\n    cooked: regular binary data, one packet per point";
    std::cerr << "\n    full:   as for cooked but with additional underlying data";
    std::cerr << "\n";
    std::cerr << "\nCoordinate frame:";
    std::cerr << "\n    The inno_point struct in Innovusion SDK describes the sensor frame as";
    std::cerr << "\n    x up, y right, and z forward. This driver follows that convention.";
    std::cerr << "\n";
    std::cerr << "\nTime frame:";
    std::cerr << "\n    The raw data is in TAI time. To convert to UTC apply a time offset";
    std::cerr << "\n";
    std::cerr << "\nExamples:";
    std::cerr << "\n    " << comma::verbose.app_name() << " --address 192.168.10.40 \\";
    std::cerr << "\n        | io-publish tcp:4444 \\";
    std::cerr << "\n              --size $( " << comma::verbose.app_name() << " --output-format | csv-format size ) \\";
    std::cerr << "\n              -m 1000 --no-flush";
    std::cerr << "\n";
    std::cerr << "\n    # convert timestamps to UTC";
    std::cerr << "\n    " << comma::verbose.app_name() << " --address 192.168.10.40 --time-offset -37";
    std::cerr << "\n";
    std::cerr << "\nUsing Innovusion LIDAR API version " << inno_api_version();
    std::cerr << "\n";
    std::cerr << std::endl;
    exit( 0 );
}

enum class output_type_t { none, raw, cooked, full };

output_type_t output_type_from_string( const std::string& output_type_str )
{
    if( output_type_str == "none" )   { return output_type_t::none; }
    if( output_type_str == "raw" )    { return output_type_t::raw; }
    if( output_type_str == "cooked" ) { return output_type_t::cooked; }
    if( output_type_str == "full" ) { return output_type_t::full; }
    { COMMA_THROW( comma::exception, "unknown output type \"" << output_type_str << "\"" ); }
}

static output_type_t output_type = output_type_t::cooked;

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" ) ) bash_completion( argc, argv );
        if( options.exists( "--debug" )) { comma::verbose.init( true, argv[0] ); }

        output_type = output_type_from_string( options.value< std::string >( "--output-type", snark::innovusion::default_output_type ));
        bool checksum = options.exists( "--checksum" );

        comma::verbose << "starting..." << std::endl;

        if( checksum )
        {
            switch( output_type )
            {
                case output_type_t::cooked: return snark::innovusion::app< snark::innovusion::checksummed< snark::innovusion::output_data_t > >().run( options );
                default: std::cerr << "--checksum only supported for --output-type=cooked" << std::endl; return 1;
            }
        }
        else
        {
            switch( output_type )
            {
                case output_type_t::raw:    return snark::innovusion::app< snark::innovusion::raw_output >().run( options );
                case output_type_t::cooked: return snark::innovusion::app< snark::innovusion::output_data_t >().run( options );
                case output_type_t::full:   return snark::innovusion::app< snark::innovusion::output_data_full_t >().run( options );
                case output_type_t::none:   return snark::innovusion::app< snark::innovusion::null_output >().run( options );
            }
        }
    }
    catch( std::exception& ex )
    {
        std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << comma::verbose.app_name() << ": unknown exception" << std::endl;
    }
    return 1;
}
