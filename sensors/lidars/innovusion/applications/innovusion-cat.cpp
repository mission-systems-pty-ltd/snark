// Copyright (c) 2021,2022,2025 Mission Systems Pty Ltd

#include "app.h"
#include "traits.h"
#include "version.h"

#include <comma/application/command_line_options.h>

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

const std::string default_output_type( "cooked" );
const unsigned int default_max_latency( 0 );

static void bash_completion( unsigned int const ac, char const* const* av )
{
    static const char* completion_options =
        " --help -h --verbose -v --debug"
        " --output-fields --output-format --output-type"
        " --address --port --name"
#if INNOVUSION_VERSION_MAJOR == 3
        " --udp-port"
#endif
        " --max-latency --sample-data --time-offset --checksum"
        ;
    std::cout << completion_options << std::endl;
    exit( 0 );
}

static void usage( bool verbose = false )
{
    std::cerr << "\nStream data from an Innovusion lidar";
    std::cerr << "\n";
    std::cerr << "\nUsage: " << comma::verbose.app_name() << " [<options>]";
    std::cerr << "\n";
    std::cerr << "\nOptions:";
    std::cerr << "\n    --help,-h:             show this help";
    std::cerr << "\n    --verbose,-v:          more output to stderr";
    std::cerr << "\n    --debug:               even more output";
    std::cerr << "\n    --address=<ip>:        device address; default=" << snark::innovusion::default_address;
    std::cerr << "\n    --port=<num>:          device port; default=" << snark::innovusion::default_port;
#if INNOVUSION_VERSION_MAJOR == 3
    std::cerr << "\n    --udp-port=<num>:      device udp port; default=" << snark::innovusion::default_udp_port;
#endif
    std::cerr << "\n    --max-latency=<ms>:    maximum latency in ms; default=" << default_max_latency;
    std::cerr << "\n    --name=<name>:         device name (max 32 chars); default="; // TODO: << default_name;
    std::cerr << "\n    --checksum:            add crc checksum to output (cooked data only)";
    std::cerr << "\n    --output-fields:       print output fields for cooked or full data and exit";
    std::cerr << "\n    --output-format:       print output format for cooked or full data and exit";
    std::cerr << "\n    --output-type=<type>:  one of none, raw, cooked, full; default=" << default_output_type;
    std::cerr << "\n    --sample-data=[<dir>]: TODO: read saved data from <dir>";
    std::cerr << "\n    --time-offset=[<sec>]: offset timestamps by given seconds";
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
static inno_timestamp_us_t max_latency = 0;

template<> std::string snark::innovusion::app< snark::innovusion::raw_output >::output_fields()
    { COMMA_THROW( comma::exception, "raw data does not have output fields" ); }
template<> std::string snark::innovusion::app< snark::innovusion::raw_output >::output_format()
    { COMMA_THROW( comma::exception, "raw data does not have output format" ); }

template<> std::string snark::innovusion::app< snark::innovusion::null_output >::output_fields()
    { COMMA_THROW( comma::exception, "null data does not have output fields" ); }
template<> std::string snark::innovusion::app< snark::innovusion::null_output >::output_format()
    { COMMA_THROW( comma::exception, "null data does not have output format" ); }

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" ) ) bash_completion( argc, argv );
        if( options.exists( "--debug" )) { comma::verbose.init( true, argv[0] ); }

        output_type = output_type_from_string( options.value< std::string >( "--output-type", default_output_type ));
        max_latency = options.value< unsigned int >( "--max-latency", default_max_latency ) * 1000; // µs
        bool checksum = options.exists( "--checksum" );

        comma::verbose << "starting..." << std::endl;

        if( checksum )
        {
            switch( output_type )
            {
                case output_type_t::cooked: return snark::innovusion::app< snark::innovusion::checksummed< snark::innovusion::output_data_t > >::run( options );
                default: std::cerr << "--checksum only supported for --output-type=cooked" << std::endl; return 1;
            }
        }
        else
        {
            switch( output_type )
            {
                case output_type_t::raw:    return snark::innovusion::app< snark::innovusion::raw_output >::run( options );
                case output_type_t::cooked: return snark::innovusion::app< snark::innovusion::output_data_t >::run( options );
                case output_type_t::full:   return snark::innovusion::app< snark::innovusion::output_data_full_t >::run( options );
                case output_type_t::none:   return snark::innovusion::app< snark::innovusion::null_output >::run( options );
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
