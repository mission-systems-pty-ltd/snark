// Copyright (c) 2021-2025 Mission Systems Pty Ltd

#include "../inno_lidar_sdk.h"

#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/last_error.h>
#include <comma/csv/stream.h>
#include <thread>
#include <tbb/concurrent_queue.h>

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

const std::string default_name( "innovusion_lidar" );
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
    std::cerr << "\n    --name=<name>:         device name (max 32 chars); default=" << default_name;
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

static std::atomic< bool > shutdown_requested = false;
static std::atomic< inno_timestamp_us_t > latest_frame_start_time;
static output_type_t output_type = output_type_t::cooked;
static inno_timestamp_us_t max_latency = 0;
static bool fatal_error = false;
static int64_t timeframe_offset_us = 0;

// https://spec.oneapi.io/versions/latest/elements/oneTBB/source/containers/concurrent_queue_cls.html
static tbb::concurrent_queue< inno_frame* > queue;

namespace snark { namespace innovusion {
struct raw_output {};
struct null_output {};
}; };

template< typename T >
struct writer
{
#if INNOVUSION_VERSION_MAJOR == 1
    static void process()
    {
        while( !shutdown_requested )
        {
            inno_frame* frame;
            // non-blocking, as opposed to concurrent_bounded_queue::pop()
            while( queue.try_pop( frame ) && !shutdown_requested )
            {
                // latest_frame is the latest frame received on the queue
                // frame is the frame we are processing now
#if INNOVUSION_VERSION_MAJOR == 3
                InnoTimestampUs latency = latest_frame_start_time - frame->common.ts_start_us;
#else
                inno_timestamp_us_t latency = latest_frame_start_time - frame->ts_us_start;
#endif
                if( comma::verbose )
                {
                    if( latency > 0 ) { std::cerr << comma::verbose.app_name() << ": latency = " << latency << "µs"; }
                    if( latency > max_latency ) { std::cerr << "...dropping frame"; }
                    if( latency > 0 ) { std::cerr << std::endl; }
                }
                if( latency <= max_latency ) { output( frame ); }
                free( (void*)frame );
            }
            std::this_thread::sleep_for( std::chrono::milliseconds( 40 ));
        }
    }
#endif

    // We roll the output routine by hand rather than use comma::csv::binary_output_stream
    // so that we can use a custom write() routine that writes to the stdout file descriptor
    // rather than using iostreams std::cout.
    //
    // This is because we were seeing output errors under load. Similar to that
    // seen in other high data rate multithreaded applications.
    static void output( inno_frame* frame )
    {
        // Implement these few lines as a lambda so they only get executed on
        // the first run through when the static "os" is set.
        // Making this a separate method would complicate the template
        // overloading for raw_output and null_output
        auto make_binary = []()
        {
            comma::csv::options output_csv;
            output_csv.format( comma::csv::format::value< T >() );
            return comma::csv::binary< T >( output_csv.format().string(), output_csv.fields, output_csv.full_xpath, T() );
        };
        static comma::csv::binary< T > binary = make_binary();
        static const size_t record_size = binary.format().size();

        // buf is large enough to contain one complete frame.
        //
        // The size of a frame can vary slightly from one to the next, and will
        // vary from one device to another, so we dynamically size once we
        // receive some data. This typically happens just once upon receipt of
        // the first frame.
        static std::vector< char > buf;

#if INNOVUSION_VERSION_MAJOR == 3
        comma::verbose << "received " << frame->item_number << " points" << std::endl;
        if( frame->item_number * record_size > buf.size() )
        {
            std::cerr << "innovusion-cat: received " << frame->item_number << " points, resizing output buffer" << std::endl;
            buf.resize( static_cast< size_t >( frame->item_number * record_size * 1.05 ));     // 5% buffer to minimise resizes
        }

        for( unsigned int i = 0; i < frame->item_number; i++ )
        {
            binary.put( T( frame, i, timeframe_offset_us ), &buf[ i * record_size ] );
        }
        write( &buf[0], frame->item_number * record_size );
#else
        if( frame->points_number * record_size > buf.size() )
        {
            std::cerr << "innovusion-cat: received " << frame->points_number << " points, resizing output buffer" << std::endl;
            buf.resize( static_cast< size_t >( frame->points_number * record_size * 1.05 ));     // 5% buffer to minimise resizes
        }

        for( unsigned int i = 0; i < frame->points_number; i++ )
        {
            binary.put( T( frame, i, timeframe_offset_us ), &buf[ i * record_size ] );
        }
        write( &buf[0], frame->points_number * record_size );
#endif
    }

    static void write( const char* buf, unsigned int count )
    {
        while( count > 0 )
        {
            ssize_t bytes_written = ::write( 1, buf, count );
            if( bytes_written == -1 ) { COMMA_THROW( comma::last_error::exception, "error" ); }
            if( bytes_written == 0 ) { COMMA_THROW( comma::exception, "write() wrote 0 bytes" ); } // shouldn't occur with stdout
            count -= bytes_written;
            buf += bytes_written;
        }
    }
};

static std::unique_ptr< snark::innovusion::log > api_log;

template< typename T >
struct app
{
    static std::string output_fields() { return comma::join( comma::csv::names< T >( T::full_xpath ), ',' ); }
    static std::string output_format() { return comma::csv::format::value< T >(); }

    static int run( const comma::command_line_options& options )
    {
        if( options.exists( "--output-fields" )) { std::cout << output_fields() << std::endl; return 0; }
        if( options.exists( "--output-format" )) { std::cout << output_format() << std::endl; return 0; }

        std::string address = options.value< std::string >( "--address", snark::innovusion::default_address );
        int port = options.value< unsigned int >( "--port", snark::innovusion::default_port );
#if INNOVUSION_VERSION_MAJOR == 3
        int udp_port = options.value< unsigned int >( "--udp-port", snark::innovusion::default_udp_port );
#endif
        std::string name = options.value< std::string >( "--name", default_name );
        timeframe_offset_us = options.value< int64_t >( "--time-offset", 0 ) * 1000000;

        // set the handling for messages from the api
        api_log.reset( new snark::innovusion::log() );
        api_log->set_logs();

        // default level is "warn"
        if( options.exists( "--debug" )) { api_log->set_log_level( snark::innovusion::log::Level::debug ); }
        else if( options.exists( "--verbose,-v" )) { api_log->set_log_level( snark::innovusion::log::Level::info ); }

        comma::signal_flag is_shutdown;
        inno_lidar_setup_sig_handler();

#if INNOVUSION_VERSION_MAJOR == 1
        std::thread output_thread( &writer<T>::process );
#endif

        snark::innovusion::lidar lidar;
#if INNOVUSION_VERSION_MAJOR == 3
        lidar.init( name, address, port, udp_port, message_callback, data_callback, status_callback );
#else
        lidar.init( name, address, port, alarm_callback, frame_callback );
#endif
        lidar.start();

        while( !is_shutdown && !fatal_error && std::cout.good() )
        {
            std::this_thread::sleep_for( std::chrono::milliseconds( 500 ));
        }
        shutdown_requested = true;
#if INNOVUSION_VERSION_MAJOR == 1
        output_thread.join();
#endif
        if( is_shutdown ) { std::cerr << comma::verbose.app_name() << ": interrupted by signal" << std::endl; }
        if( fatal_error ) { std::cerr << comma::verbose.app_name() << ": fatal error, exiting" << std::endl; return 1; }
        return 0;
    }

#if INNOVUSION_VERSION_MAJOR == 3
    static bool has_newline( const char* str)
    {
        if( !str ) return false;
        size_t len = std::strlen( str );
        return( len > 0 && str[len-1] == '\n' );
    }

    static void message_callback( int lidar_handle, void* context, uint32_t from_remote
                                , enum InnoMessageLevel level, enum InnoMessageCode code, const char* error_message )
    {
        if( api_log->publish_msg( level ))
        {
            std::cerr << comma::verbose.app_name() << ": Msg " << error_message;
            if( ! has_newline( error_message )) { std::cerr << std::endl; }
        }
        if( level == INNO_MESSAGE_LEVEL_FATAL || level == INNO_MESSAGE_LEVEL_CRITICAL ) { fatal_error = true; }
    }

    static int status_callback( int lidar_handle, void* context, const InnoStatusPacket* status )
    {
        return 0;
    }
#else
    static void alarm_callback( int lidar_handle, void* context
                              , enum inno_alarm error_level, enum inno_alarm_code alarm_code, const char* error_message )
    {
        std::cerr << comma::verbose.app_name() << ": [" << snark::innovusion::alarm_type_to_string( error_level ) << "] "
                  << snark::innovusion::alarm_code_to_string( alarm_code )
                  << " \"" << error_message << "\"" << std::endl;
        if( error_level == INNO_ALARM_FATAL || error_level == INNO_ALARM_CRITICAL ) { fatal_error = true; }
    }
#endif

#if INNOVUSION_VERSION_MAJOR == 3
    static int data_callback( int lidar_handle, void* context, const InnoDataPacket* frame )
    {
        //static inno_timestamp_us_t oldest_valid_time = 1609419600000000;        // 2021-01-01
        static inno_timestamp_us_t oldest_valid_time = 0;        // 1970-01-01

        if( frame->confidence_level != INNO_FULL_CONFIDENCE )
        {
            std::cerr << comma::verbose.app_name() << ": dropping frame " << frame->idx
                      << ", confidence level = " << snark::innovusion::confidence_level_to_string( static_cast< InnoConfidenceLevel >( frame->confidence_level )) << std::endl;
            return 1;
        }
        if( frame->common.ts_start_us < oldest_valid_time )
        {
            std::cerr << comma::verbose.app_name() << ": dropping frame " << frame->idx
                      << ", start time of " << frame->common.ts_start_us/1000000 << " (seconds from epoch) too old" << std::endl;
            return 1;
        }
        latest_frame_start_time = frame->common.ts_start_us;
        writer< T >::output( frame );
        return 0;        // they free memory
    }
#else
    static int frame_callback( int lidar_handle, void* context, inno_frame* frame )
    {
        static inno_timestamp_us_t oldest_valid_time = 1609419600000000;        // 2021-01-01

        if( frame->conf_level != 255 )
        {
            std::cerr << comma::verbose.app_name() << ": dropping frame " << frame->idx
                      << ", confidence level = " << (int)frame->conf_level << std::endl;
            return 1;
        }
        if( frame->ts_us_start < oldest_valid_time )
        {
            std::cerr << comma::verbose.app_name() << ": dropping frame " << frame->idx
                      << ", start time of " << frame->ts_us_start/1000000 << " (seconds from epoch) too old" << std::endl;
            return 1;
        }
        latest_frame_start_time = frame->ts_us_start;
        queue.push( frame );
        return 1;        // we free memory
    }
#endif
};

template<> std::string app< snark::innovusion::raw_output >::output_fields()
    { COMMA_THROW( comma::exception, "raw data does not have output fields" ); }
template<> std::string app< snark::innovusion::raw_output >::output_format()
    { COMMA_THROW( comma::exception, "raw data does not have output format" ); }

#if INNOVUSION_VERSION_MAJOR == 3
template<> void writer< snark::innovusion::raw_output >::output( const InnoDataPacket* frame )
{
    std::cout.write( (const char*)&frame->xyz_points[0], frame->item_number * sizeof( InnoXyzPoint ));
}
#else
template<> void writer< snark::innovusion::raw_output >::output( inno_frame* frame )
{
    std::cout.write( (const char*)&frame->points[0], frame->points_number * sizeof( inno_point ));
}
#endif

template<> std::string app< snark::innovusion::null_output >::output_fields()
    { COMMA_THROW( comma::exception, "null data does not have output fields" ); }
template<> std::string app< snark::innovusion::null_output >::output_format()
    { COMMA_THROW( comma::exception, "null data does not have output format" ); }

template<> void writer< snark::innovusion::null_output >::output( inno_frame* frame ) {}

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
                case output_type_t::cooked: return app< snark::innovusion::checksummed< snark::innovusion::output_data_t > >::run( options );
                default: std::cerr << "--checksum only supported for --output-type=cooked" << std::endl; return 1;
            }
        }
        else
        {
            switch( output_type )
            {
                case output_type_t::raw:    return app< snark::innovusion::raw_output >::run( options );
                case output_type_t::cooked: return app< snark::innovusion::output_data_t >::run( options );
                case output_type_t::full:   return app< snark::innovusion::output_data_full_t >::run( options );
                case output_type_t::none:   return app< snark::innovusion::null_output >::run( options );
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
