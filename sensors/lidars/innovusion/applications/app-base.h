// Copyright (c) 2025 Mission Systems Pty Ltd
//
// the app coordinates logging and messages handling

#pragma once

#include "lidar.h"
#include "log.h"
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/format.h>
#include <comma/csv/names.h>
#include <thread>

namespace snark { namespace innovusion {

const std::string default_name( "innovusion_lidar" );
const std::string default_output_type( "cooked" );

static bool fatal_error = false;

template< typename T >
class app_base
{
public:
    app_base()
        : shutdown_requested( false )
    { instance = this; }

    virtual ~app_base() = default;

    static std::vector< std::string_view >& available_options();
    static std::vector< std::string >& option_descriptions();

    std::string output_fields() { return comma::join( comma::csv::names< T >( T::full_xpath ), ',' ); }
    std::string output_format() { return comma::csv::format::value< T >(); }

    bool init( const comma::command_line_options& options
             , const std::string& default_address
             , int default_port )
    {
        if( options.exists( "--output-fields" )) { std::cout << this->output_fields() << std::endl; return false; }
        if( options.exists( "--output-format" )) { std::cout << this->output_format() << std::endl; return false; }

        address = options.value< std::string >( "--address", default_address );
        port = options.value< unsigned int >( "--port", default_port );
        name = options.value< std::string >( "--name", default_name );
        timeframe_offset_us = options.value< int64_t >( "--time-offset", 0 ) * 1000000;

        // set the handling for messages from the api
        api_log->set_logs();

        // default level is "warn"
        if( options.exists( "--debug" )) { api_log->set_log_level( snark::innovusion::log_base::Level::debug ); }
        else if( options.exists( "--verbose,-v" )) { api_log->set_log_level( snark::innovusion::log_base::Level::info ); }

        return true;
    }

    int run()
    {
        comma::signal_flag is_shutdown;

        while( !is_shutdown && !fatal_error && std::cout.good() )
        {
            std::this_thread::sleep_for( std::chrono::milliseconds( 500 ));
        }
        shutdown_requested = true;
        if( is_shutdown ) { std::cerr << comma::verbose.app_name() << ": interrupted by signal" << std::endl; }
        if( fatal_error ) { std::cerr << comma::verbose.app_name() << ": fatal error, exiting" << std::endl; return 1; }
        return 0;
    }

protected:
    static app_base<T>* instance;

    std::unique_ptr< snark::innovusion::log_base > api_log;
    std::atomic< bool > shutdown_requested;

    std::string address;
    int port;
    std::string name;
};

template<>
std::vector< std::string_view >& app_base<void>::available_options()
{
    static std::vector< std::string_view > options = {
        "--help", "-h", "--verbose", "-v", "--debug",
        "--output-fields", "--output-format", "--output-type",
        "--address", "--port", "--name",
        "--max-latency", "--sample-data", "--time-offset", "--checksum"
    };
    return options;
}

template<>
std::vector< std::string >& app_base<void>::option_descriptions()
{
    static std::vector< std::string > descriptions = {
        "--help,-h:             show this help",
        "--verbose,-v:          more output to stderr",
        "--debug:               even more output",
        "--name=<name>:         device name (max 32 chars); default=" + snark::innovusion::default_name,
        "--checksum:            add crc checksum to output (cooked data only)",
        "--output-fields:       print output fields for cooked or full data and exit",
        "--output-format:       print output format for cooked or full data and exit",
        "--output-type=<type>:  one of none, raw, cooked, full; default=" + snark::innovusion::default_output_type,
        "--sample-data=[<dir>]: TODO: read saved data from <dir>",
        "--time-offset=[<sec>]: offset timestamps by given seconds"
    };
    return descriptions;
}

template< typename T >
app_base<T>* app_base<T>::instance = nullptr;

template<> std::string snark::innovusion::app_base< snark::innovusion::raw_output >::output_fields()
    { COMMA_THROW( comma::exception, "raw data does not have output fields" ); }
template<> std::string snark::innovusion::app_base< snark::innovusion::raw_output >::output_format()
    { COMMA_THROW( comma::exception, "raw data does not have output format" ); }

template<> std::string snark::innovusion::app_base< snark::innovusion::null_output >::output_fields()
    { COMMA_THROW( comma::exception, "null data does not have output fields" ); }
template<> std::string snark::innovusion::app_base< snark::innovusion::null_output >::output_format()
    { COMMA_THROW( comma::exception, "null data does not have output format" ); }

} } // namespace snark { namespace innovusion {
