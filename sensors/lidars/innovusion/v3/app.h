// Copyright (c) 2025 Mission Systems Pty Ltd
//
// the app coordinates logging and messages handling

#pragma once

#include "lidar.h"
#include "log.h"
#include "types.h"
#include <comma/application/signal_flag.h>
#include <atomic>
#include <thread>

namespace snark { namespace innovusion {

const std::string default_name( "innovusion_lidar" );

static bool fatal_error = false;
static std::atomic< inno_timestamp_us_t > latest_frame_start_time;
static std::atomic< bool > shutdown_requested = false;

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
        int udp_port = options.value< unsigned int >( "--udp-port", snark::innovusion::default_udp_port );
        std::string name = options.value< std::string >( "--name", default_name );
        snark::innovusion::timeframe_offset_us = options.value< int64_t >( "--time-offset", 0 ) * 1000000;

        // set the handling for messages from the api
        api_log.reset( new log() );
        api_log->set_logs();

        // default level is "warn"
        if( options.exists( "--debug" )) { api_log->set_log_level( snark::innovusion::log::Level::debug ); }
        else if( options.exists( "--verbose,-v" )) { api_log->set_log_level( snark::innovusion::log::Level::info ); }

        comma::signal_flag is_shutdown;
        inno_lidar_setup_sig_handler();

        snark::innovusion::lidar lidar;
        lidar.init( name, address, port, udp_port, message_callback, data_callback, status_callback );
        lidar.start();

        while( !is_shutdown && !fatal_error && std::cout.good() )
        {
            std::this_thread::sleep_for( std::chrono::milliseconds( 500 ));
        }
        shutdown_requested = true;
        if( is_shutdown ) { std::cerr << comma::verbose.app_name() << ": interrupted by signal" << std::endl; }
        if( fatal_error ) { std::cerr << comma::verbose.app_name() << ": fatal error, exiting" << std::endl; return 1; }
        return 0;
    }

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
        snark::innovusion::writer< T >::output( frame );
        return 0;        // they free memory
    }
};

} } // namespace snark { namespace innovusion {
