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
#include <tbb/concurrent_queue.h>

namespace snark { namespace innovusion {

const std::string default_name( "innovusion_lidar" );

static bool fatal_error = false;
static std::atomic< inno_timestamp_us_t > latest_frame_start_time;
static std::atomic< bool > shutdown_requested = false;

// https://spec.oneapi.io/versions/latest/elements/oneTBB/source/containers/concurrent_queue_cls.html
static tbb::concurrent_queue< inno_frame* > queue;

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

        std::thread output_thread( &snark::innovusion::writer<T>::process );

        snark::innovusion::lidar lidar;
        lidar.init( name, address, port, alarm_callback, frame_callback );
        lidar.start();

        while( !is_shutdown && !fatal_error && std::cout.good() )
        {
            std::this_thread::sleep_for( std::chrono::milliseconds( 500 ));
        }
        shutdown_requested = true;
        output_thread.join();
        if( is_shutdown ) { std::cerr << comma::verbose.app_name() << ": interrupted by signal" << std::endl; }
        if( fatal_error ) { std::cerr << comma::verbose.app_name() << ": fatal error, exiting" << std::endl; return 1; }
        return 0;
    }

    static void alarm_callback( int lidar_handle, void* context
                              , enum inno_alarm error_level, enum inno_alarm_code alarm_code, const char* error_message )
    {
        std::cerr << comma::verbose.app_name() << ": [" << snark::innovusion::alarm_type_to_string( error_level ) << "] "
                  << snark::innovusion::alarm_code_to_string( alarm_code )
                  << " \"" << error_message << "\"" << std::endl;
        if( error_level == INNO_ALARM_FATAL || error_level == INNO_ALARM_CRITICAL ) { fatal_error = true; }
    }

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
};

} } // namespace snark { namespace innovusion {
