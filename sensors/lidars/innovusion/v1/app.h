// Copyright (c) 2025 Mission Systems Pty Ltd
//
// the app coordinates logging and messages handling

#pragma once

#include "../common/app.h"
#include "lidar.h"
#include "log.h"
#include "types.h"
#include <tbb/concurrent_queue.h>

namespace snark { namespace innovusion {

template< typename T >
class app : public app_base<T>
{
public:
    static std::vector< std::string >& option_descriptions();

    int run( const comma::command_line_options& options )
    {
        this->api_log = std::make_unique< log >();

        if( app_base<T>::init( options, default_address, default_port ))
        {
            inno_timestamp_us_t max_latency = options.value< unsigned int >( "--max-latency", default_max_latency ) * 1000; // µs

            inno_lidar_setup_sig_handler();

            std::thread output_thread( &snark::innovusion::writer<T>::process, std::ref( queue ), std::ref( this->shutdown_requested ), max_latency );

            snark::innovusion::lidar lidar;
            lidar.init( this->name, this->address, this->port, alarm_callback, frame_callback );
            lidar.start();

            int result = app_base<T>::run();
            output_thread.join();
            return result;
        }
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
        static_cast< app* >( app_base<T>::instance )->queue.push( frame );
        return 1;        // we free memory
    }

private:
    // https://spec.oneapi.io/versions/latest/elements/oneTBB/source/containers/concurrent_queue_cls.html
    tbb::concurrent_queue< inno_frame* > queue;
};

template<>
std::vector< std::string >& app<void>::option_descriptions()
{
    static std::vector< std::string > descriptions = app_base<void>::option_descriptions();
    static std::vector< std::string > extra_descriptions = {
        "--address=<ip>:        device address; default=" + snark::innovusion::default_address,
        "--port=<num>:          device port; default=" + std::to_string( snark::innovusion::default_port ),
        "--max-latency=<ms>:    maximum latency in ms; default=" + std::to_string( snark::innovusion::default_max_latency ),
    };
    descriptions.insert( descriptions.end(), extra_descriptions.begin(), extra_descriptions.end() );
    return descriptions;
}

} } // namespace snark { namespace innovusion {
