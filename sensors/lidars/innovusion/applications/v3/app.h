// Copyright (c) 2025 Mission Systems Pty Ltd
//
// the app coordinates logging and messages handling

#pragma once

#include "../app-base.h"
#include "lidar.h"
#include "log.h"
#include "types.h"
#include <atomic>

namespace snark { namespace innovusion {

static std::atomic< inno_timestamp_us_t > latest_frame_start_time;

template< typename T >
class app : public app_base<T>
{
public:
    static std::vector< std::string_view >& available_options();
    static std::vector< std::string >& option_descriptions();

    int run( const comma::command_line_options& options )
    {
        comma::verbose << "appv3::run()" << std::endl;
        this->api_log = std::make_unique< log >();

        if( app_base<T>::init( options, default_address, default_port ))
        {
            int udp_port = options.value< unsigned int >( "--udp-port", snark::innovusion::default_udp_port );

            inno_lidar_setup_sig_handler();

            snark::innovusion::lidar lidar;
            lidar.init( this->name, this->address, this->port, udp_port, message_callback, data_callback, status_callback );
            lidar.start();

            return app_base<T>::run();
        }
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
        if( static_cast< app* >( app_base<T>::instance )->api_log->publish_msg( level ))
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

template<>
std::vector< std::string_view >& app<void>::available_options()
{
    static std::vector< std::string_view > options = app_base<void>::available_options();
    options.push_back( "--udp-port" );
    return options;
}

template<>
std::vector< std::string >& app<void>::option_descriptions()
{
    static std::vector< std::string > descriptions = app_base<void>::option_descriptions();
    static std::vector< std::string > extra_descriptions = {
        "--address=<ip>:        device address; default=" + snark::innovusion::default_address,
        "--port=<num>:          device port; default=" + std::to_string( snark::innovusion::default_port ),
        "--max-latency=<ms>:    maximum latency in ms; default=" + std::to_string( snark::innovusion::default_max_latency ),
        "--udp-port=<num>:      device udp port; default=" + std::to_string( snark::innovusion::default_udp_port )
    };
    descriptions.insert( descriptions.end(), extra_descriptions.begin(), extra_descriptions.end() );
    return descriptions;
}

} } // namespace snark { namespace innovusion {
