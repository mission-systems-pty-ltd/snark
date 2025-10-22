// Copyright (c) 2021 Mission Systems Pty Ltd

#include "lidar.h"
#include <comma/application/verbose.h>
#include <comma/base/exception.h>

namespace snark { namespace innovusion {

int64_t timeframe_offset_us = 0;

lidar::lidar()
    : handle( 0 )
{}

lidar::~lidar()
{
    if( handle != 0 )
    {
        // note that if there's no connection to the lidar, this call hangs
        int result = inno_lidar_stop( handle );
        if( result != 0 ) { std::cerr << "inno_lidar_stop() returned " << result << std::endl; }
        result = inno_lidar_close( handle );
        if( result != 0 ) { std::cerr << "inno_lidar_close() returned " << result << std::endl; }
    }
}

void lidar::init( const std::string& name
                , const std::string& address, unsigned int port
                , inno_lidar_alarm_callback_t alarm_callback
                , inno_lidar_frame_callback_t frame_callback
                , void* context )
{
    handle = inno_lidar_open_live( name.c_str(), address.c_str(), port, 1 );

    // TODO: set model (not required for live?) and config yaml file
    if( inno_lidar_set_parameters( handle, nullptr, nullptr, nullptr ) != 0 )
    { COMMA_THROW( comma::exception, "inno_lidar_set_parameters() failed" ); }

    if( inno_lidar_set_reflectance_mode( handle, REFLECTANCE_MODE_INTENSITY ) != 0 )
    { COMMA_THROW( comma::exception, "inno_lidar_set_reflectance_mode() failed" ); }

    if( inno_lidar_set_callbacks( handle, alarm_callback, frame_callback, nullptr, context ) != 0 )
    { COMMA_THROW( comma::exception, "inno_lidar_set_callbacks() failed" ); }
}

void lidar::start()
{
    int result = inno_lidar_start( handle );
    if( result != 0 ) { COMMA_THROW( comma::exception, "inno_lidar_start() failed with result " << result ); }
}

} } // namespace snark { namespace innovusion {
