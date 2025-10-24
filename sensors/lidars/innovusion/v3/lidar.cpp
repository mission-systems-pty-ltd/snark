// Copyright (c) 2025 Mission Systems Pty Ltd

#include "lidar.h"
#include <comma/application/verbose.h>
#include <comma/base/exception.h>

namespace snark { namespace innovusion {

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
                , const std::string& address
                , unsigned int port
                , unsigned int udp_port
                , InnoMessageCallback message_callback
                , InnoDataPacketCallback data_callback
                , InnoStatusPacketCallback status_callback
                , void* context )
{
    comma::verbose << "inno_lidar_open_live() for " << name << ", " << address << ":" << port << std::endl;
    handle = inno_lidar_open_live( name.c_str(), address.c_str(), port, INNO_LIDAR_PROTOCOL_PCS_UDP, port );
    comma::verbose << "returned handle=" << handle << std::endl;
    if( handle < 0 ) { COMMA_THROW( comma::exception, "inno_lidar_open_live() failed" ); }

    // set SDK to callback with XYZ FRAME
    // other options are XYZ_PACKET and SPHERE_PACKET
    // you'll have to dive in to the sdk headers to see all the implications
    // also look at demo.cpp and sphere2xyz.cpp from the example applications
    //
    // note that the ROS driver at https://github.com/Seyond-Inc/seyond_ros_driver
    // takes the default SPHERE_PACKET and converts to xyz
    //
    // getting the lidar to do the polar to cartesian conversion should be more efficient for the host
    comma::verbose << "setting data type" << std::endl;
    if( inno_lidar_set_callbacks_data_type( handle, INNO_CALLBACK_XYZ_FRAME ) != 0 )
    { COMMA_THROW( comma::exception, "inno_lidar_set_callbacks_data_type() failed" ); }

    comma::verbose << "setting reflectance mode" << std::endl;
    if( inno_lidar_set_reflectance_mode( handle, INNO_REFLECTANCE_MODE_INTENSITY ) != 0 )
    { COMMA_THROW( comma::exception, "inno_lidar_set_reflectance_mode() failed" ); }

    comma::verbose << "setting callbacks" << std::endl;
    if( inno_lidar_set_callbacks( handle, message_callback, data_callback, status_callback, nullptr, context ) != 0 )
    { COMMA_THROW( comma::exception, "inno_lidar_set_callbacks() failed" ); }

    comma::verbose << "initialisation complete" << std::endl;
}

void lidar::start()
{
    int result = inno_lidar_start( handle );
    if( result != 0 ) { COMMA_THROW( comma::exception, "inno_lidar_start() failed with result " << result ); }
}

} } // namespace snark { namespace innovusion {
