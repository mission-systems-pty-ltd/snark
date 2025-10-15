// Copyright (c) 2021 Mission Systems Pty Ltd

#pragma once

#include <inno_lidar_api.h>
#include <string>

namespace snark { namespace innovusion {

const std::string default_address( "172.168.1.10" );
const unsigned int default_port( 8001 );

// thin wrapper around the Innovusion API

class lidar
{
public:
    lidar();
    ~lidar();

    void init( const std::string& name
             , const std::string& address, unsigned int port
             , inno_lidar_alarm_callback_t alarm_callback
             , inno_lidar_frame_callback_t frame_callback
             , void* context = nullptr );

    void start();

private:
    int handle;
};

} } // namespace snark { namespace innovusion {
