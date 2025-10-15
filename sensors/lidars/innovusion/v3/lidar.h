// Copyright (c) 2025 Mission Systems Pty Ltd

#pragma once

#include <sdk_common/inno_lidar_api.h>
#include <string>

namespace snark { namespace innovusion {

const std::string default_address( "172.168.1.10" );
const unsigned int default_port( 8010 );
const unsigned int default_udp_port( 8010 );

// thin wrapper around the Innovusion API

class lidar
{
public:
    lidar();
    ~lidar();

    void init( const std::string& name
             , const std::string& address
             , unsigned int port
             , unsigned int udp_port
             , InnoMessageCallback message_callback
             , InnoDataPacketCallback data_callback
             , InnoStatusPacketCallback status_callback
             , void* context = nullptr );

    void start();

private:
    int handle;
};

} } // namespace snark { namespace innovusion {
