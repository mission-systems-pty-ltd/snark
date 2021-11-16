// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2017 The University of Sydney

#pragma once

#include "messages.h"
#include "stream.h"
#include <vector>

namespace snark { namespace navigation { namespace advanced_navigation {

// spatial dual device class
class device
{
public:
    /// name is serial or network port or - for stdin
    device( const std::string& name, const advanced_navigation::options& options=advanced_navigation::options() );
    virtual ~device() {}
    void process();
    void send_ntrip( const std::vector<char>& buf );
    void send( const messages::command& command );
    comma::io::file_descriptor fd();

protected:
    virtual void handle( const messages::acknowledgement* msg ) {}
    virtual void handle( const messages::system_state* msg ) {}
    virtual void handle( const messages::position_standard_deviation* msg ) {}
    virtual void handle( const messages::velocity_standard_deviation* msg ) {}
    virtual void handle( const messages::orientation_standard_deviation* msg ) {}
    virtual void handle( const messages::raw_sensors* msg ) {}
    virtual void handle( const messages::satellites* msg ) {}
    virtual void handle( const messages::magnetic_calibration_status* msg ) {}
    virtual void handle_raw( messages::header* msg_header, const char* msg_data,std::size_t msg_data_length) {}

private:
    std::unique_ptr< advanced_navigation::stream > stream;
    std::vector< char > buf;
    unsigned int index;
    unsigned int head;
    messages::header* msg_header;
};

} } } //namespace snark { namespace navigation { namespace advanced_navigation {
