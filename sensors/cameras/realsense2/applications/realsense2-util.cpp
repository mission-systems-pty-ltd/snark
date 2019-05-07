// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2018 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <comma/base/exception.h>
#include <librealsense2/rs.hpp>
#include <vector>
#include <string>
#include <iostream>

void usage(bool detail)
{
    std::cerr<<"Intel realsense2 API utility" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "usage: " << comma::verbose.app_name() << "<operation> [ <options> ]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations" << std::endl;
    std::cerr << "    list,list-devices: output available devices and exit, fields: name, serial no, port"<< std::endl;
    std::cerr << std::endl;
    std::cerr << "    reset: send hardware reset to all devices"<< std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --all-devices,--all; reset all available devices" << std::endl;
    std::cerr << "            --device=[<serial_number>]; device to reset (multiple --device options allowed)" << std::endl;
    std::cerr << std::endl;
}

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        std::vector<std::string> unnamed=options.unnamed( "--verbose,-v,--all-devices,--all", "-.*" );
        if(unnamed.size()!=1) { COMMA_THROW( comma::exception, "expected one unnamed arguement, got: "<<unnamed.size()); }
        rs2::context context;
        auto devices=context.query_devices();
        if(unnamed[0]=="list" || unnamed[0]=="list-devices")
        {
            for(rs2::device dev : devices)
            {
                std::cout<<dev.get_info(RS2_CAMERA_INFO_NAME)<<","<<dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)<<","<<dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT)<<std::endl;
            }
        }
        else if(unnamed[0]=="reset")
        {
            bool all=options.exists("--all-devices,--all");
            std::vector<std::string> device_serial_nos=options.values<std::string>("--device");
            if(!all && device_serial_nos.empty()) { COMMA_THROW(comma::exception, "reset: either --all or --device must be specified"); }
            for(rs2::device dev : devices)
            {
                if(all)
                {
                    dev.hardware_reset();
                }
                else
                {
                    std::string serial_no=dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                    for(auto& i : device_serial_nos)
                    {
                        if(i==serial_no)
                        {
                            dev.hardware_reset();
                            break;
                        }
                    }
                }
            }
        }
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << comma::verbose.app_name() << ": exception: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl;
    }
    return 1;
}
