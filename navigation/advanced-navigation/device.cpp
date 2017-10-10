// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2017 The University of Sydney
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

/// @author Navid Pirmarzdashti

#include "device.h"
#include <iostream>
#include <comma/application/verbose.h>

namespace snark { namespace navigation { namespace advanced_navigation {

device::device(const std::string& name,int baud_rate) : port(service,name),buf(2600),index(0),head(0),msg_header(NULL)
{
    port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    port.set_option(boost::asio::serial_port_base::character_size(8));
    port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
}
void device::process()
{
    static messages::header* skipper=NULL;
    static unsigned debug_count=0;
    if(head>0 && index > buf.size()/2)
    {
        if(index-head>0)
        {
            //relocate
            memmove(&buf[0],&buf[head],index-head);
            index-=head;
            head=0;
        }
        else
        {
            index=head=0;
        }
    }
    int to_read=buf.size()-index;
    if(to_read>0)
    {
        boost::system::error_code ec;
        unsigned read_size=port.read_some(boost::asio::buffer(&buf[index],to_read),ec);
//         unsigned read_size=boost::asio::read(port, boost::asio::buffer(&buf[index],to_read));
        if(read_size==0)
            return;
        if(read_size>(unsigned)to_read)
            comma::verbose<<"read long "<<read_size<<" vs "<<to_read<<std::endl;
        index+=read_size;
    }
    while(index-head>=5)
    {
        if(!msg_header)
        {
            //find the header
            for(; index-head>=5; head++)
            {
                msg_header=reinterpret_cast<messages::header*>(&buf[head]);
                if(msg_header->is_valid())
                {
                    break;
                }
                if(!skipper)
                    skipper=msg_header;
                debug_count++;
                msg_header=NULL;
            }
        }
        if(msg_header)
        {
            if( (index-head-5) < unsigned(msg_header->length()))
            {
                return;
            }
            if(msg_header->check_crc(&buf[head+5]))
            {
                handle_raw(msg_header,&buf[head+5],msg_header->length());
                switch(msg_header->id())
                {
                case messages::system_state::id:
                    handle(reinterpret_cast<messages::system_state*>(&buf[head+5]));
                    break;
                case messages::raw_sensors::id:
                    handle(reinterpret_cast<messages::raw_sensors*>(&buf[head+5]));
                    break;
                case messages::satellites::id:
                    handle(reinterpret_cast<messages::satellites*>(&buf[head+5]));
                    break;
                case messages::position_standard_deviation::id:
                    handle(reinterpret_cast<messages::position_standard_deviation*>(&buf[head+5]));
                    break;
                case messages::velocity_standard_deviation::id:
                    handle(reinterpret_cast<messages::velocity_standard_deviation*>(&buf[head+5]));
                    break;
                case messages::orientation_standard_deviation::id:
                    handle(reinterpret_cast<messages::orientation_standard_deviation*>(&buf[head+5]));
                    break;
                default:
                    comma::verbose<<"unhandled msg id: "<<int(msg_header->id())<<" len "<<int(msg_header->length())<<" "<<head<<" "<<index<<std::endl;
                    break;
                }
                if(debug_count)
                {
                    if(!skipper)
                        comma::verbose<<" skipped "<<debug_count<<std::endl;
                    else
                        comma::verbose<<" skipped "<<debug_count<<"; "<<unsigned(skipper->LRC())<<" "<<unsigned(skipper->id())<<" "<<unsigned(skipper->length())<<std::endl;
                    debug_count=0;
                    skipper=NULL;
                }
            }
            else
            {
                comma::verbose<<"crc failed "<<unsigned(msg_header->LRC())<<" "<<unsigned(msg_header->id())<<" "<<unsigned(msg_header->length())<<std::endl;
            }
            head+=unsigned(msg_header->length())+5;
            msg_header=NULL;
        }
    }
}

void device::send_ntrip(std::vector<char> buf)
{
//     comma::verbose<<"send_ntrip "<<buf.size()<<std::endl;
    unsigned index=0;
    while(index<buf.size())
    {
        unsigned size=std::min<unsigned>(buf.size()-index,255);
        messages::rtcm_corrections msg(&buf[index],size);
        index+=size;
//         comma::verbose<<"rtcm_corrections "<<size<<std::endl;
        std::size_t to_write=size+messages::header::size;
        std::size_t written=boost::asio::write(port, boost::asio::buffer(msg.data(),to_write));
        if(written!=to_write) { std::cerr<<"writing ntrip msg failed (expected "<<to_write<<" actual "<<written<<" )"<<std::endl; }
    }
}

    
} } } //namespace snark { namespace navigation { namespace advanced_navigation {
    
