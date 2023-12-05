// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
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


#ifndef SNARK_BATTERY_OCEAN_STDIO_QUERY_H
#define SNARK_BATTERY_OCEAN_STDIO_QUERY_H

#include <cstdlib>
#include <cstring>
#include <string>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/bind/bind.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <comma/base/types.h>

using boost::asio::deadline_timer;
using boost::posix_time::time_duration;
using boost::asio::posix::stream_descriptor;

namespace snark { namespace ocean {
using namespace snark;

class stdio_query 
{
    static const int max_length = 1024;
    boost::system::error_code error_code_;
    std::size_t received_length_;
    boost::asio::io_service io_service_;
    boost::asio::posix::stream_descriptor_service desc_service_;
    stream_descriptor input_;
    stream_descriptor output_;

    deadline_timer deadline_;
        
public:
    stdio_query(); 
    ~stdio_query(); 
    
    std::size_t write( const char* buf, std::size_t size );
    
    void receive( std::string& data, const time_duration& dur=boost::posix_time::seconds(10) );
    
    boost::asio::io_service& ioservice() { return io_service_; }
    boost::system::error_code error_code() { return error_code_; }
    std::size_t received_length() { return received_length_; }
    /// For async receive, true for no data yet
    bool code_would_block() { return error_code_ == boost::asio::error::would_block; }
    bool code_success() { return error_code_ == boost::system::errc::success; }
    /// starts an asynchronous receive/listen
    /// The caller needs to call io_service().run_one() and check that error_code is not would_block
    /// to see if operation finished
    void async_receive( const boost::asio::mutable_buffer& buffer, 
                        boost::posix_time::time_duration dur=boost::posix_time::seconds(5) );

    bool check_async()
    {
        io_service_.poll();
        if( code_success() ) { return true; }
        else if( code_would_block() ) { return false; }
        else { return false; }
    }
private:
    /// A blocking receive but has timeout
    std::size_t receive(const boost::asio::mutable_buffer& buffer,
                        boost::posix_time::time_duration timeout, boost::system::error_code& ec);
    
    /// Checks if deadline has passed, if so cancel th read operation
    void check_deadline()
    {
        if (deadline_.expires_at() <= deadline_timer::traits_type::now())
        {
            input_.cancel();
            
            // no deadline again
            deadline_.expires_at(boost::posix_time::pos_infin);
        }
        
        // set the checking
        deadline_.async_wait(boost::bind(&stdio_query::check_deadline, this));
    }
    
    static void handle_receive(
        const boost::system::error_code& ec, std::size_t length,
        boost::system::error_code* out_ec, std::size_t* out_length)
    {
        *out_ec = ec;
        *out_length = length;
    } 
};


} } // namespace snark { namespace ocean { 

#endif // SNARK_BATTERY_OCEAN_STDIO_QUERY_H
