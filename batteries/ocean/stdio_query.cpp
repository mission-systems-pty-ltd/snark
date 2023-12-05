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


#include "stdio_query.h"
#include <comma/base/exception.h>

namespace snark { namespace ocean { 

stdio_query::~stdio_query()
{
    boost::system::error_code ec;
    deadline_.cancel( ec );
    input_.cancel( ec );

}


std::size_t stdio_query::write(const char* buf, std::size_t size)
{
    boost::system::error_code ec;
    std::size_t sent = output_.write_some(boost::asio::buffer( buf, size ), ec );

    if( ec.value() != boost::system::errc::success )
    {
        std::ostringstream ss;
        ss << "stdin write failed with error: " << ec.value() 
           << " category: " << ec.category().name() << " - " << ec.message() << std::endl;
        COMMA_THROW( comma::exception, ss.str() );
    }
    
    return sent;

}



stdio_query::stdio_query() 
    : error_code_( ),
      received_length_(0),
      desc_service_( io_service_ ),
      input_( io_service_, ::dup(STDIN_FILENO ) ),
      output_( io_service_, ::dup(STDOUT_FILENO ) ),
      deadline_( io_service_ )
{
    /// set deadline as infinite - no deadline
    deadline_.expires_at(boost::posix_time::pos_infin);
    
    // Start the deadline handler 
    check_deadline();
    
}

void stdio_query::receive( std::string& data, const boost::posix_time::time_duration& timeout_duration )
{
    boost::asio::mutable_buffer buf( &data[0], data.size() );
    boost::system::error_code ec;
    std::size_t s = receive( buf, timeout_duration, ec );
    
    if( ec.value() != boost::system::errc::success )
    {
        std::ostringstream ss;
        ss << "stdin read failed with error: " << ec.value() 
           << " category: " << ec.category().name() << " - " << ec.message() << std::endl;
        COMMA_THROW( comma::exception, ss.str() );
    }
    //success
    data.resize(s);
}

void stdio_query::async_receive(const boost::asio::mutable_buffer& buffer, boost::posix_time::time_duration timeout)
{
    /// deadline updated for receive
    deadline_.expires_from_now(timeout);
    
    error_code_ = boost::asio::error::would_block;
    received_length_ = 0;
    
    input_.async_read_some(boost::asio::buffer(buffer),
                          boost::bind(&stdio_query::handle_receive, boost::placeholders::_1, boost::placeholders::_2, &error_code_, &received_length_));
}


std::size_t stdio_query::receive(const boost::asio::mutable_buffer& buffer, 
                                       boost::posix_time::time_duration timeout, boost::system::error_code& ec)
{
    // Set a deadline for the asynchronous operation.
    deadline_.expires_from_now(timeout);
    
    // error of result of operation
    ec = boost::asio::error::would_block;
    std::size_t length = 0;
    
    input_.async_read_some(boost::asio::buffer(buffer),
                          boost::bind(&stdio_query::handle_receive, boost::placeholders::_1, boost::placeholders::_2, &ec, &length));
    // boost::asio::async_read( input_, boost::asio::buffer(buffer),
    //                         boost::asio::transfer_at_least(1),
    //                         boost::bind(&stdio_query::handle_receive, boost::placeholders::_1, boost::placeholders::_2, &ec, &length));
    
    // Block until the asynchronous operation has completed.
    do io_service_.run_one(); while (ec == boost::asio::error::would_block);
    
    return length;
}

} } // namespace snark { namespace ocean { 