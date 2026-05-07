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


#ifndef WIN32
#include <stdlib.h>
#endif

#include <iostream>
#include <boost/asio/ip/tcp.hpp>
#include <comma/application/signal_flag.h>
#include <gtest/gtest.h>
#include "../ibeo/protocol.h"

TEST( sick, DISABLED_protocol )
{
    #if ( BOOST_VERSION < 108700 )
        boost::asio::ip::tcp::endpoint e( boost::asio::ip::address::from_string( "127.0.0.1" ), 1234 );
    #else
        boost::asio::ip::tcp::endpoint e( boost::asio::ip::make_address( "127.0.0.1" ), 1234 );
    #endif
    boost::asio::ip::tcp::iostream stream( e );
    if( !stream ) { std::cerr << "---> connect failed" << std::endl; exit( -1 ); }
    std::cerr << "---> connected" << std::endl;
    if( !stream.good() || stream.eof() ) { std::cerr << "---> stream bad" << std::endl; exit( -1 ); }
    
    stream << "hello world" << std::endl;
    stream.flush();
    std::cerr << "---> hello" << std::endl;
    
    stream.write( "goodbye moon\n", ::strlen( "goodbye moon\n" ) );
    std::cerr << "---> bye" << std::endl;
    
    comma::signal_flag flag;
    
    std::string line( 1024, 0 );
    
    std::cerr << "---> reading..." << std::endl;
    stream.read( &line[0], 6 );
    std::cerr << "---> signal flag is " << ( flag ? "" : "not " ) << "set" << std::endl;
    
    std::cerr << "---> got " << stream.gcount() << " byte(s): " << line << std::endl;    
}

int main( int argc, char* argv[] )
{
    ::testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
    return 0;
}
