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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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

#ifndef SNARK_OCEAN_IO_QUERY_H
#define SNARK_OCEAN_IO_QUERY_H
#include <comma/base/types.h>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/array.hpp>
#include <boost/utility/binary.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <vector>
#include <iomanip>
#include <iostream>
#include "commands.h"
#include "battery.h"
#include <aero/navigation/applications/wmm/GeomagnetismHeader.h>

namespace snark { namespace ocean {
    
class stdio_query 
{
public:
    ocean8 read()
    {
        char c;
        std::cin.read( &c, 1u );
        return c;
    }
    
    void write( ocean8 value )
    {
        std::cout.write( (const char*) &value, 1u );
    };
};

template < int B, ocean8 ADDR, typename IO >
comma::uint16 cmd_query( )
{
    IO io;
    io.write( command_bits< ocean8( B ), ADDR >::value );
    ocean8 lsbyte = io.read();
    io.write( ocean8(0) );
    comma::uint16 msbyte = io.read();
    
    return comma::uint16( ( msbyte << 8 )  | lsbyte );
//     comma::uint16 result = ( ( msbyte << 8 )  | lsbyte );
    
//     std::cerr << "query " << B << " address: " << int(ADDR) 
//               << " lsb: " << int( lsbyte ) << " msb: " << int( msbyte )
//               << " value: " << result << std::endl;
//     return result;
}

template < int B, ocean8 ADDR, typename IO >
data_t query()
{
    return data_t( ADDR, cmd_query< B, ADDR, IO >() );
}

namespace impl_ {

/// Looping through each battery for N to 1, B=N initially
/// N is total number of battery, B is battery number
template < int B, int N, typename IO >
struct update_controller
{
    static void update( controller< N >& controller )
    {
        controller.batteries[B-1] & query< B, address::current, IO >();
        controller.batteries[B-1] & query< B, address::average_current, IO >();
        controller.batteries[B-1] & query< B, address::temperature, IO >();
        controller.batteries[B-1] & query< B, address::voltage, IO >();
        controller.batteries[B-1] & query< B, address::rel_state_of_charge, IO >();
        controller.batteries[B-1] & query< B, address::remaining_capacity, IO >();
        controller.batteries[B-1] & query< B, address::run_time_to_empty, IO >();
        controller.batteries[B-1] & query< B, address::status, IO >();
        
        update_controller< B - 1, N, IO >::update( controller );
    }
};

template < int N, typename IO >
struct update_controller< 0, N, IO >
{
    static void update( controller< N >& controller ) {}
};
    
} //namespace impl_ {

    
template < int N, typename IO >
bool query( controller< N >& controller )
{
    impl_::update_controller< N, N, IO >::update( controller );
    return true;
}
    
} } // namespace snark { namespace ocean {

#endif // SNARK_OCEAN_IO_QUERY_H