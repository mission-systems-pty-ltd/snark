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


#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <comma/base/types.h>
#include <comma/base/exception.h>
#include "../hdl64/packet.h"
#include "../scan_tick.h"
#include "thin.h"

namespace snark {  namespace velodyne { namespace thin {

static boost::mt19937 generator;

void thin( velodyne::hdl64::packet& packet, float rate )
{
    boost::uniform_real< float > distribution( 0, 1 ); // watch performance
    boost::variate_generator< boost::mt19937&, boost::uniform_real< float > > random( generator, distribution );
    thin::thin( packet, rate, random );
}

static void set( char* ids, unsigned int i, bool value = true )
{
    if( value ) { *( ids + ( i / 8 ) ) |= ( 1 << ( i % 8 ) ); }
    else { *( ids + ( i / 8 ) ) &= ~( 1 << ( i % 8 ) ); }
}

static bool get( const char* ids, unsigned int i )
{
    return *( ids + ( i / 8 ) ) & ( 1 << ( i % 8 ) );
}

const unsigned int idsSize = 64 / 8;

static std::size_t serialize( const velodyne::hdl64::packet::laser_block& upper, const velodyne::hdl64::packet::laser_block& lower, char* buf )
{
    char* begin = buf;
    ::memcpy( buf, upper.rotation.data(), 2 );
    buf += 2;
    char* ids = buf;
    ::memset( ids, 0, idsSize );
    buf += idsSize;
    bool modified = false;
    for( unsigned int i = 0; i < upper.lasers.size(); ++i )
    {
        if( !upper.lasers[i].range() != 0 ) { continue; }
        set( ids, i );
        ::memcpy( buf, upper.lasers[i].range.data(), 2 );
        buf += 2;
        modified = true;
    }
    for( unsigned int i = 0; i < lower.lasers.size(); ++i )
    {
        if( !lower.lasers[i].range() != 0 ) { continue; }
        set( ids, i + lower.lasers.size() );
        ::memcpy( buf, lower.lasers[i].range.data(), 2 );
        buf += 2;
        modified = true;
    }
    return modified ? buf - begin : 0;
}

// quick and dirty, dirties the buffer
std::size_t serialize( const velodyne::hdl64::packet& packet, char* buf, comma::uint32 scan )
{
    char* begin = buf;
    ::memcpy( buf, &scan, sizeof( comma::uint32 ) );
    buf += sizeof( comma::uint32 );
    char& ids = *buf++;
    ids = 0x00;
    for( unsigned int i = 0; i < packet.blocks.size(); i += 2 )
    {
        unsigned int size = serialize( packet.blocks[ i ], packet.blocks[ i + 1 ], buf );
        if( size > 0 )
        {
            ids |= ( 1 << ( i >> 1 ) );
            buf += size;
        }
    }
    return buf - begin;
}

static std::size_t deserialize( velodyne::hdl64::packet::laser_block& upper, velodyne::hdl64::packet::laser_block& lower, const char* buf )
{
    upper.id = velodyne::hdl64::packet::upper_block_id();
    lower.id = velodyne::hdl64::packet::lower_block_id();
    const char* begin = buf;
    ::memcpy( upper.rotation.data(), buf, 2 );
    ::memcpy( lower.rotation.data(), buf, 2 );
    buf += 2;
    const char* ids = buf;
    buf += idsSize;
    for( unsigned int i = 0; i < upper.lasers.size(); ++i )
    {
        if( !get( ids, i ) ) { continue; }
        ::memcpy( upper.lasers[i].range.data(), buf, 2 );
        buf += 2;
    }
    for( unsigned int i = 0; i < lower.lasers.size(); ++i )
    {
        if( !get( ids, i + lower.lasers.size() ) ) { continue; }
        ::memcpy( lower.lasers[i].range.data(), buf, 2 );
        buf += 2;
    }
    return buf - begin;
}

comma::uint32 deserialize( velodyne::hdl64::packet& packet, const char* buf )
{
    packet.clear();
    comma::uint32 scan;
    memcpy( &scan, buf, sizeof( comma::uint32 ) );
    buf += sizeof( comma::uint32 );
    const char& ids = *buf++;
    for( unsigned int i = 0; i < packet.blocks.size(); i += 2 )
    {
        if( !( ids & ( 1 << ( i >> 1 ) ) ) ) { continue; }
        buf += deserialize( packet.blocks[i], packet.blocks[ i + 1 ], buf );
    }
    return scan;
}

std::pair< velodyne::hdl64::packet, comma::uint32 > deserialize( const char* buf )
{
    std::pair< velodyne::hdl64::packet, comma::uint32 > p;
    p.second = deserialize( p.first, buf );
    return p;
}

} } } // namespace snark {  namespace velodyne { namespace thin {
