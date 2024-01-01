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


#include <comma/base/exception.h>
#include <comma/base/types.h>
#include "../../../../timing/time.h"
#include "proprietary_reader.h"

#ifdef WIN32
#include <fcntl.h>
#include <io.h>
#endif

namespace snark {
    
proprietary_reader::proprietary_reader( const std::string& filename )
    : m_offset( 0 )
    , m_end( 0 )
{
    if( filename == "-" )
    {
        #ifdef WIN32
        if( filename == "-" ) { _setmode( _fileno( stdin ), _O_BINARY ); }
        #endif
        m_istream = &std::cin;
    }
    else
    {
        m_ifstream.reset( new std::ifstream( filename.c_str(), std::ios::binary | std::ios::in ) );
        if( m_ifstream->bad() ) { COMMA_THROW( comma::exception, "failed to open file " << filename ); }
        m_istream = m_ifstream.get();
    }
}

proprietary_reader::~proprietary_reader() { close(); }

const char* proprietary_reader::read() // quick and dirty
{
    static const boost::array< signed char, 2 > start = {{ -78, 85 }}; // see QLib::Bytestreams::GetDefaultstartDelimiter()
    static const boost::array< signed char, 2 > end = {{ 117, -97 }}; // see QLib::Bytestreams::GetDefaultstartDelimiter()
    while(    m_offset >= m_end
           || ::memcmp( &m_buffer[ m_offset ], &start[0], 2 ) != 0
           || ::memcmp( &m_buffer[ m_offset + packetSize - 2 ], &end[0], 2 ) != 0 )
    {
        if( m_offset + packetSize > m_end )
        {
            if( m_istream->bad() || m_istream->eof() ) { return NULL; }
            std::size_t size = m_buffer.size();
            std::size_t len = 0;
            if( m_offset < m_end )
            {
                len = m_end - m_offset;
                ::memcpy( &m_buffer[0], &m_buffer[m_offset], len );
                size -= len;
            }
            m_istream->read( &m_buffer[len], size );
            if( m_istream->gcount() <= 0 ) { return NULL; }
            m_end = m_istream->gcount() + len;
            if( m_end < packetSize ) { return NULL; }
            m_offset = 0;
        }
        else
        {
            ++m_offset;
        }
    }
    char* t = &m_buffer[m_offset] + headerSize;
    comma::uint64 seconds;
    comma::uint32 nanoseconds;
    ::memcpy( &seconds, t, 8 );
    ::memcpy( &nanoseconds, t + 8, 4 );
    m_timestamp = boost::posix_time::ptime( snark::timing::epoch, boost::posix_time::seconds( seconds ) + boost::posix_time::microseconds( static_cast< long >( nanoseconds / 1000 ) ) );
    m_offset += packetSize;
    return t + timestampSize;
}

boost::posix_time::ptime proprietary_reader::timestamp() const { return m_timestamp; }

void proprietary_reader::close() { if( m_ifstream ) { m_ifstream->close(); } }

} 

