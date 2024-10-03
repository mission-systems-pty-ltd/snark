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

/// @author vsevolod vlaskine

#include <comma/base/exception.h>
#include "string.h"

namespace snark { namespace nmea {

static unsigned int hex_to_int( char c )
{
    if( '0' <= c && c <= '9' ) { return c - '0'; }
    if( 'A' <= c && c <= 'F' ) { return c - 'A' + 10; }
    if( 'a' <= c && c <= 'f' ) { return c - 'a' + 10; }
    COMMA_THROW( comma::exception, "expected a hexadecimal digit, got: " << c );
}

string::string( const std::string& s, bool ignore_checksum ): valid_( false ), complete_( false )
{
    if( s[0] != '$' ) { return; }
    bool has_cr = s[ s.size() - 1 ] == '\r';
    std::string::size_type p = s.find_last_of( '*' );
    if( p == std::string::npos ) { return; }
    if( p + 3 + has_cr != s.size() ) { return; }
    if( !ignore_checksum )
    {
        unsigned char sum = 0;
        unsigned char checksum;
        try { checksum = 16 * hex_to_int( s[ p + 1 ] ) + hex_to_int( s[ p + 2 ] ); }
        catch( ... ) { return; }
        for( unsigned int i = 1; i < p; sum ^= s[i++] );
        if( sum != checksum ) { return; }
    }
    valid_ = true;
    values_ = comma::split( s.substr( 1, p - 1 ), ',' );
    complete_ = true;
    for( unsigned int i = 0; i < values_.size() && complete_; complete_ = !values_[i++].empty() );
}

bool string::valid() const { return valid_; }

bool string::complete() const { return complete_; }

bool string::is_proprietary() const { return values_[0][0] == 'P'; }

std::string string::manufacturer_code() const { return values_[0].substr( 1, 3 ); }

std::string string::talker_id() const { return values_[0].substr( 0, 2 ); }

std::string string::message_type() const { return values_[0].substr( 2, 3 ); }

const std::vector< std::string >& string::values() const { return values_; }

unsigned char string::checksum( const std::string& s )
{
    if( s.empty() ) { COMMA_THROW( comma::exception, "expected non-empty string" ); }
    unsigned int begin = s[0] != '$' ? 0 : 1;
    std::string::size_type checksum_delimiter = s.find_last_of( '*' );
    unsigned int end = checksum_delimiter != std::string::npos ? checksum_delimiter : s.size();
    unsigned char sum = 0;
    for( unsigned int i = begin; i < end; sum ^= s[i++] );
    return sum;
}
    
} } // namespace snark { namespace nmea {
