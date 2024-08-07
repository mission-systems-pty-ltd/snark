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

#pragma once

#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include <comma/string/split.h>
#include "../colour.h"

namespace snark { namespace render { namespace colours {

template < typename T >
struct named
{
    static colour< T > red( T a = colour_traits< T >::max() ) { return colour< T >( colour_traits< T >::max(), colour_traits< T >::min(), colour_traits< T >::min(), a ); }
    static colour< T > green( T a = colour_traits< T >::max() ) { return colour< T >( colour_traits< T >::min(), colour_traits< T >::max(), colour_traits< T >::min(), a ); }
    static colour< T > blue( T a = colour_traits< T >::max() ) { return colour< T >( colour_traits< T >::min(), colour_traits< T >::min(), colour_traits< T >::max(), a ); }
    static colour< T > white( T a = colour_traits< T >::max() ) { return colour< T >( colour_traits< T >::max(), colour_traits< T >::max(), colour_traits< T >::max(), a ); }
    static colour< T > black( T a = colour_traits< T >::max() ) { return colour< T >( colour_traits< T >::min(), colour_traits< T >::min(), colour_traits< T >::min(), a ); }
    static colour< T > cyan( T a = colour_traits< T >::max() ) { return colour< T >( colour_traits< T >::min(), colour_traits< T >::max(), colour_traits< T >::max(), a ); }
    static colour< T > yellow( T a = colour_traits< T >::max() ) { return colour< T >( colour_traits< T >::max(), colour_traits< T >::max(), colour_traits< T >::min(), a ); }
    static colour< T > magenta( T a = colour_traits< T >::max() ) { return colour< T >( colour_traits< T >::max(), colour_traits< T >::min(), colour_traits< T >::max(), a ); }

    static colour< T > from_string( const std::string& name )
    {
        if( name == "red" ) { return red(); }
        if( name == "green" ) { return green(); }
        if( name == "blue" ) { return blue(); }
        if( name == "white" ) { return white(); }
        if( name == "black" ) { return black(); }
        if( name == "cyan" ) { return cyan(); }
        if( name == "yellow" ) { return yellow(); }
        if( name == "magenta" ) { return magenta(); }
        try
        {
            const auto& v = comma::split_as< double >( name, ',' ); // because boost::lexical_cast< unsigned char >( 0 ) throws
            COMMA_ASSERT( v.size() == 3 || v.size() == 4, "expected 3- or 4-channel colour, got: '" << name << "'" );
            return colour< T >( v[0], v[1], v[2], v.size() == 4 ? v[3] : colour_traits< T >::max() );
        }
        catch( std::exception& ex ) { COMMA_THROW( comma::exception, "unsupported or invalid colour: '" << name << "': " << ex.what() ); }
        catch( ... ) { COMMA_THROW( comma::exception, "unsupported or invalid colour: '" << name << "': unknown exception" ); }
    }
};

} } } // namespace snark { namespace render { namespace colours {
