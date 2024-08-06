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
        else if( name == "green" ) { return green(); }
        else if( name == "blue" ) { return blue(); }
        else if( name == "white" ) { return white(); }
        else if( name == "black" ) { return black(); }
        else if( name == "cyan" ) { return cyan(); }
        else if( name == "yellow" ) { return yellow(); }
        else if( name == "magenta" ) { return magenta(); }
        else { COMMA_THROW( comma::exception, "unknown colour: " << name ); }
    }
};

} } } // namespace snark { namespace render { namespace colours {
