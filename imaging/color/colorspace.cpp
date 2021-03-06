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

#include "colorspace.h"
#include <comma/string/string.h>
#include <comma/base/exception.h>

namespace snark { namespace imaging {

    colorspace::operator std::string() const
    {
        switch( value ) {
            case( none ):  return "none";  break;
            case( rgb ):   return "rgb";   break;
            case( ycbcr ): return "ycbcr"; break;
            case( ypbpr ): return "ypbpr"; break;
        }
        return "none"; // avoid a warning
    }

    const std::vector< std::string > & colorspace::field_names( cspace c )
    {
        static const std::map< cspace, std::vector< std::string > > & m = {
            { rgb,   comma::split( "r,g,b" , ',' ) },
            { ycbcr, comma::split( "y,cb,cr", ',' ) },
            { ypbpr, comma::split( "y,pb,pr", ',' ) },
            { none,  comma::split( "channel[0],channel[1],channel[2]", ',' ) }
        };
        return m.at( c );
    }

    range colorspace::default_range( cspace c )
    {
        switch ( c ) {
            case rgb:   return ub;
            case ycbcr: return ub;
            case ypbpr: return f;
            default:
                COMMA_THROW( comma::exception, "unknown colorspace '" << c << "'" );
        }
    }

} } // namespace snark { namespace imaging {
