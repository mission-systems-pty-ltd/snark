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

#include <boost/array.hpp>
#include <boost/optional/optional.hpp>
#include "../colour.h"

namespace snark { namespace render { namespace colours {

class map // quick and dirty
{
    public:
        typedef boost::array< unsigned char, 3 > pixel;
        typedef boost::array< pixel, 256 > values;

        enum channels { red = 0, green = 1, blue = 2 };

        static values constant( unsigned char r, unsigned char g, unsigned char b );
        static values temperature( unsigned char offset_r, unsigned char offset_g );
        static values jet();
        static values magma();
        static values viridis();
        static values twilight();
        static pixel contrast_to( const values& v );

        map() = default;
        map( double from, double to, const colour< unsigned char >& from_colour, const colour< unsigned char >& to_colour );
        map( double from, double to, const values& v );

        colour< unsigned char > mapped( const double scalar ) const;
        colour< unsigned char > operator()( const double scalar ) const { return mapped( scalar ); }

    private:
        double from;
        double to;
        double diff;
        boost::optional< values > values_;
        colour< unsigned char > from_colour;
        colour< unsigned char > to_colour;
};

} } } // namespace snark { namespace render { namespace colours {
