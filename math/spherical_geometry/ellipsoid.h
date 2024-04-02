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

#include <vector>
#include <boost/optional.hpp>
#include "coordinates.h"

namespace snark { namespace spherical {

/// uses:
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  */
/* Vincenty Inverse Solution of Geodesics on the Ellipsoid ( c ) Chris Veness 2002-2012           */
/*                                                                                                */
/* from: Vincenty inverse formula - T Vincenty, "Direct and Inverse Solutions of Geodesics on the */
/*       Ellipsoid with application of nested equations", Survey Review, vol XXII no 176, 1975    */
/*       http://www.ngs.noaa.gov/PUBS_LIB/inverse.pdf                                             */
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  */
//
// See http://www.movable-type.co.uk/scripts/latlong-vincenty.html

// The MIT License (MIT)
// 
// Copyright (c) 2014 Chris Veness
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
struct ellipsoid
{
    /// constructor
    ellipsoid( double major_semiaxis, double minor_semiaxis ): major_semiaxis( major_semiaxis ), minor_semiaxis( minor_semiaxis ) {}

    double major_semiaxis{0};

    double minor_semiaxis{0};

    /// return distance between from and to coordinates, using vincenty's formulae
    double distance( const coordinates &from, const coordinates &to ) const;

    /// return coordinates at a given distance from begin at given bearing, using vincenty's formulae
    /// todo: an extensive comment on what kind of bearing is used
    coordinates at( const coordinates &begin, double distance, double bearing ) const;

    //great circle arc
    struct arc
    {
        coordinates begin;
        coordinates end;
    };

    //small circle
    struct circle
    {
        //spherical coordinates in radian
        coordinates centre;
        //radius is distance in metric unit
        double radius{0};
        struct arc;
        std::vector< coordinates > discretize( const ellipsoid &ellipsoid, const boost::optional< double > &resolution, const boost::optional< unsigned int > &circle_size ) const;
    };
};

//circular arc
struct ellipsoid::circle::arc
{
    /// circle of which this is an arc
    ellipsoid::circle circle;
    /// begin bearing
    double begin{0};
    /// end bearing
    double end{0};
    arc() = default;
    arc( const ellipsoid::circle &circle, double begin, double end ) : circle( circle ), begin( begin ), end( end ) {}
    std::vector< coordinates > discretize( const ellipsoid &ellipsoid, const boost::optional< double > &resolution, const boost::optional< unsigned int > &circle_size ) const;

    //heading degree from north; east is positive 90
    static double bearing( double heading )
    {
        //bearing standard geometric radian; ie east=0,north=pi/2,south=-pi/2
        return ( heading * -1 ) + ( M_PI / 2 );
    }
};

} } // namespace snark { namespace spherical {
