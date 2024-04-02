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

#include <sstream>
#include "../../math/spherical_geometry/ellipsoid.h"

namespace snark { namespace geodesy { namespace grs67 {

static const double eccentricity = 1.0 / 298.247167427;
static const double major_semiaxis = 6378160.0;
static const double minor_semiaxis = 6356774.516;
static const double radius = major_semiaxis;
static const std::string name( "grs67" );

/*
 cached from http://en.wikipedia.org/wiki/Earth_ellipsoid#Historical_Earth_ellipsoids
 At the 1967 meeting of the IUGG held in Lucerne, Switzerland, the ellipsoid called GRS-67 (Geodetic Reference System 1967) in the listing was recommended for adoption.
 The new ellipsoid was not recommended to replace the International Ellipsoid (1924), but was advocated for use where a greater degree of accuracy is required.
 It became a part of the GRS-67 which was approved and adopted at the 1971 meeting of the IUGG held in Moscow. It is used in Australia for the Australian Geodetic Datum and in South America for the South American Datum 1969.
*/

inline std::string help()
{
    std::stringstream oss;
    oss << "        GRS67: Geodesic Reference System 1967 (6,378,160; 6,356,774.516); 298.247167427" << std::endl;
    return oss.str();
}

inline std::string info()
{
    std::stringstream oss;
    oss << "GRS67,Geodesic Reference System 1967," << major_semiaxis << "," << minor_semiaxis << "," << eccentricity << std::endl;
    return oss.str();
}

struct geoid : public spherical::ellipsoid
{
    //geoid() : ellipsoid( major_semiaxis, minor_semiaxis ) {}
    geoid() : ellipsoid( 6378160.0, 6356774.516 ) {} // todo: properly fix compiler warning about the initialisation 
};

} } } // namespace snark { namespace geodesy { namespace agd84 {
