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

#include "geoids.h"
#include <boost/algorithm/string.hpp>
#include <sstream>

namespace snark { namespace geodesy {
    
namespace impl {

wgs84::geoid wgs84;
agd84::geoid agd84;
snark::spherical::ellipsoid sphere( earth_average_radius, earth_average_radius );
snark::spherical::ellipsoid unit( 1, unit_minor_semiaxis );
grs67::geoid grs67;

static std::string sphere_info()
{
    std::stringstream os;
    os  << "shere,sphere with equal major and minor semiaxis," << earth_average_radius << "," << earth_average_radius << ",1" << std::endl;
    return os.str();
}

static std::string unit_info()
{
    std::stringstream os;
    os << "unit,scaled down WGS84,1," << unit_minor_semiaxis << "," << wgs84::eccentricity << std::endl;
    return os.str();
}

} // namespace impl

const snark::spherical::ellipsoid& geoids::select( const std::string& n )
{
    if ( n.empty()) { return impl::wgs84; }
    std::string name = n;
    boost::algorithm::to_lower( name );
    if ( name == wgs84::name ) { return impl::wgs84; }
    if ( name == agd84::name ) { return impl::agd84; }
    if ( name == grs67::name ) { return impl::grs67; }
    if ( name == "sphere" ) { return impl::sphere; }
    if ( name == "unit" ) { return impl::unit; }
    COMMA_THROW( comma::exception, "geoid not supported: '" << name << "'" );
}

std::string geoids::info( const std::string& n )
{
    if ( n.empty()) { return wgs84::info(); }
    std::string name = n;
    boost::algorithm::to_lower( name );
    if ( name == wgs84::name ) { return wgs84::info(); }
    if ( name == agd84::name ) { return agd84::info(); }
    if ( name == grs67::name ) { return grs67::info(); }
    if ( name == "sphere" ) { return impl::sphere_info(); }
    if ( name == "unit" ) { return impl::unit_info(); }
    COMMA_THROW( comma::exception, "can't get info on geoid: '" << name << "'" );
}

std::string geoids::help()
{
    std::stringstream os;
    os << "        sphere: sphere with earth average radius for comparision/testing ; " << earth_average_radius << ";" << earth_average_radius << "); 1" << std::endl;
    os << "        unit: WGS84 scaled down to unit, useful for getting distance in equivalent radian for instance (1; " << unit_minor_semiaxis << ");" << wgs84::eccentricity << std::endl;
    return os.str();
}

} } // snark geodesy
