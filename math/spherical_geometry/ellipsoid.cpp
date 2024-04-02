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

#include <cmath>
#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include "../angle.h"
#include "ellipsoid.h"

namespace snark { namespace spherical {

static double square( double x ) { return x * x; }

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

// super-quick and dirty; a phenomenally dirty implementation
// todo: what are all those magic constants? are they specific to geoid?
double ellipsoid::distance( const coordinates& p1, const coordinates& p2 ) const
{
    double middle = ( major_semiaxis - minor_semiaxis ) / major_semiaxis;
    double L = p2.longitude - p1.longitude;
    double U1 = std::atan( ( 1.0 - middle ) * std::tan( p1.latitude ) );
    double U2 = std::atan( ( 1.0 - middle ) * std::tan( p2.latitude) );
    double sinU1 = std::sin( U1 ), cosU1 = std::cos( U1 );
    double sinU2 = std::sin( U2 ), cosU2 = std::cos( U2 );

    double lambda = L;
    double lambdaP, cosSqAlpha, sigma, sinSigma, cosSigma, cos2SigmaM;
    double sinLambda, cosLambda;
    static const unsigned int max_count = 100;
    unsigned int count = 0;

    for ( ;count < max_count;++count )
    {
        sinLambda = std::sin( lambda ), cosLambda = std::cos( lambda );
        sinSigma = std::sqrt( 
            square( cosU2 * sinLambda ) +
            square( cosU1 * sinU2 - sinU1 * cosU2 * cosLambda ) );

        if ( sinSigma == 0.0 ) { return 0.0; }  // co-incident points

        cosSigma = sinU1 * sinU2 + cosU1 * cosU2 * cosLambda;
        sigma = std::atan2( sinSigma, cosSigma );
        double sinAlpha = cosU1 * cosU2 * sinLambda / sinSigma;
        cosSqAlpha = 1.0 - square( sinAlpha );

        // check for equatorial line
        static const double epsilon = 1e-12;
        if ( comma::math::equal( cosSqAlpha, 0, epsilon ) ) { cos2SigmaM = 0.0; }
        else { cos2SigmaM = cosSigma - 2.0 * sinU1 * sinU2 / cosSqAlpha; }

        double C = middle / 16.0 * cosSqAlpha * ( 4.0 + middle * ( 4.0 - 3.0 * cosSqAlpha ) );
        lambdaP = lambda;
        lambda = L + ( 1.0 - C ) * middle * sinAlpha *
          ( sigma + C * sinSigma * ( cos2SigmaM + C * cosSigma *
            ( 2 * square( cos2SigmaM ) - 1.0 ) ) );

        if ( comma::math::equal( lambda, lambdaP, epsilon ) ) { break; }
    }

    if( count >= max_count ) { COMMA_THROW( comma::exception, "failed to converge after " << max_count << " iterations" ); }

    double uSq = cosSqAlpha * ( square( major_semiaxis ) - square( minor_semiaxis ) ) / square( minor_semiaxis );
    double A = 1 + uSq / 16384 * ( 4096 + uSq *
        ( -768 + uSq * ( 320 - 175 * uSq ) ) );
    double B = uSq / 1024 * ( 256 + uSq * ( -128 + uSq * ( 74 - 47 * uSq ) ) );
    double deltaSigma = B * sinSigma * ( cos2SigmaM + B / 4 *
        ( cosSigma * ( -1 + 2 * square( cos2SigmaM ) )-
            B / 6 * cos2SigmaM * ( -3 + 4 * square( sinSigma ) ) *
                ( -3 + 4 * square( cos2SigmaM ) ) ) );
    double s = minor_semiaxis * A * ( sigma - deltaSigma );

//     double bearing = rad2deg( restrict_0_2pi( std::atan2( cosU2 * sinLambda,
//         cosU1 * sinU2 - sinU1 * cosU2 * cosLambda ) ) );
//     double end_bearing = rad2deg( restrict_0_2pi( std::atan2( cosU1 * sinLambda,
//         -sinU1 * cosU2 + cosU1 * sinU2 * cosLambda ) ) );

    return s;
}
 
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  */
/* Vincenty Direct Solution of Geodesics on the Ellipsoid ( c ) Chris Veness 2005-2012              */
/*                                                                                                */
/* from: Vincenty direct formula - T Vincenty, "Direct and Inverse Solutions of Geodesics on the  */
/*       Ellipsoid with application of nested equations", Survey Review, vol XXII no 176, 1975    */
/*       http://www.ngs.noaa.gov/PUBS_LIB/inverse.pdf                                             */
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  */
//
// See http://www.movable-type.co.uk/scripts/latlong-vincenty-direct.html

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

// super-quick and dirty; a phenomenally dirty implementation
// todo: what are all those magic constants? are they specific to geoid?
coordinates ellipsoid::at( const coordinates& p, double distance, double bearing ) const
{
    double middle = ( major_semiaxis - minor_semiaxis ) / major_semiaxis;
    double s = distance;
    double alpha1 = bearing;
    double sinAlpha1 = std::sin( alpha1 );
    double cosAlpha1 = std::cos( alpha1 );

    double tanU1 = ( 1.0 - middle ) * std::tan( p.latitude );
    double cosU1 = 1.0 / std::sqrt( ( 1.0 + square( tanU1 ) ) );
    double sinU1 = tanU1 * cosU1;
    double sigma1 = std::atan2( tanU1, cosAlpha1 );
    double sinAlpha = cosU1 * sinAlpha1;
    double cosSqAlpha = 1.0 - sinAlpha*sinAlpha;
    double uSq = cosSqAlpha * ( square( major_semiaxis ) - square( minor_semiaxis ) ) / square( minor_semiaxis );
    double A = 1.0 + uSq / 16384 * ( 4096 + uSq * ( -768 + uSq * ( 320 - 175 * uSq ) ) );
    double B = uSq / 1024 * ( 256 + uSq * ( -128 + uSq * ( 74 - 47 * uSq ) ) );

    double sigma = s / ( minor_semiaxis * A );
    double sigmaP = 2.0 * M_PI;
    double sinSigma = 0.0, cosSigma = 0.0, cos2SigmaM = 0.0;

    while ( std::fabs( sigma - sigmaP ) > 1e-12 )
    {
        cos2SigmaM = std::cos( 2.0 * sigma1 + sigma );
        sinSigma = std::sin( sigma );
        cosSigma = std::cos( sigma );
        double deltaSigma = B * sinSigma * ( cos2SigmaM + B / 4 *
            ( cosSigma * ( -1.0 + 2.0 * square( cos2SigmaM ) ) -
                B / 6 * cos2SigmaM * ( -3 + 4 * square( sinSigma ) ) *
                    ( -3 + 4 * square( cos2SigmaM ) ) ) );
        sigmaP = sigma;
        sigma = s / ( minor_semiaxis * A ) + deltaSigma;
    }

    double tmp = sinU1 * sinSigma - cosU1 * cosSigma * cosAlpha1;
    double lat2 = std::atan2( sinU1 * cosSigma + cosU1 * sinSigma*cosAlpha1,
      ( 1.0 - middle ) * std::sqrt( square( sinAlpha ) + square( tmp ) ) );
    double lambda = std::atan2( sinSigma * sinAlpha1,
        cosU1 * cosSigma - sinU1 * sinSigma * cosAlpha1 );
    double C = middle / 16 * cosSqAlpha * ( 4 + middle * ( 4 - 3 * cosSqAlpha ) );
    double L = lambda - ( 1.0 - C ) * middle * sinAlpha *
        ( sigma + C * sinSigma * ( cos2SigmaM + C * cosSigma *
            ( -1 + 2 * square( cos2SigmaM ) ) ) );
    double lon2 = math::angle< double >( math::radians( p.longitude + L + M_PI ) ).as_radians() - M_PI;

    return coordinates( lat2, lon2 );
}

std::vector< coordinates > ellipsoid::circle::discretize( const ellipsoid &ellipsoid, const boost::optional< double > &resolution, const boost::optional< unsigned int > &circle_size ) const
{
    return arc( *this, 0, 2 * M_PI ).discretize( ellipsoid, resolution, circle_size );
}

std::vector< coordinates > ellipsoid::circle::arc::discretize( const ellipsoid &ellipsoid, const boost::optional< double > &resolution, const boost::optional< unsigned int > &circle_size ) const
{
    boost::optional< long double > r;
    if ( resolution ) { r = *resolution; }
    boost::optional< long double > s;
    long double planar_radius = circle.radius / ellipsoid.major_semiaxis;
    if ( circle_size ) { s = M_PI * 2 * planar_radius / *circle_size; }
    if ( ( !r && s ) || ( s && *r > *s ) ) { r = s; }
    if ( !r ) { COMMA_THROW( comma::exception, "expected either resolution or circle size, got none" ); }
    if ( *r >= M_PI || *r < 0 ) { COMMA_THROW( comma::exception, "expected positive resolution less than 180 degrees; got " << ( *r * 180 / M_PI ) ); }
    long double step = *r / planar_radius;
    unsigned int size = M_PI * 2 / step;
    std::vector< coordinates > v;
    v.reserve( size + 1 );
    long double diff = std::abs( end - begin );
    long double sign = begin > end ? -1 : 1;
    for ( long double a( 0 ), b( begin ); a < diff;  b += ( step * sign ), a += step )
    {
        v.push_back( ellipsoid.at( circle.centre, circle.radius, bearing( b ) ) );
    }
    const coordinates &end_coordinates = ellipsoid.at( circle.centre, circle.radius, bearing( end ) );
    if ( v.back() != end_coordinates ) { v.push_back( end_coordinates ); }
    return v;
}

} } // namespace snark { namespace spherical {

