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

/// @author andrew maclean

#pragma once

#include <cmath>
#include <boost/optional.hpp>
#include "Ellipsoid.h"
#include "MapGrid.h"

//! Convert between various coordinate systems. 
namespace snark { namespace detail { namespace GeographicGeodeticRectangular {

  //! This class converts between geographic and grid coordinates using Redfearn's formulae.
  /*!
  CLASS
    CRedfearn

    When using this class, you must instantiate it with an instance of the ellipsoid 
    and map grid.

    Redfearn's formulae were published in the "Empire Survey Review", No. 69, 1948. 
    They may be used to  convert between latitude & longitude and easting, 
    northing & zone for a Transverse Mercator projection, such as the Map
    Grid of Australia (MGA). These formulae are accurate to better than 
    1 mm in any zone of  the Map Grid of Australia and for the purposes 
    of definition may be regarded as exact. 
    See "Geocentric Datum of Australia, Technical Manual" Version 2.2 for 
    the algorithms. Avaiable from http://www.icsm.gov.au/icsm/gda/gdatm/index.html 
    as a pdf file.

      \version 1.0
  first version
  
  \date 16-Sept-2003
  
  \author Andrew Maclean

  \par license
  This software is distributed WITHOUT ANY WARRANTY; without even the 
  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
  YOU, as the USER, take SOLE RESPONSIBILITY for, and are responsible for 
  determining the FITNESS and APPLICABILITY of this software for a particular 
  purpose. The author or any other person or entity are not liable in any way 
  for the use or misuse of any program or software using this software. 
  
  \todo

  \bug

*/
  class CRedfearn
  {
  public:
    //! Construct using a selected ellipsoid and map grid.
    CRedfearn (Ellipsoid::CEllipsoid Ellipsoid, MapGrid::CMapGrid MapGrid) 
    {
      E = Ellipsoid;
      M = MapGrid;
    }

    virtual ~CRedfearn(void) {}

  public:
    //! Convert geographic coordinates to grid coordinates.
    /*!
      Input:

      @param Latitude - in degrees.
      @param Longitude - in degrees.
      
      Output:

      @param Zone
      @param Easting - in meters.
      @param Northing - in meters.
      @param GridConvergence
      @param PointScale
      @param FixedZone - enforce a given zone
    */
    void GetGridCoordinates ( 
        double const & Latitude, double const & Longitude, 
        int & Zone, double & Easting, double & Northing,
        double & GridConvergence, double & PointScale, boost::optional< unsigned int > FixedZone = boost::none );

    //! Convert geographic coordinates to grid coordinates for a specified zone.
    /*!

      Useful when it is necessary to compute grid coordinates in a particular 
      zone for a point whose grid coordinates are known in the adjacent zone.

      Input:

      @param Latitude - in degrees.
      @param Longitude - in degrees.
      @param Zone 
      
      Output:

      @param Easting - in meters.
      @param Northing - in meters.
      @param GridConvergence
      @param PointScale
    */
    void GetZoneGridCoordinates ( 
        double const & Latitude, double const & Longitude, 
        const int & Zone, double & Easting, double & Northing,
        double & GridConvergence, double & PointScale);

    //! Convert grid coordinates to geographic coordinates.
    /*!
      Input:

      @param Zone
      @param Easting - in meters.
      @param Northing - in meters.

      Output:

      @param Latitude - in degrees.
      @param Longitude - in degrees.
      @param GridConvergence
      @param PointScale
    */
    void GetGeographicCoordinates ( 
        int const &Zone, double const & Easting, double const & Northing, 
        double & Latitude, double & Longitude, 
        double & GridConvergence, double & PointScale);

  private:
    Ellipsoid::CEllipsoid E;
    MapGrid::CMapGrid M;
  };

} } } // namespace snark { namespace detail { namespace GeographicGeodeticRectangular {
