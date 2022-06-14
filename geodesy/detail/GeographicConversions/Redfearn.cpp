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

#include "Redfearn.h"

using namespace snark::detail::GeographicGeodeticRectangular;
using namespace snark::detail::Ellipsoid;
using namespace snark::detail::MapGrid;
using namespace std;

void CRedfearn::GetGridCoordinates ( 
    double const & Latitude, double const & Longitude, 
    int & Zone, double & Easting, double & Northing,
    double & GridConvergence, double & PointScale, boost::optional< unsigned int > FixedZone )
{
  // Degrees to radians and vice versa.
  static const double dtr = atan((double)1.0)/45.0;
  static const double rtd = 45.0/atan((double)1.0);

  double lat = Latitude * dtr;
  //double lon = Longitude * dtr;

  // The ellipsoid
  double a = this->E.SemimajorAxis();
  double e = this->E.Eccentricity();

  // Calculate the meridian distance.
  // Use a series expansion.
  double e2 = e*e;
  double e4 = e2*e2;
  double e6 = e4*e2;
  double A0 = 1-(e2/4.0)-(3.0*e4/64.0)-(5.0*e6/256.0);
  double A2 = (3.0/8.0)*(e2+e4/4.0+15.0*e6/128.0);
  double A4 = (15.0/256.0)*(e4+3.0*e6/4.0);
  double A6 = 35.0*e6/3072.0;
  double s = sin(lat);
  double s2 = sin(2.0*lat);
  double s4 = sin(4.0*lat);
  double s6 = sin(6.0*lat);
  double m = a*(A0*lat-A2*s2+A4*s4-A6*s6);

  // Radii of curvature.
  double rho = a*(1-e2)/pow(1.0-e2*s*s,3.0/2.0);
  double nu = a/sqrt(1-e2*s*s);
  double psi = nu / rho;
  double psi2 = psi*psi;
  double psi3 = psi*psi2;
  double psi4 = psi*psi3;

  // Geographical to Grid

  Zone = int( FixedZone ? *FixedZone : ( Longitude - this->M.LWEZone0() ) / this->M.ZoneWidth() );
  double LongCMZone = Zone * this->M.ZoneWidth() + this->M.CMZone0();
  double w = (Longitude - LongCMZone)*dtr;
  double w2 = w*w;
  double w3 = w*w2;
  double w4 = w*w3;
  double w5 = w*w4;
  double w6 = w*w5;
  double w7 = w*w6;
  double w8 = w*w7;

  double c = cos(lat);
  double c2 = c*c;
  double c3 = c*c2;
  double c4 = c*c3;
  double c5 = c*c4;
  double c6 = c*c5;
  double c7 = c*c6;

  double t = tan(lat);
  double t2 = t*t;
  double t3 = t*t2;
  double t4 = t*t3;
  double t5 = t*t4;
  double t6 = t*t5;

  // Northing
  double term1 = w2*c/2.0;
  double term2 = w4*c3*(4.0*psi2+psi-t2)/24.0;
  double term3 = w6*c5*(8.0*psi4*(11.0-24.0*t2)-28*psi3*(1-6.0*t2)+psi2*(1-32*t2)-psi*(2.0*t2)+t4)/720.0;
  double term4 = w8*c7*(1385.0-3111.0*t2+543.0*t4-t6)/40320.0;
  Northing = this->M.CentralScaleFactor()*(m+nu*s*(term1+term2+term3+term4))+this->M.FalseNorthing();

  // Easting
  term1 = w*c;
  term2 = w3*c3*(psi-t2)/6.0;
  term3 = w5*c5*(4.0*psi3*(1.0-6.0*t2)+psi2*(1.0+8.0*t2)-psi*(2.0*t2)+t4)/120.0;
  term4 = w7*c7*(61.0-479.0*t2+179.0*t4-t6)/5040.0;
  Easting = nu*this->M.CentralScaleFactor()*(term1+term2+term3+term4)+this->M.FalseEasting();

  // Grid Convergence
  term1 = -w;
  term2 = -w3*c2*(2.0*psi2-psi)/3.0;
  term3 = -w5*c4*(psi4*(11.0-24.0*t2)-psi3*(11.0-36.0*t2)+2.0*psi2*(1.0-7.0*t2)+psi*t2)/15.0;
  term4 = w7*c6*(17.0-26.0*t2+2.0*t4)/315.0;
  GridConvergence = s*(term1+term2+term3+term4)*rtd;

  // Point Scale
  term1 = 1.0+(w2*c2*psi)/2.0;
  term2 = w4*c4*(4.0*psi3*(1.0-6.0*t2)+psi2*(1.0+24.0*t2)-4.0*psi*t2)/24.0;
  term3 = w6*c6*(61.0-148.0*t2+16.0*t4)/720.0;
  PointScale = this->M.CentralScaleFactor()*(term1+term2+term3);
}

void CRedfearn::GetZoneGridCoordinates ( 
    double const & Latitude, double const & Longitude, 
    const int & Zone, double & Easting, double & Northing,
    double & GridConvergence, double & PointScale)
{
  // Degrees to radians and vice versa.
  static const double dtr = atan((double)1.0)/45.0;
  static const double rtd = 45.0/atan((double)1.0);

  double lat = Latitude * dtr;
  //double lon = Longitude * dtr;

  // The ellipsoid
  double a = this->E.SemimajorAxis();
  double e = this->E.Eccentricity();

  // Calculate the meridian distance.
  // Use a series expansion.
  double e2 = e*e;
  double e4 = e2*e2;
  double e6 = e4*e2;
  double A0 = 1-(e2/4.0)-(3.0*e4/64.0)-(5.0*e6/256.0);
  double A2 = (3.0/8.0)*(e2+e4/4.0+15.0*e6/128.0);
  double A4 = (15.0/256.0)*(e4+3.0*e6/4.0);
  double A6 = 35.0*e6/3072.0;
  double s = sin(lat);
  double s2 = sin(2.0*lat);
  double s4 = sin(4.0*lat);
  double s6 = sin(6.0*lat);
  double m = a*(A0*lat-A2*s2+A4*s4-A6*s6);

  // Radii of curvature.
  double rho = a*(1-e2)/pow(1.0-e2*s*s,3.0/2.0);
  double nu = a/sqrt(1-e2*s*s);
  double psi = nu / rho;
  double psi2 = psi*psi;
  double psi3 = psi*psi2;
  double psi4 = psi*psi3;

  // Geographical to Grid

  //Zone = int((Longitude-this->M.LWEZone0())/this->M.ZoneWidth());
  double LongCMZone = Zone * this->M.ZoneWidth() + this->M.CMZone0();
  double w = (Longitude - LongCMZone)*dtr;
  double w2 = w*w;
  double w3 = w*w2;
  double w4 = w*w3;
  double w5 = w*w4;
  double w6 = w*w5;
  double w7 = w*w6;
  double w8 = w*w7;

  double c = cos(lat);
  double c2 = c*c;
  double c3 = c*c2;
  double c4 = c*c3;
  double c5 = c*c4;
  double c6 = c*c5;
  double c7 = c*c6;

  double t = tan(lat);
  double t2 = t*t;
  double t3 = t*t2;
  double t4 = t*t3;
  double t5 = t*t4;
  double t6 = t*t5;

  // Northing
  double term1 = w2*c/2.0;
  double term2 = w4*c3*(4.0*psi2+psi-t2)/24.0;
  double term3 = w6*c5*(8.0*psi4*(11.0-24.0*t2)-28*psi3*(1-6.0*t2)+psi2*(1-32*t2)-psi*(2.0*t2)+t4)/720.0;
  double term4 = w8*c7*(1385.0-3111.0*t2+543.0*t4-t6)/40320.0;
  Northing = this->M.CentralScaleFactor()*(m+nu*s*(term1+term2+term3+term4))+this->M.FalseNorthing();

  // Easting
  term1 = w*c;
  term2 = w3*c3*(psi-t2)/6.0;
  term3 = w5*c5*(4.0*psi3*(1.0-6.0*t2)+psi2*(1.0+8.0*t2)-psi*(2.0*t2)+t4)/120.0;
  term4 = w7*c7*(61.0-479.0*t2+179.0*t4-t6)/5040.0;
  Easting = nu*this->M.CentralScaleFactor()*(term1+term2+term3+term4)+this->M.FalseEasting();

  // Grid Convergence
  term1 = -w;
  term2 = -w3*c2*(2.0*psi2-psi)/3.0;
  term3 = -w5*c4*(psi4*(11.0-24.0*t2)-psi3*(11.0-36.0*t2)+2.0*psi2*(1.0-7.0*t2)+psi*t2)/15.0;
  term4 = w7*c6*(17.0-26.0*t2+2.0*t4)/315.0;
  GridConvergence = s*(term1+term2+term3+term4)*rtd;

  // Point Scale
  term1 = 1.0+(w2*c2*psi)/2.0;
  term2 = w4*c4*(4.0*psi3*(1.0-6.0*t2)+psi2*(1.0+24.0*t2)-4.0*psi*t2)/24.0;
  term3 = w6*c6*(61.0-148.0*t2+16.0*t4)/720.0;
  PointScale = this->M.CentralScaleFactor()*(term1+term2+term3);
}

void CRedfearn::GetGeographicCoordinates ( 
    int const &Zone, double const & Easting, double const & Northing, 
    double & Latitude, double & Longitude, 
    double & GridConvergence, double & PointScale)
{
  // Degrees to radians and vice versa.
  static const double dtr = atan((double)1.0)/45.0;
  static const double rtd = 45.0/atan((double)1.0);

  // The ellipsoid
  double a = this->E.SemimajorAxis();
  double e = this->E.Eccentricity();

  // Calculate the meridian distance.
  // Use a series expansion.
  double e2 = e*e;
  //double e4 = e2*e2;
  //double e6 = e4*e2;
  double m = (Northing-this->M.FalseNorthing())/this->M.CentralScaleFactor();

  // Foot-point latitude
  // This is the latitude for which the meridian distance equals 
  // the true northing divided by the central scale factor.
  double n = this->E.N();
  double n2 = n*n;
  double n3 = n*n2;
  double n4 = n*n3;
  double sig = m / this->E.G() * dtr;
  double sig2 = sin(2.0*sig);
  double sig4 = sin(4.0*sig);
  double sig6 = sin(6.0*sig);
  double sig8 = sin(8.0*sig);
  double fpl = sig+((3.0*n/2.0)-(27.0*n3/32.0))*sig2+
    ((21.0*n2/16.0)-(55.0*n4/32.0))*sig4+(151.0*n3/96.0)*sig6+
    (1097.0*n4/512.0)*sig8;

  double s = sin(fpl);

  // Radii of curvature.
  double rho = a*(1-e2)/pow(1.0-e2*s*s,3.0/2.0);
  double nu = a/sqrt(1-e2*s*s);
  double psi = nu / rho;
  double psi2 = psi*psi;
  double psi3 = psi*psi2;
  double psi4 = psi*psi3;

  double ss = 1.0/cos(fpl); // secant

  double t = tan(fpl);
  double t2 = t*t;
  double t3 = t*t2;
  double t4 = t*t3;
  double t5 = t*t4;
  double t6 = t*t5;

  //double tknu = t / (this->CentralScaleFactor() * nu );

  double K0 = this->M.CentralScaleFactor();
  double E = Easting-this->M.FalseEasting();
  double Ek = E/K0;

  // Ok generate some more powers.
  double x = Ek/nu;
  double x2 = x*x;
  double x3 = x*x2;
  double x4 = x*x3;
  double x5 = x*x4;
  double x6 = x*x5;
  double x7 = x*x6;

  double y = Ek*Ek/(nu*rho);
  double y2 = y*y;
  double y3 = y*y2;

  // Latitude
  double term1 = (x*E/2.0);
  double term2 = (x3*E/24.0)*(-4.0*psi2+9.0*psi*(1.0-t2)+12.0*t2);
  double term3 = (x5*E/720.0)*(8.0*psi4*(11.0-24.0*t2)-12.0*psi3*(21.0-71.0*t2)+15.0*psi2*(15.0-98.0*t2+15*t4)+180.0*psi*(5.0*t2-3.0*t4)+360*t4);
  double term4 = (x7*E/40320.0)*(1385.0+3633.0*t2+4095.0*t4+1575.0*t6);
  Latitude = (fpl+(t/(K0*rho))*(-term1+term2-term3+term4))*rtd;

  // Longitude
  double CentralMeridian = Zone*this->M.ZoneWidth()+this->M.CentralMeridianZone1()-this->M.ZoneWidth();
  term1 = x;
  term2 = (x3/6.0)*(psi+2.0*t2);
  term3 = (x5/120.0)*(-4.0*psi3*(1.0-6.0*t2)+psi2*(9.0-68.0*t2)+72.0*psi*t2+24.0*t4);
  term4 = (x7/5040.0)*(61.0+662.0*t2+1320.0*t4+720.0*t6);
  Longitude = CentralMeridian+ss*(term1-term2+term3-term4)*rtd;

  // Grid convergence
  term1 = -t*x;
  term2 = (t*x3/3.0)*(-2.0*psi2+3.0*psi+t2);
  term3 = -(t*x5/15.0)*(psi4*(11.0-24.0*t2)-3.0*psi3*(8.0-23.0*t2)+5.0*psi2*(3.0-14.0*t2)+30.0*psi*t2+3.0*t4);
  term4 = (t*x7/315.0)*(17.0+77.0*t2+105.0*t4+45.0*t6);
  GridConvergence = (term1+term2+term3+term4)*rtd;

  // Point scale
  term1 = 1.0+y/2.0;
  term2 = (y2/24.0)*(4.0*psi*(1-6*t2)-3*(1-16*t2)-24*t2/psi);
  term3 = y3/720.0;
  PointScale = this->M.CentralScaleFactor()*(term1+term2+term3);
}
