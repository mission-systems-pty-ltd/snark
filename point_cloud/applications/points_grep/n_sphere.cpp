// Copyright (c) 2014 The University of Sydney

#include <Eigen/Eigen>
#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include "n_sphere.h"

namespace snark { namespace geometry {

n_sphere::n_sphere( const Eigen::VectorXd& center, const double radius )
    : center( center )
    , radius( radius )
    , squared_radius( radius * radius )
{
}

bool n_sphere::contains( const Eigen::VectorXd& p, const double epsilon ) const
{
    if( p.size() != center.size() ) { COMMA_THROW( comma::exception, "expected same dimension as n-sphere: "<< center.size() << "; got dimension: " << p.size() ); }
    return !comma::math::less( squared_radius, ( p - center ).squaredNorm(), epsilon );
}

bool n_sphere::intersects( const n_sphere& s, const double epsilon ) const
{
    if( s.center.size() != center.size() ) { COMMA_THROW( comma::exception, "expected same dimension as n-sphere: "<< center.size() << "; got dimension: " << s.center.size() ); }
    return !comma::math::less( s.radius + radius, ( s.center - center ).norm(), epsilon );
}

} } // namespace snark{ { namespace geometry {
