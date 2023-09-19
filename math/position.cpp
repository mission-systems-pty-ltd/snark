// Copyright (c) 2023 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include "position.h"
#include "rotation_matrix.h"

namespace snark {

::Eigen::Affine3d position::affine() const
{
    ::Eigen::Translation3d t;
    t.vector() = coordinates;
    return ::Eigen::Affine3d( t * snark::rotation_matrix::rotation( orientation ) );
}

::Eigen::Affine3d position::inverse_affine() const
{
    ::Eigen::Translation3d t;
    t.vector() = coordinates;
    return ::Eigen::Affine3d( snark::rotation_matrix::rotation( orientation ).transpose() * t.inverse() );
}

position& position::from( const position& frame ) // todo! for many applications, this is very suboptimal; should be: calculate frame once, use many times
{
    Eigen::Matrix3d lhs = snark::rotation_matrix::rotation( frame.orientation );
    Eigen::Matrix3d rhs = snark::rotation_matrix::rotation( orientation );
    coordinates = frame.affine() * coordinates;
    orientation = snark::rotation_matrix::roll_pitch_yaw( lhs * rhs );
    return *this;
}

position& position::to( const position& frame ) // todo! for many applications, this is very suboptimal; should be: calculate frame once, use many times
{
    Eigen::Matrix3d lhs = snark::rotation_matrix::rotation( frame.orientation ).transpose();
    Eigen::Matrix3d rhs = snark::rotation_matrix::rotation( orientation );
    coordinates = frame.inverse_affine() * coordinates;
    orientation = snark::rotation_matrix::roll_pitch_yaw( lhs * rhs );
    return *this;
}

} // namespace snark
