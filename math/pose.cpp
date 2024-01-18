// Copyright (c) 2023 Vsevolod Vlaskine
// All rights reserved.

#include <comma/base/exception.h>
#include "pose.h"
#include "position.h"
#include "rotation_matrix.h"

namespace snark {

::Eigen::Affine3d pose::affine() const
{
    ::Eigen::Translation3d t;
    t.vector() = translation;
    return ::Eigen::Affine3d( t * snark::rotation_matrix::rotation( rotation ) );
}

::Eigen::Affine3d pose::inverse_affine() const
{
    ::Eigen::Translation3d t;
    t.vector() = translation;
    return ::Eigen::Affine3d( snark::rotation_matrix::rotation( rotation ).transpose() * t.inverse() );
}

pose& pose::from( const pose& frame ) // todo! use affine; also, for many applications, this is very suboptimal; should be: calculate frame once, use many times
{
    Eigen::Matrix3d lhs = snark::rotation_matrix::rotation( frame.rotation );
    Eigen::Matrix3d rhs = snark::rotation_matrix::rotation( rotation );
    translation = frame.affine() * translation;
    rotation = snark::rotation_matrix::roll_pitch_yaw( lhs * rhs );
    return *this;
}

pose& pose::to( const pose& frame ) // todo! use affine; also, for many applications, this is very suboptimal; should be: calculate frame once, use many times
{
    Eigen::Matrix3d lhs = snark::rotation_matrix::rotation( frame.rotation ).transpose();
    Eigen::Matrix3d rhs = snark::rotation_matrix::rotation( rotation );
    translation = frame.inverse_affine() * translation;
    rotation = snark::rotation_matrix::roll_pitch_yaw( lhs * rhs );
    return *this;
}

::Eigen::Vector3d pose::tangent_velocity( const roll_pitch_yaw& frame_rotation_velocity ) const
{
    COMMA_ASSERT( std::abs( frame_rotation_velocity.roll() ) <= M_PI, "high angular speeds not supported (todo); got roll: " << frame_rotation_velocity.roll() );
    COMMA_ASSERT( std::abs( frame_rotation_velocity.pitch() ) <= M_PI / 2, "high angular speeds not supported (todo); got pitch: " << frame_rotation_velocity.pitch() );
    COMMA_ASSERT( std::abs( frame_rotation_velocity.yaw() ) <= M_PI, "high angular speeds not supported (todo); got yaw: " << frame_rotation_velocity.yaw() );
    Eigen::AngleAxisd a( snark::rotation_matrix::rotation( frame_rotation_velocity ) );
    return Eigen::Vector3d( a.axis().cross( translation ) * a.angle() ); // Eigen::AngleAxis seems to guarantee that axis is normalized
}

pose pose::velocity_from( const pose& frame, const pose& frame_velocity ) const
{
    return pose{ frame_velocity.translation + snark::rotation_matrix::rotation( frame.rotation ) * tangent_velocity( frame_velocity.rotation ), frame_velocity.rotation };
}

pose::operator position() const { return position{ translation, rotation }; }

} // namespace snark {
