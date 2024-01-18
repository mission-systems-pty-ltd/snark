// Copyright (c) 2023 Vsevolod Vlaskine
// All rights reserved.

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

// see e.g. https://physics.stackexchange.com/questions/672712/angular-velocity-via-extrinsic-euler-angles
::Eigen::Vector3d pose::tangent_velocity( const roll_pitch_yaw& r ) const
{
    const auto& roll = snark::rotation_matrix::rotation( roll_pitch_yaw( r.roll(), 0, 0 ) );
    const auto& pitch = snark::rotation_matrix::rotation( roll_pitch_yaw( 0, r.pitch(), 0 ) );
    //const auto& yaw = snark::rotation_matrix::rotation( roll_pitch_yaw( 0, 0, r.yaw() ) );
    Eigen::Vector3d v = Eigen::Vector3d( r.roll(), 0, 0 ) + roll * ( Eigen::Vector3d( 0, r.pitch(), 0 ) + pitch * Eigen::Vector3d( 0, 0, r.yaw() ) );
    double angle = v.norm();
    Eigen::AngleAxisd a( snark::rotation_matrix::rotation( r ) ); //Eigen::Vector3d axis = v / angle; // todo! use v instead of Eigen::AngleAxisd
    return Eigen::Vector3d( a.axis().cross( translation ) * angle ); // Eigen::AngleAxis seems to guarantee that axis is normalized
}

pose pose::velocity_from( const pose& frame, const pose& frame_velocity ) const
{
    return pose{ frame_velocity.translation + snark::rotation_matrix::rotation( frame.rotation ) * tangent_velocity( frame_velocity.rotation ), frame_velocity.rotation };
}

pose::operator position() const { return position{ translation, rotation }; }

} // namespace snark {
