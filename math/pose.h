// Copyright (c) 2023 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "roll_pitch_yaw.h"

namespace snark { 

struct position;

// todo! consolidate snark::position and snark::pose types
// this terrible type duplication was due too
// very unfortunate lack of communication
// when frame transforms were implemented
struct pose
{
    ::Eigen::Vector3d translation;

    snark::roll_pitch_yaw rotation; // todo? use Eigen::EulerAngles?

    pose() : translation( ::Eigen::Vector3d::Zero() ), rotation( 0, 0, 0 ) {}

    pose( const ::Eigen::Vector3d& translation, const snark::roll_pitch_yaw& rotation ): translation( translation ), rotation( rotation ) {}

    pose( double x, double y, double z, double roll = 0, double pitch = 0, double yaw = 0 ): translation( x, y, z ), rotation( roll, pitch, yaw ) {}

    ::Eigen::Affine3d affine() const;

    ::Eigen::Affine3d inverse_affine() const;

    pose& from( const pose& frame );

    pose& to( const pose& frame );

    /// @todo current limitation: only resulting rotational velocity under 1Hz is currently supported
    ///       it's not too hard to add element-wise
    ::Eigen::Vector3d tangent_velocity( const roll_pitch_yaw& frame_rotation_velocity ) const;

    /// @todo current limitation: only single rotational velocity is currently supported, i.e.
    ///       it won't work if e.g. you have your aircraft rolling and your sensor mounted on
    ///       the wing spinning
    pose velocity_from( const pose& frame, const pose& frame_velocity ) const;

    double& x() { return translation.x(); }
    double x() const { return translation.x(); }
    double& y() { return translation.y(); }
    double y() const { return translation.y(); }
    double& z() { return translation.z(); }
    double z() const { return translation.z(); }
    double& roll() { return rotation.roll(); }
    double roll() const { return rotation.roll(); }
    double& pitch() { return rotation.pitch(); }
    double pitch() const { return rotation.pitch(); }
    double& yaw() { return rotation.yaw(); }
    double yaw() const { return rotation.yaw(); }
    bool operator==( const pose& rhs ) const { return translation == rhs.translation && rotation == rhs.rotation; }
    bool operator!=( const pose& rhs ) const { return !operator==( rhs ); }
    bool near( const pose& rhs, const pose& epsilon = pose( 0.001, 0.001, 0.001, 0.001, 0.001, 0.001 ) ) const; // todo
    operator position() const;
};

} // namespace snark {
