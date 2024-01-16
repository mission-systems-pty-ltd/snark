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

    ::Eigen::Affine3d affine() const;

    ::Eigen::Affine3d inverse_affine() const;

    pose& from( const pose& frame );

    pose& to( const pose& frame );

    pose velocity_from( const pose& frame_velocity ) const;

    bool operator==( const pose& rhs ) const { return translation == rhs.translation && rotation == rhs.rotation; }

    bool operator!=( const pose& rhs ) const { return !operator==( rhs ); }

    bool near( const pose& rhs, const pose& epsilon = pose{{0.001, 0.001, 0.001}, {0.001, 0.001, 0.001}} ) const; // todo

    operator position() const;
};

} // namespace snark {
