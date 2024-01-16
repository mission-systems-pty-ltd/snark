// Copyright (c) 2023 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "roll_pitch_yaw.h"
#include "position.h"

namespace snark { 

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

    operator position() const;
};

} // namespace snark {
