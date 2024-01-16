// Copyright (c) 2022 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "pose.h"

namespace snark {

// todo! consolidate snark::position and snark::pose types
// this terrible type duplication was due too
// very unfortunate lack of communication
// when frame transforms were implemented
struct position
{
    Eigen::Vector3d coordinates;
    roll_pitch_yaw orientation;

    position( const Eigen::Vector3d& coordinates = Eigen::Vector3d::Zero(), const Eigen::Vector3d& orientation = Eigen::Vector3d::Zero() ): coordinates( coordinates ), orientation( orientation ) {}

    ::Eigen::Affine3d affine() const;

    ::Eigen::Affine3d inverse_affine() const;

    position& from( const position& frame );

    position& to( const position& frame );

    operator pose() const;
};

} // namespace snark
