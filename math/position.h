// Copyright (c) 2022 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "roll_pitch_yaw.h"

namespace snark {

struct position // todo!!! consolidate with frames::transform; also: watch performance
{
    Eigen::Vector3d coordinates;
    roll_pitch_yaw orientation;

    position( const Eigen::Vector3d& coordinates = Eigen::Vector3d::Zero(), const Eigen::Vector3d& orientation = Eigen::Vector3d::Zero() ): coordinates( coordinates ), orientation( orientation ) {}

    ::Eigen::Affine3d affine() const;

    ::Eigen::Affine3d inverse_affine() const;

    position& from( const position& frame );

    position& to( const position& frame );
};

} // namespace snark
