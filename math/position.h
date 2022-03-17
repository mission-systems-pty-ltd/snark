// Copyright (c) 2022 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include "roll_pitch_yaw.h"
#include <Eigen/Core>

namespace snark {

struct position
{
    Eigen::Vector3d coordinates;
    roll_pitch_yaw orientation;
    position( const Eigen::Vector3d& coordinates = Eigen::Vector3d::Zero(), const Eigen::Vector3d& orientation = Eigen::Vector3d::Zero() ): coordinates( coordinates ), orientation( orientation ) {}
};

} // namespace snark
