// Copyright (c) 2019 The University of Sydney
// Copyright (c) 2022 Mission Systems Pty Ltd

#pragma once

#include <string>
#include <vector>
#include <comma/base/types.h>

namespace snark { namespace ouster { namespace lidar { namespace config {

struct beam_intrinsics_t
{
    std::vector< double > beam_altitude_angles;
    std::vector< double > beam_azimuth_angles;
};

struct imu_intrinsics_t
{
    std::vector< double > imu_to_sensor_transform;
};

struct lidar_intrinsics_t
{
    std::vector< double > lidar_to_sensor_transform;
};


} // namespace config
struct config_t
{
    config::beam_intrinsics_t beam_intrinsics;
    config::imu_intrinsics_t imu_intrinsics;
    config::lidar_intrinsics_t lidar_intrinsics;
};

} } } // namespace snark { namespace ouster { namespace lidar
