// Copyright (c) 2019 The University of Sydney
// Copyright (c) 2022 Mission Systems Pty Ltd

#pragma once

#include <string>
#include <vector>
#include <comma/base/types.h>

namespace snark { namespace ouster { namespace lidar { namespace config {

struct device_t
{
    std::string firmware;
};

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

struct lidar_data_format_t
{
    unsigned int pixels_per_column;
};

} // namespace config

struct config_t
{
    config::device_t device;
    config::beam_intrinsics_t beam_intrinsics;
    config::imu_intrinsics_t imu_intrinsics;
    config::lidar_intrinsics_t lidar_intrinsics;
    config::lidar_data_format_t lidar_data_format;
};

} } } // namespace snark { namespace ouster { namespace lidar
