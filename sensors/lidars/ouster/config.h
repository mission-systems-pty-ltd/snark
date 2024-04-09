// Copyright (c) 2019 The University of Sydney
// Copyright (c) 2022-2024 Mission Systems Pty Ltd

#pragma once

#include <comma/base/types.h>
#include <string>
#include <vector>

namespace snark { namespace ouster { namespace lidar { namespace config {

struct time_standard_t
{
    enum ts { utc = 0, tai };
    ts value;
    time_standard_t( const std::string& s );
    time_standard_t( time_standard_t::ts v ) : value( v ) {}
    time_standard_t() : value( utc ) {}         // default to UTC

    operator ts() const { return value; }
    operator std::string() const;
};

inline std::ostream& operator<<( std::ostream& os, const time_standard_t& ts )
{
    os << std::string( ts );
    return os;
}

struct device_t
{
    std::string firmware;
    time_standard_t time_standard;
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
