// Copyright (c) 2019 The University of Sydney
// Copyright (c) 2020,2022 Mission Systems Pty Ltd

#pragma once

#include "config.h"
#include <comma/base/types.h>
#include <comma/packed/packed.h>
#include <array>

namespace snark { namespace ouster { namespace lidar {

// Note that both the 64 and 16 beam lidar have the same raw packet structure,
// with 64 channels of data. For the 16 beam channel 1, containing signal and
// reflectivity values, is zero for all non-existent beams.
const std::size_t pixels_per_column = 64;

const comma::uint32 packet_status_good = 0xffffffff;
const comma::uint32 packet_status_bad = 0;
const unsigned int encoder_ticks_per_rev = 90112;

struct data_block_t : public comma::packed::packed_struct< data_block_t, 12 >
{
    comma::packed::little_endian::uint32 range;
    comma::packed::little_endian::uint16 reflectivity;
    comma::packed::little_endian::uint16 signal;
    comma::packed::little_endian::uint16 noise;
    comma::packed::little_endian::uint16 unused;
};

struct imu_block_t : public comma::packed::packed_struct< imu_block_t, 48 >
{
    comma::packed::little_endian::uint64 start_read_time;
    comma::packed::little_endian::uint64 acceleration_read_time;
    comma::packed::little_endian::uint64 gyro_read_time;
    comma::packed::little_endian::float32 acceleration_x;
    comma::packed::little_endian::float32 acceleration_y;
    comma::packed::little_endian::float32 acceleration_z;
    comma::packed::little_endian::float32 angular_acceleration_x;
    comma::packed::little_endian::float32 angular_acceleration_y;
    comma::packed::little_endian::float32 angular_acceleration_z;
};

struct beam_angle_lut_entry
{
    double altitude;
    double azimuth;

    beam_angle_lut_entry() : altitude( 0 ), azimuth( 0 ) {}

    beam_angle_lut_entry( double altitude_, double azimuth_ )
        : altitude( altitude_ )
        , azimuth( azimuth_ )
    {}
};

typedef std::array< beam_angle_lut_entry, pixels_per_column > beam_angle_lut_t;
beam_angle_lut_t get_beam_angle_lut( const config::beam_intrinsics_t& beam_intrinsics );

namespace v1 {

struct azimuth_block_t : public comma::packed::packed_struct< azimuth_block_t, 20 + pixels_per_column * sizeof( data_block_t ) >
{
    comma::packed::little_endian::uint64 timestamp;
    comma::packed::little_endian::uint16 measurement_id;
    comma::packed::little_endian::uint16 frame_id;
    comma::packed::little_endian::uint32 encoder_count;
    std::array< data_block_t, pixels_per_column > data_blocks;
    comma::packed::little_endian::uint32 packet_status;
};

} // namespace v1
} } } // namespace snark { namespace ouster { namespace lidar
