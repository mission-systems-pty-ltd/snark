// Copyright (c) 2019 The University of Sydney
// Copyright (c) 2020,2022 Mission Systems Pty Ltd

#pragma once

#include "packet.h"
#include "../../../math/roll_pitch_yaw.h"
#include "../../../timing/timestamped.h"
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <Eigen/Geometry>

namespace snark { namespace ouster { namespace lidar {

struct transform_t
{
    Eigen::Vector3d translation;
    snark::roll_pitch_yaw rotation;

    transform_t() {}

    transform_t( const Eigen::Vector3d& translation_, const snark::roll_pitch_yaw& rotation_ )
        : translation( translation_ )
        , rotation( rotation_ )
    {}

    transform_t( std::vector< double >& transform_vector );

    std::vector< double > frame() const;
};

struct output_azimuth_block_t
{
    boost::posix_time::ptime t;
    comma::uint16 measurement_id;
    comma::uint16 frame_id;
    comma::uint32 encoder_count;
    comma::uint32 block_id;

    output_azimuth_block_t()
        : measurement_id( 0 )
        , frame_id( 0 )
        , encoder_count( 0 )
        , block_id( 0 )
    {}

    output_azimuth_block_t( const v1::azimuth_block_t& azimuth_block
                          , comma::uint32 block_id );
};

struct output_data_block_t
{
    comma::uint16 channel;
    double range;
    double bearing;
    double elevation;
    comma::uint16 signal;
    comma::uint16 reflectivity;
    comma::uint16 ambient;
    double x;
    double y;
    double z;

    output_data_block_t()
        : channel( 0 )
        , range( 0 )
        , bearing( 0 )
        , elevation( 0 )
        , signal( 0 )
        , reflectivity( 0 )
        , ambient( 0 )
        , x( 0 ), y( 0 ), z( 0 )
    {}

    output_data_block_t( double azimuth_encoder_angle
                       , const data_block_t& data_block
                       , comma::uint16 channel
                       , const beam_angle_lut_t& beam_angle_lut
                       , const transform_t& lidar_transform );
};

struct output_lidar_t
{
    output_azimuth_block_t azimuth_block;
    output_data_block_t data_block;

    output_lidar_t() {}

    output_lidar_t( const output_azimuth_block_t& a, const output_data_block_t& d )
        : azimuth_block( a )
        , data_block( d )
    {}
};

struct output_imu_t
{
    boost::posix_time::ptime start_time;
    snark::timestamped< Eigen::Vector3d > acceleration;
    snark::timestamped< Eigen::Vector3d > angular_acceleration;

    output_imu_t()
        : acceleration( snark::timestamped< Eigen::Vector3d >( Eigen::Vector3d( 0, 0, 0 )))
        , angular_acceleration( snark::timestamped< Eigen::Vector3d >( Eigen::Vector3d( 0, 0, 0 )))
    {}

    output_imu_t( const imu_block_t& imu_block );
};

} } } // namespace snark { namespace ouster { namespace lidar {
