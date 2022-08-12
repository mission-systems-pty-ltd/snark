// Copyright (c) 2019 The University of Sydney
// Copyright (c) 2020,2022 Mission Systems Pty Ltd

#pragma once

#include "config.h"
#include "packet.h"
#include "types.h"
#include "../../../timing/traits.h"
#include "../../../visiting/eigen.h"
#include <comma/visiting/traits.h>

namespace comma { namespace visiting {

template <> struct traits< snark::ouster::lidar::config::beam_intrinsics_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::ouster::lidar::config::beam_intrinsics_t& t, Visitor& v )
    {
        v.apply( "beam_altitude_angles", t.beam_altitude_angles );
        v.apply( "beam_azimuth_angles", t.beam_azimuth_angles );
    }
};

template <> struct traits< snark::ouster::lidar::config::imu_intrinsics_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::ouster::lidar::config::imu_intrinsics_t& t, Visitor& v )
    {
        v.apply( "imu_to_sensor_transform", t.imu_to_sensor_transform );
    }
};

template <> struct traits< snark::ouster::lidar::config::lidar_intrinsics_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::ouster::lidar::config::lidar_intrinsics_t& t, Visitor& v )
    {
        v.apply( "lidar_to_sensor_transform", t.lidar_to_sensor_transform );
    }
};

template <> struct traits< snark::ouster::lidar::config_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::ouster::lidar::config_t& t, Visitor& v )
    {
        v.apply( "beam_intrinsics", t.beam_intrinsics );
        v.apply( "imu_intrinsics", t.imu_intrinsics );
        v.apply( "lidar_intrinsics", t.lidar_intrinsics );
    }
};

template <> struct traits< snark::ouster::lidar::data_block_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::ouster::lidar::data_block_t& t, Visitor& v )
    {
        v.apply( "range", t.range );
        v.apply( "reflectivity", t.reflectivity );
        v.apply( "signal", t.signal );
        v.apply( "noise", t.noise );
        v.apply( "unused", t.unused );
    }
};

template <> struct traits< snark::ouster::lidar::v1::azimuth_block_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::ouster::lidar::v1::azimuth_block_t& t, Visitor& v )
    {
        v.apply( "timestamp", t.timestamp );
        v.apply( "measurement_id", t.measurement_id );
        v.apply( "frame_id", t.frame_id );
        v.apply( "encoder_count", t.encoder_count );
        v.apply( "data_blocks", t.data_blocks );
        v.apply( "packet_status", t.packet_status );
    }
};

template <> struct traits< snark::ouster::lidar::imu_block_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::ouster::lidar::imu_block_t& t, Visitor& v )
    {
        v.apply( "start_read_time", t.start_read_time );
        v.apply( "acceleration_read_time", t.acceleration_read_time );
        v.apply( "gyro_read_time", t.gyro_read_time );
        v.apply( "acceleration_x", t.acceleration_x );
        v.apply( "acceleration_y", t.acceleration_y );
        v.apply( "acceleration_z", t.acceleration_z );
        v.apply( "angular_acceleration_x", t.angular_acceleration_x );
        v.apply( "angular_acceleration_y", t.angular_acceleration_y );
        v.apply( "angular_acceleration_z", t.angular_acceleration_z );
    }
};

template <> struct traits< snark::ouster::lidar::output_azimuth_block_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::ouster::lidar::output_azimuth_block_t& t, Visitor& v )
    {
        v.apply( "t", t.t );
        v.apply( "measurement_id", t.measurement_id );
        v.apply( "frame_id", t.frame_id );
        v.apply( "encoder_count", t.encoder_count );
        v.apply( "block", t.block_id );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::ouster::lidar::output_azimuth_block_t& t, Visitor& v )
    {
        v.apply( "t", t.t );
        v.apply( "measurement_id", t.measurement_id );
        v.apply( "frame_id", t.frame_id );
        v.apply( "encoder_count", t.encoder_count );
        v.apply( "block", t.block_id );
    }
};

template <> struct traits< snark::ouster::lidar::output_data_block_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::ouster::lidar::output_data_block_t& t, Visitor& v )
    {
        v.apply( "channel", t.channel );
        v.apply( "range", t.range );
        v.apply( "bearing", t.bearing );
        v.apply( "elevation", t.elevation );
        v.apply( "signal", t.signal );
        v.apply( "reflectivity", t.reflectivity );
        v.apply( "ambient", t.ambient );
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::ouster::lidar::output_data_block_t& t, Visitor& v )
    {
        v.apply( "channel", t.channel );
        v.apply( "range", t.range );
        v.apply( "bearing", t.bearing );
        v.apply( "elevation", t.elevation );
        v.apply( "signal", t.signal );
        v.apply( "reflectivity", t.reflectivity );
        v.apply( "ambient", t.ambient );
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
    }
};

template <> struct traits< snark::ouster::lidar::output_lidar_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::ouster::lidar::output_lidar_t& t, Visitor& v )
    {
        v.apply( "azimuth_block", t.azimuth_block );
        v.apply( "data_block", t.data_block );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::ouster::lidar::output_lidar_t& t, Visitor& v )
    {
        v.apply( "azimuth_block", t.azimuth_block );
        v.apply( "data_block", t.data_block );
    }
};

template <> struct traits< snark::ouster::lidar::output_imu_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::ouster::lidar::output_imu_t& t, Visitor& v )
    {
        v.apply( "start_time", t.start_time );
        v.apply( "acceleration", t.acceleration );
        v.apply( "angular_acceleration", t.angular_acceleration );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::ouster::lidar::output_imu_t& t, Visitor& v )
    {
        v.apply( "start_time", t.start_time );
        v.apply( "acceleration", t.acceleration );
        v.apply( "angular_acceleration", t.angular_acceleration );
    }
};

} } // namespace comma { namespace visiting {
