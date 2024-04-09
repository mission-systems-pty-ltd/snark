// Copyright (c) 2019 The University of Sydney
// Copyright (c) 2020-2024 Mission Systems Pty Ltd

#include "types.h"
#include "../../../math/range_bearing_elevation.h"
#include "../../../math/rotation_matrix.h"

namespace snark { namespace ouster { namespace lidar {

transform_t::transform_t( std::vector< double >& transform_vector )
{
    Eigen::Matrix4d mat = Eigen::Map< Eigen::Matrix< double, 4, 4, Eigen::RowMajor > >( transform_vector.data() );
    translation = Eigen::Vector3d( mat.block< 3, 1 >( 0, 3 ) * 0.001 );
    Eigen::Matrix3d rotation_matrix = mat.block< 3, 3 >( 0, 0 );
    rotation = snark::rotation_matrix( rotation_matrix ).roll_pitch_yaw();
}

std::vector< double > transform_t::frame() const
{
    return std::vector< double >({ translation.x(), translation.y(), translation.z()
                                 , rotation.roll(), rotation.pitch(), rotation.yaw()
                                 });
}

output_azimuth_block_t::output_azimuth_block_t( const v1::azimuth_block_t& azimuth_block
                                              , comma::uint32 block_id
                                              , timestamp_converter_t& timestamp_converter )
    : t( timestamp_converter.to_utc( azimuth_block.timestamp() ))
    , measurement_id( azimuth_block.measurement_id() )
    , frame_id( azimuth_block.frame_id() )
    , encoder_count( azimuth_block.encoder_count() )
    , block_id( block_id )
{
}

output_azimuth_block_t::output_azimuth_block_t( const v2::measurement_block_t<16>& measurement_block
                                              , comma::uint32 block_id
                                              , timestamp_converter_t& timestamp_converter )
    : t( timestamp_converter.to_utc( measurement_block.timestamp() ))
    , measurement_id( measurement_block.measurement_id() )
    , frame_id( measurement_block.frame_id() )
    , encoder_count( measurement_block.encoder_count() )
    , block_id( block_id )
{
}

output_azimuth_block_t::output_azimuth_block_t( const v2::measurement_block_t<32>& measurement_block
                                              , comma::uint32 block_id
                                              , timestamp_converter_t& timestamp_converter )
    : t( timestamp_converter.to_utc( measurement_block.timestamp() ))
    , measurement_id( measurement_block.measurement_id() )
    , frame_id( measurement_block.frame_id() )
    , encoder_count( measurement_block.encoder_count() )
    , block_id( block_id )
{
}

output_azimuth_block_t::output_azimuth_block_t( const v2::measurement_block_t<64>& measurement_block
                                              , comma::uint32 block_id
                                              , timestamp_converter_t& timestamp_converter )
    : t( timestamp_converter.to_utc( measurement_block.timestamp() ))
    , measurement_id( measurement_block.measurement_id() )
    , frame_id( measurement_block.frame_id() )
    , encoder_count( measurement_block.encoder_count() )
    , block_id( block_id )
{
}

output_azimuth_block_t::output_azimuth_block_t( const v2::measurement_block_t<128>& measurement_block
                                              , comma::uint32 block_id
                                              , timestamp_converter_t& timestamp_converter)
    : t( timestamp_converter.to_utc( measurement_block.timestamp() ))
    , measurement_id( measurement_block.measurement_id() )
    , frame_id( measurement_block.frame_id() )
    , encoder_count( measurement_block.encoder_count() )
    , block_id( block_id )
{
}

output_data_block_t::output_data_block_t( double azimuth_encoder_angle
                                        , const data_block_t& data_block
                                        , comma::uint16 channel
                                        , const beam_angle_lut_t& beam_angle_lut
                                        , const transform_t& lidar_transform )
    : channel( channel )
    , signal( data_block.signal() )
    , reflectivity( data_block.reflectivity() )
    , ambient( data_block.noise() )
{
    // See §4.1 Sensor Coordinate Frame in the Software User Guide
    range = static_cast< double >( data_block.range() & 0x000FFFFF ) / 1000;
    bearing = M_PI * 2 - ( azimuth_encoder_angle + beam_angle_lut[ channel ].azimuth );
    elevation = beam_angle_lut[ channel ].altitude;

    // The transform to go from lidar to sensor frame is encoded in the
    // lidar_to_sensor_transform field from get_lidar_intrinsics, but we know from
    // the documentation that it's a 180° rotation about Z and that will never change.
    //
    // We also know that it's a translation in the Z axis of 36.180mm but that might
    // change so we take that number from the config that the device provides to us.

    Eigen::Vector3d cartesian = snark::range_bearing_elevation( range, bearing - M_PI, elevation ).to_cartesian() + lidar_transform.translation;
    x = cartesian[0];
    y = cartesian[1];
    z = cartesian[2];
}

const double g0 = 9.80665;    // m/s^2, as per ISO 80000

output_imu_t::output_imu_t( const imu_block_t& imu_block, timestamp_converter_t& timestamp_converter  )
    : start_time( timestamp_converter.to_utc( imu_block.start_read_time() ))
    , acceleration( timestamp_converter.to_utc( imu_block.acceleration_read_time() )
                  , Eigen::Vector3d( imu_block.acceleration_x() * g0
                                   , imu_block.acceleration_y() * g0
                                   , imu_block.acceleration_z() * g0 ))
    , angular_acceleration( timestamp_converter.to_utc( imu_block.gyro_read_time() )
                          , Eigen::Vector3d( imu_block.angular_acceleration_x() * M_PI / 180
                                           , imu_block.angular_acceleration_y() * M_PI / 180
                                           , imu_block.angular_acceleration_z() * M_PI / 180 ))
{
}

} } } // namespace snark { namespace ouster { namespace lidar {
