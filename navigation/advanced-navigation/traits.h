// Copyright (c) 2017 The University of Sydney
// Copyright (c) 2022 Mission Systems Pty Ltd

#pragma once

#include "messages.h"
#include "../../math/spherical_geometry/traits.h"
#include <comma/visiting/traits.h>
#include <string>

namespace comma { namespace visiting {

template < unsigned int Size, bool Signed, bool Floating, std::size_t N > struct traits< boost::array<comma::packed::detail::endian< comma::packed::detail::little, Size,Signed,Floating>, N > >
{
    template< typename K, typename V > static void visit( const K& k, const boost::array<comma::packed::detail::endian< comma::packed::detail::little, Size,Signed,Floating>, N >& t, V& v )
    {
        for( std::size_t i = 0; i < t.size(); i++ ) { v.apply( std::string(1,'x'+i).c_str(), t[i]() ); }
    }
};

template <>
struct traits< snark::navigation::advanced_navigation::messages::request >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::navigation::advanced_navigation::messages::request& p, Visitor& v )
    {
        v.apply( "packet_id", p.packet_id() );
    }
    template < typename Key, class Visitor > static void visit( const Key&, snark::navigation::advanced_navigation::messages::request& p, Visitor& v )
    {
        auto a = p.packet_id();
        v.apply( "packet_id", a );
        p.packet_id = a;
    }
};

template <>
struct traits< snark::navigation::advanced_navigation::messages::reset >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::navigation::advanced_navigation::messages::reset& p, Visitor& v )
    {
        v.apply( "verification_sequence", p.verification_sequence() );
    }
    template < typename Key, class Visitor > static void visit( const Key&, snark::navigation::advanced_navigation::messages::reset& p, Visitor& v )
    {
        auto a = p.verification_sequence();
        v.apply( "verification_sequence", a );
        p.verification_sequence = a;
    }
};

template <>
struct traits< snark::navigation::advanced_navigation::messages::system_state >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::navigation::advanced_navigation::messages::system_state& p, Visitor& v )
    {
        v.apply( "system_status", p.system_status() );
        v.apply( "filter_status", p.filter_status() );
        v.apply( "t", p.t() );
        v.apply( "coordinates", p.coordinates() );
        v.apply( "height", p.height() );
        v.apply( "velocity", p.velocity );
        v.apply( "body_acceleration", p.body_acceleration );
        v.apply( "g_force", p.g_force() );
        v.apply( "orientation", p.orientation );
        v.apply( "angular_velocity", p.angular_velocity );
        v.apply( "position_stddev", p.position_stddev );
    }
};

template <>
struct traits< snark::navigation::advanced_navigation::messages::unix_time >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::navigation::advanced_navigation::messages::unix_time& p, Visitor& v )
    {
        v.apply( "t", p.t() );
    }
};

template <>
struct traits< snark::navigation::advanced_navigation::messages::filter_status_description >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::navigation::advanced_navigation::messages::filter_status_description& p, Visitor& v )
    {
        v.apply( "status", p.status );
        v.apply( "orientation_filter_initialised", p.orientation_filter_initialised() );
        v.apply( "navigation_filter_initialised", p.navigation_filter_initialised() );
        v.apply( "heading_initialised", p.heading_initialised() );
        v.apply( "utc_time_initialised", p.utc_time_initialised() );
        v.apply( "gnss_fix", p.gnss_fix() );
        v.apply( "event_1_occurred", p.event_1_occurred() );
        v.apply( "event_2_occurred", p.event_2_occurred() );
        v.apply( "internal_gnss_enabled", p.internal_gnss_enabled() );
        v.apply( "dual_antenna_heading_active", p.dual_antenna_heading_active() );
        v.apply( "velocity_heading_enabled", p.velocity_heading_enabled() );
        v.apply( "atmospheric_altitude_enabled", p.atmospheric_altitude_enabled() );
        v.apply( "external_position_active", p.external_position_active() );
        v.apply( "external_velocity_active", p.external_velocity_active() );
        v.apply( "external_heading_active", p.external_heading_active() );
    }
};

template <>
struct traits< snark::navigation::advanced_navigation::messages::system_status_description >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::navigation::advanced_navigation::messages::system_status_description& p, Visitor& v )
    {
        v.apply( "status", p.status );
        v.apply( "system_failure", p.system_failure() );
        v.apply( "accelerometer_sensor_failure", p.accelerometer_sensor_failure() );
        v.apply( "gyroscope_sensor_failure", p.gyroscope_sensor_failure() );
        v.apply( "magnetometer_sensor_failure", p.magnetometer_sensor_failure() );
        v.apply( "pressure_sensor_failure", p.pressure_sensor_failure() );
        v.apply( "gnss_failure", p.gnss_failure() );
        v.apply( "accelerometer_over_range", p.accelerometer_over_range() );
        v.apply( "gyroscope_over_range", p.gyroscope_over_range() );
        v.apply( "magnetometer_over_range", p.magnetometer_over_range() );
        v.apply( "pressure_over_range", p.pressure_over_range() );
        v.apply( "minimum_temperature_alarm", p.minimum_temperature_alarm() );
        v.apply( "maximum_temperature_alarm", p.maximum_temperature_alarm() );
        v.apply( "low_voltage_alarm", p.low_voltage_alarm() );
        v.apply( "high_voltage_alarm", p.high_voltage_alarm() );
        v.apply( "gnss_antenna_short_circuit", p.gnss_antenna_short_circuit() );
        v.apply( "data_output_overflow_alarm", p.data_output_overflow_alarm() );
    }
};

template <>
struct traits< snark::navigation::advanced_navigation::messages::raw_sensors >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::navigation::advanced_navigation::messages::raw_sensors& p, Visitor& v )
    {
        v.apply( "accelerometer", p.accelerometer );
        v.apply( "gyroscope", p.gyroscope );
        v.apply( "magnetometer", p.magnetometer );
        v.apply( "imu_temperature", p.imu_temperature() );
        v.apply( "pressure", p.pressure() );
        v.apply( "pressure_temperature", p.pressure_temperature() );
    }
};

template <>
struct traits< snark::navigation::advanced_navigation::messages::satellites >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::navigation::advanced_navigation::messages::satellites& p, Visitor& v )
    {
        v.apply( "hdop", p.hdop() );
        v.apply( "vdop", p.vdop() );
        v.apply( "gps_satellites", p.gps_satellites() );
        v.apply( "glonass_satellites", p.glonass_satellites() );
        v.apply( "beidou_satellites", p.beidou_satellites() );
        v.apply( "galileo_satellites", p.galileo_satellites() );
        v.apply( "sbas_satellites", p.sbas_satellites() );
    }
};

template <>
struct traits< snark::navigation::advanced_navigation::messages::geodetic_position >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::navigation::advanced_navigation::messages::geodetic_position& p, Visitor& v )
    {
        v.apply("latitude", p.latitude());
        v.apply("longitude", p.longitude());
        v.apply("height", p.height());
    }
};

template <>
struct traits< snark::navigation::advanced_navigation::messages::acceleration >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::navigation::advanced_navigation::messages::acceleration& p, Visitor& v )
    {
        v.apply( "x", p.x() );
        v.apply( "y", p.y() );
        v.apply( "z", p.z() );
    }
};

template <>
struct traits< snark::navigation::advanced_navigation::messages::euler_orientation >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::navigation::advanced_navigation::messages::euler_orientation& p, Visitor& v )
    {
        v.apply( "roll", p.roll() );
        v.apply( "pitch", p.pitch() );
        v.apply( "heading", p.heading() );
    }
};

template <>
struct traits< snark::navigation::advanced_navigation::messages::quaternion_orientation >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::navigation::advanced_navigation::messages::quaternion_orientation& p, Visitor& v )
    {
        v.apply("qs", p.qs());
        v.apply("qx", p.qx());
        v.apply("qy", p.qy());
        v.apply("qz", p.qz());
    }
};

template <>
struct traits< snark::navigation::advanced_navigation::messages::angular_velocity >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::navigation::advanced_navigation::messages::angular_velocity& p, Visitor& v )
    {
        v.apply( "x", p.x() );
        v.apply( "y", p.y() );
        v.apply( "z", p.z() );
    }
};

template <>
struct traits< snark::navigation::advanced_navigation::messages::external_time >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::navigation::advanced_navigation::messages::external_time& p, Visitor& v )
    {
        v.apply( "t", p.t() );
    }
    template < typename Key, class Visitor > static void visit( const Key&, snark::navigation::advanced_navigation::messages::external_time& p, Visitor& v )
    {
        boost::posix_time::ptime timestamp;
        v.apply( "t", timestamp );
        boost::posix_time::time_duration time_since_epoch = timestamp - boost::posix_time::ptime( boost::gregorian::date( 1970, 1, 1 ));
        p.unix_time_seconds = time_since_epoch.total_seconds();
        p.microseconds = time_since_epoch.total_microseconds() - time_since_epoch.total_seconds() * 1000000;
    }
};

template <>
struct traits< snark::navigation::advanced_navigation::messages::filter_options >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::navigation::advanced_navigation::messages::filter_options& p, Visitor& v )
    {
        v.apply( "permanent", p.permanent() );
        v.apply( "vehicle_types", p.vehicle_types() );
        v.apply( "internal_gnss_enabled", p.internal_gnss_enabled() );
        v.apply( "magnetic_heading_enabled", p.magnetic_heading_enabled() );
        v.apply( "atmospheric_altitude_enabled", p.atmospheric_altitude_enabled() );
        v.apply( "velocity_heading_enabled", p.velocity_heading_enabled() );
        v.apply( "reversing_detection_enabled", p.reversing_detection_enabled() );
        v.apply( "motion_analysis_enabled", p.motion_analysis_enabled() );
        v.apply( "automatic_magnetic_calibration_enabled", p.automatic_magnetic_calibration_enabled() );
        v.apply( "reserved", p.reserved() );
    }
    template < typename Key, class Visitor > static void visit( const Key&, snark::navigation::advanced_navigation::messages::filter_options& p, Visitor& v )
    {
        auto permanent = p.permanent();
        v.apply( "permanent", permanent );
        p.permanent = permanent;
        auto vehicle_types = p.vehicle_types();
        v.apply( "vehicle_types", vehicle_types );
        p.vehicle_types = vehicle_types;
        auto internal_gnss_enabled = p.internal_gnss_enabled();
        v.apply( "internal_gnss_enabled", internal_gnss_enabled );
        p.internal_gnss_enabled = internal_gnss_enabled;
        auto magnetic_heading_enabled = p.magnetic_heading_enabled();
        v.apply( "magnetic_heading_enabled", magnetic_heading_enabled );
        p.magnetic_heading_enabled = magnetic_heading_enabled;
        auto atmospheric_altitude_enabled = p.atmospheric_altitude_enabled();
        v.apply( "atmospheric_altitude_enabled", atmospheric_altitude_enabled );
        p.atmospheric_altitude_enabled = atmospheric_altitude_enabled;
        auto velocity_heading_enabled = p.velocity_heading_enabled();
        v.apply( "velocity_heading_enabled", velocity_heading_enabled );
        p.velocity_heading_enabled = velocity_heading_enabled;
        auto reversing_detection_enabled = p.reversing_detection_enabled();
        v.apply( "reversing_detection_enabled", reversing_detection_enabled );
        p.reversing_detection_enabled = reversing_detection_enabled;
        auto motion_analysis_enabled = p.motion_analysis_enabled();
        v.apply( "motion_analysis_enabled", motion_analysis_enabled );
        p.motion_analysis_enabled = motion_analysis_enabled;
        auto automatic_magnetic_calibration_enabled = p.automatic_magnetic_calibration_enabled();
        v.apply( "automatic_magnetic_calibration_enabled", automatic_magnetic_calibration_enabled );
        p.automatic_magnetic_calibration_enabled = automatic_magnetic_calibration_enabled;
        auto reserved = p.reserved();
        v.apply( "reserved", reserved );
        p.reserved = reserved;
    }
};

template <>
struct traits< snark::navigation::advanced_navigation::messages::magnetic_calibration_configuration >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::navigation::advanced_navigation::messages::magnetic_calibration_configuration& p, Visitor& v )
    {
        v.apply("action", p.action());
    }
    template < typename Key, class Visitor > static void visit( const Key&, snark::navigation::advanced_navigation::messages::magnetic_calibration_configuration& p, Visitor& v )
    {
        auto a=p.action();
        v.apply("action", a);
        p.action=a;
    }
};

template <>
struct traits< snark::navigation::advanced_navigation::messages::magnetic_calibration_status >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::navigation::advanced_navigation::messages::magnetic_calibration_status& p, Visitor& v )
    {
        v.apply("status", p.status());
        v.apply("progress", p.progress());
        v.apply("error", p.error());
    }
};

} } // namespace comma { namespace visiting {
