// Copyright (c) 2017 The University of Sydney

#pragma once

#include "../../math/spherical_geometry/coordinates.h"
#include <comma/base/types.h>
#include <comma/packed/packed.h>
#include <boost/array.hpp>
#include <boost/date_time/posix_time/ptime.hpp>


namespace snark { namespace navigation { namespace advanced_navigation {

namespace messages {

/// header packet
struct header : public comma::packed::packed_struct< header, 5 >
{
    comma::packed::uint8 LRC;
    comma::packed::uint8 id;
private:
    comma::packed::uint8 length;
public:
    comma::packed::little_endian::uint16 msg_crc;

    bool is_valid() const;
    // The CRC is a CRC16-CCITT. The starting value is 0xFFFF. The CRC covers only the packet data.
    bool check_crc( const char* data ) const;   // length is from header
    header();
    header( unsigned char id, unsigned char length, const char* data );
    void reset( unsigned char id, unsigned char length, const char* data );
    unsigned int len() const { return (unsigned int)( length() ); }
};

struct acknowledgement : public comma::packed::packed_struct< acknowledgement, 4 >
{
    enum { id = 0 };
    comma::packed::uint8 packet_id;
    comma::packed::little_endian::uint16 crc;
    comma::packed::uint8 result;

    const char* result_msg() const;
};

struct system_state : public comma::packed::packed_struct< system_state, 100 >
{
    enum { id = 20 };
    comma::packed::little_endian::uint16 system_status;
    comma::packed::little_endian::uint16 filter_status;
    comma::packed::little_endian::uint32 unix_time_seconds;
    comma::packed::little_endian::uint32 microseconds;
    comma::packed::little_endian::float64 latitude;  //rad
    comma::packed::little_endian::float64 longitude; //rad
    comma::packed::little_endian::float64 height;    //m
    boost::array< comma::packed::little_endian::float32, 3 > velocity;  // north, east, down m/s
    boost::array< comma::packed::little_endian::float32, 3 > body_acceleration; //x,y,z m/s/s
    comma::packed::little_endian::float32 g_force;   //g
    boost::array< comma::packed::little_endian::float32, 3 > orientation;   //roll,pitch,heading radians
    boost::array< comma::packed::little_endian::float32, 3 > angular_velocity;  //x,y,z rad/s
    boost::array< comma::packed::little_endian::float32, 3 > position_stddev;    //latitude,longitude,height m

    boost::posix_time::ptime t() const;
    snark::spherical::coordinates coordinates() const;
};

struct system_status_description
{
    system_status_description( uint16_t status = 0 );

    static std::string string( uint16_t status) ;
    static void description( std::ostream& os );

    unsigned int system_failure() const;
    unsigned int accelerometer_sensor_failure() const;
    unsigned int gyroscope_sensor_failure() const;
    unsigned int magnetometer_sensor_failure() const;
    unsigned int pressure_sensor_failure() const;
    unsigned int gnss_failure() const;
    unsigned int accelerometer_over_range() const;
    unsigned int gyroscope_over_range() const;
    unsigned int magnetometer_over_range() const;
    unsigned int pressure_over_range() const;
    unsigned int minimum_temperature_alarm() const;
    unsigned int maximum_temperature_alarm() const;
    unsigned int low_voltage_alarm() const;
    unsigned int high_voltage_alarm() const;
    unsigned int gnss_antenna_short_circuit() const;
    unsigned int data_output_overflow_alarm() const;

    static const std::vector< std::string > text;
    uint16_t status;
};

struct filter_status_description
{
    filter_status_description( uint16_t status = 0 );

    static std::string string( uint16_t status );
    static std::string full_description( uint16_t status );
    static void description( std::ostream& os );
    static void gnss_fix_description( std::ostream& os );

    unsigned int gnss_fix() const;
    unsigned int orientation_filter_initialised() const;
    unsigned int navigation_filter_initialised() const;
    unsigned int heading_initialised() const;
    unsigned int utc_time_initialised() const;
    unsigned int event_1_occurred() const;
    unsigned int event_2_occurred() const;
    unsigned int internal_gnss_enabled() const;
    unsigned int dual_antenna_heading_active() const;
    unsigned int velocity_heading_enabled() const;
    unsigned int atmospheric_altitude_enabled() const;
    unsigned int external_position_active() const;
    unsigned int external_velocity_active() const;
    unsigned int external_heading_active() const;

    static const std::vector< std::string > text;
    static const std::vector< std::string > gnss_fix_text;
    uint16_t status;
};

struct position_standard_deviation : public comma::packed::packed_struct< position_standard_deviation, 12 >
{
    enum { id = 24 };
    boost::array< comma::packed::little_endian::float32, 3 > stddev;
};

struct velocity_standard_deviation : public comma::packed::packed_struct< velocity_standard_deviation, 12 >
{
    enum { id = 25 };
    boost::array< comma::packed::little_endian::float32, 3 > stddev;
};

//euler_orientation_standard_deviation_packet_t
struct orientation_standard_deviation : public comma::packed::packed_struct< velocity_standard_deviation, 12 >
{
    enum { id = 26 };
    boost::array< comma::packed::little_endian::float32, 3 > stddev;
};

struct raw_sensors : public comma::packed::packed_struct< raw_sensors, 48 >
{
    enum { id = 28 };
    boost::array< comma::packed::little_endian::float32, 3 > accelerometer; //x,y,z m/s/s
    boost::array< comma::packed::little_endian::float32, 3 > gyroscope; //x,y,z rad/s
    boost::array< comma::packed::little_endian::float32, 3 > magnetometer;  //x,y,z mG
    comma::packed::little_endian::float32 imu_temperature;   //deg C
    comma::packed::little_endian::float32 pressure;  //Pascals
    comma::packed::little_endian::float32 pressure_temperature;  //deg C
};

struct satellites : public comma::packed::packed_struct< satellites, 13 >
{
    enum { id = 30 };
    comma::packed::little_endian::float32 hdop;
    comma::packed::little_endian::float32 vdop;
    comma::packed::uint8 gps_satellites;
    comma::packed::uint8 glonass_satellites;
    comma::packed::uint8 beidou_satellites;
    comma::packed::uint8 galileo_satellites;
    comma::packed::uint8 sbas_satellites;
};

struct rtcm_corrections : public comma::packed::packed_struct< rtcm_corrections, 260 >
{
    enum { id = 55 };
    messages::header header;
    boost::array< comma::packed::uint8, 255 > msg_data;

    rtcm_corrections() {}
    rtcm_corrections( const char* buf, unsigned int size );
};

struct command : public comma::packed::packed_struct< command, 260 >
{
    messages::header header;
    boost::array< comma::packed::uint8, 255 > msg_data;

    command() {}
    command( uint8_t id, const char* buf, unsigned int size );
};

struct magnetic_calibration_configuration : public comma::packed::packed_struct< magnetic_calibration_configuration, 1 >
{
    enum { id = 190 };
    comma::packed::uint8 action;

    command get_command() const;
};

struct magnetic_calibration_status : public comma::packed::packed_struct< magnetic_calibration_status, 3 >
{
    enum { id = 191 };
    comma::packed::uint8 status;
    comma::packed::uint8 progress;
    comma::packed::uint8 error;

    static void status_description( std::ostream& os );
};

} //namespace messages {

} } } //namespace snark { namespace navigation { namespace advanced_navigation {
