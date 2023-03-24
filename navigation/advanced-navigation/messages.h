// Copyright (c) 2017 The University of Sydney
// Copyright (c) 2021,2022 Mission Systems Pty Ltd

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

struct command : public comma::packed::packed_struct< command, 260 >
{
    messages::header header;
    boost::array< comma::packed::uint8, 255 > msg_data;

    command() {}
    command( uint8_t id, const char* buf, unsigned int size );
};

// each packet contains the following:
//   id - packet id
//   data - which is assembled in the packed struct
//
// packets that can be sent also have the setting
//   expects_response - do we expect a packet in response?

struct acknowledgement : public comma::packed::packed_struct< acknowledgement, 4 >
{
    enum { id = 0 };
    comma::packed::uint8 packet_id;
    comma::packed::little_endian::uint16 crc;
    comma::packed::uint8 result;

    const char* result_msg() const;
};

// only supports single packet requests, the underlying message supports multiple packets
struct request : public comma::packed::packed_struct< request, 1 >
{
    enum { id = 1 };
    enum : bool { expects_response = true };

    comma::packed::uint8 packet_id;

    command get_command() const { return command( id, data(), size ); }
};

struct reset : public comma::packed::packed_struct< reset, 4 >
{
    enum { id = 5 };
    enum : bool { expects_response = true };

    // 0x21057A7E for hot start reset (equivalent to power cycle), 0x9A5D38B7 for cold start reset
    comma::packed::little_endian::uint32 verification_sequence;

    command get_command() const { return command( id, data(), size ); }
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

struct unix_time : public comma::packed::packed_struct< unix_time, 8 >
{
    enum { id = 21 };
    comma::packed::little_endian::uint32 unix_time_seconds;
    comma::packed::little_endian::uint32 microseconds;

    boost::posix_time::ptime t() const;
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

struct orientation_standard_deviation : public comma::packed::packed_struct< orientation_standard_deviation, 12 >
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

struct acceleration : public comma::packed::packed_struct< acceleration, 12 >
{
    enum { id = 37 };
    comma::packed::little_endian::float32 x;
    comma::packed::little_endian::float32 y;
    comma::packed::little_endian::float32 z;
};

struct euler_orientation : public comma::packed::packed_struct< euler_orientation, 12 >
{
    enum { id = 39 };
    comma::packed::little_endian::float32 roll;
    comma::packed::little_endian::float32 pitch;
    comma::packed::little_endian::float32 heading;
};

struct angular_velocity : public comma::packed::packed_struct< angular_velocity, 12 >
{
    enum { id = 37 };
    comma::packed::little_endian::float32 x;
    comma::packed::little_endian::float32 y;
    comma::packed::little_endian::float32 z;
};

struct external_time : public comma::packed::packed_struct< external_time, 8 >
{
    enum { id = 52 };
    enum : bool { expects_response = false };

    comma::packed::little_endian::uint32 unix_time_seconds;
    comma::packed::little_endian::uint32 microseconds;

    boost::posix_time::ptime t() const;

    command get_command() const { return command( id, data(), size ); }
};

struct rtcm_corrections : public comma::packed::packed_struct< rtcm_corrections, 260 >
{
    enum { id = 55 };
    enum : bool { expects_response = true };

    messages::header header;
    boost::array< comma::packed::uint8, 255 > msg_data;

    rtcm_corrections() {}
    rtcm_corrections( const char* buf, unsigned int size );
};

struct filter_options : public comma::packed::packed_struct< filter_options, 17 >
{
    enum { id = 186 };
    enum : bool { expects_response = true };

    comma::packed::uint8 permanent;
    comma::packed::uint8 vehicle_types;
    comma::packed::uint8 internal_gnss_enabled;
    comma::packed::uint8 magnetic_heading_enabled;
    comma::packed::uint8 atmospheric_altitude_enabled;
    comma::packed::uint8 velocity_heading_enabled;
    comma::packed::uint8 reversing_detection_enabled;
    comma::packed::uint8 motion_analysis_enabled;
    comma::packed::uint8 automatic_magnetic_calibration_enabled;
    comma::packed::little_endian::uint64 reserved;

    command get_command() const { return command( id, data(), size ); }
};

struct magnetic_calibration_configuration : public comma::packed::packed_struct< magnetic_calibration_configuration, 1 >
{
    enum { id = 190 };
    enum : bool { expects_response = true };

    comma::packed::uint8 action;

    command get_command() const { return command( id, data(), size ); }
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
