// Copyright (c) 2017 The University of Sydney
// Copyright (c) 2021 Mission Systems Pty Ltd

#include "messages.h"
#include <comma/application/verbose.h>
#include "boost/date_time/posix_time/posix_time.hpp"

namespace snark { namespace navigation { namespace advanced_navigation {

namespace messages {

namespace detail {
static const uint16_t crc16_table[256] =
{
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

uint16_t calculate_crc( const uint8_t *bytes, unsigned int length )
{
    uint16_t crc = 0xFFFF, i;
    for( i = 0; i < length; i++ )
    {
        crc = (uint16_t)((crc << 8) ^ crc16_table[ uint8_t( (crc >> 8) ^ bytes[i] ) ] );
    }
    return crc;
}

uint8_t calculate_LRC( const uint8_t* buf )
{
//     comma::verbose<<unsigned(buf[1] + buf[2] + buf[3] + buf[4])<<" "<<unsigned((buf[1] + buf[2] + buf[3] + buf[4]) ^ 0xFF)<<" "<<unsigned(buf[0])<<std::endl;
    return uint8_t((( buf[1] + buf[2] + buf[3] + buf[4] ) ^ 0xFF ) + 1 );
}

} //namespace detail {

// ------
// header
// ------
bool header::is_valid() const
{
    uint8_t* buf = (uint8_t*)data();
    bool res = ( buf[0] == detail::calculate_LRC( buf ) );
    if( res && length() == 0 )
    {
//         std::cerr<<"message length "<<unsigned(length())<<std::endl;
        return false;
    }
//     if(!res)
//         comma::verbose<<short(buf[0])<<" "<<short(buf[1])<<" "<<short(buf[2])<<" "<<short(buf[3])<<" "<<short(buf[4])<<std::endl;
    return res;
}
bool header::check_crc( const char* data ) const
{
    return detail::calculate_crc( (const uint8_t *)data, (unsigned int)( length() )) == msg_crc();
}

header::header() { LRC = 1; id = 255; length = 0; msg_crc = 0; }   // invalid header

header::header( unsigned char i, unsigned char l, const char* buf )
{
    reset( i, l, buf );
}

void header::reset( unsigned char i, unsigned char l, const char* buf )
{
    id = i;
    length = l;
    msg_crc = detail::calculate_crc( (const uint8_t*)buf, length() );
    LRC = detail::calculate_LRC( (uint8_t*)data() );
}

// -------
// command
// -------
command::command( uint8_t id, const char* buf, unsigned int size )
    : header( id, size, buf )
{
    std::memcpy( &msg_data[0], buf, size );
}

// ---------------
// acknowledgement
// ---------------
const char* acknowledgement::result_msg() const
{
    switch( result() )
    {
        case 0:
            return "Acknowledge success";
        case 1:
            return "Acknowledge failure, CRC error";
        case 2:
            return "Acknowledge failure, packet size incorrect";
        case 3:
            return "Acknowledge failure, values outside of valid ranges";
        case 4:
            return "Acknowledge failure, system flash memory failure";
        case 5:
            return "Acknowledge failure, system not ready";
        case 6:
            return "Acknowledge failure, unknown packet";
        default:
            { COMMA_THROW( comma::exception, "Acknowledge result invalid value (result msg)"); }
    }
}

// ------------
// system_state
// ------------
boost::posix_time::ptime system_state::t() const
{
    return boost::posix_time::ptime( boost::gregorian::date( 1970, 1, 1 )
                                   , boost::posix_time::seconds( unix_time_seconds() ) + boost::posix_time::microseconds( microseconds() ));
}

snark::spherical::coordinates system_state::coordinates() const
{
    return snark::spherical::coordinates( latitude(), longitude() );
}

// -------------------------
// system_status_description
// -------------------------
const std::vector< std::string > system_status_description::text({
    "System Failure",
    "Accelerometer Sensor Failure",
    "Gyroscope Sensor Failure",
    "Magnetometer Sensor Failure",
    "Pressure Sensor Failure",
    "GNSS Failure",
    "Accelerometer Over Range",
    "Gyroscope Over Range",
    "Magnetometer Over Range",
    "Pressure Over Range",
    "Minimum Temperature Alarm",
    "Maximum Temperature Alarm",
    "Low Voltage Alarm",
    "High Voltage Alarm",
    "GNSS Antenna Short Circuit",
    "Data Output Overflow Alarm"
});

std::string system_status_description::string( uint16_t status )
{
    if( !status ) { return "null"; }
    std::stringstream ss;
    unsigned int bit = 1;
    for( unsigned int i = 0; i < text.size(); ++i )
    {
        if( status & bit ) { ss << i << ": " << text[i] << "; "; }
        bit <<= 1;
    }
    return ss.str();
}
void system_status_description::description( std::ostream& os )
{
    for( unsigned int i = 0; i < text.size(); ++i )
    {
        os << i << "," << "\"" << text[i] << "\"" << std::endl;
    }
}

system_status_description::system_status_description( uint16_t status ) : status( status ) {}
unsigned int system_status_description::system_failure() const { return (status & 1) ? 1 : 0; }
unsigned int system_status_description::accelerometer_sensor_failure() const { return ( status & 2 ) ? 1 : 0; }
unsigned int system_status_description::gyroscope_sensor_failure() const { return ( status & 4 ) ? 1 : 0; }
unsigned int system_status_description::magnetometer_sensor_failure() const { return ( status & 8 ) ? 1 : 0; }
unsigned int system_status_description::pressure_sensor_failure() const { return ( status & 0x10 ) ? 1 : 0; }
unsigned int system_status_description::gnss_failure() const { return ( status & 0x20 ) ? 1 : 0; }
unsigned int system_status_description::accelerometer_over_range() const { return ( status & 0x40 ) ? 1 : 0; }
unsigned int system_status_description::gyroscope_over_range() const { return ( status & 0x80 ) ? 1 : 0; }
unsigned int system_status_description::magnetometer_over_range() const { return ( status & 0x100 ) ? 1 : 0; }
unsigned int system_status_description::pressure_over_range() const { return ( status & 0x200 ) ? 1 : 0; }
unsigned int system_status_description::minimum_temperature_alarm() const { return ( status & 0x400 ) ? 1 : 0; }
unsigned int system_status_description::maximum_temperature_alarm() const { return ( status & 0x800 ) ? 1 : 0; }
unsigned int system_status_description::low_voltage_alarm() const { return ( status & 0x1000 ) ? 1 : 0; }
unsigned int system_status_description::high_voltage_alarm() const { return ( status & 0x2000 ) ? 1 : 0; }
unsigned int system_status_description::gnss_antenna_short_circuit() const { return ( status & 0x4000 ) ? 1 : 0; }
unsigned int system_status_description::data_output_overflow_alarm() const { return ( status & 0x8000 ) ? 1 : 0; }

// -------------------------
// filter_status_description
// -------------------------
const std::vector< std::string > filter_status_description::text({
    "Orientation Filter Initialised",
    "Navigation Filter Initialised",
    "Heading Initialised",
    "UTC Time Initialised",
    "",
    "",
    "",
    "Event 1 Occurred",
    "Event 2 Occurred",
    "Internal GNSS Enabled",
    "Dual Antenna Heading Active",
    "Velocity Heading Enabled",
    "Atmospheric Altitude Enabled",
    "External Position Active",
    "External Velocity Active",
    "External Heading Active"
});

const std::vector< std::string > filter_status_description::gnss_fix_text({
    "No GNSS fix",
    "2D GNSS fix",
    "3D GNSS fix",
    "SBAS GNSS fix",
    "Differential GNSS fix",
    "Omnistar/Starfire GNSS fix",
    "RTK Float GNSS fix",
    "RTK Fixed GNSS fix"
});

std::string filter_status_description::full_description( uint16_t status )
{
    std::stringstream ss;
    unsigned int index = ( status >> 4 ) & 7;
    ss << "GNSS fix " << index << ": " << gnss_fix_text[index] << ";";
    unsigned int bit = 1;
    for( unsigned int i = 0; i < text.size(); ++i )
    {
        if( !text[i].empty() )
        {
            //split last word
            std::string::size_type index = text[i].find_last_of( ' ' );
            //prolog
            ss << i << "," << text[i].substr( 0, index );
            if( !(status & bit) )
                ss << " NOT";
            //epilog
            ss << text[i].substr(index) << "," << (( status & bit ) ? 1 : 0 ) << ";";
        }
        bit <<= 1;
    }
    return ss.str();
}

std::string filter_status_description::string( uint16_t status )
{
    std::stringstream ss;
    unsigned int index = ( status >> 4 ) & 7;
    ss << "GNSS fix " << index << ": " << gnss_fix_text[index] << "; ";
    unsigned int bit = 1;
    for( unsigned int i = 0; i < text.size(); ++i )
    {
        if(( status & bit ) && !text[i].empty() ) { ss << i << ": " << text[i] << "; "; }
        bit <<= 1;
    }
    return ss.str();
}

void filter_status_description::description( std::ostream& os )
{
    for( unsigned int i = 0; i < text.size(); ++i )
    {
        if( !text[i].empty() )
            os << i << "," << "\"" << text[i] << "\"" << std::endl;
    }
}
void filter_status_description::gnss_fix_description( std::ostream& os )
{
    for( unsigned int i = 0; i < gnss_fix_text.size(); ++i )
    {
        os << i << "," << "\"" << gnss_fix_text[i] << "\"" << std::endl;
    }
}

filter_status_description::filter_status_description( uint16_t status ) : status( status ) {}

unsigned int filter_status_description::gnss_fix() const { return ( status >> 4 ) & 7; }
unsigned int filter_status_description::orientation_filter_initialised() const { return ( status & 0x1 ) ? 1 : 0; }
unsigned int filter_status_description::navigation_filter_initialised() const { return ( status & 0x2 ) ? 1 : 0; }
unsigned int filter_status_description::heading_initialised() const { return ( status & 0x4 ) ? 1 : 0; }
unsigned int filter_status_description::utc_time_initialised() const { return ( status & 0x8 ) ? 1 : 0; }
unsigned int filter_status_description::event_1_occurred() const { return ( status & 0x80 ) ? 1 : 0; }
unsigned int filter_status_description::event_2_occurred() const { return ( status & 0x100 ) ? 1 : 0; }
unsigned int filter_status_description::internal_gnss_enabled() const { return ( status & 0x200 ) ? 1 : 0; }
unsigned int filter_status_description::dual_antenna_heading_active() const { return ( status & 0x400 ) ? 1 : 0; }
unsigned int filter_status_description::velocity_heading_enabled() const { return ( status & 0x800 ) ? 1 : 0; }
unsigned int filter_status_description::atmospheric_altitude_enabled() const { return ( status & 0x1000 ) ? 1 : 0; }
unsigned int filter_status_description::external_position_active() const { return ( status & 0x2000 ) ? 1 : 0; }
unsigned int filter_status_description::external_velocity_active() const { return ( status & 0x4000 ) ? 1 : 0; }
unsigned int filter_status_description::external_heading_active() const { return ( status & 0x8000 ) ? 1 : 0; }

// ----------------
// rtcm_corrections
// ----------------
rtcm_corrections::rtcm_corrections( const char* buf, unsigned int size )
    : header( id, size, buf)
{
    std::memcpy( &msg_data[0], buf,size );
}

// ---------------------------
// magnetic_calibration_status
// ---------------------------
void magnetic_calibration_status::status_description( std::ostream& os )
{
    os << "0,Magnetic calibration not completed" << std::endl;
    os << "1,2D magnetic calibration completed" << std::endl;
    os << "2,3D magnetic calibration completed" << std::endl;
    os << "3,Custom values magnetic calibration completed" << std::endl;
    os << "5,2D calibration in progress" << std::endl;
    os << "6,3D calibration in progress" << std::endl;
    os << "7,2D calibration error: excessive roll" << std::endl;
    os << "8,2D calibration error: excessive pitch" << std::endl;
    os << "9,Calibration error: sensor over range event" << std::endl;
    os << "10,Calibration error: time-out" << std::endl;
    os << "11,Calibration error: system error" << std::endl;
    os << "12,Calibration error: interference error" << std::endl;
}

} //namespace messages {

} } } //namespace snark { namespace navigation { namespace advanced_navigation {
