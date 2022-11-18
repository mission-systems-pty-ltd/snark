// Copyright (c) 2018-2022 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <array>
#include <memory>
#include <comma/visiting/traits.h>
#include "packet.h"

namespace comma { namespace visiting { // todo: move to cpp file

// in progress...

namespace detail {

template < typename T > struct size { static constexpr unsigned int value{T::size}; };
template < typename T, long unsigned int Size > struct size< std::array< T, Size > > { static constexpr unsigned int value{Size}; };

template < typename K, typename T, typename V > inline void visit_as_bytes( const K& k, const T& t, V& v ) // quick and dirty
{
    std::array< unsigned int, size< T >::value > a;
    for( unsigned int i = 0; i < size< T >::value; ++i ) { a[i] = int( t.data()[i] ) & 0xff; }
    v.apply( k, a );
}

inline char _hex( unsigned char v ) { return v > 9 ? ( 'a' + v - 10 ) : ( '0' + v ); }

template < typename K, typename T, typename V > inline void visit_as_hexadecimal( const K& k, const T& t, V& v ) // uber-quick and dirty
{
    std::string a, space{};
    //for( unsigned int i = 0; i < size< T >::value; ++i ) { a[i] = std::string( "0x" ) + _hex( ( unsigned char )( t.data()[i] ) >> 4 ) + _hex( ( unsigned char )( t.data()[i] ) & 0x0f ); }
    for( unsigned int i = 0; i < size< T >::value; ++i ) { a += space + std::string( 1, _hex( ( unsigned char )( t.data()[i] ) >> 4 ) ) + std::string( 1, _hex( ( unsigned char )( t.data()[i] ) & 0x0f ) ); space = " "; }
    v.apply( k, a );
}

template < typename K, typename T, typename V > inline void visit_as_bytes( const K& k, T& t, V& v ) // quick and dirty
{
    std::array< char, size< T >::value > a;
    v.apply( k, a );
    std::memcpy( t.data(), &a[0], size< T >::value );
}

template < typename K, typename T, typename V > inline void visit_as_byte_path( const K& k, T& t, V& v, char delimiter = '.' ) // quick and dirty
{
    std::string a, dot{};
    for( unsigned int i = 0; i < size< T >::value; ++i ) { a += dot + boost::lexical_cast< std::string >( int( t.data()[i] ) & 0xff ); dot = delimiter; }
    v.apply( k, a );
}

} // namespace detail {

template <> struct traits< snark::robosense::difop::header >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::robosense::difop::header& p, Visitor& v ) { detail::visit_as_hexadecimal( "sentinel", p.sentinel, v ); }
    //template < typename Key, class Visitor > static void visit( const Key&, snark::robosense::difop::header& p, Visitor& v ) { detail::visit_as_bytes( "sentinel", p.sentinel, v ); }
};

template <> struct traits< snark::robosense::difop::tail >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::robosense::difop::tail& p, Visitor& v ) { detail::visit_as_hexadecimal( "sentinel", p.sentinel, v ); }
    //template < typename Key, class Visitor > static void visit( const Key&, snark::robosense::difop::tail& p, Visitor& v ) { detail::visit_as_bytes( "sentinel", p.sentinel, v ); }
};

template <> struct traits< snark::robosense::helios_16p::difop::data::fov_setting_t >
{
    template < typename Key, class Visitor > static void visit( const Key& k, const snark::robosense::helios_16p::difop::data::fov_setting_t& p, Visitor& v )
    {
        v.apply( "start", p.start.as_degrees() );
        v.apply( "end", p.end.as_degrees() );
    }
};

template <> struct traits< snark::robosense::helios_16p::difop::data::ethernet_t >
{
    template < typename Key, class Visitor > static void visit( const Key& k, const snark::robosense::helios_16p::difop::data::ethernet_t& p, Visitor& v )
    {
        detail::visit_as_byte_path( "lidar_ip", p.lidar_ip, v );
        detail::visit_as_byte_path( "dest_pc_ip", p.dest_pc_ip, v );
        detail::visit_as_hexadecimal( "mac_addr", p.mac_addr, v );
        v.apply( "port1", p.port1() );
        v.apply( "port2", p.port2() );
        v.apply( "port3", p.port3() );
        v.apply( "port4", p.port4() );
    }
};

template <> struct traits< snark::robosense::utc_time >
{
    template < typename Key, class Visitor > static void visit( const Key& k, const snark::robosense::utc_time& p, Visitor& v )
    {
        v.apply( "seconds", p.seconds() );
        v.apply( "nanoseconds", p.nanoseconds() );
    }
};

template <> struct traits< snark::robosense::helios_16p::difop::data::operating_status_t >
{
    template < typename Key, class Visitor > static void visit( const Key& k, const snark::robosense::helios_16p::difop::data::operating_status_t& p, Visitor& v )
    {
        detail::visit_as_hexadecimal( "ldat1_reg", p.ldat1_reg, v );
        detail::visit_as_hexadecimal( "vdat", p.vdat, v );
        detail::visit_as_hexadecimal( "vdat_12v_reg", p.vdat_12v_reg, v );
        detail::visit_as_hexadecimal( "vdat_5v_reg", p.vdat_5v_reg, v );
        detail::visit_as_hexadecimal( "vdat_2v5_reg", p.vdat_2v5_reg, v );
        detail::visit_as_hexadecimal( "vdat_apd", p.vdat_apd, v );
    }
};

template <> struct traits< snark::robosense::helios_16p::difop::data::fault_diagnosis_t::gps_status_t >
{
    template < typename Key, class Visitor > static void visit( const Key& k, const snark::robosense::helios_16p::difop::data::fault_diagnosis_t::gps_status_t& p, Visitor& v )
    {
        v.apply( "pps_lock", p.pps_lock );
        v.apply( "fprmc_lock", p.fprmc_lock );
        v.apply( "utc_lock", p.utc_lock );
        v.apply( "gprmc_input_status", p.gprmc_input_status );
        v.apply( "pps_input_status", p.pps_input_status );
    }
};

template <> struct traits< snark::robosense::helios_16p::difop::data::fault_diagnosis_t >
{
    template < typename Key, class Visitor > static void visit( const Key& k, const snark::robosense::helios_16p::difop::data::fault_diagnosis_t& p, Visitor& v )
    {
        v.apply( "temperature1", p.temperature1.as_celcius() );
        v.apply( "temperature2", p.temperature2.as_celcius() );
        v.apply( "temperature3", p.temperature3.as_celcius() );
        v.apply( "temperature4", p.temperature4.as_celcius() );
        v.apply( "temperature5", p.temperature5.as_celcius() );
        v.apply( "r_rpm", p.r_rpm() );
        v.apply( "lane_up", int( p.lane_up() ) && 0xff );
        v.apply( "lane_up_cnt", p.lane_up_cnt() );
        detail::visit_as_hexadecimal( "top_status", p.top_status, v );
        v.apply( "gps_status", p.gps_status() );
    }
};

template <> struct traits< snark::robosense::helios_16p::difop::data >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::robosense::helios_16p::difop::data& p, Visitor& v )
    {
        v.apply( "motor_rotation_speed", p.motor_rotation_speed() );
        v.apply( "ethernet", p.ethernet );
        v.apply( "fov_setting", p.fov_setting );
        v.apply( "motor_phase_lock", p.motor_phase_lock() );
        detail::visit_as_hexadecimal( "top_board_firmware_version", p.top_board_firmware_version, v );
        detail::visit_as_hexadecimal( "bottom_board_firmware_version", p.bottom_board_firmware_version, v );
        detail::visit_as_hexadecimal( "bottom_board_software_version", p.bottom_board_software_version, v );
        detail::visit_as_hexadecimal( "motor_firmware_version", p.motor_firmware_version, v );
        detail::visit_as_hexadecimal( "sensor_hardware_version", p.sensor_hardware_version, v );
        detail::visit_as_hexadecimal( "web_page_cgi_verion", p.web_page_cgi_verion, v );
        v.apply( "top_board_backup_crc", p.top_board_backup_crc() );
        v.apply( "bottom_board_backup_crc", p.bottom_board_backup_crc() );
        v.apply( "software_app_backup_crc", p.software_app_backup_crc() );
        v.apply( "web_page_cgi_backup_crc", p.web_page_cgi_backup_crc() );
        detail::visit_as_byte_path( "ethernet_gateway", p.ethernet_gateway, v );
        detail::visit_as_byte_path( "subnet_mask", p.subnet_mask, v );
        detail::visit_as_hexadecimal( "serial_number", p.serial_number, v );
        v.apply( "zero_angle_offset", p.zero_angle_offset.as_degrees() );
        v.apply( "return_mode", p.return_mode.name() );
        v.apply( "time_synchronization_mode", int( p.time_synchronization_mode() ) & 0xff );
        v.apply( "synchronization_status", int( p.synchronization_status() ) & 0xff );
        v.apply( "operating_status", p.operating_status );
        v.apply( "fault_diagnosis", p.fault_diagnosis );
        v.apply( "code_wheel_is_calibrated", p.code_wheel_is_calibrated() );
        v.apply( "gps_pps_pulse_trigger_mode", p.gps_pps_pulse_trigger_mode() );
        v.apply( "time", p.time );
        v.apply( "gprmc", std::string( p.gprmc.data() ) + "\0" );
        v.apply( "corrected_vertical_angles", p.corrected_vertical_angles.as_degrees() );
        v.apply( "corrected_horizontal_angles", p.corrected_horizontal_angles.as_degrees() );
    }
};

template <> struct traits< snark::robosense::helios_16p::difop::packet >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::robosense::helios_16p::difop::packet& p, Visitor& v )
    {
        v.apply( "header", p.header );
        v.apply( "data", p.data );
        v.apply( "tail", p.tail );
    }

    template < typename Key, class Visitor > static void visit( const Key&, snark::robosense::helios_16p::difop::packet& p, Visitor& v )
    {
        v.apply( "header", p.header );
        v.apply( "data", p.data );
        v.apply( "tail", p.tail );
    }
};

} } // namespace comma { namespace visiting {
