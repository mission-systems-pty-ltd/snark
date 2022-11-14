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

template < typename K, typename T, typename V > inline void visit_as_bytes( const K& k, T& t, V& v ) // quick and dirty
{
    std::array< char, size< T >::value > a;
    v.apply( k, a );
    std::memcpy( t.data(), &a[0], size< T >::value );
}

} // namespace detail {

template <> struct traits< snark::robosense::difop::header >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::robosense::difop::header& p, Visitor& v ) { detail::visit_as_bytes( "sentinel", p.sentinel, v ); }
    template < typename Key, class Visitor > static void visit( const Key&, snark::robosense::difop::header& p, Visitor& v ) { detail::visit_as_bytes( "sentinel", p.sentinel, v ); }
};

template <> struct traits< snark::robosense::difop::tail >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::robosense::difop::tail& p, Visitor& v ) { detail::visit_as_bytes( "sentinel", p.sentinel, v ); }
    template < typename Key, class Visitor > static void visit( const Key&, snark::robosense::difop::tail& p, Visitor& v ) { detail::visit_as_bytes( "sentinel", p.sentinel, v ); }
};

template <> struct traits< snark::robosense::helios_16p::difop::data::fov_setting_t >
{
    template < typename Key, class Visitor > static void visit( const Key& k, const snark::robosense::helios_16p::difop::data::fov_setting_t& p, Visitor& v )
    {
        v.apply( "start", p.start.as_degrees() );
        v.apply( "end", p.end.as_degrees() );
    }
};

template <> struct traits< snark::robosense::helios_16p::difop::data >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::robosense::helios_16p::difop::data& p, Visitor& v )
    {
        v.apply( "motor_rotation_speed", p.motor_rotation_speed() );
        v.apply( "fov_setting", p.fov_setting );
        v.apply( "motor_phase_lock", p.motor_phase_lock() );
        detail::visit_as_bytes( "top_board_firmware_version", p.top_board_firmware_version, v );
        detail::visit_as_bytes( "bottom_board_firmware_version", p.bottom_board_firmware_version, v );
        detail::visit_as_bytes( "bottom_board_software_version", p.bottom_board_software_version, v );
        detail::visit_as_bytes( "motor_firmware_version", p.motor_firmware_version, v );
        v.apply( "zero_angle_offset", p.zero_angle_offset.as_degrees() );
        v.apply( "gprmc", std::string( p.gprmc.data() ) + "\0" );
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
