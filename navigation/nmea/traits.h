// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// Copyright (c) 2024 Vsevolod Vlaskine
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/// @authors David Nah, vsevolod vlaskine

#pragma once

#include <cmath>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/visiting/apply.h>
#include <comma/visiting/traits.h>
#include "messages.h"

namespace comma { namespace visiting {

template <> struct traits< snark::nmea::messages::coordinate >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::nmea::messages::coordinate& p, Visitor& v ) // hyper-quick and dirty
    {
        double c{0};
        v.apply( "value", c );
        int degrees = c / 100;
        double fractions = ( c - degrees * 100 ) / 60;
        p.value = ( degrees + fractions ) * M_PI / 180;
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::nmea::messages::coordinate& p, Visitor& v ) // hyper-quick and dirty
    {
        double value = std::abs( p.value );
        int degrees = value;
        double fractions = ( value - degrees ) * 60.0;
        v.apply( "value", degrees * 100 + fractions );
    }
};

template <> struct traits< snark::nmea::messages::latitude_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key& k, snark::nmea::messages::latitude_t& p, Visitor& v ) // hyper-quick and dirty
    {
        comma::visiting::traits< snark::nmea::messages::coordinate >::visit( k, static_cast< snark::nmea::messages::coordinate& >( p ), v );
        std::string hemisphere;
        v.apply( "hemisphere", hemisphere );
        p.value *= hemisphere == "N" ? 1 : -1;
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key& k, const snark::nmea::messages::latitude_t& p, Visitor& v ) // hyper-quick and dirty
    {
        comma::visiting::traits< snark::nmea::messages::coordinate >::visit( k, static_cast< const snark::nmea::messages::coordinate& >( p ), v );
        v.apply( "hemisphere", std::string( p.value < 0 ? "S" : "N" ) );
    }
};

template <> struct traits< snark::nmea::messages::longitude_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key& k, snark::nmea::messages::longitude_t& p, Visitor& v ) // hyper-quick and dirty
    {
        comma::visiting::traits< snark::nmea::messages::coordinate >::visit( k, static_cast< snark::nmea::messages::coordinate& >( p ), v );
        std::string hemisphere;
        v.apply( "hemisphere", hemisphere );
        p.value *= hemisphere == "E" ? 1 : -1;
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key& k, const snark::nmea::messages::longitude_t& p, Visitor& v ) // hyper-quick and dirty
    {
        comma::visiting::traits< snark::nmea::messages::coordinate >::visit( k, static_cast< const snark::nmea::messages::coordinate& >( p ), v );
        v.apply( "hemisphere", std::string( p.value < 0 ? "W" : "E" ) );
    }
};

template <> struct traits< snark::nmea::messages::coordinates >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::nmea::messages::coordinates& p, Visitor& v ) // hyper-quick and dirty
    {
        v.apply( "latitude", p.latitude );
        v.apply( "longitude", p.longitude );
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::nmea::messages::coordinates& p, Visitor& v ) // hyper-quick and dirty
    {
        v.apply( "latitude", p.latitude );
        v.apply( "longitude", p.longitude );
    }
};

template <> struct traits< snark::nmea::messages::time > // pain
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::nmea::messages::time& p, Visitor& v ) // hyper-quick and monster-dirty
    {
        std::string t;
        v.apply( "time_of_day", t );
        if( !t.empty() ) { p.value = boost::posix_time::ptime( boost::posix_time::microsec_clock::universal_time().date(), boost::posix_time::from_iso_string( "19700101T" + t ).time_of_day() ); }
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::nmea::messages::time& p, Visitor& v ) // hyper-quick and monster-dirty
    {
        v.apply( "time_of_day", p.value.is_not_a_date_time() ? std::string() : boost::posix_time::to_iso_string( p.value ).substr( 9, 6 ) );
    }
};

template <> struct traits< snark::nmea::messages::date > // pain
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::nmea::messages::date& p, Visitor& v ) // hyper-quick and monster-dirty
    {
        std::string t;
        v.apply( "date", t );
        if( !t.empty() )
        {
            int date_int = boost::lexical_cast< int >( t );
            int year = 2000 + date_int % 100;
            int month = ( date_int / 100 ) % 100;
            int day = date_int / 10000;
            p.value = boost::gregorian::date( year, month, day );
        }
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::nmea::messages::date& p, Visitor& v ) // hyper-quick and monster-dirty
    {
        std::string t; // todo! set from p
        v.apply( "date", t );
    }
};


template <> struct traits< snark::nmea::message >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::nmea::message& p, Visitor& v ) { v.apply( "id", p.id ); }
    template < typename Key, class Visitor > static void visit( const Key&, const snark::nmea::message& p, Visitor& v ) { v.apply( "id", p.id ); }
};

template <> struct traits< snark::nmea::messages::gga >
{
    template < typename Key, class Visitor >
    static void visit( const Key& k, snark::nmea::messages::gga& p, Visitor& v ) // hyper-quick and monster-dirty
    {
        comma::visiting::traits< snark::nmea::message >::visit( k, static_cast< snark::nmea::message& >( p ), v );
        v.apply( "time", p.time );
        v.apply( "coordinates", p.coordinates );
        unsigned int q = static_cast< unsigned int >( p.quality );
        v.apply( "quality", q );
        p.quality = static_cast< snark::nmea::messages::gga::quality_t::values >( q );
        v.apply( "satellites_in_use", p.satellites_in_use );
        v.apply( "hdop", p.hdop );
        v.apply( "orthometric_height", p.orthometric_height );
        v.apply( "height_unit", p.height_unit );
        v.apply( "geoid_separation", p.geoid_separation );
        v.apply( "geoid_separation_unit", p.geoid_separation_unit );
        v.apply( "age_of_differential_gps_data_record", p.age_of_differential_gps_data_record );
        v.apply( "reference_station_id", p.reference_station_id );
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key& k, const snark::nmea::messages::gga& p, Visitor& v ) // hyper-quick and monster-dirty
    {
        comma::visiting::traits< snark::nmea::message >::visit( k, static_cast< const snark::nmea::message& >( p ), v );
        v.apply( "time", p.time );
        v.apply( "coordinates", p.coordinates );
        v.apply( "quality", static_cast< unsigned int >( p.quality ) );        
        v.apply( "satellites_in_use", p.satellites_in_use );
        v.apply( "hdop", p.hdop );
        v.apply( "orthometric_height", p.orthometric_height );
        v.apply( "height_unit", p.height_unit );
        v.apply( "geoid_separation", p.geoid_separation );
        v.apply( "geoid_separation_unit", p.geoid_separation_unit );
        v.apply( "age_of_differential_gps_data_record", p.age_of_differential_gps_data_record );
        v.apply( "reference_station_id", p.reference_station_id );
    }
};

template <> struct traits< snark::nmea::messages::rmc>
{
    template < typename Key, class Visitor >
    static void visit( const Key& k, snark::nmea::messages::rmc& p, Visitor& v ) // hyper-quick and monster-dirty
    {
        comma::visiting::traits< snark::nmea::message >::visit( k, static_cast< snark::nmea::message& >( p ), v );
        v.apply( "time", p.time );
        v.apply( "validity", p.validity );
        v.apply( "coordinates", p.coordinates );
        v.apply( "speed_in_knots", p.speed_in_knots );
        v.apply( "true_course", p.true_course);
        v.apply( "date", p.date);
        v.apply( "magnetic_variation", p.magnetic_variation );
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key& k, const snark::nmea::messages::rmc& p, Visitor& v ) // hyper-quick and monster-dirty
    {
        comma::visiting::traits< snark::nmea::message >::visit( k, static_cast< const snark::nmea::message& >( p ), v );
        v.apply( "time", p.time );
        v.apply( "validity", p.validity );
        v.apply( "coordinates", p.coordinates );
        v.apply( "speed_in_knots", p.speed_in_knots );
        v.apply( "true_course", p.true_course);
        v.apply( "date", p.date);
        v.apply( "magnetic_variation", p.magnetic_variation );
    }
};

template <> struct traits< snark::nmea::messages::angle >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::nmea::messages::angle& p, Visitor& v )
    {
        double a = p.value * 180 / M_PI;
        v.apply( "value", a );
        p.value = a * M_PI / 180;
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::nmea::messages::angle& p, Visitor& v )
    {
        double a = p.value * 180 / M_PI;
        v.apply( "value", a );
    }
};

template <> struct traits< snark::nmea::messages::trimble::message >
{
    template < typename Key, class Visitor >
    static void visit( const Key& k, snark::nmea::messages::trimble::message& p, Visitor& v )
    {
        comma::visiting::traits< snark::nmea::message >::visit( k, static_cast< snark::nmea::message& >( p ), v );
        v.apply( "message_type", p.message_type );
    }

    template < typename Key, class Visitor >
    static void visit( const Key& k, const snark::nmea::messages::trimble::message& p, Visitor& v )
    {
        comma::visiting::traits< snark::nmea::message >::visit( k, static_cast< const snark::nmea::message& >( p ), v );
        v.apply( "message_type", p.message_type );
    }
};

template <> struct traits< snark::nmea::messages::trimble::avr >
{
    template < typename Key, class Visitor >
    static void visit( const Key& k, snark::nmea::messages::trimble::avr& p, Visitor& v )
    {
        comma::visiting::traits< snark::nmea::messages::trimble::message >::visit( k, static_cast< snark::nmea::messages::trimble::message& >( p ), v );
        v.apply( "time", p.time );
        v.apply( "yaw", p.yaw );
        v.apply( "yaw_string", p.yaw_string );
        v.apply( "tilt", p.tilt );
        v.apply( "tilt_string", p.tilt_string );
        v.apply( "roll", p.roll );
        v.apply( "roll_string", p.roll_string );
        v.apply( "range", p.range );
        unsigned int q = static_cast< unsigned int >( p.quality );
        v.apply( "quality", q );
        p.quality = static_cast< snark::nmea::messages::trimble::avr::quality_t::values >( q );
        v.apply( "pdop", p.pdop );
        v.apply( "satellites_in_use", p.satellites_in_use );
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key& k, const snark::nmea::messages::trimble::avr& p, Visitor& v )
    {
        comma::visiting::traits< snark::nmea::messages::trimble::message >::visit( k, static_cast< const snark::nmea::messages::trimble::message& >( p ), v );
        v.apply( "time", p.time );
        v.apply( "yaw", p.yaw );
        v.apply( "yaw_string", p.yaw_string );
        v.apply( "tilt", p.tilt );
        v.apply( "tilt_string", p.tilt_string );
        v.apply( "roll", p.roll );
        v.apply( "roll_string", p.roll_string );
        v.apply( "range", p.range );
        v.apply( "quality", static_cast< unsigned int >( p.quality ) );        
        v.apply( "pdop", p.pdop );
        v.apply( "satellites_in_use", p.satellites_in_use );
    }
};

} } // namespace comma { namespace visiting {
