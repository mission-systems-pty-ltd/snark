// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
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

/// @author vsevolod vlaskine

#pragma once

#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/lexical_cast.hpp>
#include <comma/string/string.h>
#include "../../math/spherical_geometry/coordinates.h"
#include "string.h"

// http://www.nmea.org/
namespace snark { namespace nmea {

struct message
{
    std::string id;
};
    
namespace messages {

struct coordinate { double value{0}; };

struct latitude_t: public coordinate {};

struct longitude_t: public coordinate {};

struct coordinates
{
    latitude_t latitude;
    longitude_t longitude;
    
    spherical::coordinates operator()() const { return spherical::coordinates( latitude.value, longitude.value ); }
};

struct time { boost::posix_time::ptime value; };

struct date { boost::gregorian::date value; };

struct angle { double value{0}; };
    
// http://www.gpsinformation.org/dale/nmea.htm#GGA
// todo: the website above does not exist anymore; find another one
struct gga : message
{
    static const std::string type;
    
    struct quality_t { enum values { fix_not_valid = 0, gps_fix = 1, differential_gps_fix = 2, pps_fix = 3, real_time_kinematic_fixed_integers = 4, real_time_kinematic_float_integers = 5, estimated = 6, manual_input = 7, simulation = 8 }; };

    nmea::messages::time time;
    nmea::messages::coordinates coordinates;
    quality_t::values quality{quality_t::fix_not_valid};
    unsigned int satellites_in_use{0};
    double hdop{0};
    double orthometric_height{0};
    std::string height_unit{"M"};
    double geoid_separation{0};
    std::string geoid_separation_unit{"M"};
    double age_of_differential_gps_data_record{0};
    std::string reference_station_id;

    gga(): message{"$GPGGA"} {}
};

struct rmc : message
{
    static const std::string type;

    nmea::messages::time time;
    std::string validity{"A"}; // A: data valid V: data invalid
    nmea::messages::coordinates coordinates;
    double speed_in_knots{0};
    double true_course{0};
    nmea::messages::date date;
    longitude_t magnetic_variation;

    rmc(): message{"$GPRMC"} {}
};

// http://www.gpsinformation.org/dale/nmea.htm#ZDA
// http://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_ZDA.html
struct zda : message
{
    static const std::string type;
    // $GPZDA, hhmmss.ss, dd, mm, yyyy, tzh, tzm
    boost::posix_time::ptime time;
    // num ber of seconds offset, between -13 * 3600 to 13 * 3600
    int local_time_zone_offset{0};

    zda(): message{"$GPZDA"} {}
    zda( const nmea::string& s ): message{"$GPZDA"}
    {
        const std::vector< std::string >& values=s.values();
        time=boost::posix_time::from_iso_string( values[4] + values[3] + values[2] + "T"+ values[1]);
        local_time_zone_offset = boost::lexical_cast<int>(values[5]) * 3600 + boost::lexical_cast<int>(values[6]) * 60;
    }
};

namespace trimble {

static const std::string manufacturer_code = "TNL";

struct message : nmea::message
{
    std::string message_type; // todo!!! set in subsclass default constructor; see gga() above
};

struct string : nmea::string { std::string message_type() const { return values()[1]; } };

struct avr : message
{
    static const std::string type;

    struct quality_t { enum values { fix_not_valid = 0, autonomous_gps_fix = 1, differential_carrier_phase_solution_rtk_float = 2, differential_carrier_phase_solution_rtk_int = 3, differential_code_based_solution_dgps = 4 }; };
    
    nmea::messages::time time;
    nmea::messages::angle yaw;
    std::string yaw_string;
    nmea::messages::angle tilt;
    std::string tilt_string;
    nmea::messages::angle roll;
    std::string roll_string;
    double range{0};
    quality_t::values quality;
    double pdop{0};
    unsigned int satellites_in_use{0};

    avr(): message{"$GPAVR"} {}
};

}; // namespace trimble {

const std::string gga::type = "GGA";
const std::string rmc::type = "RMC";
const std::string zda::type = "ZDA";
const std::string trimble::avr::type = "AVR";

} // namespace messages {

} } // namespace snark { namespace nmea {
