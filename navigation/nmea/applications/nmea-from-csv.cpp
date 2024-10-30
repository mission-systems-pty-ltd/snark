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

#include <functional>
#include <iostream>
#include <comma/application/command_line_options.h>
#include <comma/base/exception.h>
#include <comma/csv/impl/fieldwise.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/io/serial.h>
#include <comma/io/traits.h>
#include <comma/name_value/parser.h>
#include <comma/string/string.h>
#include <comma/timing/timestamped.h>
#include <comma/timing/traits.h>
#include "../messages.h"
#include "../string.h"
#include "../traits.h"

static void usage( bool verbose )
{
    std::cerr << R"(

read csv data on stdin, output nmea strings to stdout

options
    --input-fields; print input fields to stdout and exit
    --number-of-satellites,--satellites=[<n>]; default number of satellites
    --quality=[<n>]; default quality
    --output=<what>; default=gga,gsa,gsv,rmc; <what>: <type>[,<type>]..., what to
                     output in a given order
                     <type>: gga,gsa,gsv,rmc
    --output-address,--address=<address>; default='-'; e.g: --address=serial:/dev/ttyUSB0
    --permissive,--force; errors will tell you when to use --force
    --verbose,-v: more output to stderr

fields
    default: t,latitude,longitude,z

examples
    basics
        echo '$GPGGA,021220,3348.6819,S,15109.4862,E,1,08,0.9,656.168,M,46.9,M,,*52' \
            | nmea-to-csv --verbose --ignore-missing-time \
            | nmea-from-csv --fields ,latitude,longitude --permissive

        echo '$GPGGA,021220,3348.6819,S,15109.4862,E,1,08,0.9,656.168,M,46.9,M,,*52' \
            | nmea-to-csv --verbose --ignore-missing-time \
            | nmea-from-csv --fields ,latitude,longitude --permissive \
            | nmea-to-csv --verbose --ignore-missing-time

)" << std::endl;
    std::cerr << "csv options" << std::endl << comma::csv::options::usage( verbose ) << std::endl;
    exit( 0 );
}

struct position
{
    double latitude{0};
    double longitude{0};
    double z{0};
};

struct orientation
{
    double roll{0};
    double pitch{0};
    double yaw{0};

    orientation() = default;
    orientation( double roll, double pitch, double yaw ) : roll( roll ), pitch( pitch ), yaw( yaw ) {}
};

struct input
{
    struct data
    {
        ::position position; // todo: make optional? or simply check for zeroes?
        ::orientation orientation; // todo: make optional? or simply check for zeroes?
        comma::uint32 number_of_satellites{0};
        comma::int32 quality{snark::nmea::messages::gga::quality_t::gps_fix};
        double hdop{0}; // relative accuracy of horizontal position
        double geoid_separation{0};
        comma::uint32 age_of_differential_gps_data_record{0};
        std::string reference_station_id;
        double speed{0};
        double true_course{0};
        double magnetic_variation{0};
    };

    typedef comma::timestamped< data > type;
};

namespace comma { namespace visiting {

template <> struct traits< position >
{
    template < typename Key, class Visitor > static void visit( const Key&, position& p, Visitor& v )
    {
        v.apply( "latitude", p.latitude );
        v.apply( "longitude", p.longitude );
        v.apply( "z", p.z );
    }

    template < typename Key, class Visitor > static void visit( const Key&, const position& p, Visitor& v )
    {
        v.apply( "latitude", p.latitude );
        v.apply( "longitude", p.longitude );
        v.apply( "z", p.z );
    }
};

template <> struct traits< orientation >
{
    template < typename Key, class Visitor > static void visit( const Key&, orientation& p, Visitor& v )
    {
        v.apply( "roll", p.roll );
        v.apply( "pitch", p.pitch );
        v.apply( "yaw", p.yaw );
    }

    template < typename Key, class Visitor > static void visit( const Key&, const orientation& p, Visitor& v )
    {
        v.apply( "roll", p.roll );
        v.apply( "pitch", p.pitch );
        v.apply( "yaw", p.yaw );
    }
};

template <> struct traits< input::data >
{   
    template < typename Key, class Visitor > static void visit( const Key&, input::data& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "orientation", p.orientation );
        v.apply( "number_of_satellites", p.number_of_satellites );
        //v.apply( "quality", p.quality );
        v.apply( "hdop", p.hdop );
        v.apply( "geoid_separation", p.geoid_separation );
        v.apply( "age_of_differential_gps_data_record", p.age_of_differential_gps_data_record );
        v.apply( "reference_station_id", p.reference_station_id );
        v.apply( "speed", p.speed );
        v.apply( "true_course", p.true_course );
        v.apply( "magnetic_variation", p.magnetic_variation );
    }

    template < typename Key, class Visitor > static void visit( const Key&, const input::data& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "orientation", p.orientation );
        v.apply( "number_of_satellites", p.number_of_satellites );
        //v.apply( "quality", p.quality );
        v.apply( "hdop", p.hdop );
        v.apply( "geoid_separation", p.geoid_separation );
        v.apply( "age_of_differential_gps_data_record", p.age_of_differential_gps_data_record );
        v.apply( "reference_station_id", p.reference_station_id );
        v.apply( "speed", p.speed );
        v.apply( "true_course", p.true_course );
        v.apply( "magnetic_variation", p.magnetic_variation );
    }
};

} } // namespace comma { namespace visiting {

static input::type input_;

static void write_to_stdout( const std::string& s ) { std::cout << s << std::endl; }

static void write_to_serial( const std::string& a, const std::string& s )
{
    static comma::io::serial::port p( comma::name_value::parser( "name", ';', '=' ).get< comma::io::serial::port::properties >( a ) );
    p.write( &s[0], s.size() );
    p.write( "\r\n", 1 ); // todo? should it be "\n"? parametrize through a command-line option
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--input-fields" ) ) { std::cout << comma::join( comma::csv::names< input::type >( false ), ',' ) << std::endl; return 0; }
        const auto& output_types = comma::split( options.value< std::string >( "--output", "gga,rmc" ), ',', true );
        //for( const auto& t: output_types ) { COMMA_ASSERT_BRIEF( t == "gga" || t == "gsa" || t == "gsv" || t == "rmc", "expected nmea message type; got unsupported type: '" << t << "'" ); }
        for( const auto& t: output_types ) { COMMA_ASSERT_BRIEF( t == "gga" || t == "rmc", "expected nmea message type; got unsupported type: '" << t << "'" ); }
        bool permissive = options.exists( "--permissive" );
        comma::csv::options csv( options, "t,latitude,longitude,z" );
        std::vector< std::string > v = comma::split( csv.fields, ',', true );
        for( unsigned int i = 0; i < v.size(); ++i ) // todo: use alias map in csv::options constructor
        {
            if( v[i] == "latitude" || v[i] == "longitude" ) { v[i] = "data/position/" + v[i]; }
            else if( v[i] == "z" ) { v[i] = "data/position/" + v[i]; }
            else if( v[i] == "roll" || v[i] == "pitch" || v[i] == "yaw" ) { v[i] = "data/orientation/" + v[i]; }
            else if( v[i] == "number_of_satellites" || v[i] == "quality" ) { v[i] = "data/" + v[i]; }
            else if( v[i] == "hdop" ) { v[i] = "data/hdop"; }
            else if( v[i] == "geoid_separation" ) { v[i] = "data/geoid_separation"; }
            else if( v[i] == "age_of_differential_gps_data_record" ) { v[i] = "data/age_of_differential_gps_data_record"; }
            else if( v[i] == "reference_station_id" ) { v[i] = "data/reference_station_id"; }
            else if( v[i] == "speed" ) { v[i] = "data/speed"; }
            else if( v[i] == "true_course" ) { v[i] = "data/true_course"; }
            else if( v[i] == "magnetic_variation" ) { v[i] = "data/magnetic_variation"; }
        }
        csv.fields = comma::join( v, ',' );
        input::type sample;

        // GGA input flags --------------------------------------------------------------------------------------
        //sample.data.quality = options.value( "--quality", 0 );
        //sample.data.quality = static_cast<snark::nmea::messages::gga::quality_t::values>(options.value( "--quality", 0 ));
        sample.data.number_of_satellites = options.value( "--number-of-satellites,--satellites", 0 );
        sample.data.hdop = options.value( "--hdop", 0.0 );
        sample.data.geoid_separation = options.value( "--geoid-separation", 0.0 );
        sample.data.age_of_differential_gps_data_record = options.value( "--age-of-differential-gps-data-record", 0.0 );
        sample.data.reference_station_id = options.value< std::string >( "--reference-station-id", "" );

        // RMC input flags --------------------------------------------------------------------------------------
        sample.data.speed = options.value( "--speed", 0.0 );
        sample.data.true_course = options.value( "--true-course", 0.0 );
        sample.data.magnetic_variation = options.value( "--magnetic-variation", 0.0 );

        comma::csv::input_stream< input::type > is( std::cin, csv, sample );
        comma::csv::options o;
        o.quote.reset();
        comma::csv::ascii< snark::nmea::messages::gga > gga( o );
        comma::csv::ascii< snark::nmea::messages::rmc > rmc( o );
        std::function< void( const std::string& ) > output;
        std::string address = options.value< std::string >( "--address", "-" );
        if( address == "-" ) { output = write_to_stdout; }
        else if( address.substr( 0, 7 ) == "serial:" ) { output = std::bind( write_to_serial, address.substr( 7 ), std::placeholders::_1 ); }
        else { COMMA_THROW_BRIEF( comma::exception, "expected --address='-' or --address='serial:...', got: --address='" << address << "': not supported" ); }
        while( is.ready() || std::cin.good() )
        {
            auto p = is.read();
            if( !p ) { break; }
            // gga stuff
            COMMA_ASSERT_BRIEF( p->data.number_of_satellites > 0 || permissive, "got 0 satellites, use --permissive to override" );
            for( const auto& t: output_types )
            {
                // todo: convert input into nmea strings, write to stdout
                //if( t == "gga,gsa,gsv,rmc" )
                if( t == "gga" )
                {
                    // todo: output gga
                    snark::nmea::messages::gga m;
                    m.time.value = p->t;
                    m.coordinates.latitude.value = p->data.position.latitude;
                    m.coordinates.longitude.value = p->data.position.longitude;
                    // todo: fill the remaining fields
                    m.satellites_in_use = p->data.number_of_satellites;
                    //m.quality = p->data.quality;
                    m.hdop = p->data.hdop;
                    m.orthometric_height = p->data.position.z;
                    m.geoid_separation = p->data.geoid_separation;
                    m.age_of_differential_gps_data_record = p->data.age_of_differential_gps_data_record;
                    m.reference_station_id = p->data.reference_station_id;
                    std::string line;
                    gga.put( m, line );
                    output( line + snark::nmea::string::checksum_string( line, true ) );
                    continue;
                }
                if( t == "rmc" )
                {
                    snark::nmea::messages::rmc m;
                    m.time.value = p->t;
                    m.coordinates.latitude.value = p->data.position.latitude;
                    m.coordinates.longitude.value = p->data.position.longitude;
                    m.speed_in_knots = p->data.speed * 1.943844; // whatever
                    m.true_course = p->data.true_course; // degrees true course
                    m.magnetic_variation.value = p->data.magnetic_variation; // always positive
                    std::string line;
                    rmc.put( m, line );
                    output( line + snark::nmea::string::checksum_string( line, true ) );
                    continue;
                }
            }
        }
        return 0;
    }
    catch( std::exception& ex ) { comma::say() << ex.what() << std::endl; }
    catch( ... ) { comma::say() << "unknown exception" << std::endl; }
    return 1;
}
