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

#include <iostream>
#include <comma/application/command_line_options.h>
#include <comma/base/exception.h>
#include <comma/csv/impl/fieldwise.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/string/string.h>
#include <comma/timing/timestamped.h>
#include <comma/timing/traits.h>
#include "../messages.h"
#include "../string.h"
#include "../traits.h"

bool zda_only=true;

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
    --permissive,--force; errors will tell you when to use --force
    --verbose,-v: more output to stderr

fields
    default: t,latitude,longitude,z

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
        //comma::int32 quality{0};
        //snark::nmea::messages::gga::quality_t::values quality;
        comma::uint32 height_unit{0}; //incorrect
        comma::uint32 hdop{0};
        comma::uint32 height_of_geoid{0};
        comma::uint32 geoid_separation_unit{0};
        comma::uint32 age_of_differential_gps_data_record{0};
        comma::uint32 reference_station_id{0};
        comma::uint32 speed_in_knots{0};
        comma::uint32 variation{0};
        comma::uint32 east_west{0};
        comma::uint32 validity{0};
        comma::uint32 true_course{0};
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
        v.apply( "height_unit", p.height_unit );
        v.apply( "hdop", p.hdop );
        v.apply( "height_of_geoid", p.height_of_geoid );
        v.apply( "geoid_separation_unit", p.geoid_separation_unit );
        v.apply( "age_of_differential_gps_data_record", p.age_of_differential_gps_data_record );
        v.apply( "reference_station_id", p.reference_station_id );
        v.apply( "speed_in_knots", p.speed_in_knots );
        v.apply( "variation", p.variation );
        v.apply( "east_west", p.east_west );
        v.apply( "validity", p.validity );
        v.apply( "true_course", p.true_course );
    }

    template < typename Key, class Visitor > static void visit( const Key&, const input::data& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "orientation", p.orientation );
        v.apply( "number_of_satellites", p.number_of_satellites );
        //v.apply( "quality", p.quality );
        v.apply( "height_unit", p.height_unit );
        v.apply( "hdop", p.hdop );
        v.apply( "height_of_geoid", p.height_of_geoid );
        v.apply( "geoid_separation_unit", p.geoid_separation_unit );
        v.apply( "age_of_differential_gps_data_record", p.age_of_differential_gps_data_record );
        v.apply( "reference_station_id", p.reference_station_id );
        v.apply( "speed_in_knots", p.speed_in_knots );
        v.apply( "variation", p.variation );
        v.apply( "east_west", p.east_west );
        v.apply( "validity", p.validity );
        v.apply( "true_course", p.true_course );
    }
};

} } // namespace comma { namespace visiting {

static input::type input_;

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
            if( v[i] == "roll" || v[i] == "pitch" || v[i] == "yaw" ) { v[i] = "data/orientation/" + v[i]; }
            else if( v[i] == "number_of_satellites" || v[i] == "quality" ) { v[i] = "data/" + v[i]; }
        }
        csv.fields = comma::join( v, ',' );
        input::type sample;
        sample.data.number_of_satellites = options.value( "--number-of-satellites,--satellites", 0 );
        sample.data.height_unit = options.value( "--height_unit", 0 );
        //sample.data.quality = options.value( "--quality", 0 );
        //sample.data.quality = static_cast<snark::nmea::messages::gga::quality_t::values>(options.value( "--quality", 0 ));
        sample.data.hdop = options.value( "--hdop", 0 );
        sample.data.height_of_geoid = options.value( "--height_of_geoid", 0 );
        sample.data.geoid_separation_unit = options.value( "--geoid_separation_unit", 0 );
        sample.data.age_of_differential_gps_data_record = options.value( "--age_of_differential_gps_data_record", 0 );
        sample.data.reference_station_id = options.value( "--reference_station_id", 0 );

        sample.data.speed_in_knots = options.value( "--speed_in_knots", 0 );
        sample.data.variation = options.value( "--variation", 0 );
        sample.data.east_west = options.value( "--east_west", 0 );
        sample.data.validity = options.value( "--validity", 0 );
        sample.data.true_course = options.value( "--true_course", 0 );
        
        comma::csv::input_stream< input::type > is( std::cin, csv, sample );
        comma::csv::ascii< snark::nmea::messages::gga > gga;
        while( is.ready() && std::cin.good() )
        {
            const input::type* p = is.read();
            if( !p ) { break; }
            // gga stuff
            COMMA_ASSERT_BRIEF( p->data.number_of_satellites > 0 || permissive, "got 0 satellites, use --permissive to override" );
            //COMMA_ASSERT_BRIEF( p->data.quality > 0 || permissive, "got quality 0, use --permissive to override" );
            COMMA_ASSERT_BRIEF( p->data.height_unit > 0 || permissive, "got height_unit 0, use --permissive to override" );
            COMMA_ASSERT_BRIEF( p->data.hdop > 0 || permissive, "got hdop 0, use --permissive to override" );
            COMMA_ASSERT_BRIEF( p->data.height_of_geoid > 0 || permissive, "got height_of_geoid 0, use --permissive to override" );
            COMMA_ASSERT_BRIEF( p->data.geoid_separation_unit > 0 || permissive, "got geoid_separation_unit 0, use --permissive to override" );
            COMMA_ASSERT_BRIEF( p->data.age_of_differential_gps_data_record > 0 || permissive, "got age_of_differential_gps_data_record 0, use --permissive to override" );
            COMMA_ASSERT_BRIEF( p->data.reference_station_id > 0 || permissive, "got reference_station_id 0, use --permissive to override" );

            // rmc stuff
            COMMA_ASSERT_BRIEF( p->data.speed_in_knots > 0 || permissive, "got speed_in_knots 0, use --permissive to override" );
            COMMA_ASSERT_BRIEF( p->data.variation > 0 || permissive, "got variation 0, use --permissive to override" );
            COMMA_ASSERT_BRIEF( p->data.east_west > 0 || permissive, "got east_west 0, use --permissive to override" );
            COMMA_ASSERT_BRIEF( p->data.validity > 0 || permissive, "got validity 0, use --permissive to override" );
            COMMA_ASSERT_BRIEF( p->data.true_course > 0 || permissive, "got true_course 0, use --permissive to override" );

            std::cerr << "==> p: " << p->data.position.latitude << "," << p->data.position.longitude << " satellites: " << p->data.number_of_satellites << std::endl;

            for( const auto& t: output_types )
            {
                // todo: convert input into nmea strings, write to stdout
                //if( t == "gga,gsa,gsv,rmc" )
                if( t == "gga" )
                {
                    // todo: output gga
                    snark::nmea::messages::gga m;
                    m.coordinates.latitude.value = p->data.position.latitude;
                    m.coordinates.longitude.value = p->data.position.longitude;
                    // todo: fill the remaining fields
                    m.satellites_in_use = p->data.number_of_satellites;
                    //m.quality = p->data.quality;
                    m.hdop = p->data.hdop;
                    m.orthometric_height = p->data.position.z;
                    m.height_unit = p->data.height_unit;
                    m.geoid_separation = p->data.height_of_geoid;
                    m.geoid_separation_unit = p->data.geoid_separation_unit;
                    m.age_of_differential_gps_data_record = p->data.age_of_differential_gps_data_record;
                    m.reference_station_id = p->data.reference_station_id;

                    std::string line;
                    gga.put( m, line );
                    std::cout << line << std::endl;
                    continue;
                }
                // if( t == "gsa" )
                // {
                //     // todo: not needed for sagetech so on hold till someone else needs this
                //     continue;
                // }
                // if( t == "gsv" )
                // {
                //     // todo: not needed for sagetech so on hold till someone else needs this
                //     continue;
                // }
                if( t == "rmc" )
                {
                    snark::nmea::messages::rmc r;
                    r.validity = p->data.validity;
                    r.coordinates.latitude.value = p->data.position.latitude;
                    r.coordinates.longitude.value = p->data.position.longitude;
                    r.speed_in_knots = p->data.speed_in_knots;
                    r.true_course = p->data.true_course;
                    r.variation = p->data.variation;
                    r.east_west = p->data.east_west;
                    // todo: output rmc
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
