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
        comma::int32 quality{0};
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
        v.apply( "quality", p.quality );
    }

    template < typename Key, class Visitor > static void visit( const Key&, const input::data& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "orientation", p.orientation );
        v.apply( "number_of_satellites", p.number_of_satellites );
        v.apply( "quality", p.quality );
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
        const auto& output_types = comma::split( options.value< std::string >( "--output", "gga,gsa,gsv,rmc" ), ',', true );
        for( const auto& t: output_types ) { COMMA_ASSERT_BRIEF( t == "gga" || t == "gsa" || t == "gsv" || t == "rmc", "expected nmea message type; got unsupported type: '" << t << "'" ); }
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
        sample.data.quality = options.value( "--quality", 0 );
        comma::csv::input_stream< input::type > is( std::cin, csv, sample );
        while( is.ready() && std::cin.good() )
        {
            const input::type* p = is.read();
            if( !p ) { break; }
            COMMA_ASSERT_BRIEF( p->data.number_of_satellites > 0 || permissive, "got 0 satellites, use --permissive to override" );
            COMMA_ASSERT_BRIEF( p->data.quality > 0 || permissive, "got quality 0, use --permissive to override" );

            //std::cerr << "==> p: " << p->data.position.latitude << "," << p->data.position.longitude << " satellites: " << p->data.number_of_satellites << std::endl;

            for( const auto& t: output_types )
            {
                // todo: convert input into nmea strings, write to stdout
                //if( t == "gga,gsa,gsv,rmc" )
                if( t == "gga" )
                {
                    // todo: output gga
                    continue;
                }
                if( t == "gsa" )
                {
                    // todo: output gga
                    continue;
                }
                if( t == "gsv" )
                {
                    // todo: output gga
                    continue;
                }
                if( t == "rmc" )
                {
                    // todo: output gga
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
