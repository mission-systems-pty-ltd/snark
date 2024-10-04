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

#include <iostream>
#include <comma/application/command_line_options.h>
#include <comma/csv/impl/fieldwise.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/application/verbose.h>
#include "../../../math/spherical_geometry/coordinates.h"
#include "../../../math/spherical_geometry/traits.h"
#include "../../../timing/timestamped.h"
#include "../../../timing/traits.h"
#include "../messages.h"
#include "../string.h"
#include "../traits.h"

bool zda_only=true;

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "convert nmea data to csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: netcat localhost 12345 | nmea-to-csv [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: output help; --help --verbose: more help" << std::endl;
    std::cerr << "    --fields,-f=<fields>: select output fields" << std::endl;
    std::cerr << "    --output-all,--all: if present, output records on every gps update," << std::endl;
    std::cerr << "                        even if values of output fields have not changed" << std::endl;
    std::cerr << "    --output-fields: print output fields and exit" << std::endl;
    std::cerr << "    --output-format: print output format and exit" << std::endl;
    std::cerr << "    --output-ignore-checksum,--ignore-checksum: handle record, even if checksum invalid" << std::endl;
    std::cerr << "    --output-ignore-missing-timestamp,--ignore-missing-time" << std::endl;
    std::cerr << "    --output-on-gga-only: output on every GGA message only" << std::endl;
    std::cerr << "    --permissive: skip record, if checksum invalid" << std::endl;
    std::cerr << "    --verbose,-v: more output to stderr" << std::endl;
    std::cerr << "    --no-zda; (WARNING: for backward compatibility only) updates time from any message, used when input has no ZDA message"<< std::endl;
    std::cerr << "        when not specified, time field is only updated from ZDA messages"<< std::endl;
    std::cerr << std::endl;
    std::cerr << std::endl;
    std::cerr << "fields" << std::endl;
    std::cerr << "    default: t,latitude,longitude,z,roll,pitch,yaw,number_of_satellites,quality" << std::endl;
    std::cerr << std::endl;
    if( verbose ) { std::cerr << std::endl << "binary options" << std::endl << comma::csv::options::usage() << std::endl; }
    exit( 0 );
}

struct position
{
    snark::spherical::coordinates coordinates;
    double z{0};

    position() = default;
    position( const snark::spherical::coordinates& coordinates, double z ) : coordinates( coordinates ), z( z ) {}
};

struct orientation
{
    double roll{0};
    double pitch{0};
    double yaw{0};

    orientation() = default;
    orientation( double roll, double pitch, double yaw ) : roll( roll ), pitch( pitch ), yaw( yaw ) {}
};

struct output
{
    struct data
    {
        ::position position; // todo: make optional? or simply check for zeroes?
        ::orientation orientation; // todo: make optional? or simply check for zeroes?
        comma::uint32 number_of_satellites{0};
        comma::int32 quality{0};
    };

    typedef snark::timestamped< data > type;
};

namespace comma { namespace visiting {

template <> struct traits< position >
{
    template < typename Key, class Visitor > static void visit( const Key&, position& p, Visitor& v )
    {
        v.apply( "coordinates", p.coordinates );
        v.apply( "z", p.z );
    }

    template < typename Key, class Visitor > static void visit( const Key&, const position& p, Visitor& v )
    {
        v.apply( "coordinates", p.coordinates );
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

template <> struct traits< output::data >
{
    template < typename Key, class Visitor > static void visit( const Key&, output::data& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "orientation", p.orientation );
        v.apply( "number_of_satellites", p.number_of_satellites );
        v.apply( "quality", p.quality );
    }

    template < typename Key, class Visitor > static void visit( const Key&, const output::data& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "orientation", p.orientation );
        v.apply( "number_of_satellites", p.number_of_satellites );
        v.apply( "quality", p.quality );
    }
};

} } // namespace comma { namespace visiting {

static bool _ignore_missing_timestamp{false};
static output::type output_;

static comma::csv::fieldwise make_fieldwise( const comma::csv::options& csv )
{
    std::vector< std::string > v = comma::split( csv.fields, ',' );
    for( unsigned int i = 0; i < v.size(); ++i ) { if( v[i] == "t" ) { v[i] = ""; } }
    comma::csv::options c = csv;
    c.fields = comma::join( v, ',' );
    return comma::csv::fieldwise( output::type(), c );
}

static void output( const comma::csv::fieldwise& fieldwise, const output::type& v, comma::csv::output_stream< output::type >& os )
{
    static std::string last;
    std::string current;
    if( os.is_binary() )
    {
        current.resize( os.binary().binary().format().size() );
        os.binary().binary().put( v, &current[0] );
        if( !fieldwise.binary().equal( &last[0], &current[0] ) ) { std::cout.write( &current[0], os.binary().binary().format().size() ); std::cout.flush(); }
    }
    else
    {
        os.ascii().ascii().put( v, current );
        if( !fieldwise.ascii().equal( last, current ) ) { std::cout << current << std::endl; }
    }
    last = current;
}

using namespace snark;

bool handle( const nmea::messages::zda& zda )
{
    output_.t = zda.time;
    return true;
}

bool handle( const nmea::messages::gga& v )
{
    if( v.quality==snark::nmea::messages::gga::quality_t::fix_not_valid ) { return false; }
    // GGA message contains only time_of_day, so need to have loaded the actual date from
    // a different message already.
    bool valid_time = _ignore_missing_timestamp || !output_.t.is_not_a_date_time();
    if( !zda_only && valid_time && !_ignore_missing_timestamp ) { output_.t = boost::posix_time::ptime( output_.t.date(), v.time.value.time_of_day() ); }
    output_.data.position.coordinates = v.coordinates();
    output_.data.position.z = v.orthometric_height;
    output_.data.number_of_satellites = v.satellites_in_use;
    output_.data.quality = v.quality;
    return valid_time;
}

bool handle( const nmea::messages::rmc& v )
{
    if(!zda_only && v.validity == "A")
    {
        output_.t = boost::posix_time::ptime(v.date.value, v.time.value.time_of_day());
    }
    output_.data.position.coordinates = v.coordinates();
    return true;
}

bool handle( const nmea::messages::trimble::avr& m )
{
    if(m.quality==snark::nmea::messages::trimble::avr::quality_t::fix_not_valid) { return false; }
    // GGA message contains only time_of_day, so need to have loaded the actual date from
    // a different message already.
    bool valid_time = _ignore_missing_timestamp || !output_.t.is_not_a_date_time();
    if(!zda_only && valid_time && !_ignore_missing_timestamp)
    {
        output_.t = boost::posix_time::ptime(output_.t.date(), m.time.value.time_of_day());
    }
    output_.data.orientation.roll = m.roll.value;
    output_.data.orientation.pitch = m.tilt.value;
    output_.data.orientation.yaw = m.yaw.value;
    //output_.data.number_of_satellites = m.value.satellites_in_use;
    return valid_time;
}

template < typename T > bool handle( const nmea::string& s )
{
    static comma::csv::ascii< T > ascii;
    return handle( ascii.get( s.values() ) );
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< output::type >( false ), ',' ) << std::endl; return 0; }
        if( options.exists( "--output-format" ) ) { std::cout << comma::csv::format::value< output::type >( options.value< std::string >( "--fields,-f", "" ), false ) << std::endl; return 0; }
        bool output_all = options.exists( "--output-all,--all" );
        bool output_on_gga_only = options.exists( "--output-on-gga-only" );
        _ignore_missing_timestamp = options.exists( "--output-ignore-missing-timestamp,--ignore-missing-time" );
        bool verbose = options.exists( "--verbose,-v" );
        bool permissive = options.exists( "--permissive" );
        bool ignore_checksum = options.exists( "--output-ignore-checksum,--ignore-checksum" );
        if( options.exists( "--ignore-parsing-errors" ) ) { std::cerr << "nmea-to-csv: --ignore-parsing-errors: deprecated; use --permissive" << std::endl; return 1; }
        zda_only=!options.exists("--no-zda");
        comma::csv::options csv( options );
        csv.full_xpath = true; // for future, e.g. to plug in errors
        if( csv.fields.empty() ) { csv.fields = comma::join( comma::csv::names< output::type >( csv.full_xpath ), ',' ); }
        else if( csv.full_xpath )
        {
            std::vector< std::string > v = comma::split( csv.fields, ',' );
            for( unsigned int i = 0; i < v.size(); ++i )
            {
                if( v[i] == "latitude" || v[i] == "longitude" ) { v[i] = "data/position/coordinates/" + v[i]; }
                else if( v[i] == "z" ) { v[i] = "data/position/" + v[i]; }
                if( v[i] == "roll" || v[i] == "pitch" || v[i] == "yaw" ) { v[i] = "data/orientation/" + v[i]; }
                else if( v[i] == "number_of_satellites" || v[i] == "quality" ) { v[i] = "data/" + v[i]; }
            }
            csv.fields = comma::join( v, ',' );
        }
        if( options.exists( "--binary" ) ) { csv.format( comma::csv::format::value< output::type >( csv.fields, csv.full_xpath ) ); }
        comma::csv::fieldwise fieldwise = make_fieldwise( csv ); // todo: plug in
        comma::csv::output_stream< output::type > os( std::cout, csv );
        while( std::cin.good() )
        {
            std::string line;
            std::getline( std::cin, line );
            if( line.empty() ) { continue; }
            nmea::string s( line, ignore_checksum );
            if( !s.valid() )
            {
                comma::say() << ( permissive ? "skipped": "got" ) << " invalid nmea string: \"" << line << "\"" << std::endl;
                if( permissive ) { continue; }
                return 1;
            }
            try
            {
                if( s.is_proprietary() )
                {
                    if( s.manufacturer_code() == nmea::messages::trimble::manufacturer_code )
                    {
                        if( static_cast< const nmea::messages::trimble::string& >( s ).message_type() == nmea::messages::trimble::avr::type ) { if( !handle< nmea::messages::trimble::avr >( s ) ) { continue; } }
                        else { if( verbose ) { std::cerr << "nmea-to-csv: discarded unimplemented trimble message: \"" << line << "\"" << std::endl; } continue; }
                    }
                    else
                    {
                        if( verbose ) { std::cerr << "nmea-to-csv: discarded unimplemented proprietary nmea message: \"" << line << "\"" << std::endl; }
                        continue;
                    }
                }
                else if( s.message_type() == nmea::messages::gga::type ) { if( !handle< nmea::messages::gga >( s ) ) { continue; } }
                else if( s.message_type() == nmea::messages::rmc::type ) { if( !handle< nmea::messages::rmc>( s ) ) { continue; } }
                else if( s.message_type() == nmea::messages::zda::type ) { if( !handle(nmea::messages::zda( s ) ) ) { continue; } }
                else { if( verbose ) { std::cerr << "nmea-to-csv: discarded unimplemented string: \"" << line << "\"" << std::endl; } continue; }
                if( output_on_gga_only ) { if( s.message_type() == nmea::messages::gga::type ) { os.write( output_ ); } }
                else if( output_all ) { os.write( output_ ); }
                else { output( fieldwise, output_, os ); }
            }
            catch( std::exception& ex )
            {
                std::cerr << "nmea-to-csv: " << ex.what() << " on nmea string: \"" << comma::join( s.values(), ',' ) << "\"" << std::endl;
                if( !permissive ) { return 1; }
            }
            catch( ... )
            {
                std::cerr << "nmea-to-csv: unknown exception on nmea string: \"" <<  comma::join( s.values(), ',' ) << "\"" << std::endl;
                if( !permissive ) { return 1; }
            }
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "nmea-to-csv: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "nmea-to-csv: unknown exception" << std::endl; }
    return 1;
}
