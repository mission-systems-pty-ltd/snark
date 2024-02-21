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

#include <boost/optional.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/exception.h>
#include <comma/base/none.h>
#include <comma/base/types.h>
#include <comma/csv/format.h>
#include <comma/csv/names.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include "../../../../timing/clocked_time_stamp.h"
#include "../hdl64/stream.h"
#include "../impl/pcap_reader.h"
#include "../impl/proprietary_reader.h"
#include "../impl/thin_reader.h"
#include "../impl/udp_reader.h"
#include "../impl/stream_reader.h"
#include "../impl/velodyne_stream.h"
#include "../puck/calculator.h"
#include "../puck/packet.h"
#include "../puck/stream.h"

//#include <google/profiler.h>

class adjust_timestamp
{
public:
    static unsigned const DEFAULT_PERIOD = 288;
    static unsigned const DEFAULT_THRESHOLD = 550;
    static unsigned const DEFAULT_RESET = 300;

    enum mode_kind { none = 0, average, hard };

    class mode
    {
    private:
        adjust_timestamp::mode_kind m_kind;

    public:
        mode( void ) : m_kind( adjust_timestamp::none ) {}
        mode( std::string const& i_mode_string );

        adjust_timestamp::mode_kind kind( void ) const { return m_kind; }
    };

    boost::posix_time::ptime operator()( boost::posix_time::ptime t );

    adjust_timestamp( std::string const& i_mode_string, unsigned const i_adjusted_threshold, unsigned const i_adjusted_reset );

private:
    mode mode_;
    snark::timing::clocked_time_stamp adjusted_timestamp_;
    snark::timing::periodic_time_stamp periodic_time_stamp_;
    boost::posix_time::ptime last_timestamp_;
};

static void usage( bool )
{
    std::cerr << std::endl;
    std::cerr << "Takes velodyne packets and outputs coordinate datapoints to stdout" << std::endl;
    std::cerr << "Usage: cat velodyne*.bin | velodyne-to-csv <options>" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --output-fields: print output fields and exit" << std::endl;
    std::cerr << "    --output-format,--format: output full binary format and exit (see examples)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "input stream options" << std::endl;
    std::cerr << "    default : read velodyne data directly from stdin in the format: <timestamp><packet>" << std::endl;
    std::cerr << "              <timestamp>: 8-byte unsigned int, microseconds from linux epoch" << std::endl;
    std::cerr << "              <packet>: regular velodyne 1206-byte packet" << std::endl;
    std::cerr << "    --pcap : if present, velodyne data is read from pcap packets" << std::endl;
    std::cerr << "    --thin : if present, velodyne data is thinned (e.g. by velodyne-thin)" << std::endl;
    std::cerr << "    --udp-port <port> : read velodyne data directly from udp port" << std::endl;
    std::cerr << std::endl;
    std::cerr << "input format options" << std::endl;
    std::cerr << "    --model=<model>: velodyne model" << std::endl;
    std::cerr << "        <model>" << std::endl;
    std::cerr << "            hdl64,hdl-64,64: hdl-64" << std::endl;
    std::cerr << "            vlp16,vlp-16,puck: puck vlp-16" << std::endl;
    std::cerr << "    --proprietary,-q : read velodyne data directly from stdin using the proprietary protocol" << std::endl;
    std::cerr << "        <header, 16 bytes><timestamp, 12 bytes><packet, 1206 bytes><footer, 4 bytes>" << std::endl;
    std::cerr << "    --puck: same as --model=puck" << std::endl;
    std::cerr << "    --64,--hdl64 : same as --model=hdl64" << std::endl;
    std::cerr << "    default input format: <timestamp, 8 bytes><packet, 1206 bytes>" << std::endl;
    std::cerr << std::endl;
    std::cerr << "configuration options:" << std::endl;
    std::cerr << "    hdl-64" << std::endl;
    std::cerr << "        --db=<db.xml file>; default=/usr/local/etc/db.xml" << std::endl;
    std::cerr << "            if the file is a version 0 then the legacy option is used for timing and azimuth calculation" << std::endl;
    std::cerr << "    rs-lidar-16" << std::endl;
    std::cerr << "        --angles=<filename>; default: as in spec" << std::endl;
    std::cerr << std::endl;
    std::cerr << "output options:" << std::endl;
    std::cerr << "    --binary,-b[=<format>]: if present, output in binary equivalent of csv" << std::endl;
    std::cerr << "    --fields <fields>: e.g. t,x,y,z,scan" << std::endl;
    std::cerr << "    --flush: if present, flush output stream after each write" << std::endl;
    std::cerr << "    --min-range=<value>: do not output points closer than <value>; default 0" << std::endl;
    std::cerr << "    --max-range=<value>: do not output points farther away than <value>; default output all points" << std::endl;
    std::cerr << "    --ntp=[<threshold>],[<\"permissive\">]: get data timestamps from ntp data in packets (default: system time from input stream)" << std::endl;
    std::cerr << "                                            if times differ by more than <threshold>, use system time if permissive" << std::endl;
    std::cerr << "    --output-invalid-points: output also invalid laser returns" << std::endl;
    std::cerr << "    --scans [<from>]:[<to>] : output only scans in given range" << std::endl;
    std::cerr << "                               e.g. 1:3 for scans 1, 2, 3" << std::endl;
    std::cerr << "                                    5: for scans 5, 6, ..." << std::endl;
    std::cerr << "                                    :3 for scans 0, 1, 2, 3" << std::endl;
    std::cerr << "           attention! by default scans will start at 90 degrees so that the data in front and at the back" << std::endl;
    std::cerr << "                      of the lidar are not split by the scan boundary" << std::endl;
    std::cerr << "                      --scan-begin-angle: todo" << std::endl;
    std::cerr << "    --raw-intensity: output intensity data without any correction" << std::endl;
    std::cerr << "    --legacy: use old timetable and old algorithm for azimuth calculation" << std::endl;
    std::cerr << "    default output columns: " << comma::join( comma::csv::names< snark::velodyne_point >(), ',' ) << std::endl;
    std::cerr << "    default binary format: " << comma::csv::format::value< snark::velodyne_point >() << std::endl;
    std::cerr << "time options:" << std::endl;
    std::cerr << "    --adjusted-time=<mode>: adjust input time to match velodyne period; " << std::endl;
    std::cerr << "        mode is string name for method of adjustment:" << std::endl;
    std::cerr << "            'average': adjust total deviation" << std::endl;
    std::cerr << "            'hard': use closest point as base then add period for later points" << std::endl;
    std::cerr << "        when using on log files, the timestamps may change depending on start of data" << std::endl;
    std::cerr << "    --adjusted-time-threshold=<threshold>: upper bound for adjusting timestamp, only effective with --adjusted-time; unit: microseconds, default: "<< adjust_timestamp::DEFAULT_THRESHOLD << std::endl;
    std::cerr << "        if input timestamp is greater than period * ticks + <threshold>, it will reset adjusted time and the input timestamp will be used without change" << std::endl;
    std::cerr << "    --adjusted-time-reset=<reset>: reset adjusted time after this time, only effective with --adjusted-time; unit: seconds, default: "<< adjust_timestamp::DEFAULT_RESET << std::endl;
    std::cerr << "        if input timestamp + <reset> is greater than last reset time, it will reset adjusted time and the input timestamp will be used without change" << std::endl;
    std::cerr << "packet options:" << std::endl;
    std::cerr << "    --discard-invalid-scans: don't output scans with missing packets" << std::endl;
    std::cerr << "    --missing-packets-threshold=<n>: number of consecutive missing packets for new/invalid scan" << std::endl;
    std::cerr << "        if the data is missing more than <n> consecutive packets (calculated based on timestamp), then the next packet after the gap will be marked as the beginning of a new scan" << std::endl;
    std::cerr << "        use --discard-invalid-scans option together with this option to discard all the scans that have missing consecutive packets greater than <n>" << std::endl;
    std::cerr << "        by default (when --missing-packets-threshold is not specified) it will mark new/invalid scan if more than 25 millisecond of data is missing (half a circle at 20Hz)" << std::endl;
    std::cerr << "    --packet-duration: print packet duration in milliseconds and exit, --model needs to be specified" << std::endl;
    std::cerr << "    --packet-angle: print packet angle in degrees and exit; calculates packet angle based on packet duration and rotation speed, use --rotation-per-second to specify rotation speed" << std::endl;
    std::cerr << "    --rotation-per-second: specify rotation speed, see --packet-angle; default: 20" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples:" << std::endl;
    std::cerr << "    output csv points to file:" << std::endl;
    std::cerr << "    cat raw/*.bin | velodyne-to-csv --db db.xml > velodyne.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    output csv only valid points to file:" << std::endl;
    std::cerr << "    cat raw/*.bin | velodyne-to-csv --db db.xml | grep \",1$\" > velodyne.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    output binary points to file, then view file (colour by laser id):" << std::endl;
    std::cerr << "    (the output could be directed straight to view-points," << std::endl;
    std::cerr << "    just the command line would be longer)" << std::endl;
    std::cerr << "    raw/*.bin | velodyne-to-csv --db db.xml --binary | > velodyne.bin" << std::endl;
    std::cerr << "    cat velodyne.bin | view-points --fields \",id,,,,,x,y,z\" --binary $(velodyne-to-csv --output-format)" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

adjust_timestamp::mode::mode( std::string const& i_mode_string ): m_kind( adjust_timestamp::none )
{
    if( i_mode_string.empty() ) { m_kind = adjust_timestamp::none; }    
    else if( i_mode_string == "average" ) { m_kind = adjust_timestamp::average; }
    else if( i_mode_string == "hard" ) { m_kind = adjust_timestamp::hard; }
    else { COMMA_THROW(comma::exception, "invalid name for adjusted time: "<< i_mode_string <<" (valid values: average, hard)"); }
}

adjust_timestamp::adjust_timestamp( std::string const& i_mode_string, unsigned const i_adjusted_threshold, unsigned const i_adjusted_reset )
    : mode_( i_mode_string )
    , adjusted_timestamp_( boost::posix_time::microseconds( DEFAULT_PERIOD ) )
    , periodic_time_stamp_( boost::posix_time::microseconds( DEFAULT_PERIOD ), boost::posix_time::microseconds( i_adjusted_threshold ), boost::posix_time::seconds( i_adjusted_reset ) )
{
}

boost::posix_time::ptime adjust_timestamp::operator()( boost::posix_time::ptime timestamp )
{
    if( adjust_timestamp::none == mode_.kind() ) { return timestamp; }
    if( !last_timestamp_.is_not_a_date_time() && ( timestamp - last_timestamp_ ) > periodic_time_stamp_.adjusted_threshold() )
    {
        adjusted_timestamp_.reset();
        //periodic_time_stamp_.reset();
    }
    last_timestamp_ = timestamp;
    switch( mode_.kind() )
    {
        case adjust_timestamp::average: return adjusted_timestamp_.adjusted( timestamp );
        case adjust_timestamp::hard: return periodic_time_stamp_.adjusted( timestamp );
        default: { COMMA_THROW(comma::exception, "invalid adjust time mode "<< mode_.kind() ); }
    }
}

static std::string fields_( const std::string& s ) // parsing fields, quick and dirty
{
    if( s == "" ) { return s; }
    std::vector< std::string > v = comma::split( s, ',' );
    for( std::size_t i = 0; i < v.size(); ++i )
    {
        if( v[i] == "x" ) { v[i] = "ray/second/x"; }
        else if( v[i] == "y" ) { v[i] = "ray/second/y"; }
        else if( v[i] == "z" ) { v[i] = "ray/second/z"; }
        else if( v[i] == "r" ) { v[i] = "range"; } // convenience renaming
        else if( v[i] == "bearing" || v[i] == "b" ) { v[i] = "azimuth"; } // convenience renaming
        else if( v[i] == "block" ) { v[i] = "scan"; } // convenience renaming
    }
    return comma::join( v, ',' );
}

static comma::csv::format format_( const std::string& s, const std::string& fields )
{
    if( !s.empty() ) { try { return comma::csv::format( s ); } catch( ... ) {} }
    if( fields.empty() ) { return comma::csv::format::value< snark::velodyne_point >(); }
    std::vector< std::string > v = comma::split( fields, ',' );
    comma::csv::format format;
    for( std::size_t i = 0; i < v.size(); ++i )
    {
        if( v[i] == "t" ) { format += "t"; }
        else if( v[i] == "id" ) { format += "ui"; }
        else if( v[i] == "intensity" ) { format += "ui"; }
        else if( v[i] == "valid" ) { format += "b"; }
        else if( v[i] == "scan" ) { format += "ui"; }
        else if( v[i] == "ray" ) { format += "6d"; }
        else if( v[i] == "ray/first" ) { format += "3d"; }
        else if( v[i] == "ray/second" ) { format += "3d"; }
        else { format += "d"; }
    }
    return format;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        bool verbose = options.exists( "--verbose,-v" );
        if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< snark::velodyne_point >(), ',' ) << std::endl; return 0; }
        std::string fields = fields_( options.value< std::string >( "--fields", "" ) );
        comma::csv::format format = format_( options.value< std::string >( "--binary,-b", "" ), fields );
        if( options.exists( "--output-format,--format" ) ) { std::cout << format.string() << std::endl; return 0; }
        bool output_invalid_points = options.exists( "--output-invalid-points" );
        bool discard_invalid_scans=options.exists("--discard-invalid-scans");
        boost::optional< std::size_t > from = boost::make_optional< std::size_t >( false, 0 );
        boost::optional< std::size_t > to = boost::make_optional< std::size_t >( false, 0 );
        if( options.exists( "--scans" ) )
        {
            std::string range = options.value< std::string >( "--scans" );
            std::vector< std::string > v = comma::split( range, ':' );
            if( v.size() != 2 ) { COMMA_THROW( comma::exception, "expected range in format <from>:<to>, got: \"" << range << "\"" ); }
            from = v[0] == "" ? 0 : boost::lexical_cast< std::size_t >( v[0] );
            if( v[1] != "" ) { to = boost::lexical_cast< std::size_t >( v[1] ); }
            if( from && to && *from > *to ) { COMMA_THROW( comma::exception, "expected <from> not greater than <to> in the range, got: \"" << range << "\"" ); }
        }
        comma::csv::options csv;
        csv.fields = fields;
        csv.full_xpath = true;
        csv.flush = options.exists( "--flush" );
        if( options.exists( "--binary,-b" ) ) { csv.format( format ); }
        options.assert_mutually_exclusive( "--pcap,--thin,--udp-port,--proprietary,-q" );
        options.assert_mutually_exclusive( "--puck,--db" );
        options.assert_mutually_exclusive( "--puck,--model" );
        double min_range = options.value( "--min-range", 0.0 );
        boost::optional<double> max_range=options.optional<double>("--max-range");
        bool raw_intensity=options.exists( "--raw-intensity" );
        bool legacy = options.exists( "--legacy");
        adjust_timestamp adjust_timestamp_functor( options.value<std::string>("--adjusted-time","")
                                                 , options.value<unsigned>("--adjusted-time-threshold", adjust_timestamp::DEFAULT_THRESHOLD )
                                                 , options.value<unsigned>("--adjusted-time-reset", adjust_timestamp::DEFAULT_RESET ) );
        snark::velodyne::calculator* calculator = NULL;
        snark::velodyne::stream* s = NULL;
        boost::scoped_ptr< snark::velodyne::hdl64::db > db;
        struct models { enum values { hdl64, puck }; };
        options.assert_mutually_exclusive( "--model,--puck,--hdl64,--64" );
        models::values model;
        std::string model_string = options.value< std::string >( "--model", "" );
        if( options.exists( "--puck" ) || model_string == "puck" || model_string == "vlp16" || model_string == "vlp-16" ) { model = models::puck; }
        else if( options.exists( "--hdl64,--64" ) || model_string == "hdl64" || model_string == "hdl-64" || model_string == "64" ) { model = models::hdl64; }
        else if( model_string.empty() ) { std::cerr << "velodyne-to-csv: please specify --model" << std::endl; return 1; }
        else { std::cerr << "velodyne-to-csv: expected model, got: \"" << model_string << "\"" << std::endl; return 1; }
        boost::optional< snark::velodyne::puck::ntp_t > ntp = comma::silent_none< snark::velodyne::puck::ntp_t >();
        switch( model )
        {
            case models::puck:
                if ( options.exists( "--ntp" ) )
                {
                    std::vector<std::string> v = comma::split(options.value<std::string>("--ntp"), ',');
                    switch (v.size())
                    {
                        case 0: ntp = snark::velodyne::puck::ntp_t(); break;
                        case 1: ntp = snark::velodyne::puck::ntp_t(boost::lexical_cast<double>(v[0])); break;
                        case 2: ntp = snark::velodyne::puck::ntp_t(boost::lexical_cast<double>(v[0]), v[1] == "permissive"); break;
                        default: std::cerr << "velodyne-to-csv: expected --ntp=[<threshold>],[permissive]; got --ntp=" << options.value<std::string>("--ntp") << std::endl; return 1;
                    }
                }
                calculator = new snark::velodyne::puck::calculator;
                if( options.exists( "--pcap" ) ) { s = new snark::velodyne::puck::stream< snark::pcap_reader >( new snark::pcap_reader, output_invalid_points, ntp ); }
                else if( options.exists( "--thin" ) ) { s = new snark::velodyne::puck::stream< snark::thin_reader >( new snark::thin_reader, output_invalid_points, ntp ); }
                else if( options.exists( "--udp-port" ) ) { s = new snark::velodyne::puck::stream< snark::udp_reader >( new snark::udp_reader( options.value< unsigned short >( "--udp-port" ) ), output_invalid_points, ntp ); }
                else if( options.exists( "--proprietary,-q" ) ) { s = new snark::velodyne::puck::stream< snark::proprietary_reader >( new snark::proprietary_reader, output_invalid_points, ntp ); }
                else { s = new snark::velodyne::puck::stream< snark::stream_reader >( new snark::stream_reader, output_invalid_points, ntp ); }
                break;
            case models::hdl64:
                db.reset( new snark::velodyne::hdl64::db( options.value< std::string >( "--db", "/usr/local/etc/db.xml" ) ) );
                if( !legacy && db->version == 0 ) { legacy=true; std::cerr << "velodyne-to-csv: using legacy option for old database" << std::endl; }
                if( legacy && db->version > 0 ) { std::cerr << "velodyne-to-csv: using new calibration with legacy option" << std::endl;}
                calculator = new snark::velodyne::db_calculator( *db );
                if( options.exists( "--pcap" ) ) { s = new snark::velodyne::hdl64::stream< snark::pcap_reader >( new snark::pcap_reader, output_invalid_points, legacy ); }
                else if( options.exists( "--thin" ) ) { s = new snark::velodyne::hdl64::stream< snark::thin_reader >( new snark::thin_reader, output_invalid_points, legacy ); }
                else if( options.exists( "--udp-port" ) ) { s = new snark::velodyne::hdl64::stream< snark::udp_reader >( new snark::udp_reader( options.value< unsigned short >( "--udp-port" ) ), output_invalid_points, legacy ); }
                else if( options.exists( "--proprietary,-q" ) ) { s = new snark::velodyne::hdl64::stream< snark::proprietary_reader >( new snark::proprietary_reader, output_invalid_points, legacy ); }
                else { s = new snark::velodyne::hdl64::stream< snark::stream_reader >( new snark::stream_reader, output_invalid_points, legacy ); }
                break;
        }
        if( options.exists( "--packet-duration" ) ) { std::cout<<s->packet_duration()<<std::endl; return 0; }
        if( options.exists( "--packet-angle" ) ) { std::cout<<s->packet_angle( options.value< unsigned int >( "--rotation-per-second", 20 ) ) << std::endl; return 0; }
        boost::optional<unsigned> threshold_n=options.optional<unsigned>("--missing-packets-threshold");
        if( threshold_n && *threshold_n == 0 ) { COMMA_THROW( comma::exception, "invalid value for --missing-packets-threshold; expected >=1, got " << *threshold_n ); }
        s->set_missing_packets_threshold( threshold_n );
        snark::velodyne_stream v( s, calculator, from, to, raw_intensity );
        comma::signal_flag is_shutdown;
        comma::csv::output_stream< snark::velodyne_point > ostream( std::cout, csv );
        static std::vector< snark::velodyne_point > points;
        uint32_t scan = 0;
        if( discard_invalid_scans ) { points.reserve( 130000 ); }
        //Profilerstart( "velodyne-to-csv.prof" );{
        while( !is_shutdown && v.read() )
        { 
            if( v.point().range < min_range ) { continue; }
            if( max_range && v.point().range > *max_range ) { continue; }
            snark::velodyne_point p = v.point();
            p.timestamp = adjust_timestamp_functor( p.timestamp );
            if( discard_invalid_scans )
            {
                if( scan != p.scan && points.size() )
                {
                    if( points.back().valid_scan && p.valid_scan ) { for( const auto& i : points ) { ostream.write( i ); } } // todo? improve performance? currently will flush on each point, which is inefficient, since it should be sufficient to flush once on the full scan
                    points.clear();
                }
                scan = p.scan;
                points.push_back( p );
            }
            else
            {
                ostream.write( p ); // todo? improve performance? currently will flush on each point, which is inefficient, since it should be sufficient to flush once on the full scan
            }
        }
        //Profilerstop(); }
        if( verbose ) { if( is_shutdown ) { std::cerr << "velodyne-to-csv: interrupted by signal" << std::endl; } else { std::cerr << "velodyne-to-csv: done, no more data" << std::endl; } }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "velodyne-to-csv: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "velodyne-to-csv: unknown exception" << std::endl; }
    return 1;
}
