// Copyright (c) 2018-2022 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <fstream>
#include <memory>
#include <comma/application/command_line_options.h>
#include <comma/base/types.h>
#include <comma/csv/names.h>
#include <comma/csv/stream.h>
#include <comma/io/stream.h>
#include "../../../../timing/time.h"
#include "../../../../visiting/traits.h"
#include "../calculator.h"
#include "../packet.h"

// todo
//   - helios
//     ! corrected azimuth angles from file: plug in
//     ! helios-5515 vs helios-16p: model is incorrect; tweak enum? add --force? don't check model consistency? rename helios_16p to helios_5515?
//     ! range resolution! helios-16p: from msop; lidar-16: from difop: add as (optional?) parameter of make_calculator? or template main loop on model?
//     ! template main loop on model; currently hard-coded to lidar_16, which works so far, but is not right
//     ! resolve model on helios::models
//     ! helios::models: plug in
//     ? exact laser point timing, table 12, page 26
//       - currently implied from block timestamps and firing interval (100us for both models) in packet.cpp
//       - lidar-16: 55.5us; Time_offset = 55.5 μs * (sequence_index -1) + 2.8 μs * (data_index-1)
//       - helios-16p: 55.56us; no formula, only tabular
//   ? axis directions; frame -> n-e-d
//   ? temperature from difop
//   ? curves from difop
//   ? move msop::packet::const_iterator to calculator
//   ? --distance-resolution: 1cm, 0.5cm, etc.
//   ? 32-beam support?
//   ? move difop stream to calculator?

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "usage: cat robosense*.bin | robosense-to-csv [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "limitations" << std::endl;
    std::cerr << "    only rs-lidar-16 currently supported" << std::endl;
    std::cerr << "    reflectivity curves not implemented yet" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --model=<name>; default=auto; if the data packets have a different model ids, exception will be thrown" << std::endl;
    std::cerr << "                    <name>: 'auto' or model name; run robosense-to-csv --models for list of available models" << std::endl;
    std::cerr << "    --models; print list of models and their numeric ids in msop packet header and " << std::endl;
    std::cerr << "              exit (lidar-16 currently is supported; helios-16p: implementation in progress)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "calibration options" << std::endl;
    std::cerr << "    --calibration,-c=<directory>; directory containing calibration files: angle.csv, ChannelNum.csv, curves.csv etc" << std::endl;
    std::cerr << "    --calibration-angles,--angles,--angle,-a=<filename>; default: as in spec" << std::endl;
    std::cerr << "    --calibration-azimuth-output,--output-azimuth=<how>; output azimuth correction angles to stdout" << std::endl;
    std::cerr << "        if --difop present, take horizontal angles from difop packet" << std::endl;
    std::cerr << "        <how>: 'degrees' or 'radians'" << std::endl;
    std::cerr << "    --calibration-elevation-output,--output-elevation=<how>; output vertical angles to stdout" << std::endl;
    std::cerr << "        --calibration-angles-output,--output-calibration-angles,--output-angles: deprecated" << std::endl;
    std::cerr << "        if --difop present, take vertical angles from difop packet" << std::endl;
    std::cerr << "        <how>: 'degrees' or 'radians'" << std::endl;
    std::cerr << "    --calibration-channels,--channels=<filename>; default: 450 (i.e. 4.50cm) for all channels" << std::endl;
    std::cerr << "    --range-resolution,--resolution=<metres>; default=0.005, but you most likely will want 0.005 (alternatively, --difop will contain range resolution" << std::endl;
    std::cerr << "    --output-range-resolution; if --difop present, output resolution as difop says; otherwise output default resolution" << std::endl;
    std::cerr << std::endl;
    std::cerr << "difop options" << std::endl;
    std::cerr << "    --difop=[<path>]; file or stream containing timestamped difop packets; if present, calibration data will be taken from difop packets" << std::endl;
    std::cerr << "                      currently only calibrated vertical angles are supported" << std::endl;
    std::cerr << "                      '-' means difop packets are on stdin (makes sense only with --output-angles or alike" << std::endl;
    std::cerr << "    --difop-max-number-of-packets,--difop-max=<num>; max number of difop packets to read; if not specified, read till the end of file/stream" << std::endl;
    std::cerr << "    --force; use if robosense-to-csv suggests it" << std::endl;
    std::cerr << std::endl;
    std::cerr << "output options:" << std::endl;
    std::cerr << "    --binary,-b[=<format>]: output in binary equivalent of csv" << std::endl;
    std::cerr << "    --fields <fields>: e.g. t,x,y,z,scan" << std::endl;
    std::cerr << "    --output-fields: todo: print output fields and exit" << std::endl;
    std::cerr << "    --output-invalid-points: output invalid points" << std::endl;
    std::cerr << "    --scan-discard-incomplete,--discard-incomplete-scans,--discard-incomplete: don't output scans with missing packets" << std::endl;
    std::cerr << "    --scan-max-missing-packets,--missing-packets=<n>; default 5; number of consecutive missing packets for new/invalid scan (as a rule of thumb: roughly at 20rpm 50 packets per revolution)" << std::endl;
    std::cerr << "    --temperature,-t=<celcius>; default=20; integer from 0 to 39" << std::endl;
    std::cerr << std::endl;
    std::cerr << "csv options" << std::endl;
    std::cerr << comma::csv::options::usage( verbose ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    data" << std::endl;
    std::cerr << "        default config" << std::endl;
    std::cerr << "            cat timestamped-msop.bin | robosense-to-csv" << std::endl;
    std::cerr << "        config from difop" << std::endl;
    std::cerr << "            cat timestamped-msop.bin | robosense-to-csv --difop timestamped-difop.bin" << std::endl;
    std::cerr << "        config from calibration directory" << std::endl;
    std::cerr << "            cat timestamped-msop.bin | robosense-to-csv --calibration my-calibration-dir" << std::endl;
    std::cerr << "        configure vertical pitch only" << std::endl;
    std::cerr << "            cat timestamped-msop.bin | robosense-to-csv --calibration-angles angles.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    difop" << std::endl;
    std::cerr << "        output elevation angles, if present in difop" << std::endl;
    std::cerr << "            cat timestamped-difop.bin | robosense-to-csv --difop - --output-elevation=radians > angles.csv" << std::endl;
    std::cerr << "        output elevation angles, if present in difop, otherwise default angles" << std::endl;
    std::cerr << "            cat timestamped-difop.bin  | robosense-to-csv --difop - --output-elevation=radians --force > angles.csv" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

namespace comma { namespace visiting {
    
template <> struct traits< snark::robosense::calculator::point >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::robosense::calculator::point& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "scan", p.scan );
        v.apply( "id", p.id );
        v.apply( "range", p.range );
        v.apply( "bearing", p.bearing );
        v.apply( "elevation", p.elevation );
        v.apply( "reflectivity", p.reflectivity );
        v.apply( "coordinates", p.coordinates );
    }
};

} } // namespace comma { namespace visiting {

template < typename P > static std::pair< boost::posix_time::ptime, P* > read( std::istream& is, char* buffer )
{
    comma::uint64 microseconds;
    std::pair< boost::posix_time::ptime, P* > p( boost::posix_time::not_a_date_time, NULL );
    is.read( reinterpret_cast< char* >( &microseconds ), sizeof( comma::uint64 ) );
    if( is.gcount() < int( sizeof( comma::uint64 ) ) || is.bad() || is.eof() ) { return p; }
    is.read( buffer, P::size );
    if( is.gcount() < int( P::size ) || is.bad() || is.eof() ) { return p; }
    comma::uint64 seconds = microseconds / 1000000; //to avoid time overflow on 32bit systems with boost::posix_time::microseconds( m_microseconds ), apparently due to a bug in boost
    microseconds = microseconds % 1000000;
    static boost::posix_time::ptime epoch( snark::timing::epoch );
    p.first = epoch + boost::posix_time::seconds( seconds ) + boost::posix_time::microseconds( microseconds );
    p.second = reinterpret_cast< P* >( buffer );
    return p;
}

template < snark::robosense::models::values Model > struct model_traits; // todo? move to packets?

template <> struct model_traits< snark::robosense::models::lidar_16 >
{
    typedef snark::robosense::lidar_16 lidar;
    static double range_resolution( const snark::robosense::lidar_16::difop::packet& p ) { return p.data.range_resolution(); } // quick and dirty
    static double range_resolution_default() { return 0.01; } // todo! sort it out! quick and dirty for now
    
    static std::array< double, snark::robosense::msop::data::number_of_lasers > corrected_horizontal_angles( const snark::robosense::lidar_16::difop::packet& ) { return snark::robosense::lidar_16::difop::data::corrected_horizontal_angles_default(); }
};

template <> struct model_traits< snark::robosense::models::helios_16p >
{
    typedef snark::robosense::helios_16p lidar;
    static double range_resolution( const snark::robosense::helios_16p::difop::packet& p ) { return 0.005; } // quick and dirty, just to make it compiling for now
    static double range_resolution_default() { return 0.005; } // todo! sort it out! quick and dirty for now
    static std::array< double, snark::robosense::msop::data::number_of_lasers > corrected_horizontal_angles( const snark::robosense::helios_16p::difop::packet& p ) { return p.data.corrected_horizontal_angles.as_radians(); }
};

template < snark::robosense::models::values Model >
static snark::robosense::calculator make_calculator( const comma::command_line_options& options, const typename model_traits< Model >::lidar::msop::packet* msop_packet = nullptr ) // todo? quick and dirty; move to calculator?
{
    typedef typename model_traits< Model >::lidar lidar_t;
    typedef typename lidar_t::difop difop_t;
    options.assert_mutually_exclusive( "--calibration,-c", "--calibration-angles,--angles,--angle,-a" );
    options.assert_mutually_exclusive( "--calibration,-c", "--calibration-channels,--channels" );
    options.assert_mutually_exclusive( "--difop", "--calibration-angles,--angles,--angle,-a" );
    options.assert_mutually_exclusive( "--difop", "--calibration-channels,--channels" );
    options.assert_mutually_exclusive( "--difop", "--calibration,-c" );
    std::string calibration = options.value< std::string >( "--calibration,-c", "" );
    std::string angles = options.value< std::string >( "--calibration-angles,--angles,--angle,-a", "" );
    std::string channels = options.value< std::string >( "--calibration-channels,--channels", "" );
    std::string difop = options.value< std::string >( "--difop", "" );
    options.assert_mutually_exclusive( "" ); // why?
    if( difop.empty() )
    {
        auto range_resolution = options.optional< double >( "--range-resolution,--resolution" );
        if( calibration.empty() )
        {
            if( !angles.empty() ) { comma::say() << "config: angles from --angles" << std::endl; }
            if( !channels.empty() ) { comma::say() << "config: channels from --calibration-channels" << std::endl; }
            return snark::robosense::calculator::make< lidar_t >( "", angles, channels, range_resolution ); // todo! pass azimult filename!
        }
        comma::say() << "config from calibration directory: " << calibration << std::endl;
        return snark::robosense::calculator::make< lidar_t >( "", calibration + "/angle.csv", calibration + "/ChannelNum.csv", range_resolution ); // todo! pass azimult filename!
    }
    comma::say() << "config from difop" << std::endl;
    unsigned int difop_max_number_of_packets = options.value( "--difop-max-number-of-packets,--difop-max", 0 );
    typedef std::pair< boost::posix_time::ptime, typename difop_t::packet* > pair_t;
    comma::io::istream is( difop );
    std::vector< char > buffer( difop_t::packet::size );
    unsigned int count = 0;
    unsigned int difop_count = 0;
    pair_t p( boost::posix_time::ptime(), NULL );
    for( count = 0; is->good() && !is->eof() && ( difop_max_number_of_packets == 0 || count < difop_max_number_of_packets ); ++count )
    {
        pair_t q = read< typename difop_t::packet >( *is, &buffer[0] );
        if( !q.second ) { break; }
        if( q.second->header.valid() )
        {
            ++difop_count;
            if( q.second->data.corrected_vertical_angles.empty() ) { continue; }
            p = q;
            break;
        }
    }
    if( !p.second )
    { 
        comma::say() << "got no non-zero corrected vertical angles in " << difop_count << " DIFOP packet(s) (total packet count: " << count << ") in '" << difop << "'" << std::endl;
        if( options.exists( "--force" ) )
        {
            comma::say() << "using defaults for corrected vertical angles" << std::endl;
            return snark::robosense::calculator( difop_t::data::corrected_horizontal_angles_default(), difop_t::data::corrected_vertical_angles_default(), lidar_t::range_resolution_default() );
        }
        comma::say() << "use --force to override (defaults will be used)" << std::endl;
        exit( 1 );
    }
    comma::say() << "got DIFOP data in packet " << count << " in '" << difop << "'" << std::endl;
    std::array< double, snark::robosense::msop::data::number_of_lasers > elevation;
    for( unsigned int i = 0; i < elevation.size(); ++i ) { elevation[i] = p.second->data.corrected_vertical_angles.as_radians( i ); }
    return snark::robosense::calculator( model_traits< Model >::corrected_horizontal_angles( *p.second ), elevation, lidar_t::range_resolution( p.second, msop_packet ) );
}

static comma::csv::options csv;
static boost::optional< snark::robosense::calculator > calculator;
static unsigned int temperature;
static bool output_invalid_points;

static snark::robosense::calculator make_calculator( snark::robosense::models::values model, const comma::command_line_options& options, const void* msop_packet = nullptr ) // todo? quick and dirty; move to calculator?
{
    switch( model )
    {
        case snark::robosense::models::lidar_16: return make_calculator< snark::robosense::models::lidar_16 >( options, reinterpret_cast< const snark::robosense::lidar_16::msop::packet* >( msop_packet ) );
        case snark::robosense::models::lidar_32: 
        case snark::robosense::models::bpearl:
        case snark::robosense::models::ruby:
        case snark::robosense::models::ruby_lite:
            COMMA_THROW( comma::exception, "model \"" << snark::robosense::models::to_string( model ) << "\": not implemented" );
        case snark::robosense::models::helios: return make_calculator< snark::robosense::models::helios_16p >( options, reinterpret_cast< const snark::robosense::helios_16p::msop::packet* >( msop_packet ) ); // todo: resolve model on helios::models
    }
    COMMA_THROW( comma::exception, "never here" );
}

void write( const boost::posix_time::ptime& t, const snark::robosense::lidar_16::msop::packet* p, comma::uint32 id )
{
    static comma::csv::output_stream< snark::robosense::calculator::point > ostream( std::cout, csv );
    for( snark::robosense::msop::const_iterator it( p->data ); !it.done() && std::cout.good(); ++it )
    { 
        if( it->valid() || output_invalid_points ) { ostream.write( calculator->make_point( id, t, it, temperature ) ); }
    }
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< snark::robosense::calculator::point >( false ), ',' ) << std::endl; return 0; }
        if( options.exists( "--models" ) ) { for( const auto& m: snark::robosense::models::names ) { std::cout << m.first << "," << m.second << std::endl; } return 0; }
        output_invalid_points = options.exists( "--output-invalid-points" );
        std::vector< char > buffer( snark::robosense::lidar_16::msop::packet::size ); // quick and dirty for now; all packet sizes are the same (i hope)
        boost::optional< snark::robosense::models::values > model; // preparing for model autodetect
        std::string model_name = options.value< std::string >( "--model", "auto" );
        if( model_name != "auto" ) { model = snark::robosense::models::from_string( model_name ); calculator = make_calculator( *model, options ); }
        if( options.exists( "--output-range-resolution" ) )
        { 
            if( calculator ) { std::cout << calculator->range_resolution() << std::endl; return 0; }
            comma::say() << "--output-range-resolution: please specify --model explicitly"  << std::endl; return 1;
        }
        if( options.exists( "--calibration-azimuth-output,--output-azimuth" ) )
        {
            if( !calculator ) { comma::say() << "--calibration-azimuth-output: please specify --model explicitly"  << std::endl; return 1; }
            const auto& how = options.value< std::string >( "--calibration-azimuth-output,--output-azimuth" );
            double factor = 0;
            if( how == "radians" ) { factor = 1; }
            else if( how == "degrees" ) { factor = 180. / M_PI; }
            else
            { 
                comma::say() << "please specify --calibration-azimuth-output=degrees or --calibration-azimuth-output=radians"  << std::endl;
                comma::say() << "ATTENTION: angles in the configuration file (e.g. angle.csv) should be in DEGREES"  << std::endl;
                return 1;
            }
            for( auto a: calculator->azimuth() ) { std::cout << ( a * factor ) << std::endl; }
            return 0;
        }
        if( options.exists( "--calibration-angles-output,--output-calibration-angles,--output-angles,--calibration-elevation-output,--output-elevation" ) )
        {
            if( options.exists( "--calibration-angles-output,--output-calibration-angles,--output-angles" ) ) { std::cerr << "--calibration-angles-output,--output-calibration-angles,--output-angles: deprecated; use: --calibration-elevation-output,--output-elevation" << std::endl; }
            if( !calculator ) { comma::say() << "--calibration-elevation-output: please specify --model explicitly"  << std::endl; return 1; }
            const auto& how = options.value< std::string >( "--calibration-angles-output,--output-calibration-angles,--output-angles,--calibration-elevation-output,--output-elevation" );
            double factor = 0;
            if( how == "radians" ) { factor = 1; }
            else if( how == "degrees" ) { factor = 180. / M_PI; }
            else
            { 
                comma::say() << "please specify --calibration-elevation-output=degrees or --calibration-elevation-output=radians"  << std::endl;
                comma::say() << "ATTENTION: angles in the configuration file (e.g. angle.csv) should be in DEGREES"  << std::endl;
                return 1;
            }
            for( auto a: calculator->elevation() ) { std::cout << ( a * factor ) << std::endl; }
            return 0;
        }
        temperature = options.value( "--temperature,-t", 20 );
        if( temperature > 40 ) { comma::say() << "expected temperature between 0 and 40; got: " << temperature << std::endl; return 1; }
        snark::robosense::calculator::scan scan( options.value( "--scan-max-missing-packets,--missing-packets", 10 ) );
        bool discard_incomplete_scans = options.exists( "--scan-discard-incomplete,--discard-incomplete-scans,--discard-incomplete" );
        csv = comma::csv::options( options );
        csv.full_xpath = false;
        std::vector< std::pair< boost::posix_time::ptime, snark::robosense::lidar_16::msop::packet > > scan_buffer; //std::vector< std::pair< boost::posix_time::ptime, std::array< char, snark::robosense::msop::packet::size > > > scan_buffer;
        if( discard_incomplete_scans ) { scan_buffer.reserve( 120 ); } // quick and dirty
        while( std::cin.good() && !std::cin.eof() )
        {
            auto p = read< snark::robosense::lidar_16::msop::packet >( std::cin, &buffer[0] ); // todo?! template on the packet type?!
            if( !p.second ) { break; }
            if( !snark::robosense::msop::valid( *p.second ) ) { continue; }
            auto current_model = snark::robosense::msop::detect_model( p.second->header.data() );            
            if( !model )
            {
                model = current_model;
                comma::say() << "auto-detected model: \"" << snark::robosense::models::to_string( *model ) << "\" (model numeric id: " << int( *model ) << ")" << std::endl;
                calculator = make_calculator( *model, options, ( void* )( p.second ) );
            }
            // todo: uncomment once we know the answer about helios-16p glitch: if( current_model != *model ) { COMMA_THROW( comma::exception, "expected packet for model \"" << snark::robosense::models::to_string( *model ) << "\"; got: \"" << snark::robosense::models::to_string( current_model )<< "\"" ); }
            scan.update( p.first, p.second->data );
            if( discard_incomplete_scans )
            {
                if( scan.current().is_new() )
                {
                    if( scan.is_complete( scan.last() ) ) { for( const auto& b: scan_buffer ) { write( b.first, &b.second, scan.last().id ); } }
                    scan_buffer.clear();
                }
                scan_buffer.push_back( std::pair< boost::posix_time::ptime, snark::robosense::lidar_16::msop::packet >() );
                scan_buffer.back().first = p.first;
                scan_buffer.back().second = *p.second;
            }
            else
            {
                write( p.first, p.second, scan.current().id );
            }
        }
        if( discard_incomplete_scans )
        {
            if( scan.is_complete( scan.current() ) ) { for( const auto& b: scan_buffer ) { write( b.first, &b.second, scan.last().id ); } }
            scan_buffer.clear();
        }
        return 0;
    }
    catch( std::exception& ex ) { comma::say() << "" << ex.what() << std::endl; }
    catch( ... ) { comma::say() << "unknown exception" << std::endl; }
    return 1;
}
