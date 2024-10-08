// Copyright (c) 2021,2022 Mission Systems Pty Ltd

#include "../device.h"
#include "../messages.h"
#include "../stream.h"
#include "../traits.h"
#include "../../../math/roll_pitch_yaw.h"
#include "../../../visiting/traits.h"
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/application/verbose.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/io/select.h>
#include <comma/name_value/serialize.h>
#include <regex>
#include <thread>

namespace messages = snark::navigation::advanced_navigation::messages;

static unsigned int sleep_us = 10000;
static bool flush;

void usage( bool verbose )
{
    std::cerr << "\nconvert Advanced Navigation raw data to csv";
    std::cerr << "\n";
    std::cerr << "\nusage: <raw-data> | advanced-navigation-to-csv  <packet> [<options>]";
    std::cerr << "\n       echo <value> | advanced-navigation-to-csv  --status=<packet>";
    std::cerr << "\n       advanced-navigation-to-csv --status-description=<packet>";
    std::cerr << "\n";
    std::cerr << "\n    where <packet> selects output and is one of (packet ids in brackets):";
    std::cerr << "\n        system-state (20)";
    std::cerr << "\n        unix-time (21)";
    std::cerr << "\n        raw-sensors (28)";
    std::cerr << "\n        satellites (30)";
    std::cerr << "\n        geodetic-position (32)";
    std::cerr << "\n        acceleration (37)";
    std::cerr << "\n        euler-orientation (39)";
    std::cerr << "\n        quaternion-orientation (40)";
    std::cerr << "\n        angular-velocity (42)";
    std::cerr << "\n        filter-options (186)";
    std::cerr << "\n        magnetic-calibration (191)";
    std::cerr << "\n";
    std::cerr << "\n    or a combination or subset of packets";
    std::cerr << "\n        navigation:     navigation data from system-state packet (default)";
    std::cerr << "\n        imu:            combine unix-time and raw-sensors packets";
    std::cerr << "\n        geodetic-pose:  combine several packets as described below";
    std::cerr << "\n        all:            combine several packets as described below";
    std::cerr << "\n";
    std::cerr << "\n        packet-ids:     just display id's of all received packets";
    std::cerr << "\n";
    std::cerr << "\n        imu is a combination of unix-time(21), raw-sensors(28),";
    std::cerr << "\n        euler-orientation-std-dev(26), euler-orientation(39) and";
    std::cerr << "\n        angular-velocity(42)";
    std::cerr << "\n";
    std::cerr << "\n        geodetic-pose is a combination of ";
    std::cerr << "\n        unix-time(21), geodetic-position(32), quaternion-orientation(40)";
    std::cerr << "\n";    
    std::cerr << "\n        all is a combination of system-state(20), velocity-std-dev(25),";
    std::cerr << "\n        euler-orientation-std-dev(26), raw-sensors(28) and satellites(30).";
    std::cerr << "\n";
    std::cerr << "\n        The combination packets output on receipt of their last packet.";
    std::cerr << "\n        If you are seeing no output use the packet-ids option to see what";
    std::cerr << "\n        packets are being sent.";
    std::cerr << "\n";
    std::cerr << "\noptions:";
    std::cerr << "\n    --help,-h:         show help";
    std::cerr << "\n    --verbose,-v:      show detailed messages";
    std::cerr << "\n    --flush:           flush output stream after each write";
    std::cerr << "\n    --json:            format --status=<packet> output in json";
    std::cerr << "\n    --magnetic-calibration-description: print description table";
    std::cerr << "\n    --output-fields:   print output fields and exit";
    std::cerr << "\n    --output-format:   print output format and exit";
    std::cerr << "\n    --sleep=<microseconds>: sleep between reading, default " << sleep_us;
    std::cerr << "\n    --status=<packet>: print out expanded status bit map of input values";
    std::cerr << "\n    --status-description=<packet>: print description of status bit field";
    std::cerr << "\n";
    std::cerr << "\n    where <packet> is one of:";
    std::cerr << "\n        system_status,filter_status for --status";
    std::cerr << "\n        system_status,filter_status,gnss_fix for --status-description";
    std::cerr << "\n";
    std::cerr << "\ncsv options";
    std::cerr << comma::csv::options::usage( verbose );
    std::cerr << "\n";
    std::cerr << "\nexamples";
    if( verbose )
    {
        std::cerr << "\n    <raw-data> | advanced-navigation-to-csv all";
        std::cerr << "\n    <raw-data> | advanced-navigation-to-csv imu";
        std::cerr << "\n    <raw-data> | advanced-navigation-to-csv system-state";
        std::cerr << "\n";
        std::cerr << "\n    --- see description of system_status values ---";
        std::cerr << "\n    <raw-data> | advanced-navigation-to-csv system-state \\";
        std::cerr << "\n        | advanced-navigation-to-csv --fields system_status \\";
        std::cerr << "\n                                     --status system_status";
        std::cerr << "\n    echo 128 | advanced-navigation-to-csv  --status system_status --json";
        std::cerr << "\n";
        std::cerr << "\n    --- see description of filter_status values ---";
        std::cerr << "\n    <raw-data> | advanced-navigation-to-csv system-state \\";
        std::cerr << "\n        | advanced-navigation-to-csv --fields ,filter_status \\";
        std::cerr << "\n                                     --status filter_status";
        std::cerr << "\n    echo 1029 | advanced-navigation-to-csv --status filter_status";
        std::cerr << "\n    advanced-navigation-to-csv --status-description filter_status";
        std::cerr << "\n";
        std::cerr << "\n    --- see packet data rates (hz,id) ---";
        std::cerr << "\n    <raw-data> | timeout 2 advanced-navigation-to-csv packet-ids \\";
        std::cerr << "\n        | csv-paste - value=0 | csv-calc --fields id size \\";
        std::cerr << "\n        | csv-eval --fields c --format d 'c*=0.5'";
        std::cerr << "\n";
        std::cerr << "\n  where <raw-data> is coming from advanced-navigation-cat or similar";
    }
    else
    {
        std::cerr << "\n    run --help --verbose for more csv options and examples...";
    }
    std::cerr << "\n" << std::endl;
}

static void bash_completion()
{
    std::cout << "--help --verbose"
              << " system-state unix-time raw-sensors satellites magnetic-calibration navigation all packet-ids"
              << " --output-fields --output-format --send --json"
              << " --magnetic-calibration-description --status --status-description"
              << std::endl;
}

// Packet ID field from message header. See §13.2 of the Spatial FOG Dual Reference Manual
struct output_packet_id
{
    output_packet_id() : packet_id( 0 ) {}
    uint8_t packet_id;
};

struct output_nav
{
    output_nav() : height(0), system_status(0), filter_status(0) {}
    boost::posix_time::ptime t;
    snark::spherical::coordinates coordinates;
    double height;
    snark::roll_pitch_yaw orientation;
    uint16_t system_status;
    uint16_t filter_status;
};

struct output_geodetic_pose
{
    output_geodetic_pose() {}
    messages::unix_time unix_time;
    messages::geodetic_position geodetic_position;
    messages::quaternion_orientation quaternion_orientation;
};

struct output_imu
{
    output_imu() {}
    messages::unix_time unix_time;
    messages::raw_sensors raw_sensors;
    Eigen::Vector3f orientation_stddev;
    messages::euler_orientation orientation;
    messages::angular_velocity angular_velocity;
};

struct output_all
{
    output_all() : velocity_stddev( 0, 0, 0 ), orientation_stddev( 0, 0, 0 ) {}
    messages::system_state system_state;
    messages::raw_sensors raw_sensors;
    Eigen::Vector3f velocity_stddev;
    Eigen::Vector3f orientation_stddev;
    messages::satellites satellites;
};

struct status_data
{
    uint16_t status;
};

namespace comma { namespace visiting {

template < unsigned int S, bool P, bool F, std::size_t N > struct traits< boost::array< comma::packed::detail::endian< comma::packed::detail::big, S, P, F >, N > >
{
    template< typename K, typename V > static void visit( const K& k, const boost::array< comma::packed::detail::endian< comma::packed::detail::big, S, P, F >, N >& t, V& v )
    {
        for( std::size_t i = 0; i < t.size(); i++ ) { v.apply( i, t[i]() ); }
    }
};

template <> struct traits< output_packet_id >
{
    template < typename Key, class Visitor > static void visit( const Key&, const output_packet_id& p, Visitor& v ) { v.apply( "packet_id", p.packet_id ); }
    template < typename Key, class Visitor > static void visit( const Key&, output_packet_id& p, Visitor& v ) { v.apply( "packet_id", p.packet_id ); }
};

template <> struct traits< output_nav >
{
    template < typename Key, class Visitor > static void visit( const Key&, const output_nav& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "coordinates", p.coordinates );
        v.apply( "height", p.height );
        v.apply( "orientation", p.orientation );
        v.apply( "system_status", p.system_status );
        v.apply( "filter_status", p.filter_status );
    }

    template < typename Key, class Visitor > static void visit( const Key&, output_nav& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "coordinates", p.coordinates );
        v.apply( "height", p.height );
        v.apply( "orientation", p.orientation );
        v.apply( "system_status", p.system_status );
        v.apply( "filter_status", p.filter_status );
    }
};

template <> struct traits< output_geodetic_pose >
{
    template < typename Key, class Visitor > static void visit( const Key&, const output_geodetic_pose& p, Visitor& v )
    {
        v.apply( "", p.unix_time );
        v.apply( "geodetic_position", p.geodetic_position );
        v.apply( "orientation", p.quaternion_orientation );
    }
};

template <> struct traits< output_imu >
{
    template < typename Key, class Visitor > static void visit( const Key&, const output_imu& p, Visitor& v )
    {
        v.apply( "", p.unix_time );
        v.apply( "", p.raw_sensors );
        v.apply( "orientation", p.orientation );
        v.apply( "orientation_stddev", p.orientation_stddev );
        v.apply( "angular_velocity", p.angular_velocity );
    }
};

template <> struct traits< output_all >
{
    template < typename Key, class Visitor > static void visit( const Key&, const output_all& p, Visitor& v )
    {
        v.apply( "", p.system_state );
        v.apply( "", p.raw_sensors );
        v.apply( "velocity_stddev", p.velocity_stddev );
        v.apply( "orientation_stddev", p.orientation_stddev );
        v.apply( "", p.satellites );
    }
};

template <> struct traits< status_data >
{
    template < typename Key, class Visitor > static void visit( const Key&, const status_data& p, Visitor& v ) { v.apply( "status", p.status ); }
    template < typename Key, class Visitor > static void visit( const Key&, status_data& p, Visitor& v ) { v.apply( "status", p.status ); }
};

} } // namespace comma { namespace visiting {

struct app_i
{
    virtual ~app_i() {}
    virtual void run() = 0;
    virtual void output_fields() = 0;
};

struct app_base : public snark::navigation::advanced_navigation::device
{
    comma::io::select select;
    comma::signal_flag signaled;

    app_base() : device( "-" ) { select.read().add( fd() ); }

    void process()
    {
        while( !signaled && std::cout.good() )
        {
            select.wait( boost::posix_time::microseconds( sleep_us ));
            if( !signaled && select.read().ready( fd() ) ) { device::process(); }
        }
    }

    void handle( const messages::acknowledgement* msg ) { comma::say() << msg->result_msg() << std::endl; }
};

template< typename T > struct app_t : public app_base
{
    comma::csv::output_stream< T > os;

    app_t( const comma::command_line_options& options ): os( std::cout, comma::csv::options( options, "", true )) {}
    static void output_fields() { std::cout << comma::join( comma::csv::names< T >( true ), ',' ) << std::endl; }
    static void output_format() { std::cout << comma::csv::format::value< T >() << std::endl; }
};

struct app_packet_id : public app_t< output_packet_id >
{
    app_packet_id( const comma::command_line_options& options ) : app_t( options ) {}
    void handle_raw( messages::header* msg_header, const char* msg_data, std::size_t msg_data_length ) { std::cout << (unsigned int)msg_header->id() << std::endl; }
};

// ------------------------- navigation ------------------------
//
struct app_nav : public app_t< output_nav >
{
    app_nav( const comma::command_line_options& options ) : app_t( options ) {}
    void handle( const messages::system_state* msg )
    {
        output_nav o;
        o.t = msg->t();
        o.coordinates.latitude = msg->latitude();
        o.coordinates.longitude = msg->longitude();
        o.height = msg->height();
        o.orientation = snark::roll_pitch_yaw( msg->orientation[0](), msg->orientation[1](), msg->orientation[2]() );
        o.system_status = msg->system_status();
        o.filter_status = msg->filter_status();
        os.write( o );
        if( flush ) { os.flush(); }
    }
};

// ------------------------- geodetic_pose ------------------------
//
// configure packets to output at the same rate, then packets are received in
// order of lowest to highest packet id
struct app_geodetic_pose : public app_t< output_geodetic_pose >
{
    output_geodetic_pose output;
    app_geodetic_pose( const comma::command_line_options& options ): app_t( options ) {}
    // Unix Time, packet id (21)
    void handle( const messages::unix_time* msg ) { std::memcpy( output.unix_time.data(), msg->data(), messages::unix_time::size ); }
    // Geodetic Position, packet id (32)
    void handle( const messages::geodetic_position* msg ) { std::memcpy( output.geodetic_position.data(), msg->data(), messages::geodetic_position::size ); }
    // Quaternion Orientation, packet id (40)
    void handle( const messages::quaternion_orientation* msg )
    {
        std::memcpy( output.quaternion_orientation.data(), msg->data(), messages::quaternion_orientation::size );
        os.write( output );
        if( flush ) { os.flush(); }
    } 
};

// ---------------------------- imu ----------------------------
//
// accumulate unix-time and raw-sensors packets into one output record
//
// configure packets to output at the same rate, then packets are received in
// order of lowest to highest packet id
struct app_imu : public app_t< output_imu >
{
    output_imu output;

    app_imu( const comma::command_line_options& options ): app_t( options ) {}

    // Unix Time, packet id 21
    void handle( const messages::unix_time* msg ) { std::memcpy( output.unix_time.data(), msg->data(), messages::unix_time::size ); }

    // Euler Orientation Std Dev, packet id 26
    void handle( const messages::orientation_standard_deviation* msg ) { output.orientation_stddev = Eigen::Vector3f( msg->stddev[0](), msg->stddev[1](), msg->stddev[2]() ); }

    // Raw Sensors, packet id 28
    void handle( const messages::raw_sensors* msg ) { std::memcpy( output.raw_sensors.data(), msg->data(), messages::raw_sensors::size ); }

    // Euler Orientation, packet id 39
    void handle( const messages::euler_orientation* msg ) { std::memcpy( output.orientation.data(), msg->data(), messages::euler_orientation::size ); }

    // Angular Velocity, packet id 42
    void handle( const messages::angular_velocity* msg )
    {
        std::memcpy( output.angular_velocity.data(), msg->data(), messages::angular_velocity::size );
        os.write( output );
        if( flush ) { os.flush(); }
    }
};

// ---------------------------- all ----------------------------
//
// accumulate several packets into one big output record
//
// Configure packets to output at the same rate, then packets are received in
// order of lowest to highest packet id. We output when we received the last packet.
// All data is then guarenteed to align with the timestamp in the system state packet.
// See section 13.6 of the Reference Manual for details.
struct app_all : public app_t< output_all >
{
    output_all output;

    app_all( const comma::command_line_options& options ): app_t( options ) {}
    void handle( const messages::system_state* msg ) { std::memcpy( output.system_state.data(), msg->data(), messages::system_state::size ); }
    void handle( const messages::velocity_standard_deviation* msg ) { output.velocity_stddev = Eigen::Vector3f( msg->stddev[0](), msg->stddev[1](), msg->stddev[2]() ); }
    void handle( const messages::orientation_standard_deviation* msg ) { output.orientation_stddev = Eigen::Vector3f( msg->stddev[0](), msg->stddev[1](), msg->stddev[2]() ); }
    void handle( const messages::raw_sensors* msg ) { std::memcpy( output.raw_sensors.data(), msg->data(), messages::raw_sensors::size ); }
    void handle( const messages::satellites* msg )
    {
        std::memcpy( output.satellites.data(), msg->data(), messages::satellites::size );
        os.write( output );
        if( flush ) { os.flush(); }
    }
};

// ----------------------- single packets ----------------------
//
template < typename T > struct app_packet : public app_t< T >
{
    app_packet( const comma::command_line_options& options ) : app_t< T >( options ) {}

    void handle( const T* msg )
    {
        app_t< T >::os.write( *msg );
        if( flush ) { app_t< T >::os.flush(); }
    }
};

struct factory_i
{
    virtual ~factory_i() {}
    virtual void output_fields() = 0;
    virtual void output_format() = 0;
    virtual void run( const comma::command_line_options& options ) = 0;
};

template < typename T > struct factory_t : public factory_i
{
    typedef T type;
    void output_fields() { T::output_fields(); }
    void output_format() { T::output_format(); }
    void run( const comma::command_line_options& options )
    {
        T app( options );
        app.process();
    }
};

template < typename T > struct full_description
{
    comma::csv::input_stream< status_data > is;
    bool json{false};

    full_description( const comma::command_line_options& options ): is( std::cin, comma::csv::options( options )), json( options.exists( "--json" )) {}

    void process()
    {
        while( std::cin.good() )
        {
            const status_data* p = is.read();
            if( !p ) { break; }
            T description( p->status );
            boost::property_tree::ptree ptree;
            comma::to_ptree to_ptree( ptree, comma::xpath() );
            comma::visiting::apply( to_ptree ).to( description );
            std::cout.precision( 16 ); // quick and dirty
            if( json )
            {
                boost::property_tree::write_json( std::cout, ptree, false ); // comma::write_json( description, std::cout );
            }
            else
            {
                std::string s = comma::property_tree::to_path_value_string( ptree, comma::property_tree::disabled, '=', ';' ); // comma::write_path_value( description, std::cout );
                std::cout << std::regex_replace( s, std::regex( "\"([0-9]*)\"" ), "$1" ) << std::endl;
            }
        }
    }
};

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" )) { bash_completion(); return 0; }
        std::vector< std::string > unnamed = options.unnamed( comma::csv::options::valueless_options() + ",--verbose,-v,--output-fields,--output-format,--flush,--json", "-.*" );
        flush = options.exists( "--flush" );
        sleep_us = options.value< unsigned int >( "--sleep", sleep_us );
        auto opt_full_description = options.optional< std::string >( "--status" );
        if( opt_full_description )
        {
            if( *opt_full_description == "system_status" ) { full_description< messages::system_status_description >( options ).process(); }
            else if( *opt_full_description == "filter_status" ) { full_description< messages::filter_status_description >( options ).process(); }
            else { COMMA_THROW( comma::exception, "invalid field for --status. expected 'system_status' or 'filter_status', got " << *opt_full_description ); }
            return 0;
        }
        auto opt_status_description = options.optional< std::string >( "--status-description" );
        if( opt_status_description )
        {
            if( *opt_status_description == "system_status" ) { messages::system_status_description::description( std::cout ); }
            else if( *opt_status_description == "filter_status" ) { messages::filter_status_description::description( std::cout ); }
            else if( *opt_status_description == "gnss_fix" ) { messages::filter_status_description::gnss_fix_description( std::cout ); }
            else { COMMA_THROW( comma::exception, "invalid field for --status-description. expected 'system_status' or 'filter_status' or 'gnss_fix', got " << *opt_status_description ); }
            return 0;
        }
        if( options.exists( "--magnetic-calibration-description" )) { messages::magnetic_calibration_status::status_description( std::cout ); return 0; }
        std::unique_ptr< factory_i > factory;
        COMMA_ASSERT_BRIEF( unnamed.size() == 1, "expected one packet name, got: " << unnamed.size() );
        std::string packet = unnamed[0];
        if( packet == "navigation" ) { factory.reset( new factory_t< app_nav >() ); }
        else if( packet == "all" ) { factory.reset( new factory_t< app_all >() ); }
        else if( packet == "imu" ) { factory.reset( new factory_t< app_imu >() ); }
        else if( packet == "geodetic-pose" ) { factory.reset( new factory_t< app_geodetic_pose >() ); }
        else if( packet == "packet-ids" ) { factory.reset( new factory_t< app_packet_id >() ); }
        else if( packet == "system-state" ) { factory.reset( new factory_t< app_packet <messages::system_state > >() ); }
        else if( packet == "unix-time" ) { factory.reset( new factory_t< app_packet <messages::unix_time > >() ); }
        else if( packet == "raw-sensors" ) { factory.reset( new factory_t< app_packet< messages::raw_sensors > >() ); }
        else if( packet == "satellites" ) { factory.reset( new factory_t< app_packet< messages::satellites > >() ); }
        else if( packet == "geodetic-position" ) { factory.reset( new factory_t< app_packet< messages::geodetic_position > >() ); }
        else if( packet == "acceleration" ) { factory.reset( new factory_t< app_packet< messages::acceleration > >() ); }
        else if( packet == "euler-orientation" ) { factory.reset( new factory_t< app_packet< messages::euler_orientation > >() ); }
        else if( packet == "quaternion-orientation" ) { factory.reset( new factory_t< app_packet< messages::quaternion_orientation > >() ); }
        else if( packet == "angular-velocity" ) { factory.reset( new factory_t< app_packet< messages::angular_velocity > >() ); }
        else if( packet == "external-time" ) { factory.reset( new factory_t< app_packet <messages::external_time > >() ); }
        else if( packet == "filter-options" ) { factory.reset( new factory_t< app_packet< messages::filter_options > >() ); }
        else if( packet == "magnetic-calibration" ) { factory.reset( new factory_t< app_packet< messages::magnetic_calibration_status > >() ); }
        else { COMMA_THROW( comma::exception, "unsupported packet '" << packet << "; see --help for more details" );}
        if( options.exists( "--output-fields" )) { factory->output_fields(); return 0; }
        if( options.exists( "--output-format" )) { factory->output_format(); return 0; }
        comma::csv::detail::unsynchronize_with_stdio();
        factory->run( options );
        return 0;
    }
    catch( snark::navigation::advanced_navigation::eois_exception& e ) { comma::verbose<<comma::verbose.app_name() << ": " << e.what() << std::endl; return 0; }
    catch( std::exception& ex ) { comma::say() << ex.what() << std::endl; }
    catch( ... ) { comma::say() << "unknown exception" << std::endl; }
    return 1;
}
