// Copyright (c) 2021 Mission Systems Pty Ltd

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
    std::cerr << "\nusage: <raw-data> | " << comma::verbose.app_name() << " <packet> [<options>]";
    std::cerr << "\n       echo <value> | " << comma::verbose.app_name() << " --status=<packet>";
    std::cerr << "\n       " << comma::verbose.app_name() << " --status-description=<packet>";
    std::cerr << "\n";
    std::cerr << "\n    where <packet> selects output and is one of (packet ids in brackets):";
    std::cerr << "\n        system-state:   system state packet (20)";
    std::cerr << "\n        raw-sensors:    raw sensors packet (28)";
    std::cerr << "\n        satellites:     satellites packet (30)";
    std::cerr << "\n        filter-options: filter options packet (186)";
    std::cerr << "\n        magnetic-calibration: magnetic calibration status packet (191)";
    std::cerr << "\n        navigation:     navigation data from system-state packet (default)";
    std::cerr << "\n        all:            combine several packets as described below";
    std::cerr << "\n        packet-ids:     just display id's of all received packets";
    std::cerr << "\n";
    std::cerr << "\n        all is a combination of system-state(20), raw-sensors(28),";
    std::cerr << "\n        velocity-std-dev(25), euler-orientation-std-dev(26) and satellites(30)";
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
    std::cerr << "\n    --wait-for-all:    when <packet>=all don't output until all types received";
    std::cerr << "\n";
    std::cerr << "\n    where <packet> is one of:";
    std::cerr << "\n        system_status,filter_status for --status";
    std::cerr << "\n        system_status,filter_status,gnss_fix for --status-description";
    std::cerr << "\n";
    if( verbose )
    {
        std::cerr << "\ncsv options:";
        std::cerr << "\n";
        std::cerr << comma::csv::options::usage();
        std::cerr << "\nexamples:";
        std::cerr << "\n    <raw-data> | " << comma::verbose.app_name() << " all";
        std::cerr << "\n    <raw-data> | " << comma::verbose.app_name() << " raw-sensors";
        std::cerr << "\n";
        std::cerr << "\n  see description of system_status values";
        std::cerr << "\n    <raw-data> | " << comma::verbose.app_name() << " system-state \\";
        std::cerr << "\n        | " << comma::verbose.app_name() << " --fields system_status \\";
        std::cerr << "\n                                     --status system_status";
        std::cerr << "\n    echo 128 | " << comma::verbose.app_name() << " --status system_status --json";
        std::cerr << "\n";
        std::cerr << "\n  see description of filter_status values";
        std::cerr << "\n    <raw-data> | " << comma::verbose.app_name() << " system-state \\";
        std::cerr << "\n        | " << comma::verbose.app_name() << " --fields ,filter_status \\";
        std::cerr << "\n                                     --status filter_status";
        std::cerr << "\n    echo 1029 | " << comma::verbose.app_name() << " --status filter_status";
        std::cerr << "\n    " << comma::verbose.app_name() << " --status-description filter_status";
        std::cerr << "\n";
        std::cerr << "\n  where <raw-data> is coming from advanced-navigation-cat or similar";
    }
    else
    {
        std::cerr << "\nuse -v or --verbose for more csv options and examples";
    }
    std::cerr << "\n" << std::endl;
}

static void bash_completion()
{
    std::cout << "--help --verbose"
              << " raw-sensors system-state satellites magnetic-calibration navigation all packet-ids"
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

template <>
struct traits< output_packet_id >
{
    template < typename Key, class Visitor > static void visit( const Key&, const output_packet_id& p, Visitor& v )
    {
        v.apply( "packet_id", p.packet_id );
    }

    template < typename Key, class Visitor > static void visit( const Key&, output_packet_id& p, Visitor& v )
    {
        v.apply( "packet_id", p.packet_id );
    }
};

template <>
struct traits< output_nav >
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

template <>
struct traits< output_all >
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

template <>
struct traits< status_data >
{
    template < typename Key, class Visitor > static void visit( const Key&, const status_data& p, Visitor& v )
    {
        v.apply( "status", p.status );
    }
    template < typename Key, class Visitor > static void visit( const Key&, status_data& p, Visitor& v )
    {
        v.apply( "status", p.status );
    }
};

} } // namespace comma { namespace visiting {

struct app_i
{
    virtual ~app_i() {}
    virtual void run() = 0;
    virtual void output_fields() = 0;
};

struct app_base : protected snark::navigation::advanced_navigation::device
{
    comma::io::select select;
    comma::signal_flag signaled;

    app_base() : device( "-" )
    {
        select.read().add( fd() );
    }

    void process()
    {
        while( !signaled && std::cout.good() )
        {
            select.wait( boost::posix_time::microseconds( sleep_us ));
            if( !signaled && select.read().ready( fd() ) ) { device::process(); }
        }
    }

    virtual void handle( const messages::acknowledgement* msg )
    {
        std::cerr << comma::verbose.app_name() << ": " << msg->result_msg() << std::endl;
    }
};

template< typename T >
struct app_t : public app_base
{
    comma::csv::output_stream< T > os;

    app_t( const comma::command_line_options& options )
        : app_base()
        , os( std::cout, comma::csv::options( options, "", true ))
    {}

    static void output_fields()
    {
        std::cout << comma::join( comma::csv::names< T >( true ), ',' ) << std::endl;
    }

    static void output_format()
    {
        std::cout << comma::csv::format::value< T >() << std::endl;
    }
};

struct app_packet_id : public app_t< output_packet_id >
{
    app_packet_id( const comma::command_line_options& options ) : app_t( options )
    {}
    //message handlers
    void handle_raw( messages::header* msg_header, const char* msg_data, std::size_t msg_data_length )
    {
        std::cout << (unsigned int)msg_header->id() << std::endl;
    }
};

struct app_nav : public app_t< output_nav >
{
    app_nav( const comma::command_line_options& options ) : app_t( options )
    {}
    //message handlers
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

/// accumulate several packets into one big output record
struct app_all : public app_t< output_all >
{
    enum {
        raw_sensors_mask = 1,
        velocity_standard_deviation_mask = 2,
        orientation_standard_deviation_mask = 4,
        satellites_mask = 8,
        all_mask = 15
    };

    unsigned int received_messages_mask;
    unsigned int wait_for_all_counter;
    output_all output;

    app_all( const comma::command_line_options& options )
        : app_t( options )
        , received_messages_mask( 0 )
        , wait_for_all_counter( 0 )
    {
        if( !options.exists( "--wait-for-all" ))
            received_messages_mask = all_mask;
    }

    void handle( const messages::system_state* msg )
    {
        //make copy
        memcpy( output.system_state.data(), msg->data(), messages::system_state::size );

        if(( received_messages_mask & all_mask ) == all_mask )
        {
            os.write( output );
            if( flush ) { os.flush(); }
        }
        else if( wait_for_all_counter++ == 100 )
        {
            std::cerr << "(--wait-for-all specified) still waiting for messages: ";
            if( !( received_messages_mask & raw_sensors_mask ))
                std::cerr << "raw_sensors ";
            if( !( received_messages_mask & velocity_standard_deviation_mask ))
                std::cerr << "velocity_standard_deviation ";
            if( !( received_messages_mask & orientation_standard_deviation_mask ))
                std::cerr << "orientation_standard_deviation ";
            if( !( received_messages_mask & satellites_mask ))
                std::cerr << "satellites ";
            std::cerr << std::endl;
        }
    }
    void handle( const messages::raw_sensors* msg )
    {
        received_messages_mask |= raw_sensors_mask;
        std::memcpy( output.raw_sensors.data(), msg->data(), messages::raw_sensors::size );
    }
    void handle( const messages::velocity_standard_deviation* msg )
    {
        received_messages_mask |= velocity_standard_deviation_mask;
        output.velocity_stddev = Eigen::Vector3f( msg->stddev[0](), msg->stddev[1](), msg->stddev[2]() );
    }
    void handle( const messages::orientation_standard_deviation* msg )
    {
        received_messages_mask |= orientation_standard_deviation_mask;
        output.orientation_stddev = Eigen::Vector3f( msg->stddev[0](), msg->stddev[1](), msg->stddev[2]() );
    }
    void handle( const messages::satellites* msg )
    {
        received_messages_mask |= satellites_mask;
        std::memcpy( output.satellites.data(), msg->data(), messages::satellites::size );
    }
};

template< typename T >
struct app_packet : public app_t< T >
{
    app_packet( const comma::command_line_options& options ) : app_t< T >( options )
    {}

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

template< typename T >
struct factory_t : public factory_i
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

template< typename T >
struct full_description
{
    comma::csv::input_stream< status_data > is;
    bool json;

    full_description( const comma::command_line_options& options )
        : is( std::cin, comma::csv::options( options ))
        , json( options.exists( "--json" ))
    {}

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
//                 comma::write_json( description, std::cout );
                boost::property_tree::write_json( std::cout, ptree, false );
            }
            else
            {
//                 comma::write_path_value( description, std::cout );
                std::string s = comma::property_tree::to_path_value_string( ptree, comma::property_tree::disabled, '=', ';' );
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

        std::vector< std::string > unnamed = options.unnamed( comma::csv::options::valueless_options()
                                                            + ",--verbose,-v,--output-fields,--output-format,--flush,--json", "-.*" );
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

        if( unnamed.size() != 1 ) { COMMA_THROW( comma::exception, "expected one packet name, got: " << unnamed.size() ); }
        std::string packet = unnamed[0];
        if( packet == "navigation" ) { factory.reset( new factory_t< app_nav >() ); }
        else if( packet == "all" ) { factory.reset( new factory_t< app_all >() ); }
        else if( packet == "packet-ids" ) { factory.reset( new factory_t< app_packet_id >() ); }
        else if( packet == "system-state" ) { factory.reset( new factory_t< app_packet <messages::system_state > >() ); }
        else if( packet == "unix-time" ) { factory.reset( new factory_t< app_packet <messages::unix_time > >() ); }
        else if( packet == "raw-sensors" ) { factory.reset( new factory_t< app_packet< messages::raw_sensors > >() ); }
        else if( packet == "satellites" ) { factory.reset( new factory_t< app_packet< messages::satellites > >() ); }
        else if( packet == "filter-options" ) { factory.reset( new factory_t< app_packet< messages::filter_options > >() ); }
        else if( packet == "magnetic-calibration" ) { factory.reset( new factory_t< app_packet< messages::magnetic_calibration_status > >() ); }
        else { COMMA_THROW( comma::exception, packet << " is an unsupported packet. See help." );}

        if( options.exists( "--output-fields" )) { factory->output_fields(); return 0; }
        if( options.exists( "--output-format" )) { factory->output_format(); return 0; }

        comma::csv::detail::unsynchronize_with_stdio();
        factory->run( options );

        return 0;
    }
    catch( snark::navigation::advanced_navigation::eois_exception& e ) { comma::verbose<<comma::verbose.app_name() << ": " << e.what() << std::endl; return 0; }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; }
    return 1;
}
