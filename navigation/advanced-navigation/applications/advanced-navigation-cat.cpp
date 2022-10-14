// Copyright (c) 2021,2022 Mission Systems Pty Ltd

#include "../device.h"
#include "../traits.h"
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/application/verbose.h>
#include <comma/csv/stream.h>
#include <comma/io/select.h>
#include <tbb/concurrent_queue.h>
#include <thread>

namespace messages = snark::navigation::advanced_navigation::messages;

const unsigned int default_baud_rate = 115200;
const unsigned int default_sleep = 10000;
bool flush;

void usage( bool verbose )
{
    std::cerr << "\nconnect to Advanced Navigation Certus device and send commands or output data";
    std::cerr << "\n";
    std::cerr << "\nusage: " << comma::verbose.app_name() << " <options>";
    std::cerr << "\n    or " << comma::verbose.app_name() << " --send <command> <options>";
    std::cerr << "\n";
    std::cerr << "\noptions:";
    std::cerr << "\n    --help,-h:              show help";
    std::cerr << "\n    --verbose,-v:           show detailed messages";
    std::cerr << "\n    --baud-rate,--baud=<n>: baud rate for serial connection, default " << default_baud_rate;
    std::cerr << "\n    --device=<device>:      serial or network port";
    std::cerr << "\n    --flush:                flush output stream after each record";
    std::cerr << "\n    --input-fields:         print input command fields for --send and exit";
    std::cerr << "\n    --input-format:         print input command format for --send and exit";
    std::cerr << "\n    --ntrip=<stream>:       read ntrip data from stream and send it to device";
    std::cerr << "\n    --send=<command>        read data from stdin and send as command packet";
    std::cerr << "\n    --sleep=<microseconds>: sleep between reading, default " << default_sleep;
    std::cerr << "\n";
    std::cerr << "\n    <stream> can be \"-\" for stdin; or a filename or \"tcp:<host>:<port>\" etc";
    std::cerr << "\n";
    std::cerr << "\n    --send: read commands from stdin and write command message to device";
    std::cerr << "\n            csv options apply to input stream";
    std::cerr << "\n    output: acknowledgement, use --verbose for human readable message";
    std::cerr << "\n";
    std::cerr << "\n    commands:";
    std::cerr << "\n        request: request a specific packet";
    std::cerr << "\n            input field: packet_id";
    std::cerr << "\n";
    std::cerr << "\n        reset: send a hot (equivalent to power-cycle) or cold start reset ";
    std::cerr << "\n            input field: sequence";
    std::cerr << "\n";
    std::cerr << "\n        filter-options: set filter options";
    std::cerr << "\n            input fields: permanent, vehicle_types, internal_gnss_enabled,";
    std::cerr << "\n                magnetic_heading_enabled, atmospheric_altitude_enabled,";
    std::cerr << "\n                velocity_heading_enabled, reversing_detection_enabled,";
    std::cerr << "\n                motion_analysis_enabled, automatic_magnetic_calibration_enabled,";
    std::cerr << "\n                reserved";
    std::cerr << "\n";
    std::cerr << "\n        magnetic-calibration: send magnetic calibration command";
    std::cerr << "\n            input fields: action";
    std::cerr << "\n            where <action> is:";
    std::cerr << "\n                0 - Cancel magnetic calibration";
    std::cerr << "\n                2 - Start 2D magnetic calibration";
    std::cerr << "\n                3 - Start 3D magnetic calibration";
    std::cerr << "\n                4 - Reset calibration to defaults";
    std::cerr << "\n";
    if( verbose )
    {
        std::cerr << "\nexamples:";
        std::cerr << "\n    " << comma::verbose.app_name() << " --device tcp:certus.local:16718";
        std::cerr << "\n    " << comma::verbose.app_name() << " --device /dev/usb/ttyUSB0 --baud 57600";
        std::cerr << "\n";
        std::cerr << "\n  request filter options packet";
        std::cerr << "\n    echo 186 | " << comma::verbose.app_name() << " --send request";
        std::cerr << "\n";
        std::cerr << "\n  reset device with standard hot start";
        std::cerr << "\n    echo 554007166 | " << comma::verbose.app_name() << " --send reset";
        std::cerr << "\n";
        std::cerr << "\n  reset device with cold start";
        std::cerr << "\n    echo 2589800631 | " << comma::verbose.app_name() << " --send reset";
        std::cerr << "\n";
        std::cerr << "\n  set filter options packet";
        std::cerr << "\n    echo 1,0,0,0,1,0,1,0,0,0 | " << comma::verbose.app_name() << " --send filter-options";
        std::cerr << "\n";
        std::cerr << "\n  send 2D magnetic calibration command and see status";
        std::cerr << "\n    " << comma::verbose.app_name() << " --send magnetic-calibration --input-fields";
        std::cerr << "\n    echo 2 | " << comma::verbose.app_name() << " --send magnetic-calibration";
        std::cerr << "\n    " << comma::verbose.app_name() << " | advanced-navigation-to-csv magnetic-calibration";
        std::cerr << "\n    advanced-navigation-to-csv --magnetic-calibration-description";
    }
    else
    {
        std::cerr << "\nuse -v or --verbose for examples";
    }
    std::cerr << "\n" << std::endl;
}

static void bash_completion()
{
    std::cout << " --help -h --verbose -v"
              << " --baud-rate --baud --device --flush --sleep"
              << " --send --input-fields --input-format magnetic-calibration"
              << std::endl;
}

class ntrip
{
public:
    typedef std::shared_ptr< std::vector< char > > item_t;

    ntrip( const std::string& name )
        : is( name,comma::io::mode::binary, comma::io::mode::non_blocking )
        , buf( 4096 )
    {
        select.read().add( is.fd() );
    }

    bool process()
    {
        if( !is->good() ) return false;
        unsigned size = is.available_on_file_descriptor();
        if( !size ) { select.wait( boost::posix_time::microseconds( 20000 )); }
        if( size || select.read().ready( is.fd() ))
        {
            //or use size = readsome...
            is->read( &buf[0],size );
            size = is->gcount();
            if( size )
            {
                //put it in queue
                queue.push( item_t( new std::vector< char >( buf.begin(), buf.begin()+size )));
            }
        }
        return true;
    }

    static tbb::concurrent_bounded_queue< item_t > queue;

private:
    //create stream from name
    comma::io::istream is;
    comma::io::select select;
    //app_t can check the queue and write to gps, instead of sleep
    std::vector< char > buf;
};

//make a thread for reading
class ntrip_thread
{
public:
    ntrip_thread( const std::string& filename )
        : shutdown( false )
        , thread( &ntrip_thread::run, this )
        , filename( filename )
    {}

    ~ntrip_thread()
    {
        shutdown = true;
        thread.join();
    }

    void run()
    {
        while( !shutdown )
        {
            try
            {
                ntrip nt( filename );
                while( !shutdown&&nt.process() ) {}
            }
            catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
            catch( ... ) { std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; }
            //try to connect again in 3 seconds
            for( unsigned i = 0; i < 100 && !shutdown; i++ )
            {
                std::this_thread::sleep_for( std::chrono::microseconds( 30 ));
            }
        }
    }

private:
    bool shutdown;
    std::thread thread;
    std::string filename;
};

static void output_raw( messages::header* msg_header, const char* msg_data, std::size_t msg_data_length, std::ostream& os )
{
    std::vector< char > obuf( 260 );
    obuf.resize( messages::header::size + msg_data_length );
    std::memcpy( &obuf[0], msg_header->data(), messages::header::size );
    std::memcpy( &obuf[messages::header::size], msg_data, msg_data_length );
    os.write( &obuf[0], obuf.size() );
    if( flush ) { os.flush(); }
}

// open a device (serial or network) for reading
// override device::handler(...) methods to process messages
class app_base : protected snark::navigation::advanced_navigation::device
{
public:
    app_base( const comma::command_line_options& options )
        : device( options.value< std::string >( "--device" )
                , options.value< unsigned int >( "--baud-rate,--baud", default_baud_rate ))
        , us( options.value< unsigned int >( "--sleep", default_sleep ))
    {
        select.read().add( fd() );
    }

    void process()
    {
        while( !signal && std::cout.good() )
        {
            select.wait( boost::posix_time::microseconds( us ));
            if( !signal && select.read().ready( fd() )) { device::process(); }
            if( !ntrip::queue.empty() )
            {
                ntrip::item_t item;
                ntrip::queue.pop( item );
                if( item ) { send_ntrip( *item ); }
            }
        }
    }

private:
    unsigned int us;
    comma::io::select select;
    comma::signal_flag signal;
};

// app class to output raw data
class app_raw : public app_base
{
public:
    app_raw( const comma::command_line_options& options )
        : app_base( options )
        , obuf( 260 )
    {}

protected:
    void handle_raw( messages::header* msg_header, const char* msg_data, std::size_t msg_data_length )
    {
        obuf.resize( messages::header::size + msg_data_length );
        std::memcpy( &obuf[0], msg_header->data(), messages::header::size );
        std::memcpy( &obuf[messages::header::size], msg_data, msg_data_length );
        std::cout.write( &obuf[0], obuf.size() );
        if( flush ) { std::cout.flush(); }
    }

private:
    std::vector< char > obuf;
};

struct send_factory_i
{
    virtual ~send_factory_i() {}
    virtual void input_fields() = 0;
    virtual void input_format() = 0;
    virtual void run( const comma::command_line_options& options ) = 0;
};

template< typename T >
struct send_factory_t : public send_factory_i
{
    typedef T type;
    void input_fields() { T::input_fields(); }
    void input_format() { T::input_format(); }
    void run( const comma::command_line_options& options )
    {
        T app( options );
        app.process();
    }
};

// send data to device
template < typename T >
class send_app : protected snark::navigation::advanced_navigation::device
{
public:
    send_app( const comma::command_line_options& options )
        : device( options.value< std::string >( "--device" ), options.value< unsigned int >( "--baud-rate,--baud", default_baud_rate ))
        , is( std::cin, comma::csv::options( options ))
        , us( options.value< unsigned int >( "--sleep", default_sleep ))
    {
        select.read().add( fd() );
    }

    // send a command from stdin and receive a response
    // keep reading stdin until it's closed (or ctrl-c)
    // assume one response per command sent and keep reading device until all
    // responses received, then exit
    void process()
    {
        unsigned int commands_sent = 0;
        unsigned int responses_recv = 0;

        while( !signal && std::cin.good() )
        {
            const T* msg = is.read();
            if( msg )
            {
                messages::command cmd = msg->get_command();
                send( cmd );
                commands_sent++;
            }
            select.wait( boost::posix_time::microseconds( us ));
            if( !signal && select.read().ready( fd() ))
            {
                device::process();
                responses_recv++;
            }
        }
        comma::verbose << ( signal ? "signal caught" : "stdin closed" ) << std::endl;
        if( responses_recv < commands_sent )
        {
            comma::verbose << "sent " << commands_sent << " commands but only received " << responses_recv << " responses" << std::endl;
            if( !signal ) { comma::verbose << "waiting for remaining responses" << std::endl; }
        }
        while( !signal && responses_recv < commands_sent )
        {
            select.wait( boost::posix_time::microseconds( us ));
            if( !signal && select.read().ready( fd() ))
            {
                device::process();
                responses_recv++;
            }
        }
    }

    static void input_fields()
    {
        std::cout << comma::join( comma::csv::names< T >( true ), ',' ) << std::endl;
    }

    static void input_format()    //const std::string& fields
    {
        //std::cout << comma::csv::format::value< output_t >( fields, true ) << std::endl;
        std::cout << comma::csv::format::value< T >() << std::endl;
    }

protected:
    void handle_raw( messages::header* msg_header, const char* msg_data, std::size_t msg_data_length )
    {
        output_raw( msg_header, msg_data, msg_data_length, std::cout );
    }

private:
    comma::csv::input_stream< T > is;
    unsigned int us;
    comma::io::select select;
    comma::signal_flag signal;
};

tbb::concurrent_bounded_queue< ntrip::item_t > ntrip::queue;

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );

        if( options.exists( "--bash-completion" )) { bash_completion(); return 0; }

        flush = options.exists( "--flush" );

        auto opt_send = options.optional< std::string >( "--send" );
        if( opt_send )
        {
            std::unique_ptr< send_factory_i > sf;
            if( *opt_send == "request" ) { sf.reset( new send_factory_t< send_app< messages::request >>() ); }
            else if( *opt_send == "reset" ) { sf.reset( new send_factory_t< send_app< messages::reset >>() ); }
            else if( *opt_send == "filter-options" ) { sf.reset( new send_factory_t< send_app< messages::filter_options >>() ); }
            else if( *opt_send == "magnetic-calibration" ) { sf.reset( new send_factory_t< send_app< messages::magnetic_calibration_configuration >>() ); }
            else { COMMA_THROW( comma::exception, "invalid send command: " << *opt_send ); }

            if( options.exists( "--input-fields" )) { sf->input_fields(); return 0; }
            if( options.exists( "--input-format" )) { sf->input_format(); return 0; }

            sf->run( options );
            return 0;
        }

        std::unique_ptr< ntrip_thread > ntt;
        auto opt_ntrip = options.optional< std::string >( "--ntrip" );
        if( opt_ntrip ) { ntt.reset( new ntrip_thread( *opt_ntrip )); }

        app_raw( options ).process();

        return 0;
    }
    catch( snark::navigation::advanced_navigation::eois_exception& e )
    {
        // normal exit on end of input stream
        comma::verbose << comma::verbose.app_name() << ": " << e.what() << std::endl;
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; }
    return 1;
}
