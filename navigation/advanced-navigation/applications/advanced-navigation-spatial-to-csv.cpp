// Copyright (c) 2017 The University of Sydney

#include "../device.h"
#include "../messages.h"
#include "../traits.h"
#include "../../../math/roll_pitch_yaw.h"
#include "../../../math/spherical_geometry/coordinates.h"
#include "../../../math/spherical_geometry/traits.h"
#include "../../../visiting/traits.h"
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/application/verbose.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/io/select.h>
#include <comma/io/stream.h>
#include <comma/name_value/serialize.h>
#include <boost/date_time/posix_time/ptime.hpp>
#include <Eigen/Core>
#include <tbb/concurrent_queue.h>
#include <chrono>
#include <regex>
#include <thread>

using namespace snark::navigation::advanced_navigation;

const unsigned default_baud_rate=115200;
const unsigned default_sleep=10000;
bool flush=true;

void usage(bool detail)
{
    std::cerr<<"connect to Advanced Navigation Spatial device and output GPS data" << std::endl;
    std::cerr << std::endl;
    std::cerr << "DEPRECATED - use \"advanced-navigation-cat | advanced-navigation-to-csv\" instead" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "usage: " << comma::verbose.app_name() << " <what> [<options>]" << std::endl;
    std::cerr<< "    <what>: select data packet to output, default: navigation"<< std::endl;
    std::cerr<< "    or " << comma::verbose.app_name() << " --send <command> --device <port> [<options>]"<< std::endl;
    std::cerr << std::endl;
    std::cerr<< "what: " << std::endl;
    std::cerr<< "    all: combines system-state, raw-sensors and standard deviations into one record" << std::endl;
    std::cerr << "        --wait-for-all: it won't output messages for the first time until it receives all the message types" << std::endl;
    std::cerr<< "    navigation: navigation data from system state packet" << std::endl;
    std::cerr<< "    raw-sensors" << std::endl;
    std::cerr<< "    satellites" << std::endl;
    std::cerr<< "    magnetic-calibration: magnetic calibration status packet"<<std::endl;
    std::cerr<< "    system-state: full system state packet"<< std::endl;
    std::cerr << std::endl;
    std::cerr<< "send: read commands from stdin and write command message to device; csv options apply to input stream" << std::endl;
    std::cerr<< "    output: acknowledgement result value, use -v or --verbose to see human readable message on stderr"<< std::endl;
    std::cerr<< "    commands:"<< std::endl;
    std::cerr<< "        magnetic-calibration: send magnetic calibration command"<< std::endl;
    std::cerr<< "            fields: action"<< std::endl;
    std::cerr<< "            action"<< std::endl;
    std::cerr<< "                        0 Cancel magnetic calibration"<< std::endl;
    std::cerr<< "                        2 Start 2D magnetic calibration"<< std::endl;
    std::cerr<< "                        3 Start 3D magnetic calibration"<< std::endl;
    std::cerr<< "                        4 Reset calibration to defaults"<< std::endl;
    std::cerr<< "                "<< std::endl;
    std::cerr<< "    --input-fields: print input command fields and exit"<<std::endl;
    std::cerr<< "    --input-format: print input command format and exit"<<std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --baud-rate,--baud=<n>: baud rate for connection, default "<< default_baud_rate << std::endl;
    std::cerr << "    --device=<filename>; filename for serial port e.g. /dev/usb/ttyUSB0" << std::endl;
    std::cerr << "    --flush: flush output stream after each write" << std::endl;
    std::cerr << "    --help,-h:       show help" << std::endl;
    std::cerr << "    --magnetic-calibration-description: print magnetic calibration status description table and exit"<< std::endl;
    std::cerr << "    --ntrip=<stream>: read ntrip data from stream and send it to device" << std::endl;
    std::cerr << "        stream can be \"-\" for stdin; or a filename or \"tcp:<host>:<port>\" etc" << std::endl;
    std::cerr << "    --output-fields: print output fields and exit" << std::endl;
    std::cerr << "    --output-format: print output format and exit" << std::endl;
    std::cerr << "    --raw: output raw packets to stdout, " << std::endl;
    std::cerr << "    --sleep=<n>: microsecond sleep between reading, default "<< default_sleep << std::endl;
    std::cerr << "    --status=<what>; print out expanded status bit map of input values" << std::endl;
    std::cerr << "        <what>: system_status | filter_status" << std::endl;
    std::cerr << "        csv options apply to input" << std::endl;
    std::cerr << "            input fields: status" << std::endl;
    std::cerr << "        --json; format output in json"<< std::endl;
    std::cerr << "    --status-description=<what>; print bit index/value and their human readable description and then exit"<< std::endl;
    std::cerr << "        <what>: system_status | filter_status | gnss_fix" << std::endl;
    std::cerr << "    --stdin; read packets from stdin, can't be used with options that need to write to device (e.g. --ntrip)" << std::endl;
    std::cerr << "    --verbose,-v:    show detailed messages" << std::endl;
    std::cerr << std::endl;
    if(detail)
    {
        std::cerr << "csv options:" << std::endl;
        std::cerr<< comma::csv::options::usage() << std::endl;
    }
    else
    {
        std::cerr << "use -v or --verbose to see more detail" << std::endl;
    }
    std::cerr << std::endl;
    std::cerr << "examples:" << std::endl;
    std::cerr << "    sudo mknod /dev/usb/ttyUSB0 c 188 0" << std::endl;
    std::cerr << "    "<<comma::verbose.app_name()<<" all --device \"/dev/usb/ttyUSB0\" " << std::endl;
    std::cerr << "    "<<comma::verbose.app_name()<<" raw-sensors --device \"/dev/usb/ttyUSB0\" " << std::endl;
    std::cerr << std::endl;
    std::cerr << "  see description of system_status values" << std::endl;
    std::cerr << "    " << comma::verbose.app_name() << " system-state --device /dev/usb/ttyUSB0 | " << comma::verbose.app_name() << " --fields system_status --status system_status" << std::endl;
    std::cerr << "    echo 128 | " << comma::verbose.app_name() << " --status system_status --json" << std::endl;
    std::cerr << std::endl;
    std::cerr << "  see description of filter_status values" << std::endl;
    std::cerr << "    " << comma::verbose.app_name() << " system-state --device /dev/usb/ttyUSB0 | " << comma::verbose.app_name() << " --fields ,filter_status --status filter_status" << std::endl;
    std::cerr << "    echo 1029 | " << comma::verbose.app_name() << " --status filter_status" << std::endl;
    std::cerr << "    " << comma::verbose.app_name() << " --status-description filter_status" << std::endl;
    std::cerr << std::endl;
    std::cerr << "  send 2D magnetic calibration command and see status" << std::endl;
    std::cerr << "    echo 1 | " << comma::verbose.app_name() << " --send magnetic-calibration --device /dev/usb/ttyUSB0 -v"<< std::endl;
    std::cerr << "    "<< comma::verbose.app_name() << " magnetic-calibration --device /dev/usb/ttyUSB0"<< std::endl;
    std::cerr << "    "<< comma::verbose.app_name() << " --magnetic-calibration-description"<<std::endl;
    std::cerr << std::endl;
}

struct output
{
    output() : height(0), system_status(0), filter_status(0) { }
    boost::posix_time::ptime t;
    snark::spherical::coordinates coordinates;
    double height;
    snark::roll_pitch_yaw orientation;
    uint16_t system_status;
    uint16_t filter_status;
};

struct output_all
{
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
struct traits< output >
{
    template < typename Key, class Visitor > static void visit( const Key&, const output& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "coordinates", p.coordinates );
        v.apply( "height", p.height );
        v.apply( "orientation", p.orientation );
        v.apply( "system_status", p.system_status );
        v.apply( "filter_status", p.filter_status );
    }

    template < typename Key, class Visitor > static void visit( const Key&, output& p, Visitor& v )
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

struct ntrip
{
    typedef std::shared_ptr<std::vector<char>> item_t;
    //create stream from name
    comma::io::istream is;
    comma::io::select select;
    //have queue<std::vecotr<char>> (or double buffer) to put data in
    static tbb::concurrent_bounded_queue<item_t> queue;
    //app_t can check the queue and write to gps, instead of sleep
    ntrip(const std::string& name) : is(name,comma::io::mode::binary,comma::io::mode::non_blocking), buf(4096)
    {
        select.read().add( is.fd() );
    }
    std::vector<char> buf;
    bool process()
    {
        if(!is->good())
            return false;
        unsigned size=is.available_on_file_descriptor();
        if( !size ) { select.wait( boost::posix_time::microseconds( 20000 ) ); }
        if( size || select.read().ready( is.fd() ) )
        {
            //or use size=readsome...
            is->read(&buf[0],size);
            size=is->gcount();
            if(size)
            {
                //put it in queue
                queue.push(item_t(new std::vector<char>(buf.begin(),buf.begin()+size)));
            }
        }
        return true;
    }
};

struct ntrip_thread
{
    //make a thread for reading
    bool shutdown;
    std::thread thread;
    std::string filename;
    ntrip_thread(const std::string& filename,const comma::command_line_options& options) :
        shutdown(false),
        thread(&ntrip_thread::run,this),
        filename(filename)
    {
        
    }
    ~ntrip_thread()
    {
        shutdown=true;
        thread.join();
    }
    void run()
    {
        while(!shutdown)
        {
            try
            {
                ntrip nt(filename);
                while(!shutdown&&nt.process())
                {
                }
            }
            catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
            catch( ... ) { std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; }
            //try to connect again in 3 seconds
            for(unsigned i=0;i<100&&!shutdown;i++)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(30));
            }
        }
    }
};

struct app_i
{
    virtual ~app_i() { }
    virtual void run()=0;
    virtual void output_fields()=0;
};

struct app_base : protected device
{
    unsigned us;
    comma::io::select select;
    comma::signal_flag signaled;
public:
    app_base(const std::string& port,const comma::command_line_options& options) : 
        device(port,options.value<unsigned>("--baud-rate,--baud",default_baud_rate)),
        us(options.value<unsigned>("--sleep",default_sleep))
    {
        select.read().add( fd() );
    }
    void process()
    {
        while( !signaled && std::cout.good() )
        {
            select.wait( boost::posix_time::microseconds( us ) );
            if( select.read().ready( fd() ) ) { device::process(); }
            if(!ntrip::queue.empty())
            {
                ntrip::item_t item;
                ntrip::queue.pop(item);
                if(item) { send_ntrip(*item); }
            }
        }
    }
};

template<typename T>
struct app_t : public app_base
{
    comma::csv::output_stream<T> os;
    app_t(const std::string& port,const comma::command_line_options& options) : 
        app_base(port,options),
        os(std::cout,comma::csv::options(options,"",true))
    {
        
    }
    static void output_fields()
    {
        std::cout<<comma::join( comma::csv::names<T>(true), ',' ) << std::endl; 
    }
    static void output_format()    //const std::string& fields
    {
        //std::cout<<comma::csv::format::value<output_t>(fields, true) << std::endl;
        std::cout<<comma::csv::format::value<T>() << std::endl;
    }
};

struct app_nav : public app_t<output>
{
    app_nav(const std::string& port,const comma::command_line_options& options) : app_t(port,options) { }
    //message handlers
    void handle(const messages::system_state* msg)
    {
        output o;
        o.t=msg->t();
        o.coordinates.latitude=msg->latitude();
        o.coordinates.longitude=msg->longitude();
        o.height=msg->height();
        o.orientation=snark::roll_pitch_yaw(msg->orientation[0](),msg->orientation[1](),msg->orientation[2]());
        o.system_status=msg->system_status();
        o.filter_status=msg->filter_status();
        os.write(o);
        if(flush) { os.flush(); }
    }
};

struct app_raw : public app_base
{
    std::vector<char> obuf;
    app_raw(const std::string& port,const comma::command_line_options& options) : app_base(port,options), obuf(260)
    {
        
    }
    static void output_fields() { std::cout<<std::endl; }
    static void output_format() { std::cout<<std::endl;  }
protected:
    void handle_raw(messages::header* msg_header, const char* msg_data,std::size_t msg_data_length)
    {
        obuf.resize(messages::header::size+msg_data_length);
        std::memcpy(&obuf[0],msg_header->data(),messages::header::size);
        std::memcpy(&obuf[messages::header::size],msg_data,msg_data_length);
        std::cout.write(&obuf[0],obuf.size());
        if(flush) { std::cout.flush(); }
    }
};

/// accumulate several packets into one big output record
struct app_all : public app_t<output_all>
{
    enum { raw_sensors_mask=1, velocity_standard_deviation_mask=2, orientation_standard_deviation_mask=4, satellites_mask=8, all_mask=15 };
    unsigned recieved_messages_mask;
    unsigned wait_for_all_counter;

    app_all(const std::string& port,const comma::command_line_options& options) : app_t(port,options), recieved_messages_mask(0), wait_for_all_counter(0)
    {
        if(!options.exists("--wait-for-all"))
            recieved_messages_mask=all_mask;
    }
    output_all output;
    void handle(const messages::system_state* msg)
    {
        //make copy
        memcpy(output.system_state.data(), msg->data(),messages::system_state::size);
//         output.system_state=msg;

        if((recieved_messages_mask&all_mask)==all_mask)
        {
            os.write(output);
            if(flush) { os.flush(); }
        }
        else if(wait_for_all_counter++==100)
        {
            std::cerr<<"(--wait-for-all specified) still waiting for messages: ";
            if(!(recieved_messages_mask&raw_sensors_mask))
                std::cerr<<"raw_sensors ";
            if(!(recieved_messages_mask&velocity_standard_deviation_mask))
                std::cerr<<"velocity_standard_deviation ";
            if(!(recieved_messages_mask&orientation_standard_deviation_mask))
                std::cerr<<"orientation_standard_deviation ";
            if(!(recieved_messages_mask&satellites_mask))
                std::cerr<<"satellites ";
            std::cerr<<std::endl;
        }
    }
    void handle(const messages::raw_sensors* msg)
    {
        recieved_messages_mask|=raw_sensors_mask;
        std::memcpy(output.raw_sensors.data(),msg->data(),messages::raw_sensors::size);
    }
    void handle(const messages::velocity_standard_deviation* msg)
    {
        recieved_messages_mask|=velocity_standard_deviation_mask;
        output.velocity_stddev=Eigen::Vector3f(msg->stddev[0](),msg->stddev[1](),msg->stddev[2]());
    }
    void handle(const messages::orientation_standard_deviation* msg)
    {
        recieved_messages_mask|=orientation_standard_deviation_mask;
        output.orientation_stddev=Eigen::Vector3f(msg->stddev[0](),msg->stddev[1](),msg->stddev[2]());
    }
    void handle(const messages::satellites* msg)
    {
        recieved_messages_mask|=satellites_mask;
        std::memcpy(output.satellites.data(),msg->data(),messages::satellites::size);
    }
};

template<typename T>
struct app_packet : public app_t<T>
{
    app_packet(const std::string& port,const comma::command_line_options& options) : app_t<T>(port,options) { }
    void handle(const T* msg)
    {
        app_t<T>::os.write(*msg);
        if(flush) { app_t<T>::os.flush(); }
    }
};

struct factory_i
{
    virtual ~factory_i() { }
    virtual void output_fields()=0;
    virtual void output_format()=0;
    virtual void run(const std::string& input,const comma::command_line_options& options)=0;
};

template<typename T>
struct factory_t : public factory_i
{
    typedef T type;
    void output_fields() { T::output_fields(); }
    void output_format() { T::output_format(); }
    void run(const std::string& input,const comma::command_line_options& options)
    {

        T app(input,options);
        app.process();
    }
};

// template< typename T >
// struct description
// {
//     comma::csv::input_stream< status_data > is;
//     description( const comma::command_line_options& options ) : is( std::cin, comma::csv::options( options ) ) { }
//     void process()
//     {
//         while( std::cin.good() )
//         {
//             const status_data* p = is.read();
//             if( !p ) { break; }
//             std::cout << T::string( p->status ) << std::endl;
//         }
//     }
// };

template<typename T>
struct full_description
{
    comma::csv::input_stream< status_data > is;
    bool json;
    full_description( const comma::command_line_options& options ) : is( std::cin, comma::csv::options( options ) ),json(options.exists("--json")) { }
    void process()
    {
        while( std::cin.good() )
        {
            const status_data* p = is.read();
            if( !p ) { break; }
            T description(p->status);
            boost::property_tree::ptree ptree;
            comma::to_ptree to_ptree( ptree, comma::xpath() );
            comma::visiting::apply( to_ptree ).to( description );
            std::cout.precision( 16 ); // quick and dirty
            if(json)
            {
//                 comma::write_json(description,std::cout);
                boost::property_tree::write_json( std::cout, ptree, false );
            }
            else
            {
//                 comma::write_path_value(description,std::cout);
                std::string s=comma::property_tree::to_path_value_string( ptree, comma::property_tree::disabled, '=', ';' );
                std::cout<<std::regex_replace(s,std::regex("\"([0-9]*)\""),"$1")<<std::endl;
            }
        }
    }
};

static void bash_completion( int argc, char** argv )
{
    std::cout << "--help --verbose" <<
        " all magnetic-calibration navigation raw-sensors system-state satellites " <<
        " --magnetic-calibration-description --output-fields --output-format --raw --send --stdin --status --status-description --json"<< 
        std::endl;
}

struct send_factory_i
{
    virtual ~send_factory_i() { }
    virtual void input_fields()=0;
    virtual void input_format()=0;
    virtual unsigned run(const comma::command_line_options& options)=0;
};

template<typename T>
struct send_factory_t : public send_factory_i
{
    typedef T type;
    void input_fields() { T::input_fields(); }
    void input_format() { T::input_format(); }
    unsigned run(const comma::command_line_options& options)
    {

        T app(options);
        return app.process();
    }
};

template <typename T>
struct send_app : protected device 
{
    comma::csv::input_stream<T> is;
    unsigned result;
    send_app(const comma::command_line_options& options) : 
        device(options.value<std::string>("--device"),options.value<unsigned>("--baud-rate,--baud",default_baud_rate)),
        is( std::cin, comma::csv::options( options ) )
    {
        
    }
    unsigned process()
    {
        while(std::cin.good())
        {
            const T* pt=is.read();
            if(!pt) 
                break;
            messages::command cmd=pt->get_command();
            send(cmd);
        }
        return result;
    }
    static void input_fields()
    {
        std::cout<<comma::join( comma::csv::names<T>(true), ',' ) << std::endl; 
    }
    static void input_format()    //const std::string& fields
    {
        //std::cout<<comma::csv::format::value<output_t>(fields, true) << std::endl;
        std::cout<<comma::csv::format::value<T>() << std::endl;
    }
};

tbb::concurrent_bounded_queue<ntrip::item_t> ntrip::queue;

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        
        if(options.exists("--bash-completion")) { bash_completion( argc, argv ); return 0; }
        
        std::vector<std::string> unnamed=options.unnamed( comma::csv::options::valueless_options()+ ",--verbose,-v,--output-fields,--output-format,--raw,--stdin,--flush, --json", "-.*" );
        flush=options.exists("--flush");
        
//         auto opt_description=options.optional<std::string>("--description");
//         if(opt_description)
//         {
//             if( *opt_description == "system_status" ) { description< messages::system_status_description >( options ).process(); }
//             else if( *opt_description == "filter_status" ) { description< messages::filter_status_description >( options ).process(); }
//             else { COMMA_THROW( comma::exception, "invalid field for description. expected 'system_status' or 'filter_status', got " << *opt_description ); }
//             return 0;
//         }
        auto opt_full_description=options.optional<std::string>("--status");
        if(opt_full_description)
        {
            if( *opt_full_description == "system_status" ) { full_description< messages::system_status_description >( options ).process(); }
            else if( *opt_full_description == "filter_status" ) { full_description< messages::filter_status_description >( options ).process(); }
            else { COMMA_THROW( comma::exception, "invalid field for --status. expected 'system_status' or 'filter_status', got " << *opt_full_description ); }
            return 0;
        }
        auto opt_status_description=options.optional<std::string>("--status-description");
        if(opt_status_description)
        {
            if( *opt_status_description == "system_status" ) { messages::system_status_description::description( std::cout ); }
            else if( *opt_status_description == "filter_status" ) { messages::filter_status_description::description( std::cout ); }
            else if( *opt_status_description == "gnss_fix" ) { messages::filter_status_description::gnss_fix_description( std::cout ); }
            else { COMMA_THROW( comma::exception, "invalid field for --status-description. expected 'system_status' or 'filter_status' or 'gnss_fix', got " << *opt_status_description ); }
            return 0;
        }
        if( options.exists( "--magnetic-calibration-description" )) { messages::magnetic_calibration_status::status_description( std::cout ); return 0; }
        
        auto opt_send = options.optional< std::string >( "--send" );
        if( opt_send )
        {
            std::unique_ptr<send_factory_i> sf;
            if( *opt_send == "magnetic-calibration" ) { sf.reset( new send_factory_t< send_app< messages::magnetic_calibration_configuration >>() ); }
            else { COMMA_THROW( comma::exception, "invalid send command: " << *opt_send ); }
            
            if( options.exists( "--input-fields" )) { sf->input_fields(); return 0; }
            if( options.exists( "--input-format" )) { sf->input_format(); return 0; }
            
            return sf->run( options );
        }
        
        std::unique_ptr<factory_i> factory;
        if(options.exists("--raw"))
        {
            factory.reset(new factory_t<app_raw>());
        }
        else
        {
            if(unnamed.size()!=1) { COMMA_THROW( comma::exception, "expected one unnamed arguement, got: "<<unnamed.size()); }
            if(unnamed[0]=="navigation") { factory.reset(new factory_t<app_nav>()); }
            else if(unnamed[0]=="all") { factory.reset(new factory_t<app_all>()); }
            else if(unnamed[0]=="raw-sensors") { factory.reset(new factory_t<app_packet<messages::raw_sensors>>()); }
            else if(unnamed[0]=="system-state") { factory.reset(new factory_t<app_packet<messages::system_state>>()); }
            else if(unnamed[0]=="satellites") { factory.reset(new factory_t<app_packet<messages::satellites>>()); }
            else if(unnamed[0]=="magnetic-calibration") { factory.reset(new factory_t<app_packet<messages::magnetic_calibration_status>>()); }
            else { COMMA_THROW( comma::exception,"expected <what>: navigation | raw-sensors | system-state | all; got "<<unnamed[0]);}
        }

        if(options.exists("--output-fields")) { factory->output_fields(); return 0; }
        if(options.exists("--output-format")) { factory->output_format(); return 0; }

        std::unique_ptr<ntrip_thread> ntt;
        auto opt_ntrip=options.optional<std::string>("--ntrip");
        if(opt_ntrip) { ntt.reset(new ntrip_thread(*opt_ntrip,options)); }

        
        std::string input;
        if(options.exists("--stdin"))
        {
            input="-";
            if(options.exists("--device")) { COMMA_THROW( comma::exception, "--stdin conflicts with --device"); }
            if(opt_ntrip) { COMMA_THROW( comma::exception, "--stdin conflicts with --ntrip"); }
            comma::csv::detail::unsynchronize_with_stdio();
        }
        else
        {
            input=options.value<std::string>("--device");
        }

        factory->run(input,options);
        
        return 0;
    }
    catch(snark::navigation::advanced_navigation::eois_exception& e)
    {
        // normal exit on end of input stream
        comma::verbose<<comma::verbose.app_name() << ": " << e.what() << std::endl;
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; }
    return 1;
}
