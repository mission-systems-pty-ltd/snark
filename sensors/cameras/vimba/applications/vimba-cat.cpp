// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney

#include <boost/bind.hpp>
#include <comma/application/signal_flag.h>
#include <comma/application/verbose.h>
#include <comma/base/exception.h>
#include "../../../../imaging/cv_mat/serialization.h"

#include "../attribute.h"
#include "../camera.h"
#include "../error.h"
#include "../frame.h"
#include "../system.h"
#include <comma/io/publisher.h>
#include <comma/csv/stream.h>
#include "../traits.h"

static const char* possible_fields = "t,rows,cols,type,size";
static const char* default_fields = "t,rows,cols,type";
static int default_retries_on_no_frames = 3;

static void bash_completion( unsigned const ac, char const* const* av )
{
    static const char* completion_options =
        " --help -h"
        " --verbose -v"
        " --version"
        " --list-cameras"
        " --list-attributes"
        " --set --set-and-exit"
        " --id --fields"
        " --header --no-header"
        " --dont-check-frames --retries-on-no-frames"
        " --ptp-status --ptp-status-fields --ptp-status-format"
        ;

    std::cout << completion_options << std::endl;
    exit( 0 );
}

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "Capture data from an Allied Vision GigE camera" << std::endl;
    std::cerr << "Output to stdout as serialized cv::Mat" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Usage: " << comma::verbose.app_name() << " [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Options: " << std::endl;
    std::cerr << "    --help,-h:                         show this help, --help --verbose for more help" << std::endl;
    std::cerr << "    --acquisition-start-delay,--acquisition-delay=<seconds>; default=10; wait for a given time before starting acquisition" << std::endl;
    std::cerr << "    --dont-check-frames:               don't check if we've stop receiving frames" << std::endl;
    std::cerr << "    --fields=<fields>:                 header fields; default: " << default_fields << std::endl;
    std::cerr << "    --frames-buffer-size,--num-frames: default=3; camera frame acquisition buffer size" << std::endl;
    std::cerr << "    --header:                          output header only" << std::endl;
    std::cerr << "    --id=<camera id>:                  camera id; default: use first available camera" << std::endl;
    std::cerr << "    --list-cameras:                    list all cameras and exit" << std::endl;
    std::cerr << "    --list-attributes [<names>]:       list camera attributes; default: list all" << std::endl;
    std::cerr << "    --no-header:                       output image data only" << std::endl;
    std::cerr << "    --ptp-status=<stream>;             publish ptp status data to <stream> in binary"<< std::endl;
    std::cerr << "        <stream>: tcp:<port> | udp:<port> | <filename>"<< std::endl;
    std::cerr << "        fields: t,use_ptp,value"<< std::endl;
    std::cerr << "        format: ascii; t,ub,s[20]"<< std::endl;
    std::cerr << "    --ptp-status-fields;               print ptp status fields and exit"<< std::endl;
    std::cerr << "    --ptp-status-format;               print ptp status format and exit"<< std::endl;
    std::cerr << "    --retries-on-no-frames=<n>:        retry attempts if no frames detected; default: " << default_retries_on_no_frames << std::endl;
    std::cerr << "    --set <attributes>:                set camera attributes" << std::endl;
    std::cerr << "    --set-and-exit <attributes>:       set attributes and exit" << std::endl;
    std::cerr << "    --verbose,-v:                      more output" << std::endl;
    std::cerr << "    --version:                         output the vimba library version" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    Possible values for <fields> are: " << possible_fields << "." << std::endl;
    std::cerr << "    <attributes> are semicolon-separated name-value pairs." << std::endl;
    std::cerr << "    <names> are semicolon-semicolon feature names." << std::endl;
    std::cerr << std::endl;
    std::cerr << "    --list-cameras and --list-attributes provide more detail with --verbose" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    --dont-check-frames restores the old behaviour of not checking or retrying" << std::endl;
    std::cerr << "    --retries-on-no-frames=0 means the app will exit immediately if no frames" << std::endl;
    std::cerr << "    detected, otherwise it will try to stop and restart acquisition up to the" << std::endl;
    std::cerr << "    given number of times" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Examples:" << std::endl;
    std::cerr << "    " << comma::verbose.app_name() << " --id=\"02-2623A-07136\" --set \"ExposureAuto=Off;ExposureTimeAbs=60\"" << std::endl;
    std::cerr << "    " << comma::verbose.app_name() << " --list-attributes \"ExposureAuto;ExposureTimeAbs\"" << std::endl;
    std::cerr << std::endl;
    if( verbose )
    {
        std::cerr << "Camera id:" << std::endl;
        std::cerr << "    --list-cameras will provide two id's for each found camera: the camera id" << std::endl;
        std::cerr << "    and the serial number. Either can be used as the value for the --id option." << std::endl;
        std::cerr << std::endl;
        std::cerr << "Trouble-shooting:" << std::endl;
        std::cerr << "    Before running any vimba application you need to set GENICAM_GENTL64_PATH" << std::endl;
        std::cerr << "    to the path to the transport layer cti file. It is currently set to" << std::endl;
        std::cerr << "    " << getenv( "GENICAM_GENTL64_PATH" ) << "." << std::endl;
        std::cerr << std::endl;
        std::cerr << "Differences compared to gige-cat:" << std::endl;
        std::cerr << "    " << comma::verbose.app_name() << " does not support the integer id's supported by gige-cat." << std::endl;
        std::cerr << "    e.g. \"--id 180711\". Instead use one of the id's given by \"--list-cameras\"." << std::endl;
        std::cerr << std::endl;
        std::cerr << "    Many attributes have changed name or are configured in a different way." << std::endl;
        std::cerr << "    Use \"--list-attributes\" and \"--list-attributes --verbose\" to find the" << std::endl;
        std::cerr << "    appropriate new attribute." << std::endl;
        std::cerr << std::endl;
        std::cerr << "    " << comma::verbose.app_name() << " does not support image filters. Pipe to cv-cat instead." << std::endl;
        std::cerr << std::endl;
    }
    exit( 0 );
}

static comma::csv::format format_from_fields( const std::string& fields )
{
    std::vector< std::string > v = comma::split( fields, "," );
    comma::csv::format format;
    for( unsigned int i = 0; i < v.size(); ++i )
    {
        if( v[i] == "t" ) { format += "t"; }
        else { format += "ui"; }
    }
    return format;
}

// Word wrap input text to the given width, optionally inserting a prefix on each line
static std::string wrap( const std::string& text, size_t width = 80, const std::string& prefix = "")
{
    std::istringstream words( text );
    std::ostringstream wrapped;
    std::string word;

    size_t wrap_width = width - prefix.length();

    if( words >> word )
    {
        wrapped << word;
        size_t space_left = wrap_width - word.length();
        while( words >> word )
        {
            if( space_left < word.length() + 1 ) {
                wrapped << '\n' << prefix << word;
                space_left = wrap_width - word.length();
            } else {
                wrapped << ' ' << word;
                space_left -= word.length() + 1;
            }
        }
    }
    return wrapped.str();
}

struct ptp_status_writer
{
    static std::unique_ptr<ptp_status_writer> instance;
    comma::io::publisher publisher;
    std::stringstream ssbuf;
    comma::csv::output_stream<snark::vimba::ptp_status> os;
    ptp_status_writer(const std::string& name) : publisher(name,comma::io::mode::ascii), os(ssbuf,true)
    {
    }
    static void init(boost::optional<std::string> stream_name)
    {
        if(stream_name) { instance.reset(new ptp_status_writer(*stream_name)); }
    }
    inline void pwrite(const snark::vimba::ptp_status& ptp_status)
    {
        ssbuf.str("");
        os.write(ptp_status);
        std::string s=ssbuf.str();
        publisher.write(s.data(),s.size());
    }
    static inline void write(const snark::vimba::ptp_status& ptp_status)
    {
        if(instance) { instance->pwrite(ptp_status); }
    }
    static void output_fields()
    {
        std::cout<<comma::join( comma::csv::names<snark::vimba::ptp_status>(true), ',' )<<std::endl;
    }
    static void output_format()
    {
        std::cout<<comma::csv::format::value<snark::vimba::ptp_status>() << std::endl;
    }
};
std::unique_ptr<ptp_status_writer> ptp_status_writer::instance;


static void output_frame( const snark::vimba::frame& frame
                        , snark::cv_mat::serialization& serialization
                        , snark::vimba::camera& camera )
{
    snark::vimba::ptp_status ptp_status;
    snark::vimba::camera::timestamped_frame timestamped_frame = camera.frame_to_timestamped_frame( frame, ptp_status );
    if( !timestamped_frame.first.is_not_a_date_time() )
    {
        serialization.write( std::cout, timestamped_frame );
        ptp_status_writer::write(ptp_status);
    }
}

static void print_attribute_entry( const std::string& label, const std::string& value )
{
    std::string prefix( label.length() + 2, ' ' );
    std::cerr << label << ": " << wrap( value, 80, prefix ) << "\n";
}

static void print_attribute( const snark::vimba::attribute& attribute, bool verbose, const std::string& prefix="" )
{
    if( verbose )
    {
        print_attribute_entry( "Name          ", attribute.name() );
        print_attribute_entry( "Type          ", attribute.type_as_string() );
        print_attribute_entry( "Value         ", attribute.value_as_string() );
        print_attribute_entry( "Description   ", attribute.description() );
        if( !attribute.allowed_values().empty() )
            print_attribute_entry( "Allowed Values", attribute.allowed_values_as_string() );
        std::cerr << std::endl;
    }
    else
    {
        if( !prefix.empty() ) { std::cerr << prefix << ": "; }
        std::cerr << attribute.name() << "=" << attribute.value_as_string() << std::endl;
    }
}

static void print_stats( const snark::vimba::camera& camera )
{
    static const std::vector< std::string > stat_attributes
    {
        "StatFrameRate", "StatFrameDelivered", "StatFrameDropped", "StatFrameRescued",
        "StatFrameShoved", "StatFrameUnderrun", "StatLocalRate", "StatPacketErrors",
        "StatPacketMissed", "StatPackerReceived", "StatPacketRequested", "StatPacketResent",
        "StatTimeElapsed"
    };
    boost::optional< snark::vimba::attribute > a;
    for( const std::string& attribute : stat_attributes )
    {
        a = camera.get_attribute( attribute );
        if( a ) { print_attribute( *a, false, comma::verbose.app_name() ); }
    }
}


#define QUOTED( arg ) #arg
#define STRINGIZED( arg ) QUOTED( arg )

int main( int argc, char** argv )
{
    if( !getenv( "GENICAM_GENTL64_PATH" ))
    {
        setenv( "GENICAM_GENTL64_PATH", STRINGIZED( VIMBA_GENICAM_GENTL64_PATH ), 0 );
    }

    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" ) ) bash_completion( argc, argv );
        if( options.exists("--ptp-status-fields")) { ptp_status_writer::output_fields(); return 0; }
        if( options.exists("--ptp-status-format")) { ptp_status_writer::output_format(); return 0; }
        ptp_status_writer::init(options.optional<std::string>("--ptp-status"));
        

        if( options.exists( "--version" ))
        {
            VmbVersionInfo_t version = snark::vimba::system::version();
            std::cout << "Vimba library version: " << version.major << "." << version.minor << "." << version.patch << std::endl;
            std::cout << "GENICAM_GENTL64_PATH=" << getenv( "GENICAM_GENTL64_PATH" ) << std::endl;
            return 0;
        }

        snark::vimba::system system;                                   // Initialize the Vimba API

        if( options.exists( "--list-cameras" ))
        {
            AVT::VmbAPI::CameraPtrVector c = snark::vimba::system::cameras();
            if( comma::verbose ) { std::cout << "Cameras found: " << c.size() << std::endl; }
            for( AVT::VmbAPI::CameraPtrVector::const_iterator iter = c.begin(); iter != c.end(); ++iter )
            {
                snark::vimba::camera camera( *iter );
                snark::vimba::camera::name_values info = camera.info();
                if( comma::verbose )
                {
                    std::cout << "\nCamera ID    : " << info["id"]
                              << "\nCamera Name  : " << info["name"]
                              << "\nModel Name   : " << info["model"]
                              << "\nSerial Number: " << info["serial_number"]
                              << "\nInterface ID : " << info["interface_id"]  << std::endl;
                }
                else
                {
                    std::cout << "id=\"" << info["id"] << "\",name=\"" << info["name"]
                              << "\",serial=\"" << info["serial_number"] << "\"" << std::endl;
                }
            }
            return 0;
        }

        snark::vimba::camera camera( options.exists( "--id" )
                                   ? snark::vimba::camera( options.value<std::string>( "--id" ))
                                   : snark::vimba::camera( snark::vimba::system::open_first_camera()));

        if( options.exists( "--set-and-exit" ))
        {
            camera.set_features( options.value<std::string>( "--set-and-exit" ));
            return 0;
        }

        if( options.exists( "--set" ))
        {
            std::string set_option = options.value<std::string>( "--set" );
            if( set_option.front() == '"' ) { set_option.erase( 0, 1 ); set_option.erase( set_option.size() - 1 ); }
            camera.set_features( set_option );
        }

        if( options.exists( "--list-attributes" ))
        {
            std::string names_str = options.value< std::string >( "--list-attributes", "" );
            if( names_str.empty() )
            {
                std::vector< snark::vimba::attribute > attributes = camera.attributes();
                for( std::vector< snark::vimba::attribute >::const_iterator it = attributes.begin();
                     it != attributes.end();
                     ++it )
                {
                    print_attribute( *it, comma::verbose );
                }
            }
            else
            {
                std::vector< std::string > names = comma::split( names_str, ";," );
                for( std::vector< std::string >::const_iterator it = names.begin(); it != names.end(); ++it )
                {
                    boost::optional< snark::vimba::attribute > a = camera.get_attribute( *it );
                    if( a ) { print_attribute( *a, comma::verbose ); }
                }
            }
            return 0;
        }

        comma::verbose << "starting as pid " << getpid() << std::endl;
        std::string fields = options.value< std::string >( "--fields", default_fields );
        comma::csv::format format = format_from_fields( fields );
        bool header_only = false;
        bool check_frames = !options.exists( "--dont-check-frames" );
        unsigned int retries_on_no_frames = options.value< unsigned int >( "--retries-on-no-frames", default_retries_on_no_frames );
        if( options.exists( "--no-header" )) { fields = ""; }
        else { header_only = ( options.exists( "--header" )); }
        unsigned int num_frames = options.value< unsigned int >( "--frames-buffer-size,--num-frames", 3 );
        float acquisition_start_delay = options.value< unsigned int >( "--acquisition-start-delay,--acquisition-delay", 10. );
        snark::cv_mat::serialization serialization( fields, format, header_only );
        camera.set_acquisition_mode( snark::vimba::camera::ACQUISITION_MODE_CONTINUOUS );
        bool acquiring = true;
        unsigned int acquisition_restarts = 0;
        long long acquisition_time_elapsed = 0; // use timestamp instead
        int exit_code = 0;
        while( acquiring )
        {
            camera.start_acquisition( boost::bind( &output_frame, _1, boost::ref( serialization ), boost::ref( camera ) ), num_frames );
            comma::signal_flag is_shutdown;
            long frames_delivered = 0;
            do {
                sleep( 1 );
                ++acquisition_time_elapsed;
                // on the first time through, wait a few seconds before checking
                if( check_frames && acquisition_time_elapsed >= acquisition_start_delay )
                {
                    boost::optional< snark::vimba::attribute > frames_delivered_attribute = camera.get_attribute( "StatFrameDelivered" );
                    if( frames_delivered_attribute )
                    {
                        long frames_delivered_prev = frames_delivered;
                        frames_delivered = frames_delivered_attribute->int_value();
                        if( frames_delivered == frames_delivered_prev && !is_shutdown )
                        {
                            std::cerr << comma::verbose.app_name() << ": warning - no frames received in the last second" << std::endl;
                            if( comma::verbose ) { print_stats( camera ); }
                            if( acquisition_restarts == retries_on_no_frames )
                            {
                                std::cerr << comma::verbose.app_name() << ": ";
                                if( retries_on_no_frames > 0 )
                                {
                                    std::cerr << "we've tried restarting acquisition "
                                              << acquisition_restarts << " time"
                                              << ( acquisition_restarts == 1 ? "" : "s" );
                                }
                                else
                                {
                                    std::cerr << "we're not going to try restarting acquisition";
                                }
                                std::cerr << ". Exiting..." << std::endl;
                                acquiring = false;
                                exit_code = 1;
                            }
                            else
                            {
                                std::cerr << comma::verbose.app_name() << ": restarting acquisition" << std::endl;
                                acquisition_restarts++;
                            }
                            break;
                        }
                    }
                }
            } while( !is_shutdown );

            if( is_shutdown )
            {
                comma::verbose << "caught shutdown signal " << std::endl;
                acquiring = false;
            }
            comma::verbose << "exited acquisition loop" << std::endl;

            camera.stop_acquisition();
        }
        if( comma::verbose && acquisition_time_elapsed > 5 ) { print_stats( camera ); }
        comma::verbose << "exiting with code " << exit_code << std::endl;
        return exit_code;
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": unknown exception" << std::endl; }
    return 1;
}
