// Copyright (c) 2019 The University of Sydney

#include <algorithm>
#include <chrono>
#include <comma/application/signal_flag.h>
#include <comma/io/stream.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/name_value/parser.h>
#include <comma/timing/conversions.h>
#include "../../../../imaging/cv_mat/serialization.h"
#include "../../../../imaging/cv_mat/traits.h"
#include <librealsense2/rs.hpp>

namespace {

//comma::signal_flag signaled;

static void bash_completion( unsigned const ac, char const * const * av )
{
    static char const * const arguments[] =
    {
        " configure list reset",
        " --device",
        " --sensor",
        " --operations",
        " --output-fields",
        " --output-format",
        " --verbose"
    };
    std::cout << arguments << std::endl;
    exit( 0 );
}

static void operations( unsigned const indent_count = 0 )
{
    auto const indent = std::string( indent_count, ' ' );
    std::cerr << indent << "camera; acquire rgb camera data, output to stdout as cv-cat-formatted images" << std::endl;
    std::cerr << indent << "configure; configure sensor options from stdin (fields: index,value)" << std::endl;
    std::cerr << indent << "list; list devices" << std::endl;
    std::cerr << indent << "reset; reset devices" << std::endl;
}

static void usage( bool const verbose )
{
    std::cerr << R"(
show and configure realsense cameras

usage: " << comma::verbose.app_name() << " <operation> [<options>...]

operations: camera, configure, list, reset

options
    --device=<serial>; serial number(s) of device(s); TODO for camera
    --operations; print list of operations and exit
    --output-fields; print operation-dependent output fields to stdout and exit
    --output-format; print operation-dependent output format to stdout and exit

operations
    camera
        colour profiles
            width: 1920 height: 1880 fps: 8
            width: 1280 height: 720  fps: 6, 15, 30
            width:  640 height: 480  fps: 6, 15, 30
            width:  424 height: 240  fps: 6, 15, 30, 60
        options
            --fps=<framerate>; default=30
            --width=<pixels>; default=1280
            --image-format,--format=<format>; default=bgr
    configure
        options
            --sensor=<index>; serial number(s) of device(s).

examples
    realsense2-util camera | cv-cat 'view;null'
    realsense2-util configure --sensor=1 <<< '11,0'
    realsense2-util list
    realsense2-util reset --device 1234 --device 4321
)" << std::endl;
}



static void handle_info_options( comma::command_line_options const& options ) { if( options.exists( "--operations" ) ) { operations(); exit( 0 ); } }

static std::string get_operation( comma::command_line_options const& options )
{
    std::vector< std::string > unnamed = options.unnamed( "--verbose,-v,--operations", "-.*" );
    COMMA_ASSERT_BRIEF( unnamed.size() == 1, "expected one operation, got " << unnamed.size() << ": " << comma::join( unnamed, ' ' ) );
    return unnamed[0];
}

static void list_sensors( rs2::device const& device )
{
    auto sensors = device.query_sensors();
    for( auto& sensor : sensors )
    {
        std::cerr << "    Sensor: " << sensor.get_info( RS2_CAMERA_INFO_NAME ) << std::endl;
        std::cout << "        Options( index,name,description,default,min,max,current ):" << std::endl;
        for( int oi = 0; oi < static_cast< int >( RS2_OPTION_COUNT ); oi++ )
        {
            rs2_option option = static_cast< rs2_option >( oi );
            if( sensor.supports( option ) )
            {
                auto range = sensor.get_option_range( option );
                std::cerr << "            " << oi << ',' << option << ',' << sensor.get_option_description( option )
                    << ','<< range.def << ',' << range.min << ',' << range.max << ',' << sensor.get_option( option ) << std::endl;
            }
        }
        //std::cout << "        Stream Profiles( id, index, name, type):" << std::endl;
        //auto stream_profiles = sensor.get_stream_profiles();
        //for( auto const& sp : stream_profiles )
        //{
        //    std::cerr << "            " << sp.unique_id() << ',' << sp.stream_index() << ',' << sp.stream_name() << ',' << sp.stream_type() << std::endl;
        //}
    }
}

}

namespace configure
{

struct input_t
{
    int index;
    float value;
    input_t( void ) : index( 0 ), value( 0 ) {}
};

}

namespace comma { namespace visiting {

template <> struct traits< configure::input_t >
{
    template < typename K, typename V > static void visit ( K const&, configure::input_t& t, V& v )
    {
        v.apply( "index", t.index );
        v.apply( "value", t.value );
    }

    template < typename K, typename V > static void visit ( K const&, configure::input_t const& t, V& v )
    {
        v.apply( "index", t.index );
        v.apply( "value", t.value );
    }
};

} } // namespace comma { namespace visiting {

static std::map< std::string, rs2_format > image_formats =  { { "bgr8",  RS2_FORMAT_BGR8  }
                                                            , { "bgra8", RS2_FORMAT_BGRA8 }
                                                            , { "rgba8", RS2_FORMAT_RGBA8 }
                                                            , { "rgb8",  RS2_FORMAT_RGB8  }
                                                            , { "yuyv",  RS2_FORMAT_YUYV  } };

rs2_format image_format_from_string( const std::string& s )
{
    auto i = image_formats.find( s );
    COMMA_ASSERT_BRIEF( i != image_formats.end(), "expected image format, got: '" << s << "'" );
    return i->second;
}

int main( int ac, char* av[] )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--bash-completion" ) ) bash_completion( ac, av );
        handle_info_options( options );
        auto const verbose = options.exists( "--verbose" );
        auto operation = get_operation( options );
        auto device_ids = options.values< std::string >( "--device" );
        if( "configure" == operation )
        {
            rs2::context context;
            auto devices = context.query_devices();
            COMMA_ASSERT_BRIEF( devices.size() > 0, "please specify at least one --device" );
            COMMA_ASSERT_BRIEF( device_ids.size() == 1, "currently only one device can be configured at a time; got: " << device_ids.size() );
            rs2::device device;
            if( device_ids.size() == 1 )
            {
                device = devices[0];
            }
            else
            {
                for( auto const& dev : devices ) // hm... so, we do handle multiple devices after all...
                {
                    auto device_id = std::string( dev.get_info( RS2_CAMERA_INFO_SERIAL_NUMBER ) );
                    if( device_id == device_ids[0] ) { device = dev; break; }
                }
                COMMA_ASSERT_BRIEF( device, "device with serial number '"<< device_ids[0] << "' not found" );
            }
            auto sensors = device.query_sensors();
            auto const sensor_index = options.value< unsigned >( "--sensor" );
            if( sensors.size() <= sensor_index )
            { std::cerr << comma::verbose.app_name() << ": sensor index " << sensor_index << " greater than maximum: "<< sensors.size() << std::endl; return 1; }
            auto sensor = sensors[ sensor_index ];
            //auto const sensor_name = sensor.get_info( RS2_CAMERA_INFO_NAME );

            comma::csv::options csv;
            csv.fields = "index,value";
            comma::csv::input_stream< configure::input_t > istrm( std::cin, csv );
            while( istrm.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                auto record = istrm.read(); if( !record ) break;
                rs2_option option = static_cast< rs2_option >( record->index );
                //if( !sensor.supports( option ) )
                //{
                //    std::cerr << comma::verbose.app_name() << ": sensor '" << sensor_name << "' at index " << sensor_index
                //        << " does not support option " << option << " with index" << record->index << std::endl;
                //}
                //auto range = sensor.get_option_range( option );
                //if( range.min > record->value || range.max < record->value )
                //{
                //    std::cerr << comma::verbose.app_name() << ": given value " << record->value << " is out of range (" << range.min << ',' << range.max
                //        << "for option " << option << " with index" << record->index << std::endl;
                //}
                sensor.set_option( option, record->value );

            }
            return 0;
        }
        if( "list" == operation )
        {
            rs2::context context;
            auto devices = context.query_devices();
            for( auto const& dev : devices )
            {
                auto device_id = std::string( dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) );
                if( device_ids.empty() || device_ids.end() != std::find( device_ids.begin(), device_ids.end(), device_id ) )
                {
                    std::cout << dev.get_info(RS2_CAMERA_INFO_NAME)
                        << ','<< device_id
                        << ','<< dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT) << std::endl;
                    
                    if( verbose ) { list_sensors( dev ); }
                }
            }
            if( !verbose ) { std::cerr << comma::verbose.app_name() << ": pass --verbose for sensor information." << std::endl; }
            return 0;
        }
        if( "reset" == operation )
        {
            rs2::context context;
            auto devices = context.query_devices();
            for( auto dev : devices )
            {
                auto device_id = std::string( dev.get_info( RS2_CAMERA_INFO_SERIAL_NUMBER ) );
                if( device_ids.empty() || device_ids.end() != std::find( device_ids.begin(), device_ids.end(), device_id ) ) { dev.hardware_reset(); }
            }
            return 0;
        }
        if( operation == "camera" )
        {
            COMMA_ASSERT_BRIEF( !options.exists( "--device" ), "camera: --device: todo" );
            rs2::pipeline pipe;
            rs2::config config;
            unsigned int width = options.value( "--width", 1280 );
            unsigned int fps = options.value( "--fps", 30 );
            unsigned int height = 0;
            switch( width )
            {
                case 1920:
                    height = 1080;
                    COMMA_ASSERT_BRIEF( fps == 8, "expected --fps of 8 for width " << width << " got: " << fps );
                    break;
                case 1280:
                    height = 720;
                    COMMA_ASSERT_BRIEF( fps == 6 || fps == 15 || fps == 30, "expected --fps of 6, 15, or 30 for width " << width << " got: " << fps );
                    break;
                case 640:
                    height = 480;
                    COMMA_ASSERT_BRIEF( fps == 6 || fps == 15 || fps == 30, "expected --fps of 6, 15, or 30 for width " << width << " got: " << fps );
                    break;
                case 424:
                    height = 240;
                    COMMA_ASSERT_BRIEF( fps == 6 || fps == 15 || fps == 30 || fps == 60, "expected --fps of 6, 15, 30, or 60 for width " << width << " got: " << fps );
                    break;
                default:
                    COMMA_THROW_BRIEF( comma::exception, "unsupported --width=" << width );
            }
            comma::saymore() << "camera: aquisition: configuring for width: " << width << " height: " << height << " fps: " << fps << "..." << std::endl;
            config.enable_stream( RS2_STREAM_COLOR, width, height, image_format_from_string( options.value< std::string >( "--image-format", "bgr8" ) ), fps );
            comma::saymore() << "camera: aquisition: starting..." << std::endl;
            pipe.start(config);
            comma::saymore() << "camera: aquisition: running..." << std::endl;
            comma::signal_flag is_shutdown;
            snark::cv_mat::serialization::header h;
            snark::cv_mat::serialization output;
            comma::csv::binary_output_stream< snark::cv_mat::serialization::header > header_stream( std::cout, h.default_format(), h.default_fields() );
            while( std::cout.good() && !is_shutdown )
            {
                rs2::frameset frames = pipe.wait_for_frames();
                rs2::video_frame color_frame = frames.get_color_frame();
                if( !color_frame ) { continue; }
                int width = color_frame.get_width();
                int height = color_frame.get_height();
                h.timestamp = comma::timing::as_ptime( std::chrono::system_clock::now() );
                h.rows = height;
                h.cols = width;
                h.type = CV_8UC3;
                h.size = height * width * 3;
                header_stream.write( h );
                std::cout.write( reinterpret_cast< const char* >( color_frame.get_data() ), h.size );
                std::cout.flush();
            }
            comma::saymore() << "camera: aquisition: done" << std::endl;
            return 0;
        }
        comma::say() << ": expected operation; got: '" << operation << "'" << std::endl;
        return 1;
    }
    catch( rs2::error& ex ) { std::cerr << comma::verbose.app_name() << ": realsense exception: " << ex.what() << " (maybe try smaller width or lower fps?)" << std::endl; }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": unknown exception" << std::endl; }
    return 1;
}

