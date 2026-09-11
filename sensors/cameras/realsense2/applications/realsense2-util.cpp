// Copyright (c) 2019 The University of Sydney

#include <algorithm>
#include <chrono>
#include <librealsense2/rs.hpp>
#include <comma/application/signal_flag.h>
#include <comma/io/stream.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/name_value/parser.h>
#include <comma/name_value/serialize.h>
#include <comma/timing/conversions.h>
#include "../../../../imaging/camera/pinhole.h"
#include "../../../../imaging/camera/traits.h"
#include "../../../../imaging/cv_mat/serialization.h"
#include "../../../../imaging/cv_mat/traits.h"

namespace {

//comma::signal_flag signaled;

static void bash_completion( unsigned const ac, char const * const * av )
{
    static char const * const arguments[] =
    {
        " configure list reset",
        " --device",
        " --sensor",
        " --verbose"
    };
    std::cout << arguments << std::endl;
    exit( 0 );
}

static void usage( bool const verbose )
{
    std::cerr << R"(
show and configure realsense cameras

usage: relasense2-util <operation> [<options>...]

operations: color, configure, intrinsics, list, reset
    color; acquire colour camera data, output to stdout as cv-cat-formatted images
    configure; configure sensor options from stdin (fields: index,value)
    profile; output sensor profile and exit
    list; list devices
    reset; reset devices

options
    --device=<serial>; serial number(s) of device(s); TODO for camera

operations
    color
        colour profiles
            width: 1920 height: 1880 fps: 8
            width: 1280 height: 720  fps: 6, 15, 30
            width:  640 height: 480  fps: 6, 15, 30
            width:  424 height: 240  fps: 6, 15, 30, 60
        options
            --fps=<framerate>; default=30
            --width=<pixels>; default=1280; sensor width in pixels
            --image-format,--format=<format>; default=bgr

    configure
        options
            --sensor=<index>; serial number(s) of device(s)
            --width=<pixels>; default=1280; sensor width in pixels
    
    profile
        --intrinsics; output sensor intrinsics as json
        --minified; output as one-line minified json
        --sensor=<which>; default=color; choices: color, ... todo

examples
    realsense2-util camera | cv-cat 'view;null'
    realsense2-util configure --sensor=1 <<< '11,0'
    realsense2-util list
    realsense2-util reset --device 1234 --device 4321
)" << std::endl;
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

// rs-enumerate-devices -c
// ...
//  Intrinsic of "Color" / 640x480 / {YUYV/RGB8/BGR8/RGBA8/BGRA8}
//   Width:        640
//   Height:       480
//   PPX:          327.709014892578
//   PPY:          251.378204345703
//   Fx:           607.996337890625
//   Fy:           607.972229003906
//   Distortion:   Inverse Brown Conrady
//   Coeffs:       0       0       0       0       0  
//   FOV (deg):    55.51 x 43.07

static unsigned int _height( unsigned int width )
{
    unsigned int height;
    switch( width )
    {
        case 1920: height = 1080; break;
        case 1280: height = 720; break;
        case 640: height = 480; break;
        case 424: height = 240; break;
        default: COMMA_THROW_BRIEF( comma::exception, "unsupported --width=" << width );
    }
    return height;
}

int main( int ac, char* av[] )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--bash-completion" ) ) bash_completion( ac, av );
        auto const verbose = options.exists( "--verbose" );
        std::vector< std::string > unnamed = options.unnamed( "--verbose,-v,--intrinsics,--minified", "-.*" );
        COMMA_ASSERT_BRIEF( unnamed.size() == 1, "expected one operation, got " << unnamed.size() << ": " << comma::join( unnamed, ' ' ) );
        auto operation = unnamed[0];
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
        if( operation == "color" )
        {
            COMMA_ASSERT_BRIEF( !options.exists( "--device" ), "camera: --device: todo" );
            rs2::pipeline pipe;
            rs2::config config;
            unsigned int width = options.value( "--width", 1280 );
            unsigned int fps = options.value( "--fps", 30 );
            unsigned int height = _height( width );
            switch( width )
            {
                case 1920: COMMA_ASSERT_BRIEF( fps == 8, "expected --fps of 8 for width " << width << " got: " << fps ); break;
                case 1280: COMMA_ASSERT_BRIEF( fps == 6 || fps == 15 || fps == 30, "expected --fps of 6, 15, or 30 for width " << width << " got: " << fps ); break;
                case 640: COMMA_ASSERT_BRIEF( fps == 6 || fps == 15 || fps == 30, "expected --fps of 6, 15, or 30 for width " << width << " got: " << fps ); break;
                case 424: COMMA_ASSERT_BRIEF( fps == 6 || fps == 15 || fps == 30 || fps == 60, "expected --fps of 6, 15, 30, or 60 for width " << width << " got: " << fps ); break;
                default: COMMA_THROW_BRIEF( comma::exception, "unsupported --width=" << width );
            }
            comma::saymore() << "color: aquisition: enabling for width: " << width << " height: " << height << " fps: " << fps << "..." << std::endl;
            config.enable_stream( RS2_STREAM_COLOR, width, height, image_format_from_string( options.value< std::string >( "--image-format", "bgr8" ) ), fps );
            comma::saymore() << "color: aquisition: starting..." << std::endl;
            pipe.start(config);
            comma::saymore() << "color: aquisition: running..." << std::endl;
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
        if( operation == "profile" )
        {
            rs2::context context;
            auto devices = context.query_devices();
            COMMA_ASSERT_BRIEF( devices.size() > 0, "no realsense devices found" );
            COMMA_ASSERT_BRIEF( devices.size() == 1, "found " << devices.size() << " realsense devices; only 1 currently supported: todo, just ask" );
            rs2::device device = devices[0]; // todo: parametrise on devices
            rs2::sensor sensor;
            auto sensor_option = options.value< std::string >( "--which", "color" );
            bool minified = options.exists( "--minified" );
            auto which = RS2_STREAM_COLOR;
            COMMA_ASSERT_BRIEF( sensor_option == "color", "only --which=color implemented, others: todo, just ask" );
            bool found = false;
            for( auto&& s : device.query_sensors() )
            {
                for( auto&& profile: s.get_stream_profiles() ) { if( profile.stream_type() == which ) { sensor = s; found = true; break; } }
                if( found ) { break; }
            }
            COMMA_ASSERT_BRIEF( found, sensor_option << "profile: sensor not found" );
            if( options.exists( "--intrinsics" ) )
            {
                unsigned int width = options.value< unsigned int >( "--width" );
                unsigned int height = _height( width );
                for( auto&& profile: sensor.get_stream_profiles() ) // todo! quick and dirty for now; make it generic! 
                {
                    if( !profile.is< rs2::video_stream_profile >() ) { continue; }
                    auto video_profile = profile.as< rs2::video_stream_profile >();
                    if( video_profile.width() != int( width ) && video_profile.height() != int( height ) ) { continue; }
                    rs2_intrinsics intrinsics = video_profile.get_intrinsics();
                    snark::camera::pinhole::config_t config;
                    config.focal_length = ( intrinsics.fx + intrinsics.fy ) / 2.;
                    config.image_size = Eigen::Vector2i( width, height );
                    config.principal_point = Eigen::Vector2d( intrinsics.ppx, intrinsics.ppy );
                    config.distortion = snark::camera::pinhole::config_t::distortion_t();
                    config.distortion->radial.k1 = intrinsics.coeffs[0];
                    config.distortion->radial.k2 = intrinsics.coeffs[1];
                    config.distortion->radial.k3 = intrinsics.coeffs[4];
                    config.distortion->tangential.p1 = intrinsics.coeffs[2];
                    config.distortion->tangential.p2 = intrinsics.coeffs[3];
                    comma::write_json( config, std::cout, !minified );
                    return 0;
                }
                COMMA_THROW_BRIEF( comma::exception, "profile: intrinsics for image width: " << width << " height: " << height << " not found" );
            }
            COMMA_THROW_BRIEF( comma::exception, "profile: generic profile output: todo" );
        }
        comma::say() << ": expected operation; got: '" << operation << "'" << std::endl;
        return 1;
    }
    catch( rs2::error& ex ) { std::cerr << comma::verbose.app_name() << ": realsense exception: " << ex.what() << " (maybe try smaller width or lower fps?)" << std::endl; }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": unknown exception" << std::endl; }
    return 1;
}

