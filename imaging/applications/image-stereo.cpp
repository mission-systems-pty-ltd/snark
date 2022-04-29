// Copyright (c) 2019 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <comma/application/command_line_options.h>
#include <comma/csv/ascii.h>
#include <comma/csv/stream.h>
#include <comma/name_value/serialize.h>
#include "../../visiting/traits.h"
#include "../camera/stereo.h"
#include "../camera/traits.h"

void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "take on stdin pixels, perform an operation based a pair of pinhole cameras" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat pixels.csv | image-stereo <operation> <options> > points.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations" << std::endl;
    std::cerr << "    to-cartesian: take on stdin pixels on stereo pair, undistort image, append pixel's cartesian coordinates in a given frame" << std::endl;
    std::cerr << "        fields: default: first/x,first/y,second/x,second/y" << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --force,--permissive: discard invalid input instead of exiting with error" << std::endl;
    std::cerr << "            --input-fields: output input fields to stdout and exit" << std::endl;
    std::cerr << "            --output-fields: output appended fields for given operation and exit" << std::endl;
    std::cerr << "            --output-format: output appended fields for given operation and exit" << std::endl;
    std::cerr << "        examples" << std::endl;
    std::cerr << "            basics" << std::endl;
    std::cerr << "                echo 0,500,1000,500 | image-stereo to-cartesian --camera-config <( echo '{\"image_size\":{\"x\":1000,\"y\":1000},\"focal_length\":500}' ) --first-pose 0,-10,0,0,0,0 --second-pose 0,10,0,0,0,0" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    rectify-map: output opencv-style image rectification maps" << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --image-size=[<size>]; output image size as <width>,<height>, default: same as input image size" << std::endl;
    //std::cerr << "            --output-first; todo: output maps only for the first camera; convenience option" << std::endl;
    //std::cerr << "            --output-second; todo: output maps only for the second camera; convenience option" << std::endl;
    std::cerr << "            --type=<type>; default=CV_16SC2; map element type, values: CV_16SC2 or w, CV_32FC1 or f, CV_32FC2 or 2f" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --verbose,-v: more output" << std::endl;
    std::cerr << std::endl;
    std::cerr << "configuration options" << std::endl;
    std::cerr << "    --baseline=[<meters>]: convenience option, todo" << std::endl;
    std::cerr << "    --camera-config=<path>: camera config for both cameras; <path>: <filename> or <filename>:<path>" << std::endl;
    std::cerr << "    --config,-c=<path>: camera pair config; <path>: <filename> or <filename>:<path>" << std::endl;
    std::cerr << "    --config-fields: output config fields to stdout and exit" << std::endl;
    std::cerr << "    --first-camera-config,--first-config=<path>: first camera config; <path>: <filename> or <filename>:<path>" << std::endl;
    std::cerr << "    --first-pose=<x,y,z,roll,pitch,yaw>: pose of the first camera; default: whatever is in camera config" << std::endl;
    std::cerr << "    --pose-frame,--pose-frame=<frame>; default=north-east-down; down-left-forward: todo" << std::endl;
    std::cerr << "    --second-camera-config,--second-config=<path>: second camera config; <path>: <filename> or <filename>:<path>" << std::endl;
    std::cerr << "    --second-pose=<x,y,z,roll,pitch,yaw>: pose of the second camera; default: whatever is in camera config" << std::endl;
    std::cerr << "    --verbose,-v; more verbose output" << std::endl;
    std::cerr << std::endl;
    std::cerr << "csv options" << std::endl;
    std::cerr << comma::csv::options::usage( verbose ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    todo" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

static bool verbose;

template < typename S, typename T > static void output_details( const comma::command_line_options& options )
{
    if( options.exists( "--input-fields" ) ) { std::cout << comma::join( comma::csv::names< S >(), ',' ) << std::endl; exit( 0 ); }
    if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< T >(), ',' ) << std::endl; exit( 0 ); }
    if( options.exists( "--output-format" ) ) { std::cout << comma::csv::format::value< T >() << std::endl; exit( 0 ); }
}

template < typename T >
static T read_config( const std::string& config_parameters )
{
    T config;
    const std::vector< std::string >& v = comma::split( config_parameters, ":#@" );
    if( v.size() == 1 ) { comma::read( config, config_parameters, true ); }
    else { comma::read( config, config_parameters.substr( 0, config_parameters.size() - v.back().size() - 1 ), v.back(), true ); }
    return config;
}

static snark::camera::pinhole::config_t make_sample_pinhole_config()
{
    snark::camera::pinhole::config_t config;
    config.focal_length = 0.1;
    config.image_size = Eigen::Vector2i( 1000, 2000 );
    config.sensor_size = Eigen::Vector2d( 0.1, 0.2 );
    config.distortion = snark::camera::pinhole::config_t::distortion_t( snark::camera::pinhole::config_t::distortion_t::radial_t( 0.001, -0.0002, 0.003 ), snark::camera::pinhole::config_t::distortion_t::tangential_t( 0.0004, -0.0005 ) );
    return config;
}

static snark::camera::stereo::pair::config_t make_sample_config()
{
    snark::camera::stereo::pair::config_t config;
    config.first.pinhole = config.second.pinhole = make_sample_pinhole_config();
    config.first.pose = config.second.pose = snark::pose( Eigen::Vector3d( 1, 2, 3 ), snark::roll_pitch_yaw( 0.1, 0.2, 0.3 ) );
    return config;
}

static snark::camera::stereo::pair make_pair( const comma::command_line_options& options )
{
    options.assert_mutually_exclusive( "--config,-c", "--camera-config,--first-camera-config,--first-config,--second-camera-config,--second-config" );
    options.assert_mutually_exclusive( "--camera-config", "--first-camera-config,--first-config,--second-camera-config,--second-config" );
    if( options.exists( "--baseline" ) ) { std::cerr << "image-stereo: --baseline: not implemented" << std::endl; exit( 1 ); }
    snark::camera::stereo::pair::config_t config;
    if( options.exists( "--config,-c" ) )
    {
        const auto& c = options.value< std::string >( "--config,-c" );
        if( verbose ) { std::cerr << "image-stereo: using config: " << c << std::endl; }
        config = read_config< snark::camera::stereo::pair::config_t >( c );
    }
    else if( options.exists( "--camera-config" ) )
    {
        const auto& c = options.value< std::string >( "--camera-config" );
        if( verbose ) { std::cerr << "image-stereo: using same config for both cameras: " << c << std::endl; }
        config.first.pinhole = config.second.pinhole = read_config< snark::camera::pinhole::config_t >( c );
    }
    else if( options.exists( "--first-camera-config,--first-config" ) )
    {
        const auto& f = options.value< std::string >( "--first-camera-config,--first-config" );
        const auto& s = options.value< std::string >( "--second-camera-config,--second-config" );
        if( verbose ) { std::cerr << "image-stereo: using first camera config: " << f << " and second camera config: " << s << std::endl; }
        config.first.pinhole = read_config< snark::camera::pinhole::config_t >( f );
        config.second.pinhole = read_config< snark::camera::pinhole::config_t >( s );
    }
    config.first.pinhole.validate();
    config.second.pinhole.validate();
    auto first_pose = options.optional< std::string >( "--first-pose" );
    if( first_pose ) { config.first.pose = comma::csv::ascii< snark::pose >().get( first_pose ); }
    auto second_pose = options.optional< std::string >( "--second-pose" );
    if( second_pose ) { config.second.pose = comma::csv::ascii< snark::pose >().get( second_pose ); }
    return snark::camera::stereo::pair( config );
}

struct point_t: public Eigen::Vector2d
{
    snark::pose pose;
    point_t(): Eigen::Vector2d( Eigen::Vector2d::Zero() ) {}
};

typedef std::pair< point_t, point_t > input_t;

namespace comma { namespace visiting {

template <> struct traits< point_t >
{
    template < typename Key, class Visitor > static void visit( const Key& k, point_t& t, Visitor& v )
    {
        comma::visiting::traits< Eigen::Vector2d >::visit( k, t, v );
        v.apply( "pose", t.pose );
    }

    template < typename Key, class Visitor > static void visit( const Key& k, const point_t& t, Visitor& v )
    {
        comma::visiting::traits< Eigen::Vector2d >::visit( k, t, v );
        v.apply( "pose", t.pose );
    }
};

} } // namespace comma { namespace visiting {

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--config-fields" ) ) { std::cout << comma::join( comma::csv::names( true, make_sample_config() ), '\n' ) << std::endl; return 0; }
        verbose = options.exists( "--verbose,-v" );
        const std::vector< std::string >& unnamed = options.unnamed( "--force,--permissive,--input-fields,--output-fields,--output-format,--flush", "-.*" );
        if( unnamed.empty() ) { std::cerr << "image-stereo: please specify operation" << std::endl; return 1; }
        std::string operation = unnamed[0];
        comma::csv::options csv( options );
        bool force = options.exists( "--force,--permissive" );
        if( operation == "to-cartesian" )
        {
            if( csv.fields.empty() ) { csv.fields = "first/x,first/y,second/x,second/y"; }
            output_details< input_t, std::pair< Eigen::Vector3d, Eigen::Vector3d > >( options );
            auto pair = make_pair( options );
            input_t sample;
            sample.first.pose = pair.first().pose;
            sample.second.pose = pair.second().pose;
            comma::csv::input_stream< input_t > is( std::cin, csv, sample );
            comma::csv::output_stream< std::pair< Eigen::Vector3d, Eigen::Vector3d > > os( std::cout, csv.binary(), true, csv.flush );
            if( !csv.binary() ) { os.ascii().precision( csv.precision ); }
            comma::csv::tied< input_t, std::pair< Eigen::Vector3d, Eigen::Vector3d > > tied( is, os );
            while( is.ready() || std::cin.good() )
            {
                const input_t* p = is.read();
                if( !p ) { break; }
                if( comma::math::equal( ( p->first.pose.translation - p->second.pose.translation ).norm(), 0.01 ) )
                {
                    if( force ) { std::cerr << "image-stereo: expected two camera positions with sufficient paralax; got first: " << p->first.pose.translation.transpose() << " second: " << p->second.pose.translation.transpose() << "; discarded" << std::endl; }
                    else { std::cerr << "image-stereo: expected two camera positions with sufficient paralax; got first: " << p->first.pose.translation.transpose() << " second: " << p->second.pose.translation.transpose() << "; use --force to override" << std::endl; return 1; }
                }
                tied.append( pair.to_cartesian( p->first, p->second, p->first.pose, p->second.pose ) );
            }
            return 0;
        }
        if( operation == "rectify-map" )
        {
            if( options.exists( "--output-first" ) || options.exists( "--output-second" ) ) { std::cerr << "image-stereo: rectify-map: --output-first, --output-second: todo" << std::endl; return 1; }
            const std::string& t = options.value< std::string >( "--type", "w" );
            unsigned int width, height;
            std::tie( width, height ) = comma::csv::ascii< std::pair< unsigned int, unsigned int > >().get( options.value< std::string >( "--image-size" ) );
            int type;
            if( t == "w" || t == "CV_16SC2" ) { type = CV_16SC2; }
            else if( t == "f" || t == "CV_32FC1" ) { type = CV_32FC1; }
            else if( t == "2f" || t == "CV_32FC2" ) { type = CV_32FC2; }
            else { std::cerr << "image-stereo: rectify-map: expected --type=<type>; got: \"" << t << "\"" << std::endl; return 1; }
            const auto& maps = make_pair( options ).rectify_map( width, height, type );
            for( const auto& m: maps.first ) { std::cout.write( reinterpret_cast< const char* >( m.datastart ), m.dataend - m.datastart ); }
            for( const auto& m: maps.second ) { std::cout.write( reinterpret_cast< const char* >( m.datastart ), m.dataend - m.datastart ); }
            return 0;
        }
        std::cerr << "image-stereo: expected operation; got: '"<< operation << "'" << std::endl;
    }
    catch( std::exception& ex ) { std::cerr << "image-stereo: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "image-stereo: unknown exception" << std::endl; }
    return 1;
}
