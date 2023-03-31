// Copyright (c) 2011 The University of Sydney

/// @authors vsevolod vlaskine, zhe xu

#include <boost/array.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <comma/csv/stream.h>
#include <comma/math/compare.h>
#include <comma/name_value/ptree.h>
#include <comma/name_value/serialize.h>
#include "../camera/pinhole.h"
#include "../camera/metashape.h"
#include "../camera/traits.h"

void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "take on stdin pixels, perform an operation based on pinhole camera model" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat pixels.csv | image-pinhole <operation> <options> > points.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations" << std::endl;
    std::cerr << "    config: config operations" << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --from <what>; default=pinhole; what: pinhole, metashape, photoscan (same as metashape)" << std::endl;
    std::cerr << "            --to <what>; default=pinhole; what: pinhole, metashape, photoscan (same as metashape)" << std::endl;
    std::cerr << "    distort: take on stdin undistorted pixels, append their distorted values (uses distortion map file)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    distortion-map,undistort-rectify-map: build distortion map from camera parameters in config and write to stdout (binary image matrix of map x, map y)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    to-cartesian: take on stdin pixels, undistort image, append pixel's cartesian coordinates in camera frame" << std::endl;
    std::cerr << "        --normalize: normalize cartesian coordinates in camera frame" << std::endl;
    std::cerr << "        --fields: x,y,z,range" << std::endl;
    std::cerr << "            x,y: pixel coordinates" << std::endl;
    std::cerr << "            range: if present, the point in sensor plane will point out of image plane with a given range" << std::endl;
    std::cerr << "            z: if present, the point in sensor plane will be projected to the plane parallel to sensor plane with a given z coordinate (it may be confusing, i know)" << std::endl;
    std::cerr << "            default: x,y" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    to-pixels: take on stdin cartesian coordinates in camera frame, append their coordinates in pixels" << std::endl;
    std::cerr << "        --clip,--discard: discard pixels outside of image and behind camera" << std::endl;
    std::cerr << "        --fields: x,y,z; if z is given, the input points will be projected to the sensor plane; default: x,y" << std::endl;
    std::cerr << "            range field support: todo, just ask" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    undistort: take on stdin pixels, append their undistorted values" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: print help; --help --verbose for more help" << std::endl;
    std::cerr << "    --camera-config,--camera,--config,-c=<parameters>: camera configuration" << std::endl;
    std::cerr << "        <parameters>: filename of json or path-value configuration file or ';'-separated path-value pairs" << std::endl;
    std::cerr << "                      e.g: --config=\"focal_length=123;image_size/x=222;image_size/y=123.1;...\"" << std::endl;
    std::cerr << "    --input-fields: output input fields for given operation and exit" << std::endl;
    std::cerr << "    --output-config,--sample-config,--config-sample: output sample config and exit" << std::endl;
    std::cerr << "    --output-config-fields,--config-fields: output config fields and exit" << std::endl;
    std::cerr << "    --output-fields: output appended fields for given operation and exit" << std::endl;
    std::cerr << "    --output-format: output appended fields binary format for given operation and exit" << std::endl;
    std::cerr << "    --verbose,-v: more output" << std::endl;
    std::cerr << std::endl;
    std:: cerr << snark::camera::pinhole::usage() << std::endl;
    if( verbose ) { std::cerr << comma::csv::options::usage() << std::endl; }
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    todo" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

struct input_t : public Eigen::Vector3d
{
    double range;
    input_t(): Eigen::Vector3d( Eigen::Vector3d::Zero() ), range( 0 ) {}
};

namespace comma { namespace visiting {

template <> struct traits< input_t >
{
    template < typename Key, class Visitor > static void visit( const Key& k, input_t& t, Visitor& v )
    {
        comma::visiting::traits< Eigen::Vector3d >::visit( k, t, v );
        v.apply( "range", t.range );
    }
    
    template < typename Key, class Visitor > static void visit( const Key& k, const input_t& t, Visitor& v )
    {
        comma::visiting::traits< Eigen::Vector3d >::visit( k, t, v );
        v.apply( "range", t.range );
    }
};
    
} } // namespace comma { namespace visiting {

template < typename S, typename T > static void output_details( const comma::command_line_options& options )
{
    if( options.exists( "--input-fields" ) ) { std::cout << comma::join( comma::csv::names< S >(), ',' ) << std::endl; exit( 0 ); }
    if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< T >(), ',' ) << std::endl; exit( 0 ); }
    if( options.exists( "--output-format" ) ) { std::cout << comma::csv::format::value< T >() << std::endl; exit( 0 ); }
}

static snark::camera::pinhole::config_t make_config( const std::string& config_parameters )
{
    snark::camera::pinhole::config_t config;
    if( config_parameters.find_first_of( '=' ) == std::string::npos )
    {
        const std::vector< std::string >& v = comma::split( config_parameters, ":#@" );
        if( v.size() == 1 ) { comma::read( config, config_parameters, true ); }
        else { comma::read( config, config_parameters.substr( 0, config_parameters.size() - v.back().size() - 1 ), v.back(), true ); }
        config.validate();
    }
    else
    {
        boost::property_tree::ptree p = comma::property_tree::from_path_value_string( config_parameters, '=', ';', comma::property_tree::path_value::no_check, true );
        comma::from_ptree from_ptree( p, true );
        comma::visiting::apply( from_ptree ).to( config );
    }
    return config;
}

static snark::camera::pinhole::config_t make_sample_config()
{
    snark::camera::pinhole::config_t config;
    config.focal_length = 0.1;
    config.image_size = Eigen::Vector2i( 1000, 2000 );
    config.sensor_size = Eigen::Vector2d( 0.1, 0.2 );
    config.distortion = snark::camera::pinhole::config_t::distortion_t( snark::camera::pinhole::config_t::distortion_t::radial_t( 0.001, -0.0002, 0.003 ), snark::camera::pinhole::config_t::distortion_t::tangential_t( 0.0004, -0.0005 ) );
    return config;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--output-config,--sample-config,--config-sample" ) ) { comma::write_json( make_sample_config(), std::cout ); return 0; }
        if( options.exists( "--output-config-fields,--config-fields" ) ) { for( const auto& field: comma::csv::names( true, make_sample_config() ) ) { std::cout << field << std::endl; } return 0; }
        const std::vector< std::string >& unnamed = options.unnamed( "--input-fields,--output-fields,--output-format,--verbose,-v,--flush,--clip,--keep,--normalize", "-.*" );
        if( unnamed.empty() ) { std::cerr << "image-pinhole: please specify operation" << std::endl; return 1; }
        std::string operation = unnamed[0];
        if( operation == "config" ) // quick and dirty for now
        {
            std::string from = options.value< std::string >( "--from", "pinhole" );
            std::string to = options.value< std::string >( "--to", "pinhole" );
            if( from == "pinhole" )
            {
                const auto& source = comma::read< snark::camera::pinhole::config_t >( std::cin );
                if( to == "pinhole" ) { comma::write_json( source, std::cout ); return 0; }
            }
            if( from == "metashape" || from == "photoscan" )
            {
                const auto& source = comma::read< snark::metashape::camera::pinhole >( std::cin );
                if( to == "pinhole" ) { comma::write_json( source.calibration.as< snark::camera::pinhole::config_t >(), std::cout ); return 0; }
            }
            std::cerr << "image-pinhole: --from " << from << " --to " << to << ": not implemented, just ask..." << std::endl;
            return 1;
        }
        comma::csv::options csv( options );
        if( operation == "to-cartesian" )
        {
            if( csv.fields.empty() ) { csv.fields = "x,y"; }
            output_details< input_t, Eigen::Vector3d >( options );
            snark::camera::pinhole pinhole( make_config( options.value< std::string >( "--camera-config,--camera,--config,-c" ) ) );
            bool normalize = options.exists( "--normalize" );
            bool has_z = csv.has_field( "z" );
            bool has_range = csv.has_field( "range" );
            if( has_z && has_range ) { std::cerr << "image-pinhole: to-cartesian: expected either z or range field; got both" << std::endl; return 1; }
            comma::csv::input_stream< input_t > is( std::cin, csv );
            comma::csv::output_stream< Eigen::Vector3d > os( std::cout, csv.binary(), false, csv.flush );
            comma::csv::tied< input_t, Eigen::Vector3d > tied( is, os );
            while( is.ready() || std::cin.good() )
            {
                const input_t* p = is.read();
                if( !p ) { break; }
                const Eigen::Vector3d& q = pinhole.to_cartesian( Eigen::Vector2d( p->x(), p->y() ) );
                tied.append( normalize ? q.normalized()
                           : has_z ? Eigen::Vector3d( q * ( p->z() / -pinhole.config().focal_length ) )
                           : has_range ? Eigen::Vector3d( q.normalized() * -p->range )
                           : q );
            }
            return 0;
        }
        if( operation == "to-pixels" )
        {
            output_details< Eigen::Vector3d, Eigen::Vector2d >( options );
            snark::camera::pinhole pinhole( make_config( options.value< std::string >( "--camera-config,--camera,--config,-c" ) ) );
            if( csv.fields.empty() ) { csv.fields = "x,y"; }
            comma::csv::input_stream< Eigen::Vector3d > is( std::cin, csv, Eigen::Vector3d::Zero() );
            comma::csv::output_stream< Eigen::Vector2d > os( std::cout, csv.binary(), false, csv.flush );
            comma::csv::tied< Eigen::Vector3d, Eigen::Vector2d > tied( is, os );
            bool clip = options.exists( "--clip,--discard" );
            bool has_z = csv.has_field( "z" );
            auto xy = [&]( const Eigen::Vector3d& v )->Eigen::Vector2d
            {
                Eigen::Vector2d w( v.x(), v.y() );
                return has_z && !comma::math::equal( v.z(), 0 ) ? Eigen::Vector2d( w * ( -pinhole.config().focal_length / v.z() )) : w;
            };
            while( is.ready() || std::cin.good() )
            {
                const Eigen::Vector3d* p = is.read();
                if( !p ) { break; }
                const Eigen::Vector2d& pixel = pinhole.to_pixel( xy( *p ) );
                if ( !clip || (    !comma::math::less( pixel.x(), 0 )
                                && !comma::math::less( p->z(), 0 )
                                && comma::math::less( pixel.x(), pinhole.config().image_size.x() )
                                && !comma::math::less( pixel.y(), 0 )
                                && comma::math::less( pixel.y(), pinhole.config().image_size.y() ) ) )
                {
                    tied.append( pinhole.distort( pixel ) );
                }
            }
            return 0;
        }
        if( operation == "undistort" )
        {
            output_details< Eigen::Vector2d, Eigen::Vector2d >( options );
            snark::camera::pinhole pinhole( make_config( options.value< std::string >( "--camera-config,--camera,--config,-c" ) ) );
            comma::csv::input_stream< Eigen::Vector2d > is( std::cin, csv );
            comma::csv::output_stream< Eigen::Vector2d > os( std::cout, csv.binary(), false, csv.flush );
            comma::csv::tied< Eigen::Vector2d, Eigen::Vector2d > tied( is, os );
            while( is.ready() || std::cin.good() )
            {
                const Eigen::Vector2d* p = is.read();
                if( !p ) { break; }
                tied.append( pinhole.undistorted( *p ) );
            }
            return 0;
        }
        if( operation == "distort" )
        {
            output_details< Eigen::Vector2d, Eigen::Vector2d >( options );
            snark::camera::pinhole pinhole( make_config( options.value< std::string >( "--camera-config,--camera,--config,-c" ) ) );
            comma::csv::input_stream< Eigen::Vector2d > is( std::cin, csv );
            comma::csv::output_stream< Eigen::Vector2d > os( std::cout, csv.binary(), false, csv.flush );
            comma::csv::tied< Eigen::Vector2d, Eigen::Vector2d > tied( is, os );
            while( is.ready() || std::cin.good() )
            {
                const Eigen::Vector2d* p = is.read();
                if( !p ) { break; }
                tied.append( pinhole.distort( *p ) );
            }
            return 0;
        }
        if( operation=="distortion-map" || operation == "undistort-rectify-map" )
        {
            snark::camera::pinhole pinhole( make_config( options.value< std::string >( "--camera-config,--camera,--config,-c" ) ) );
            if( !pinhole.distortion_map() ) { std::cerr << "image-pinhole: no distortion specified in config" << std::endl; return 1; }
            pinhole.distortion_map()->write( std::cout );
            return 0;
        }
        std::cerr << "image-pinhole: error: unrecognized operation: "<< operation << std::endl;
    }
    catch( std::exception& ex ) { std::cerr << "image-pinhole: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "image-pinhole: unknown exception" << std::endl; }
    return 1;
}
