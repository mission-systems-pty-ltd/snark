// Copyright (c) 2022 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <cmath>
#include <iostream>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include "../../math/rotation_matrix.h"
#include "../../math/position.h"
#include "../../visiting/traits.h"

static void _layout( std::ostream& os = std::cerr )
{
    os << "    w: forward      s: backward" << std::endl;
    os << "    a: left         d: right" << std::endl;
    os << "    x: up           z: down         " << std::endl;
    os << "    q: turn left    e: turn right" << std::endl;
    os << "    r: pitch up     f: pitch down" << std::endl;
    os << "    c: roll left    v: roll right" << std::endl;
}

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "usage: io-console | control-console | my-app-using-poses-on-stdin --binary=6d" << std::endl;
    std::cerr << std::endl;
    std::cerr << "input: key codes as binary bytes; same as io-console output" << std::endl;
    std::cerr << std::endl;
    std::cerr << "output: x,y,z,roll,pitch,yaw binary records; format: 6d" << std::endl;
    std::cerr << std::endl;
    std::cerr << "keys" << std::endl;
    _layout();
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --lookaround,-l; for each keypress output 6 poses that look in 6 directions:" << std::endl;
    std::cerr << "                     top,back,left,front,right,bottom as for equirectangular cubes" << std::endl;
    std::cerr << "    --pose,-p=<initial_pose>; default=0,0,0,0,0,0; initial pose as <x>,<y>,<z>,<roll>,<pitch>,<yaw>" << std::endl;
    std::cerr << "    --step,-s=<meters>; default=1; linear increment step" << std::endl;
    std::cerr << "    --step-angle,-a=<radians>; default=0.0873; angle increment step, default: 5 degrees" << std::endl;
    std::cerr << "    --verbose,-v: print hot keys and pose on stderr" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

static snark::position _transform( const snark::position& pose, const snark::position& delta ) // todo: quick and dirty; put it in library
{
    Eigen::Translation3d translation( pose.coordinates );
    const Eigen::Matrix3d& rotation = snark::rotation_matrix::rotation( pose.orientation );
    Eigen::Affine3d transform = translation * rotation;
    return snark::position( transform * delta.coordinates, snark::rotation_matrix::roll_pitch_yaw( rotation * snark::rotation_matrix::rotation( delta.orientation ) ) );
}

static void _output( comma::csv::output_stream< snark::position >& ostream, const snark::position& pose, bool lookaround, bool verbose )
{
    if( verbose ) { std::cerr << "x,y,z: " << pose.coordinates.transpose() << " roll,pitch,yaw: " << pose.orientation.transpose() << " degrees: " << ( ( pose.orientation * 180 / M_PI ).transpose() ) << std::string( 40, ' ' ) << "\r"; }
    if( !lookaround ) { ostream.write( pose ); return; }
    static const std::vector< snark::position > faces = { { { 0., 0., 0. }, { 0.,  M_PI / 2.,         0. } }    // top
                                                        , { { 0., 0., 0. }, { 0.,         0.,       M_PI } }    // back
                                                        , { { 0., 0., 0. }, { 0.,         0.,  M_PI / 2. } }    // left
                                                        , { { 0., 0., 0. }, { 0.,         0.,         0. } }    // front
                                                        , { { 0., 0., 0. }, { 0.,         0., -M_PI / 2. } }    // right
                                                        , { { 0., 0., 0. }, { 0., -M_PI / 2.,         0. } } }; // down
    for( const auto& face: faces ) { ostream.write( _transform( pose, face ) ); }
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options csv;
        double step = options.value( "--step,-s", 1. );
        double angle = options.value( "--step-angle,-a", 5 * M_PI / 180 );
        bool lookaround = options.exists( "--lookaround,-l" );
        bool verbose = options.exists( "--verbose,-v" );
        if( verbose ) { std::cerr << std::endl; _layout(); std::cerr << std::endl; }
        csv.format( "6d" );
        csv.flush = true;
        comma::csv::output_stream< snark::position > ostream( std::cout, csv );
        snark::position pose = comma::csv::ascii< snark::position >().get( options.value< std::string >( "--pose,-p", "0,0,0,0,0,0" ) );
        _output( ostream, pose, lookaround, verbose );
        while( std::cin.good() )
        {
            int c = std::getchar();
            snark::position delta;
            switch( c )
            {
                case 'w': case 'W':           delta.coordinates.x() += step; break;
                case 's': case 'S':           delta.coordinates.x() -= step; break;
                case 'd': case 'D':           delta.coordinates.y() += step; break;
                case 'a': case 'A':           delta.coordinates.y() -= step; break;
                case 'z': case 'Z':           delta.coordinates.z() += step; break;
                case 'x': case 'X': case ' ': delta.coordinates.z() -= step; break;
                case 'c': case 'C':           delta.orientation.roll() -= angle; break;
                case 'v': case 'V':           delta.orientation.roll() += angle; break;
                case 'r': case 'R':           delta.orientation.pitch() += angle; break;
                case 'f': case 'F':           delta.orientation.pitch() -= angle; break;
                case 'q': case 'Q':           delta.orientation.yaw() -= angle; break;
                case 'e': case 'E':           delta.orientation.yaw() += angle; break;
                default: continue;
            }
            pose = _transform( pose, delta );
            _output( ostream, pose, lookaround, verbose );
        }
        return 0;
    }
    catch( std::exception& ex ) { comma::say() << ex.what() << std::endl; }
    catch( ... ) { comma::say() << "unknown exception" << std::endl; }
    return 1;
}
