// Copyright (c) 2022 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <cmath>
#include <iostream>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include "../../math/rotation_matrix.h"
#include "../../math/position.h"
#include "../../visiting/traits.h"

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
    std::cerr << "    w: forward      s: backward     a: left         d: right" << std::endl;
    std::cerr << "    q: turn left    e: turn right   r: pitch up     f: pitch down" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --pose,-p=<initial_pose>; default=0,0,0,0,0,0; initial pose as <x>,<y>,<z>,<roll>,<pitch>,<yaw>" << std::endl;
    std::cerr << "    --step,-s=<meters>; default=1; linear increment step" << std::endl;
    std::cerr << "    --step-angle,-a=<radians>; default=0.0873; angle increment step, default: 5 degrees" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options csv;
        double step = options.value( "--step,-s", 1. );
        double angle = options.value( "--step-angle,-a", 5 * M_PI / 180 );
        csv.format( "6d" );
        csv.flush = true;
        comma::csv::output_stream< snark::position > ostream( std::cout, csv );
        snark::position pose = comma::csv::ascii< snark::position >().get( options.value< std::string >( "--pose,-p", "0,0,0,0,0,0" ) );
        ostream.write( pose );
        //bool escape_next = false;
        while( std::cin.good() )
        {
            int c = std::getchar();
            //if( escape_next ) { escape_next = false; continue; }
            snark::position delta;
            switch( c )
            {
                case 'w': case 'W': delta.coordinates.x() += step; break;
                case 's': case 'S': delta.coordinates.x() -= step; break;
                case 'd': case 'D': delta.coordinates.y() += step; break;
                case 'a': case 'A': delta.coordinates.y() -= step; break;
                case 'r': case 'R': delta.orientation.pitch() += angle; break;
                case 'f': case 'F': delta.orientation.pitch() -= angle; break;
                case 'q': case 'Q': delta.orientation.yaw() -= angle; break;
                case 'e': case 'E': delta.orientation.yaw() += angle; break;
                //case 27: escape_next = true; continue;
                default: continue;
            }
            Eigen::Translation3d translation( pose.coordinates );
            Eigen::Matrix3d rotation = snark::rotation_matrix::rotation( pose.orientation );
            Eigen::Affine3d transform = translation * rotation;
            pose.coordinates = transform * delta.coordinates;
            pose.orientation = snark::rotation_matrix::roll_pitch_yaw( rotation * snark::rotation_matrix::rotation( delta.orientation ) );
            ostream.write( pose );
        }
        return 0;
    }
    catch( std::exception& ex ) { comma::say() << ex.what() << std::endl; }
    catch( ... ) { comma::say() << "unknown exception" << std::endl; }
    return 1;
}
