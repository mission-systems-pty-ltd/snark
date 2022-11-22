// Copyright (c) 2019 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <iostream>
#include <sstream>
#include <tuple>
#include <type_traits>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>
#if defined( CV_VERSION_EPOCH ) && CV_VERSION_EPOCH == 2
#include <opencv2/calib3d/calib3d.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif
#include <tbb/parallel_for.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/tuple/tuple.hpp>
#include <comma/base/exception.h>
#include <comma/csv/ascii.h>
#include <comma/visiting/traits.h>
#include "../../../math/range_bearing_elevation.h"
#include "polar_map.h"

namespace snark { namespace cv_calc { namespace polar_map {
    
std::string options()
{
    std::ostringstream oss;
    oss << "        --bearing=<from>,<step>; angles in radians" << std::endl;
    oss << "        --cartesian-size,--xy-size=<width>,<height>; cartesian image size" << std::endl;
    oss << "        --centre,--center,-c=<x>,<y>; default=0,0" << std::endl;
    oss << "        --polar-size=<width>,<height>; polar image size if --reverse" << std::endl;
    oss << "        --range=<from>,<step>; default: step: 1" << std::endl;
    oss << "        --reverse,--from-cartesian,--to-polar; generate reverse map, i.e. mapping cartesian image to polar image" << std::endl;
    oss << "        --size=<width>,<height>; if direct, desired cartesian image size; if --reverse, desired polar image size" << std::endl;
    oss << "        --transposed; polar image dimensions: bearing,range rather than range,bearing" << std::endl;
    oss << std::endl;
    return oss.str();
}

int run( const comma::command_line_options& options )
{
    bool reverse = options.exists( "--reverse,--from-polar,--to-cartesian" );
    static_assert( sizeof( float ) == 4, "expected float of size 4" );
    double width, height, range_from, range_step, bearing_from, bearing_step, cx, cy;
    std::tie( width, height ) = comma::csv::ascii< std::pair< unsigned int, unsigned int > >().get( options.value< std::string >( reverse ? "--polar-size,--size" : "--cartesian-size,--size" ) );
    std::tie( range_from, range_step ) = comma::csv::ascii< std::pair< double, double > >().get( options.value< std::string >( "--range", "0,1" ) );
    std::tie( bearing_from, bearing_step ) = comma::csv::ascii< std::pair< double, double > >().get( options.value< std::string >( "--bearing", "0,0.0174532925199" ) );
    std::tie( cx, cy ) = comma::csv::ascii< std::pair< double, double > >().get( options.value< std::string >( "--centre,--center,-c", "0,0" ) );
    bool transposed = options.exists( "--transposed" );
    double rs{range_step}, rf{range_from}, bs{bearing_step}, bf{bearing_from};
    cv::Mat x( height, width, CV_32F ); // quick and dirty; if output to stdout has problems, use serialisation class
    cv::Mat y( height, width, CV_32F ); // quick and dirty; if output to stdout has problems, use serialisation class
    if( reverse )
    {
        COMMA_THROW( comma::exception, "--reverse: todo, just ask" );
    }
    else
    {
        tbb::parallel_for( tbb::blocked_range< std::size_t >( 0, height ), [&]( const tbb::blocked_range< std::size_t >& r )
        {
            for( unsigned int v = r.begin(); v < r.end(); ++v )
            {
                for( unsigned int u = 0; u < width; ++u )
                {
                    double ox{ u - rf - cx }, oy{ v - rf - cy };
                    double r = std::sqrt( ox * ox + oy * oy ) / rs;
                    double a = ox == 0 ? 0. : ( std::atan( oy / ox ) - bf ) / bs;
                    std::tie( x.at< float >( v, u ), y.at< float >( v, u ) ) = transposed ? std::tie( a, r ) : std::tie( r, a );
                }
            }
        } );
        std::cout.write( reinterpret_cast< const char* >( x.datastart ), x.dataend - x.datastart );
        std::cout.write( reinterpret_cast< const char* >( y.datastart ), y.dataend - y.datastart );
    }
    return 0;
}

} } } // namespace snark { namespace cv_calc { namespace polar_map {
