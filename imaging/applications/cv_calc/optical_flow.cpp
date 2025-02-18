// Copyright (c) 2025 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <array>
#include <cmath>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <tbb/parallel_for.h>
#include <comma/containers/enums.h>
#include "../../../imaging/cv_mat/filters/life.h"
#include "../../../imaging/cv_mat/traits.h"
#include "../../../imaging/cv_mat/serialization.h"
#include "optical_flow.h"

namespace snark { namespace cv_calc { namespace optical_flow {

namespace farneback {

std::string options()
{
    std::ostringstream oss;
    oss << R"(        optical flow, farneback method, trivial wrapper
        around cv::calcOpticalFlowFarneback() on grey-scale images
        output is two-channel float32 image containing x and y component
        of optical flow for each pixel

        options
            --iterations=<n>; default=3; number of iterations the algorithm
                does at each pyramid level
            --levels=3; default=3; number of pyramid layers including the
                initial image; levels=1 means that no extra layers are
                created and only the original images are used
            --output-normalized,--normalized; if --output-polar, normalise
                magnitude, e.g. for debugging or visualisation
            --output-polar,--polar; instead of x,y components, output
                magnitude and angle
            --output=<what>; default=xy
                <what>
                    applied: apply x,y displacement to input image
                    polar  : output magnitude,angle instead of x,y
                    xy     : output optical flow x,y displacement
            --poly-n=<n>; default=5; size of the pixel neighborhood used to
                find polynomial expansion in each pixel; larger values mean
                that the image will be approximated with smoother surfaces,
                yielding more robust algorithm and more blurred motion
                field, typically --poly-n is 5 or 7
            --poly-sigma=<value>; standard deviation of the Gaussian that
                is used to smooth derivatives used as a basis for the
                polynomial expansion; for --poly-n=5, you can set
                --poly-sigma=1.1, for --poly-n=7, a good value would be
                --poly-sigma=1.5
            --pyramid-scale=<scale>; default=0.5; parameter, specifying the
                image scale (<1) to build pyramids for each image
                pyr_scale=0.5 means a classical pyramid, where each next
                layer is half size of the previous one
            --use-gaussian,--gaussian; uses the Gaussian winsize×winsize
                filter instead of a box filter of the same size for optical
                flow estimation usually, this option gives z more accurate
                flow than with a box filter, at the cost of lower speed
                normally, winsize for a Gaussian window should be set to a
                larger value to achieve the same level of robustness
            --use-initial-flow; uses the input flow as an initial flow
                approximation
            --window-size=<n>; default=15; averaging window size; larger
                values increase the algorithm robustness to image noise and
                give more chances for fast motion detection, but yield more
                blurred motion field)";
    return oss.str();
}

int run( const comma::command_line_options& options )
{
    auto [ input_serialization, s ] = cv_mat::serialization::make( options, true );
    cv_mat::serialization::options o; // todo: quick and dirty, improve later
    o.type = "2f";
    o.no_header = s.no_header();
    o.header_only = s.header_only();
    cv_mat::serialization output_serialization( o );
    unsigned int iterations = options.value( "--iterations", 3 );
    unsigned int levels = options.value( "--levels", 3 );
    unsigned int poly_n = options.value( "--poly-n", 5 );
    double poly_sigma = options.value( "--poly-sigma", 0.5 );
    double pyramid_scale = options.value( "--pyramid-scale", 0.5 );
    unsigned int window_size = options.value( "--window-size", 15 );
    int flags = ( options.exists( "--use-initial-flow" ) ? cv::OPTFLOW_USE_INITIAL_FLOW : 0 ) | ( options.exists( "--use-gaussian,--gaussian" ) ? cv::OPTFLOW_FARNEBACK_GAUSSIAN : 0 );
    struct output_t { enum types { applied, polar, xy }; };
    output_t::types output = comma::enums::find< output_t::types >( options.value< std::string >( "--output", "xy" ), { "applied", "polar", "xy" } );
    typedef std::pair< boost::posix_time::ptime, cv::Mat > pair_t;
    pair_t frame1, frame2, flow, applied;
    cv::Mat previous, next, normalized;
    cv::Mat cartesian[2];
    cv::Mat polar[2];
    while( std::cin.good() )
    {
        pair_t frame2 = input_serialization.read< boost::posix_time::ptime >( std::cin );
        cv::cvtColor( frame2.second, next, cv::COLOR_BGR2GRAY );
        if( frame2.second.empty() ) { return 0; }
        if( !frame1.second.empty() )
        {
            flow.first = frame2.first;
            if( flow.second.empty() ) { flow.second = cv::Mat( previous.size(), CV_32FC2 ); }
            cv::calcOpticalFlowFarneback( previous, next, flow.second, pyramid_scale, levels, window_size, iterations, poly_n, poly_sigma, flags );
            switch( output )
            {
                case output_t::applied:
                {
                    int rows = frame1.second.rows; // for brevity
                    int cols = frame1.second.cols; // for brevity
                    int channels = frame1.second.channels(); // for brevity
                    applied.first = frame2.first;
                    if( applied.second.empty() ) { applied.second = cv::Mat::zeros( frame1.second.size(), frame1.second.type() ); }
                    unsigned char* p = &frame1.second.at< unsigned char >( 0, 0 ); // todo! generalise
                    float* f = &flow.second.at< float >( 0, 0 );
                    unsigned char* s = &applied.second.at< unsigned char >( 0, 0 );
                    for( unsigned int row = 0; int( row ) < rows; ++row )
                    {
                        for( unsigned int col = 0; int( col ) < cols; ++col, p += channels, f += 2 )
                        {
                            double x{f[0] + col};
                            double y{f[1] + row};
                            int i = std::floor( x );
                            int j = std::floor( y );
                            //std::cerr << "==> a: pixel: " << col << "," << row << ": f: " << f[0] << "," << f[1] << " x: " << x << " y: " << y << std::endl;
                            if( i >= 0 && i < cols && j >= 0 && j < rows )
                            {
                                unsigned char* t = s + ( i * row + j ) * channels;
                                unsigned char* q = p;
                                double g = ( x - i ) * ( y - j );
                                for( int k = 0; k < channels; ++k, ++t, ++q ) { *t += double( *q ) * g; }
                            }
                            ++i;
                            if( i >= 0 && i < cols && j >= 0 && j < rows )
                            {
                                unsigned char* t = s + ( i * row + j ) * channels;
                                unsigned char* q = p;
                                double g = ( i - x ) * ( y - j );
                                for( int k = 0; k < channels; ++k, ++t, ++q ) { *t += double( *q ) * g; }
                            }
                            --i;
                            ++j;
                            if( i >= 0 && i < cols && j >= 0 && j < rows )
                            {
                                unsigned char* t = s + ( i * row + j ) * channels;
                                unsigned char* q = p;
                                double g = ( x - i ) * ( j - y );
                                for( int k = 0; k < channels; ++k, ++t, ++q ) { *t += double( *q ) * g; }
                            }
                            ++i;
                            if( i >= 0 && i < cols && j >= 0 && j < rows )
                            {
                                unsigned char* t = s + ( i * row + j ) * channels;
                                unsigned char* q = p;
                                double g = ( i - x ) * ( j - y );
                                for( int k = 0; k < channels; ++k, ++t, ++q ) { *t += double( *q ) * g; }
                            }
                            //std::cerr << "==> b: pixel: " << col << "," << row << ": f: " << f[0] << "," << f[1] << " x: " << x << " y: " << y << std::endl;
                        }
                    }
                    output_serialization.write_to_stdout( applied, true );
                    break;
                }
                case output_t::polar:
                    cv::split( flow.second, cartesian );
                    cv::cartToPolar( cartesian[0], cartesian[1], polar[0], polar[1], true );
                    polar[1] *= M_PI / 180;
                    // if( output_normalized ) { cv::normalize( polar[0], normalized, 0.0f, 1.0f, cv::NORM_MINMAX ); normalized.copyTo( polar[0] ); }
                    merge( polar, 2, flow.second );
                    output_serialization.write_to_stdout( flow, true );
                    break;
                case output_t::xy:
                    output_serialization.write_to_stdout( flow, true );
                    break;
            }
        }
        frame1.first = frame2.first;
        frame2.second.copyTo( frame1.second );
        next.copyTo( previous );
    }
    return 0;
}

} // namespace farneback {

} } } // namespace snark { namespace cv_calc { namespace optical_flow {
