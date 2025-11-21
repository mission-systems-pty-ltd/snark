// Copyright (c) 2020 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <sstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/lexical_cast.hpp>
#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include <comma/string/split.h>
#include "../serialization.h"
#include "../utils.h"
#include "resize.h"

namespace snark { namespace cv_mat { namespace filters {
    
template < typename H >
resize< H >::resize( unsigned int width, unsigned int height, double width_factor, double height_factor, int interpolation, bool by_orientation )
    : _width( width )
    , _height( height )
    , _width_factor( width_factor )
    , _height_factor( height_factor )
    , _interpolation( interpolation )
    , _by_orientation( by_orientation )
{
}

template < typename H >
typename resize< H >::value_type resize< H >::operator()( typename resize< H >::value_type m )
{
    typename resize< H >::value_type n{ m.first, cv::Mat() };
    bool swap = _by_orientation && m.second.cols < m.second.rows;
    unsigned int width  = swap ? ( _height == 0 ? comma::math::equal( _height_factor, 1. ) ? m.second.cols : m.second.cols * _height_factor : _height )
                               : ( _width == 0  ? comma::math::equal( _width_factor, 1. )  ? m.second.cols : m.second.cols * _width_factor  : _width );
    unsigned int height = swap ? ( _width == 0  ? comma::math::equal( _width_factor, 1. )  ? m.second.rows : m.second.rows * _width_factor  : _width )
                               : ( _height == 0 ? comma::math::equal( _height_factor, 1. ) ? m.second.rows : m.second.rows * _height_factor : _height );
    cv::resize( m.second, n.second, cv::Size( width, height ), 0, 0, _interpolation );
    return n;
}

template < typename H >
resize< H > resize< H >::make( const std::string& options )
{
    unsigned int width = 0;
    unsigned int height = 0;
    double width_factor = 1;
    double height_factor = 1;
    const std::vector< std::string >& r = comma::split( options, ',' );
    int interpolation = cv::INTER_LINEAR;
    bool by_orientation = false;
    unsigned int size = 0;
    for( const auto& e: r )
    {
        if( e[0] < 'a' || e[0] > 'z' ) { ++size; continue; }
        if( e == "by-orientation" ) { by_orientation = true; }
        else { interpolation = cv_mat::interpolation( e ); }
    }
    switch( size )
    {
        case 1:
            width_factor = height_factor = boost::lexical_cast< double >( r[0] );
            break;
        case 2:
        case 3:
            if( !r[0].empty() )
            {
                try { width = boost::lexical_cast< unsigned int >( r[0] ); }
                catch ( ... ) { width_factor = boost::lexical_cast< double >( r[0] ); }
            }
            if( !r[1].empty() )
            {
                try { height = boost::lexical_cast< unsigned int >( r[1] ); }
                catch ( ... ) { height_factor = boost::lexical_cast< double >( r[1] ); }
            }
            break;
        default:
            COMMA_THROW( comma::exception, "expected resize=<width>,[<height>],[<interpolation>] got: \"" << options << "\"" );
    }
    return resize< H >( width, height, width_factor, height_factor, interpolation, by_orientation );
}

template < typename H >
std::string resize< H >::usage( unsigned int indent )
{
    std::ostringstream oss;
    std::string i( indent, ' ' );
    oss << i << "    resize=<options>\n";
    oss << i << "        <options>\n";
    oss << i << "            <factor>[,<interpolation>][,by-orientation]\n";
    oss << i << "            <width>,<height>[,<interpolation>][,by-orientation]\n";
    oss << i << "        <interpolation>";
    oss << i << "            nearest (" << cv::INTER_NEAREST << ")\n";
    oss << i << "            linear (" << cv::INTER_LINEAR << ") - default\n";
    oss << i << "            cubic (" << cv::INTER_CUBIC << ")\n";
    oss << i << "            area (" << cv::INTER_AREA << ")\n";
    oss << i << "            lanczos4 (" << cv::INTER_LANCZOS4 << ")\n";
    oss << i << "        <width>,<height>";
    oss << i << "            if no decimal dot '.', size is in pixels\n";
    oss << i << "            if decimal dot present, size as a fraction\n";
    oss << i << "            i.e. 5 means 5 pixels; 5.0 means 5 times\n";
    oss << i << "        by-orientation: convenience option\n";
    oss << i << "            if image aspect ratio is portrait, swap <width> and <height>\n";
    oss << i << "            e.g. if resize=600,400,by-orientation\n";
    oss << i << "                - for input 1800x1200 image the output image will be 600x400\n";
    oss << i << "                - for input 1200x1800 image the output image will be 400x600\n";
    oss << i << "        examples\n";
    oss << i << "            resize=0.5,1024 : 50% of width; heigth 1024 pixels\n";
    oss << i << "            resize=512,1024,cubic : resize to 512x1024 pixels, cubic interpolation\n";
    oss << i << "            resize=0.2,0.4 : resize to 20% of width and 40% of height\n";
    oss << i << "            resize=0.5 : resize proportionally to 50%\n";
    oss << i << "            resize=0.5,1024 : 50% of width; heigth 1024 pixels\n";
    return oss.str();
}

template class snark::cv_mat::filters::resize< boost::posix_time::ptime >;
template class snark::cv_mat::filters::resize< std::vector< char > >;

} } } // namespace snark { namespace cv_mat { namespace filters {
