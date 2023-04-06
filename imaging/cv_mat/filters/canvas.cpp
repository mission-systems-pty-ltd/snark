// Copyright (c) 2023 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <boost/bind.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/lexical_cast.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include <comma/base/exception.h>
#include <comma/string/string.h>
#include "../utils.h"
#include "canvas.h"

namespace snark { namespace cv_mat { namespace filters {

template < typename H >
std::pair< typename canvas< H >::functor_t, bool > canvas< H >::make( const std::string& options, char delimiter )
{
    const auto& v = comma::split( options, delimiter );
    if( v.size() < 2 ) { COMMA_THROW( comma::exception, "expected <colormap>,<from/x>,<from/y>,<to/x>,<to/x> (at least 5 arguments); got: '" << options << "'" ); }
    canvas< H > c;
    cv::Size size{ boost::lexical_cast< int >( v[0] ), boost::lexical_cast< int >( v[1] ) };
    c._origin = cv::Point( v.size() < 3 || v[2].empty() ? 0 : boost::lexical_cast< int >( v[2] )
                         , v.size() < 4 || v[3].empty() ? 0 : boost::lexical_cast< int >( v[3] ) );
    c._canvas = cv::Mat( size, CV_8UC3, color_from_string( v.size() < 5 || v[4].empty() ? "black" : v[4] ) );
    cv::cvtColor( c._canvas, c._grey_canvas, CV_BGR2GRAY );
    return std::make_pair( boost::bind< std::pair< H, cv::Mat > >( c, _1 ), false );
}

template < typename H >
std::pair< H, cv::Mat > canvas< H >::operator()( std::pair< H, cv::Mat > m )
{
    if( m.second.type() != CV_8UC3 && m.second.type() != CV_8UC1 ) { COMMA_THROW( comma::exception, "canvas: only CV_8UC1 and CV_8UC3 currently supported; got image of type: " << type_as_string( m.second.type() ) << " (" << m.second.type() << ")" ); }
    std::pair< H, cv::Mat > n;
    n.first = m.first;
    ( m.second.type() == CV_8UC1 ? _grey_canvas : _canvas ).copyTo( n.second );
    m.second.copyTo( n.second( cv::Rect( _origin, m.second.size() ) ) );
    return n;
}

template < typename H >
std::string canvas< H >::usage( unsigned int indent )
{
    std::ostringstream oss;
    std::string i( indent, ' ' );
    oss << i << "canvas=<width>,<height>[,<x>,<y>][,<color>]; make a larger canvas under image\n";
    oss << i << "    options\n";
    oss << i << "        <width>,<height>: canvas size\n";
    oss << i << "        <x>,<y>: image origin, default: 0,0\n";
    oss << i << "        <canvas_color>: default: black\n";
    return oss.str();
}

template struct canvas< boost::posix_time::ptime >;
template struct canvas< std::vector< char > >;

} } }  // namespace snark { namespace cv_mat { namespace filters {
