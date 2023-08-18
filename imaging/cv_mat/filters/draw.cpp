// Copyright (c) 2023 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <iostream>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/lexical_cast.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <comma/base/exception.h>
#include <comma/base/none.h>
#include <comma/string/string.h>
#include "../utils.h"
#include "draw.h"

namespace snark { namespace cv_mat { namespace filters {

template < typename H >
std::pair< typename draw< H >::functor_t, bool > draw< H >::make( const std::string& options, char delimiter )
{
    const auto& v = comma::split( options, delimiter );
    auto second = v.begin();
    if( v[0] == "colorbar" ) { return colorbar::make( comma::join( ++second, v.end(), delimiter ) ); } // todo: quick and dirty; comma: implement split into a given number of strings
    if( v[0] == "grid" ) { return grid::make( comma::join( ++second, v.end(), delimiter ) ); } // todo: quick and dirty; comma: implement split into a given number of strings
    COMMA_THROW( comma::exception, "draw: expected draw primitive name; got: '" << v[0] << "'" );
}

template < typename H >
std::string draw< H >::usage( unsigned int indent )
{
    std::ostringstream oss;
    std::string i( indent, ' ' );
    oss << i << "draw=<what>[,<options>]; draw one of these:\n";
    oss << colorbar::usage( 4 + indent );
    oss << grid::usage( 4 + indent );
    return oss.str();
}

template < typename H >
std::pair< typename draw< H >::functor_t, bool > draw< H >::colorbar::make( const std::string& options, char delimiter )
{
    const auto& v = comma::split( options, delimiter );
    if( v.size() < 5 ) { COMMA_THROW( comma::exception, "expected <colormap>,<from/x>,<from/y>,<to/x>,<to/x> (at least 5 arguments); got: '" << options << "'" ); }
    colorbar c;
    boost::optional< cv::ColormapTypes > colormap = comma::silent_none< cv::ColormapTypes >();
    std::pair< cv::Scalar, cv::Scalar > color_range;
    const auto& s = comma::split( v[0], ':' );
    if( s.size() == 1 ) { colormap = colormap_from_string( v[0].empty() ? "jet" : v[0] ); }
    else { color_range = std::make_pair( color_from_string( s[0] ), color_from_string( s[1] ) ); }
    c._rectangle = cv::Rect( boost::lexical_cast< unsigned int >( v[1] ), boost::lexical_cast< unsigned int >( v[2] ), boost::lexical_cast< unsigned int >( v[3] ), boost::lexical_cast< unsigned int >( v[4] ) );
    if( c._rectangle.width < 60 || c._rectangle.height < 20 ) { COMMA_THROW( comma::exception, "colormap: expected rectangle at least 60x20; got: " << c._rectangle.width << "x" << c._rectangle.height ); }
    std::string from = v.size() > 5 && !v[5].empty() ? v[5] : "0";
    std::string to = v.size() > 6 && !v[6].empty() ? v[6] : "255";
    std::string middle = v.size() > 7 && !v[7].empty() ? v[7] : "";
    to += v.size() > 8 ? v[8] : "   "; // quick and dirty
    cv::Scalar colour = color_from_string( v.size() > 9 ? v[9] : "" );
    bool vertical{false}, reverse{false};
    for( unsigned int i = 10; i < v.size(); ++i )
    {
        if( v[i] == "vertical" ) { vertical = true; }
        else if( v[i] == "reverse" ) { reverse = true; }
    }
    cv::Mat grey( 1, 256, CV_8UC1 );
    for( unsigned int i = 0; i < 256; ++i ) { grey.at< unsigned char >( 0, i ) = reverse ? 255 - i : i; }
    cv::Mat coloured;
    if( colormap ) { cv::applyColorMap( grey, coloured, *colormap ); }
    else { apply_color_range( grey, coloured, color_range ); }
    cv::Mat bar;
    cv::resize( coloured, bar, cv::Size( c._rectangle.width, c._rectangle.height / 4 ) );
    c._bar = cv::Mat( c._rectangle.height, c._rectangle.width, CV_8UC3, cv::Scalar( 0, 0, 0 ) );
    bar.copyTo( c._bar( cv::Rect( 0, c._rectangle.height * 3 / 4, c._rectangle.width, c._rectangle.height / 4 ) ) );
    unsigned int h = c._rectangle.height / 2;
    unsigned int w = c._rectangle.width;
    #if defined( CV_VERSION_EPOCH ) && CV_VERSION_EPOCH == 2
        cv::putText( c._bar, from, cv::Point( 10, h ), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( colour * 0.5 ), 1, CV_AA );
        cv::putText( c._bar, middle, cv::Point( w / 2 - 16, h ), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( colour * 0.5 ), 1, CV_AA );
        cv::putText( c._bar, to, cv::Point( w - 55, h ), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( colour * 0.5 ), 1, CV_AA );
    #else
        cv::putText( c._bar, from, cv::Point( 10, h ), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( colour * 0.5 ), 1, cv::LINE_AA );
        cv::putText( c._bar, middle, cv::Point( w / 2 - 16, h ), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( colour * 0.5 ), 1, cv::LINE_AA );
        cv::putText( c._bar, to, cv::Point( w - 55, h ), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( colour * 0.5 ), 1, cv::LINE_AA );
    #endif
    if( vertical ) { cv::Mat transposed; cv::transpose( c._bar, transposed ); transposed.copyTo( c._bar ); }
    return std::make_pair( boost::bind< std::pair< H, cv::Mat > >( c, _1 ), false );
}

template < typename H >
std::pair< H, cv::Mat > draw< H >::bar::operator()( std::pair< H, cv::Mat > m )
{
    if( m.second.type() != CV_8UC3 ) { COMMA_THROW( comma::exception, "colorbar: only CV_8UC3 (" << CV_8UC3 << ") currently supported; got image of type: " << type_as_string( m.second.type() ) << " (" << m.second.type() << ")" ); }
    std::pair< H, cv::Mat > n;
    n.first = m.first;
    m.second.copyTo( n.second );
    _bar.copyTo( n.second( _rectangle ) );
    return n;
}

template < typename H >
std::string draw< H >::colorbar::usage( unsigned int indent )
{
    std::ostringstream oss;
    std::string i( indent, ' ' );
    oss << i << "draw=colorbar,<colormap>,<from/x>,<from/y>,<width>,<height>[,<from>,<to>[,<middle>[,<units>[,<text_color>[,vertical[,reverse]]]]]]\n";
    oss << i << "    draw colorbar on image; currently only 3-byte rgb supported\n";
    oss << i << "    options\n";
    oss << i << "        <colormap>: either colormap name, e.g. jet, see color-map filter for options\n";
    oss << i << "                    or color range: <color>:<color>, e.g. red:green; default=jet\n";
    oss << i << "        <from/x>,<from/y>,<width>,<height>: bounding rectangle in pixels\n";
    oss << i << "        <from>,<to>,<middle>: value range and number of values to display\n";
    oss << i << "                              display (as in numpy.linspace); default: from=0 to=255\n";
    oss << i << "        <units>: units to show, e.g. m\n";
    oss << i << "        <text_color>: default: white\n";
    oss << i << "        vertical: bar is vertical; default: horizontal\n";
    oss << i << "        reverse: show colors in reverse order\n";
    return oss.str();
}

// template < typename H >
// std::string draw< H >::scale::usage( unsigned int indent )
// {
//     std::ostringstream oss;
//     std::string i( indent, ' ' );
//     oss << i << "scale=<x>,<y>,<width>,<label>[,<color>[,vertical]]\n";
//     oss << i << "    draw scale on image; currently only 3-byte rgb supported\n";
//     oss << i << "    options\n";
//     oss << i << "        <x>,<y>: position\n";
//     oss << i << "        <width>: scale size in pixels\n";
//     oss << i << "        <label>: e.g. 100m\n";
//     oss << i << "        <color>: default: white\n";
//     oss << i << "        vertical: bar is vertical; default: horizontal\n";
//     return oss.str();
// }

template < typename H >
std::string draw< H >::grid::usage( unsigned int indent )
{
    std::ostringstream oss;
    std::string i( indent, ' ' );
    oss << i << "draw=grid,<from/x>,<from/y>,<step/x>,<step/y>[,<width>[,<height>[,<color>[,<ends_included>]]]]\n";
    oss << i << "    draw grid on image; currently only 3-byte rgb supported\n";
    oss << i << "    options\n";
    oss << i << "        <from/x>,<from/y>: upper left corner of bounding rectangle in pixels\n";
    oss << i << "        <width>,<height>: bounding rectangle size in pixels; if not specified, draw grid on the whole image\n";
    oss << i << "        <step/x>,<step/y>: horizontal and vertical grid step in pixels\n";
    oss << i << "        <color>: <r>,<g>,<b> in range 0-255; default: 0,0,0\n";
    oss << i << "        <ends-included>: if 1, first and last steps are included; default: 0, i.e. ends excluded\n";
    return oss.str();
}

template < typename H >
std::pair< typename draw< H >::functor_t, bool > draw< H >::grid::make( const std::string& options, char delimiter )
{
    const auto& v = comma::split( options, delimiter );
    if( v.size() < 5 ) { COMMA_THROW_BRIEF( comma::exception, "draw=grid: please specify grid origin and step (got: '" << options << "'" ); }
    boost::array< int, 10 > p = {{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }};
    for( unsigned int i = 0; i < v.size(); ++i ) { if( !v[i].empty() ) { p[i] = boost::lexical_cast< int >( v[i] ); } }
    grid g; // quick and dirty
    g._origin = {p[0], p[1]};
    g._step = {p[2], p[3]};
    g._size = {p[4], p[5]};
    if( g._size.width > 0 ) { g._end = {g._origin.x + g._size.width, g._origin.y + g._size.height}; }
    g._color = cv::Scalar( p[6], p[7], p[8] );
    g._ends_included = p[9];
    COMMA_ASSERT_BRIEF( g._size.width > 0 || !g._ends_included, "draw=grid: got ends-included flag set; please specify grid width,height" );
    return std::make_pair( boost::bind< std::pair< H, cv::Mat > >( g, _1 ), false );
}

template < typename H >
std::pair< H, cv::Mat > draw< H >::grid::operator()( std::pair< H, cv::Mat > m )
{
    std::pair< H, cv::Mat > n;
    n.first = m.first;
    m.second.copyTo( n.second );
    cv::Point end = _end.x == 0 ? cv::Point( m.second.cols, m.second.rows ) : _end;
    cv::Point begin = _origin;
    if( _ends_included ) { end += _step; } else { begin += _step; }
    for( cv::Point p{begin}, q( _origin.x, _origin.y + _size.height ); p.x < end.x; p.x += _step.x, q.x += _step.x ) { cv::line( n.second, p, q, _color ); } // , thickness, line_type, shift );
    for( cv::Point p{begin}, q( _origin.x + _size.width, _origin.x ); p.y < end.y; p.y += _step.y, q.y += _step.y ) { cv::line( n.second, p, q, _color ); } // , thickness, line_type, shift );
    return n;
}

template struct draw< boost::posix_time::ptime >;
template struct draw< std::vector< char > >;

} } }  // namespace snark { namespace cv_mat { namespace filters {