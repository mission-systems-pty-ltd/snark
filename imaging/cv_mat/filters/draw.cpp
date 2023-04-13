// Copyright (c) 2023 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <iostream>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/lexical_cast.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <comma/base/exception.h>
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
    COMMA_THROW( comma::exception, "draw: expected draw primitive name; got: '" << v[0] << "'" );
}

template < typename H >
std::string draw< H >::usage( unsigned int indent )
{
    std::ostringstream oss;
    std::string i( indent, ' ' );
    oss << i << "draw=<what>[,<options>]; draw one of these:\n";
    oss << colorbar::usage( 4 + indent );
    return oss.str();
}

template < typename H >
std::pair< typename draw< H >::functor_t, bool > draw< H >::colorbar::make( const std::string& options, char delimiter )
{
    const auto& v = comma::split( options, delimiter );
    if( v.size() < 5 ) { COMMA_THROW( comma::exception, "expected <colormap>,<from/x>,<from/y>,<to/x>,<to/x> (at least 5 arguments); got: '" << options << "'" ); }
    colorbar c;
    boost::optional< cv::ColormapTypes > colormap;
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
    oss << i << "colorbar=<colormap>,<from/x>,<from/y>,<width>,<height>[,<from>,<to>[,<middle>[,<units>[,<text_color>[,vertical[,reverse]]]]]]\n";
    oss << i << "    draw colorbar on image; currently only 3-byte rgb supported\n";
    oss << i << "    options\n";
    oss << i << "        <colormap>: either colormap name, e.g. jet, see color-map filter for options\n";
    oss << i << "                    or color range: <color>:<color>, e.g. red:green\n";
    oss << i << "        <colormap>: either colormap name, e.g. jet, see color-map filter for options; default=jet\n";
    oss << i << "        <from/x>,<from/y>,<width>,<height>: bounding rectangle in pixels\n";
    oss << i << "        <from>,<to>,<n>: value range and number of values to display\n";
    oss << i << "                         display (as in numpy.linspace); default: from=0 to=255 n=2\n";
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

template struct draw< boost::posix_time::ptime >;
template struct draw< std::vector< char > >;

} } }  // namespace snark { namespace cv_mat { namespace filters {