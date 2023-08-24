// Copyright (c) 2023 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <iostream>
#include <sstream>
#include <tuple>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/lexical_cast.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <comma/base/exception.h>
#include <comma/base/none.h>
#include <comma/name_value/parser.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include "../utils.h"
#include "draw.h"

namespace comma { namespace visiting {

template < typename H > struct _impl // quick and dirty
{
    typedef typename snark::cv_mat::filters::draw< H >::axis::properties axis_t;

    template < typename Key, class Visitor > static void visit( const Key&, axis_t& p, Visitor& v )
    {
        v.apply( "label", p.label );
        std::string extents;
        v.apply( "extents", extents );
        const auto& e = comma::split_as< float >( extents, ',' );
        COMMA_ASSERT_BRIEF( e.empty() || e.size() == 2, "expected extents; got: '" << extents << "'" );
        if( !e.empty() ) { p.extents.first = e[0]; p.extents.second = e[1]; }
        COMMA_ASSERT_BRIEF( p.extents.first != p.extents.second, "expected non-zero length extents; got: '" << extents << "'" );
        v.apply( "step", p.step );
        std::string origin;
        v.apply( "origin", origin );
        const auto& s = comma::split_as< unsigned int >( origin, ',' );
        COMMA_ASSERT_BRIEF( s.empty() || s.size() == 2, "expected origin; got: '" << origin << "'" );
        v.apply( "size", p.size );
        std::string color;
        v.apply( "color", color );
        const auto& c = comma::split_as< unsigned int >( color, ',' );
        COMMA_ASSERT_BRIEF( c.empty() || c.size() == 3 || c.size() == 4, "expected color; got: '" << color << "'" );
        if( !c.empty() ) { p.color = c.size() == 4 ? cv::Scalar( c[2], c[1], c[0], c[3] ) : cv::Scalar( c[2], c[1], c[0] ); }
        v.apply( "vertical", p.vertical );
        if( !s.empty() ) { p.geometry.first = cv::Point( s[0], s[1] ); }
        p.geometry.second = p.geometry.first;
        ( p.vertical ? p.geometry.second.y : p.geometry.second.x ) += p.size;
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, const axis_t& p, Visitor& v )
    {
        v.apply( "label", p.label );
        std::ostringstream extents;
        extents << p.extents.first << "," << p.extents.second;
        v.apply( "extents", extents.str() );
        v.apply( "step", p.step );
        std::ostringstream origin;
        extents << p.origin.x() << "," << p.origin.y();
        v.apply( "origin", origin.str() );
        v.apply( "size", p.size );
        std::ostringstream color;
        color << p.color.r() << "," << p.color.g() << "," << p.color.b() << "," << p.color.a();
        v.apply( "color", color.str() );
        v.apply( "vertical", p.vertical );
    }

    typedef typename snark::cv_mat::filters::draw< H >::status::properties status_t;

    template < typename Key, class Visitor > static void visit( const Key&, status_t& p, Visitor& v )
    {
        v.apply( "label", p.label );
        std::string origin;
        v.apply( "origin", origin );
        const auto& s = comma::split_as< int >( origin, ',' );
        COMMA_ASSERT_BRIEF( s.empty() || s.size() == 2, "expected origin; got: '" << origin << "'" );
        if( !s.empty() ) { p.origin = cv::Point( s[0], s[1] ); }
        std::string color;
        v.apply( "color", color );
        const auto& c = comma::split_as< unsigned int >( color, ',' );
        COMMA_ASSERT_BRIEF( c.empty() || c.size() == 3 || c.size() == 4, "expected color; got: '" << color << "'" );
        if( !c.empty() ) { p.color = c.size() == 4 ? cv::Scalar( c[2], c[1], c[0], c[3] ) : cv::Scalar( c[2], c[1], c[0] ); }
        std::string bg_color;
        v.apply( "bg-color", bg_color );
        const auto& bc = comma::split_as< unsigned int >( bg_color, ',' );
        COMMA_ASSERT_BRIEF( bc.empty() || bc.size() == 3 || bc.size() == 4, "expected color; got: '" << bg_color << "'" );
        if( !bc.empty() ) { p.bg_color = c.size() == 4 ? cv::Scalar( bc[2], bc[1], bc[0], bc[3] ) : cv::Scalar( bc[2], bc[1], bc[0] ); }
        v.apply( "font-size", p.font_size );
        v.apply( "alpha", p.alpha );
        v.apply( "spin-up", p.spin_up );
        v.apply( "system-time", p.system_time );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, const status_t& p, Visitor& v )
    {
        v.apply( "label", p.label );
        std::ostringstream origin;
        origin << p.origin.x() << "," << p.origin.y();
        v.apply( "origin", origin.str() );
        std::ostringstream color;
        color << p.color.r() << "," << p.color.g() << "," << p.color.b() << "," << p.color.a();
        v.apply( "color", color.str() );
        std::ostringstream bg_color;
        bg_color << p.bg_color.r() << "," << p.bg_color.g() << "," << p.bg_color.b() << "," << p.bg_color.a();
        v.apply( "bg-color", bg_color.str() );
        v.apply( "font-size", p.font_size );
        v.apply( "alpha", p.alpha );
        v.apply( "spin-up", p.spin_up );
        v.apply( "system-time", p.system_time );
    }
};

template <> struct traits< typename snark::cv_mat::filters::draw< boost::posix_time::ptime >::axis::properties >
{
    typedef snark::cv_mat::filters::draw< boost::posix_time::ptime >::axis::properties value_t;
    template < typename Key, class Visitor > static void visit( const Key& k, const value_t& t, Visitor& v ) { _impl< boost::posix_time::ptime >::visit( k, t, v ); }
    template < typename Key, class Visitor > static void visit( const Key& k, value_t& t, Visitor& v ) { _impl< boost::posix_time::ptime >::visit( k, t, v ); }
};

template <> struct traits< typename snark::cv_mat::filters::draw< std::vector< char > >::axis::properties >
{
    typedef snark::cv_mat::filters::draw< std::vector< char > >::axis::properties value_t;
    template < typename Key, class Visitor > static void visit( const Key& k, const value_t& t, Visitor& v ) { _impl< std::vector< char > >::visit( k, t, v ); }
    template < typename Key, class Visitor > static void visit( const Key& k, value_t& t, Visitor& v ) { _impl< std::vector< char > >::visit( k, t, v ); }
};

template <> struct traits< typename snark::cv_mat::filters::draw< boost::posix_time::ptime >::status::properties >
{
    typedef snark::cv_mat::filters::draw< boost::posix_time::ptime >::status::properties value_t;
    template < typename Key, class Visitor > static void visit( const Key& k, const value_t& t, Visitor& v ) { _impl< boost::posix_time::ptime >::visit( k, t, v ); }
    template < typename Key, class Visitor > static void visit( const Key& k, value_t& t, Visitor& v ) { _impl< boost::posix_time::ptime >::visit( k, t, v ); }
};

template <> struct traits< typename snark::cv_mat::filters::draw< std::vector< char > >::status::properties >
{
    typedef snark::cv_mat::filters::draw< std::vector< char > >::status::properties value_t;
    template < typename Key, class Visitor > static void visit( const Key& k, const value_t& t, Visitor& v ) { _impl< std::vector< char > >::visit( k, t, v ); }
    template < typename Key, class Visitor > static void visit( const Key& k, value_t& t, Visitor& v ) { _impl< std::vector< char > >::visit( k, t, v ); }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace cv_mat { namespace filters {

namespace impl {

#if defined( CV_VERSION_EPOCH ) && CV_VERSION_EPOCH == 2 // quick and dirty; pain...
constexpr auto line_aa = CV_AA;
constexpr auto filled = 1;
#else
constexpr auto line_aa = cv::LINE_AA;
constexpr auto filled = cv::FILLED;
#endif

} // namespace impl {

template < typename H >
std::pair< typename draw< H >::functor_t, bool > draw< H >::make( const std::string& options
                                                                , draw< H >::timestamp_functor_t get_timestamp
                                                                , char delimiter )
{
    const auto& v = comma::split( options, delimiter );
    auto second = v.begin();
    if( v[0] == "axis" ) { return axis::make( comma::join( ++second, v.end(), delimiter ), delimiter ); } // todo: quick and dirty; comma: implement split into a given number of strings
    if( v[0] == "colorbar" ) { return colorbar::make( comma::join( ++second, v.end(), delimiter ), delimiter ); } // todo: quick and dirty; comma: implement split into a given number of strings
    if( v[0] == "grid" ) { return grid::make( comma::join( ++second, v.end(), delimiter ), delimiter ); } // todo: quick and dirty; comma: implement split into a given number of strings
    if( v[0] == "status" ) { return status::make( comma::join( ++second, v.end(), delimiter ), get_timestamp, delimiter ); } // todo: quick and dirty; comma: implement split into a given number of strings
    if( v[0] == "time" ) { return time::make( comma::join( ++second, v.end(), delimiter ), get_timestamp, delimiter ); } // todo: quick and dirty; comma: implement split into a given number of strings
    COMMA_THROW( comma::exception, "draw: expected draw primitive name; got: '" << v[0] << "'" );
}

template < typename H >
std::string draw< H >::usage( unsigned int indent )
{
    std::ostringstream oss;
    std::string i( indent, ' ' );
    oss << i << "draw=<what>[,<options>]; draw one of these:\n";
    oss << axis::usage( 4 + indent );
    oss << colorbar::usage( 4 + indent );
    oss << grid::usage( 4 + indent );
    oss << status::usage( 4 + indent );
    oss << time::usage( 4 + indent );
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
    cv::putText( c._bar, from, cv::Point( 10, h ), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( colour * 0.5 ), 1, impl::line_aa );
    cv::putText( c._bar, middle, cv::Point( w / 2 - 16, h ), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( colour * 0.5 ), 1, impl::line_aa );
    cv::putText( c._bar, to, cv::Point( w - 55, h ), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( colour * 0.5 ), 1, impl::line_aa );
    if( vertical ) { cv::Mat transposed; cv::transpose( c._bar, transposed ); ; cv::flip( transposed, c._bar, 0 ); }
    return std::make_pair( boost::bind< std::pair< H, cv::Mat > >( c, _1 ), true );
}

template < typename H >
std::pair< H, cv::Mat > draw< H >::bar::operator()( std::pair< H, cv::Mat > m )
{
    if( m.second.type() != CV_8UC3 ) { COMMA_THROW( comma::exception, "colorbar: only CV_8UC3 (" << CV_8UC3 << ") currently supported; got image of type: " << type_as_string( m.second.type() ) << " (" << m.second.type() << ")" ); }
    _bar.copyTo( m.second( _rectangle ) );
    return m;
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
    return std::make_pair( boost::bind< std::pair< H, cv::Mat > >( g, _1 ), true );
}

template < typename H >
std::pair< H, cv::Mat > draw< H >::grid::operator()( std::pair< H, cv::Mat > m )
{
    cv::Point end = _end.x == 0 ? cv::Point( m.second.cols, m.second.rows ) : _end;
    cv::Point begin = _origin;
    cv::Size size = _end.x == 0 ? cv::Size( m.second.cols - _origin.x - 1, m.second.rows - _origin.y - 1 ) : _size;
    if( _ends_included ) { end += _step; } else { begin += _step; }
    for( int x{begin.x}; x < end.x; x += _step.x ) { cv::line( m.second, cv::Point( x, _origin.y ), cv::Point( x, _origin.y + size.height ), _color ); } // , thickness, line_type, shift );
    for( int y{begin.y}; y < end.y; y += _step.y ) { cv::line( m.second, cv::Point( _origin.x, y ), cv::Point( _origin.x + size.width, y ), _color ); } // , thickness, line_type, shift );
    return m;
}

template < typename H >
std::string draw< H >::axis::usage( unsigned int indent )
{
    std::ostringstream oss;
    std::string i( indent, ' ' );
    oss << i << "draw=axis,<options>\n";
    oss << i << "    draw axis on image; currently only 3-byte rgb supported\n";
    oss << i << "    <options>\n";
    oss << i << "        [label:<text>]\n";
    oss << i << "        extents:<begin>,<end>: extents of values along the axis\n";
    oss << i << "        step:<value>: value step\n";
    oss << i << "        origin:<x>,<y>: axis origin in pixels\n";
    oss << i << "        size:<pixels>: axis size in pixels\n";
    oss << i << "        color:<r>,<g>,<b>: axis color; default: 0,0,0\n";
    oss << i << "        vertical: axis is vertical; default: horizontal; todo: axis label and values\n";
    return oss.str();
}

template < typename H >
std::pair< typename draw< H >::functor_t, bool > draw< H >::axis::make( const std::string& options, char delimiter )
{
    axis a;
    a._properties = comma::name_value::parser( '|', ':' ).get< properties >( options );
    COMMA_ASSERT_BRIEF( a._properties.size > 0, "draw=axis: please specify positive size" );
    COMMA_ASSERT_BRIEF( a._properties.step != 0, "draw=axis: please specify non-zero step" );
    a._step = a._properties.size * std::abs( a._properties.step / ( a._properties.extents.second - a._properties.extents.first ) );
    a._label_position = ( a._properties.geometry.first + a._properties.geometry.second ) / 2;
    if( a._properties.vertical ) { a._label_position.x -= 16; } else { a._label_position.y += 34; }
    ( a._properties.vertical ? a._label_position.y : a._label_position.x ) -= a._properties.label.size() * 4;
    for( float v = a._properties.extents.first; v <= a._properties.extents.second; v += a._properties.step )
    { 
        std::ostringstream oss;
        oss.precision( 4 );
        oss << v;
        a._labels.push_back( oss.str() );
    }
    if( !a._properties.label.empty() )
    {
        int baseline{0};
        auto s = cv::getTextSize( a._properties.label, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline );
        a._label = cv::Mat( s.height, s.width, CV_8UC3, cv::Scalar( 0, 0, 0 ) );
        cv::putText( a._label, a._properties.label, cv::Point( 0, s.height - 1 ), cv::FONT_HERSHEY_SIMPLEX, 0.4, a._properties.color * 0.8, 1, impl::line_aa );
        if( a._properties.vertical ) { cv::Mat transposed; cv::transpose( a._label, transposed ); cv::flip( transposed, a._label, 0 ); }
        a._label_rectangle = cv::Rect( a._label_position, cv::Point( a._label_position.x + a._label.cols, a._label_position.y + a._label.rows ) );
    }
    return std::make_pair( boost::bind< std::pair< H, cv::Mat > >( a, _1 ), true );
}

template < typename H >
std::pair< H, cv::Mat > draw< H >::axis::operator()( std::pair< H, cv::Mat > m ) // todo: pre-draw in make() and then just apply on top of the image
{
    cv::line( m.second, _properties.geometry.first, _properties.geometry.second, _properties.color );
    cv::Point a = _properties.geometry.first;
    float v = _properties.extents.first;
    for( unsigned int o{0}, i{0}; o <= _properties.size; o += _step, v += _properties.step, ++i )
    {
        cv::Point b{a};
        ( _properties.vertical ? b.x : b.y ) += 3;
        cv::line( m.second, a, b, _properties.color );
        if( _properties.vertical )
        {
            // todo: draw labels
        }
        else
        {
            cv::Point c{a};
            ( _properties.vertical ? c.x : c.y ) += 16;
            ( _properties.vertical ? c.y : c.x ) -= _labels[i].size() * 4;
            cv::putText( m.second, _labels[i], c, cv::FONT_HERSHEY_SIMPLEX, 0.4, _properties.color * 0.8, 1, impl::line_aa );
        }
        if( _step == 0 ) { break; }
        ( _properties.vertical ? a.y : a.x ) += _step;
    }
    if( !_properties.label.empty() )
    {
        if( _properties.vertical ) // todo: transparent background
        {
            _label.copyTo( m.second( _label_rectangle ) );
        }
        else
        {
            cv::putText( m.second, _properties.label, _label_position, cv::FONT_HERSHEY_SIMPLEX, 0.5, _properties.color * 0.8, 1, impl::line_aa );
        }
    }
    return m;
}

template < typename H >
std::string draw< H >::time::usage( unsigned int indent )
{
    std::ostringstream oss;
    std::string i( indent, ' ' );
    oss << i << "draw=time,<x>,<y>,<color>,<fontsize>\n";
    oss << i << "    draw timestamp on image\n";
    oss << i << "    options\n";
    oss << i << "        <x>,<y>: timestamp position; default: 10,10\n";
    oss << i << "        <color>: <r>,<g>,<b>[,<a>]; default: 0,0,0\n";
    oss << i << "        <fontsize>: font size as float; default: 0.5\n";
    return oss.str();
}

template < typename H >
std::pair< typename draw< H >::functor_t, bool > draw< H >::time::make( const std::string& options, draw< H >::timestamp_functor_t get_timestamp, char delimiter )
{
    const auto& v = comma::split( options, delimiter, true );
    time t;
    if( v.size() > 0 && !v[0].empty() ) { t._origin.x = boost::lexical_cast< unsigned int >( v[0] ); }
    if( v.size() > 1 && !v[1].empty() ) { t._origin.y = boost::lexical_cast< unsigned int >( v[1] ); }
    if( v.size() > 4 ) { t._color = cv::Scalar( v[2].empty() ? 0 : boost::lexical_cast< unsigned int >( v[2] )
                                              , v[3].empty() ? 0 : boost::lexical_cast< unsigned int >( v[3] )
                                              , v[4].empty() ? 0 : boost::lexical_cast< unsigned int >( v[4] ) ); }
    if( v.size() > 5 && !v[5].empty() ) { t._font_size = boost::lexical_cast< float >( v[5] ); }
    t._timestamp = get_timestamp;
    return std::make_pair( boost::bind< std::pair< H, cv::Mat > >( t, _1 ), true );
}

template < typename H >
std::pair< H, cv::Mat > draw< H >::time::operator()( std::pair< H, cv::Mat > m )
{
    cv::putText( m.second, boost::posix_time::to_iso_string( _timestamp( m.first ) ), _origin, cv::FONT_HERSHEY_SIMPLEX, _font_size, _color, 1, impl::line_aa );
    return m;
}

template < typename H >
std::string draw< H >::status::usage( unsigned int indent )
{
    std::ostringstream oss;
    std::string i( indent, ' ' );
    oss << i << "draw=status,<options>\n";
    oss << i << "    draw status bar on image: timestamp, count, and fps; currently only 3-byte rgb supported\n";
    oss << i << "    <options>\n";
    oss << i << "        [label:<text>]: status text label\n";
    oss << i << "        origin:<x>,<y>: origin in pixels, if negative, offset\n";
    oss << i << "                        is from the bottom of the image; default: 20,20\n";
    oss << i << "        color:<r>,<g>,<b>: axis color; default: 0,0,0\n";
    oss << i << "        bg-color:<r>,<g>,<b>[,<a>]: background color; default: 220,220,220\n";
    oss << i << "                                    for now only <a>=0 for no rectangle is supported\n";
    oss << i << "        font-size:<float>: default: 0.4\n";
    oss << i << "        alpha:<float>: fps ema alpha; default: 0.5\n";
    oss << i << "        spin-up:<float>: fps spin-up; default: 1\n";
    oss << i << "        system-time: use system time for ema instead of image timestamp\n";
    return oss.str();
}

template < typename H >
std::pair< typename draw< H >::functor_t, bool > draw< H >::status::make( const std::string& options, draw< H >::timestamp_functor_t get_timestamp, char delimiter )
{
    status s;
    s._properties = comma::name_value::parser( '|', ':' ).get< properties >( options );
    s._timestamp = get_timestamp;
    int baseline{0};
    s._text_size = cv::getTextSize( s._properties.label.empty() ? std::string( "20230101T000000.000000" ) : s._properties.label, cv::FONT_HERSHEY_SIMPLEX, s._properties.font_size, 1, &baseline );
    if( s._properties.label.empty() ) { s._text_size.width = 0; }
    return std::make_pair( boost::bind< std::pair< H, cv::Mat > >( s, _1 ), false );
}

template < typename H >
std::pair< H, cv::Mat > draw< H >::status::operator()( std::pair< H, cv::Mat > m )
{
    cv::Point origin{ _properties.origin.x < 0 ? m.second.cols + _properties.origin.x : _properties.origin.x
                    , _properties.origin.y < 0 ? m.second.rows + _properties.origin.y : _properties.origin.y };
    cv::Point offset{ origin.x + _text_size.width + 30, origin.y }; // quick and dirty; arbitrary
    // todo: count: check image size
    // todo: fps: check image size
    // todo: background transparency
    if( _properties.bg_color[3] > 0 ) { cv::rectangle( m.second, cv::Point( 0, origin.y - _text_size.height - 2 ), cv::Point( m.second.rows - 1, origin.x + 3 ), _properties.bg_color, impl::filled, impl::line_aa ); }
    if( !_properties.label.empty() ) { cv::putText( m.second, _properties.label, origin, cv::FONT_HERSHEY_SIMPLEX, _properties.font_size, _properties.color, 1, impl::line_aa ); }
    cv::putText( m.second, boost::posix_time::to_iso_string( _timestamp( m.first ) ), offset, cv::FONT_HERSHEY_SIMPLEX, _properties.font_size, _properties.color, 1, impl::line_aa );
    {
        std::ostringstream oss;
        oss << "frames: " << ( _count + 1 );
        cv::putText( m.second, oss.str(), cv::Point{offset.x + 246, offset.y}, cv::FONT_HERSHEY_SIMPLEX, _properties.font_size, _properties.color, 1, impl::line_aa );
    }
    {
        auto t = _properties.system_time ? boost::posix_time::microsec_clock::universal_time() : _timestamp( m.first );
        COMMA_ASSERT( !t.is_not_a_date_time(), "frame-rate: asked to use timestamp from image, but input image has no timestamp" );
        if( !_previous.is_not_a_date_time() )
        {
            auto d = t - _previous;
            double interval = d.total_seconds() + double( d.total_microseconds() ) / 1000000;
            _average_interval += ( interval - _average_interval ) * ( _count <= _properties.spin_up ? 1. / _count : _properties.alpha );
        }
        _previous = t;
        std::ostringstream oss;
        oss.precision( 4 );
        oss << "fps: " << ( 1. / _average_interval );
        cv::putText( m.second, oss.str(), cv::Point{offset.x + 402, offset.y}, cv::FONT_HERSHEY_SIMPLEX, _properties.font_size, _properties.color, 1, impl::line_aa );
    }
    ++_count;
    return m;
}

template struct draw< boost::posix_time::ptime >;
template struct draw< std::vector< char > >;

} } }  // namespace snark { namespace cv_mat { namespace filters {