// Copyright (c) 2011 The University of Sydney

/// @author Vsevolod Vlaskine

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>
#include <qnamespace.h>
#include <comma/string/string.h>
#include "colored.h"

namespace snark { namespace graphics { namespace view {

static color_t color_from_name( const std::string& name )
{
         if( name == "black" )   { return color_t( Qt::black ); }
    else if( name == "white" )   { return color_t( Qt::white ); }
    else if( name == "red" )     { return color_t( Qt::red ); }
    else if( name == "green" )   { return color_t( Qt::green ); }
    else if( name == "blue" )    { return color_t( Qt::blue ); }
    else if( name == "yellow" )  { return color_t( Qt::yellow ); }
    else if( name == "magenta" ) { return color_t( Qt::magenta ); }
    else if( name == "cyan" )    { return color_t( Qt::cyan ); }
    else if( name == "grey" )    { return color_t( Qt::gray ); }
    else if( name == "pink" )    { return color_t( 255, 128, 128 ); }
    else if( name == "orange" )  { return color_t( 255, 128, 0 ); }
    else if( name == "salad" )   { return color_t( 128, 255, 128 ); }
    else if( name == "sky" )     { return color_t( 128, 128, 255 ); }
    else
    {
        auto split = []( const std::string& s, char delimiter )
        {
            const std::vector< std::string >& v = comma::split( s, delimiter );
            try
            {
                switch( v.size() )
                {
                    case 3: return color_t( boost::lexical_cast< int >( v[0] ), boost::lexical_cast< int >( v[1] ), boost::lexical_cast< int >( v[2] ) );
                    case 4: return color_t( boost::lexical_cast< int >( v[0] ), boost::lexical_cast< int >( v[1] ), boost::lexical_cast< int >( v[2] ), boost::lexical_cast< int >( v[3] ) );
                    default: COMMA_THROW( comma::exception, "expected color as rgb or rgba, got '" << s << "'" );
                }
            }
            catch( std::exception& ex ) { COMMA_THROW( comma::exception, "expected color, got '" << s << "'" << "(" << ex.what() << ")" ); }
            catch( ... ) { COMMA_THROW( comma::exception, "expected color, got '" << s << "' (unknown exception)" ); }
        };
        try { return split( name, '|' ); } catch ( ... ) { return split( name, ',' ); }
    }
}

static color_t multiply( const color_t& color, double scalar )
{
#if Qt3D_VERSION==1
    double red = scalar * color.redF();
    double green = scalar * color.greenF();
    double blue = scalar * color.blueF();
    double alpha = scalar * color.alphaF();
    return color_t::fromRgbF( red, green, blue, alpha );
#else
    return color * scalar;
#endif
}

static color_t add( const color_t& a, const color_t& b )
{
#if Qt3D_VERSION==1
    unsigned int red = std::min( a.red() + b.red(), 255 );
    unsigned int green = std::min( a.green() + b.green(), 255 );
    unsigned int blue = std::min( a.blue() + b.blue(), 255 );
    unsigned int alpha = std::min( a.alpha() + b.alpha(), 255 );
    return color_t( red, green, blue, alpha );
#else
    return a + b;
#endif
}


fixed::fixed( const std::string& name ) : color_( color_from_name( name ) ) {}

color_t fixed::color( const Eigen::Vector3d&, comma::uint32, double, const color_t& ) const { return color_; }

by_height::by_height( double from
                    , double to
                    , const color_t& from_color
                    , const color_t& to_color
                    , bool cyclic
                    , bool linear
                    , bool sharp )
    : from( from )
    , to( to )
    , sum( to + from )
    , diff( to - from )
    , middle( from / sum )
    , from_color( from_color )
    , to_color( to_color )
    , cyclic( cyclic )
    , linear( linear )
    , sharp( sharp )
{
    average_color = color_t( from_color.red() / 2   + to_color.red() / 2
                           , from_color.green() / 2 + to_color.green() / 2
                           , from_color.blue() / 2  + to_color.blue() / 2
                           , from_color.alpha() / 2 + to_color.alpha() / 2 );
}

color_t by_height::color( const Eigen::Vector3d& point, comma::uint32, double, const color_t& ) const
{
    if( cyclic )
    {
        double v = std::fmod( point.z() - std::floor( point.z() / sum ) * sum, sum ) / sum;
        if( sharp ) { return v < middle ? from_color : to_color; }
        if( linear )
        {
            if( v < middle * 0.5 )
            {
                return add( multiply( average_color, 1 - v ), multiply( from_color, v ) );
            }
            else if( v < ( 1 + middle ) * 0.5 )
            {
                v = v - middle * 0.5;
                return add( multiply( from_color, ( 1 - v ) ), multiply( to_color, v ) );
            }
            else
            {
                v = v - ( 1 + middle ) * 0.5;
                return add( multiply( to_color, 1 - v ), multiply( average_color, v ) );
            }
        }
        else
        {
            if( v < middle )
            {
                v = v / middle - 0.5;
                v = v * v * 4;
                return add( multiply( from_color, 1 - v ), multiply( average_color, v ) );
            }
            else
            {
                v = ( v - middle ) / ( 1 - middle ) - 0.5;
                v = v * v * 4;
                return add( multiply( average_color, v ), multiply( to_color, 1 - v ) );
            }
        }
    }
    else
    {
        double v = ( point.z() - from ) / diff;
        v = ( v < 0 ? 0 : v > 1 ? 1 : v );
        return add( multiply( from_color, 1 - v ), multiply( to_color, v ) );
    }
}

by_scalar::by_scalar( double from, double to, const color_t& from_color, const color_t& to_color, bool alpha_by_scalar, bool cyclic )
    : from( from )
    , to( to )
    , diff( to - from )
    , from_color( from_color )
    , to_color( to_color )
    , _alpha_by_scalar( alpha_by_scalar )
    , _cyclic( cyclic )
{
    if( alpha_by_scalar ) { this->from_color.rgba[3] = 0; }
}

by_scalar::by_scalar( double from, double to, const snark::render::colour_map::values& map, bool alpha_by_scalar, bool cyclic )
    : from( from )
    , to( to )
    , diff( to - from )
    , map( map )
    , _alpha_by_scalar( alpha_by_scalar )
    , _cyclic( cyclic )
{
}

color_t by_scalar::color( const Eigen::Vector3d& point, comma::uint32, double scalar, const color_t& ) const
{
    (void)point;
    double v = ( scalar - from ) / diff;
    v = _cyclic ? v - std::floor( v ) : ( v < 0 ? 0 : v > 1 ? 1 : v );
    if( !map ) { return add( multiply( from_color, 1 - v ), multiply( to_color, v ) ); }
    unsigned int i = v * 255;
    return color_t( ( *map )[i][0], ( *map )[i][1], ( *map )[i][2], _alpha_by_scalar ? i : 255 );
}

namespace impl {

static boost::array< unsigned int, 256 > colorInit()
{
    boost::array< unsigned int, 256 > a;
    for( std::size_t i = 0; i < 256; ++i ) { a[i] = i; }
    std::random_shuffle( a.begin(), a.end() );
    return a;
}
static unsigned char colorIndex = 23; // whatever
static boost::array< unsigned int, 256 > colorIndices = colorInit();

} // namespace impl {

by_id::by_id( const color_t& backgroundcolor )
    : background_( backgroundcolor )
    , has_scalar_( false )
{
}

by_id::by_id( const color_t& backgroundcolor
            , double from
            , double to )
    : background_( backgroundcolor )
    , has_scalar_( true )
    , from_( from )
    , diff_( to - from )
{
}

color_t by_id::color( const Eigen::Vector3d&, comma::uint32 id, double scalar, const color_t& ) const // quick and dirty
{
    static const float b = 0.95f;
    unsigned char i = ( id & 0xff ) + ( ( id & 0xff00 ) >> 16 );
    const float h = b * float( impl::colorIndices[ i ] ) / 255;
    const float a = 0.3f * float( 255 - impl::colorIndices[ i ] ) / 255;
    color_t color;
    switch( ( i * 13 + impl::colorIndex ) % 6 )
    {
#if Qt3D_VERSION==1
        case 0 : color = color_t::fromRgbF( b, h, a ); break;
        case 1 : color = color_t::fromRgbF( a, b, h ); break;
        case 2 : color = color_t::fromRgbF( h, a, b ); break;
        case 3 : color = color_t::fromRgbF( b, a, h ); break;
        case 4 : color = color_t::fromRgbF( h, b, a ); break;
        default: color = color_t::fromRgbF( a, h, b ); break;
#else
        case 0 : color = color_t( b, h, a ); break;
        case 1 : color = color_t( a, b, h ); break;
        case 2 : color = color_t( h, a, b ); break;
        case 3 : color = color_t( b, a, h ); break;
        case 4 : color = color_t( h, b, a ); break;
        default: color = color_t( a, h, b ); break;
#endif
    }
    if( !has_scalar_ ) { return color; }
    double v = ( scalar - from_ ) / diff_;
    v = ( v < 0 ? 0 : v > 1 ? 1 : v );
    return add( multiply( color, v ), multiply( background_, 1 - v ) );
}

by_id_color_map::by_id_color_map( const std::unordered_map< comma::uint32, color_t >& colors, color_t not_found, bool cyclic ): colors_( colors ), not_found_( not_found ), cyclic_( cyclic ) {}

color_t by_id_color_map::color( const Eigen::Vector3d&, comma::uint32 id, double scalar, const color_t& ) const
{ 
    if( cyclic_ ) { return colors_.find( id % colors_.size() )->second; }
    auto it = colors_.find( id );
    return it == colors_.end() ? not_found_ : it->second;
}

color_t by_rgb::color( const Eigen::Vector3d& , comma::uint32, double, const color_t& c ) const { return c; }

colored* color_from_string( const std::string& t, const std::string& fields, const color_t& backgroundcolor )
{
    std::string s = t.empty() ? std::string( "yellow" ) : t; // std::string s = t.empty() ? std::string( "0:1,cyan:magenta" ) : t;
    std::vector< std::string > f = comma::split( fields, ',' );
    bool has_id = false;
    for( unsigned int i = 0; !has_id && i < f.size(); ++i ) { has_id = f[i] == "id"; } // quick and dirty
    bool has_scalar = false;
    for( unsigned int i = 0; !has_scalar && i < f.size(); ++i ) { has_scalar = f[i] == "scalar"; } // quick and dirty
    bool r = false;
    bool g = false;
    bool b = false;
    bool a = false;
    for( unsigned int i = 0; !( r && g && b ) && !a && i < f.size(); ++i ) // quick and dirty
    {
        if( f[i] == "r" ) { r = true; }
        if( f[i] == "g" ) { g = true; }
        if( f[i] == "b" ) { b = true; }
        if( f[i] == "a" ) { a = true; }
    }
    if( has_id )
    {
        if( has_scalar )
        {
            const std::vector< std::string >& v = comma::split( comma::split( s, ',' )[0], ':' );
            if( v.size() != 2 ) { COMMA_THROW( comma::exception, "expected range (e.g: -5:20), got " << s ); }
            double from = boost::lexical_cast< double >( v[0] );
            double to = boost::lexical_cast< double >( v[1] );
            return new by_id( backgroundcolor, from, to );
        }
        if( t.empty() ) { return new by_id( backgroundcolor ); }
        const std::vector< std::string >& v = comma::split( t, ',' );
        std::unordered_map< comma::uint32, color_t > colors;
        bool cyclic = false;
        bool not_cyclic = false;
        color_t default_color = color_from_name( "grey" );
        for( unsigned int i = 0; i < v.size(); ++i )
        {
            const std::vector< std::string >& w = comma::split( v[i], ':' );
            switch( w.size() )
            {
                case 1:
                    cyclic = true;
                    colors[i] = color_from_name( w[0] );
                    break;
                case 2:
                    not_cyclic = true;
                    if( w[0] == "default" ) { default_color = color_from_name( w[1] ); }
                    else { colors[ boost::lexical_cast< comma::uint32 >( w[0] ) ] = color_from_name( w[1] ); }
                    break;
                default:
                    COMMA_THROW( comma::exception, "expected color definition; got --color='" << s << "'" );
            }
        }
        if( cyclic && not_cyclic ) { COMMA_THROW( comma::exception, "expected either color list or color map; got --color='" << s << "'" ); }
        return new by_id_color_map( colors, default_color, cyclic );
    }
    if( r || g || b || a ) { return new by_rgb; }
    if( has_scalar )
    {
        color_t from_color, to_color;
        double from = 0;
        double to = 0;
        boost::optional< snark::render::colour_map::values > map;
        bool alpha_by_scalar{false};
        bool cyclic{false};
        std::vector< std::string > v = comma::split( s, ',' );
        switch( v.size() )
        {
            case 1:
            {
                std::vector< std::string > w = comma::split( v[0], ':' );
                switch( w.size() )
                {
                    case 1:
                        if( w[0] == "green" ) { map = snark::render::colour_map::constant( 0, 255, 0 ); }
                        else if( w[0] == "red" ) { map = snark::render::colour_map::constant( 255, 0, 0 ); }
                        else if( w[0] == "hot" ) { map = snark::render::colour_map::temperature( 96, 96 ); }
                        else if( w[0] == "jet" ) { map = snark::render::colour_map::jet(); }
                        else if( w[0] == "magma" ) { map = snark::render::colour_map::magma(); }
                        else if( w[0] == "viridis" ) { map = snark::render::colour_map::viridis(); }
                        else if( w[0] == "twilight" ) { map = snark::render::colour_map::twilight(); }
                        else { COMMA_THROW( comma::exception, "expected colour map, got: " << s ); }
                        break;
                    case 2:
                        if( w[0][0] >= 'a' && w[0][0] <= 'z' )
                        {
                            from_color = color_from_name( w[0] );
                            to_color = color_from_name( w[1] );
                        }
                        else
                        {
                            from_color = color_from_name( "cyan" );
                            to_color = color_from_name( "magenta" );
                            from = boost::lexical_cast< double >( w[0] );
                            to = boost::lexical_cast< double >( w[1] );
                        }
                        break;
                    default:
                        COMMA_THROW( comma::exception, "expected range (e.g. -5:20,red:blue or 3:10), or colour map name, got " << s );
                }
                break;
            }
            case 2:
            case 3:
            {
                std::vector< std::string > w = comma::split( v[0], ':' );
                from = boost::lexical_cast< double >( w[0] );
                to = boost::lexical_cast< double >( w[1] );
                w = comma::split( v[1], ':' );
                switch( w.size() )
                {
                    case 1:
                        if( w[0] == "green" ) { map = snark::render::colour_map::constant( 0, 255, 0 ); }
                        else if( w[0] == "red" ) { map = snark::render::colour_map::constant( 255, 0, 0 ); }
                        else if( w[0] == "hot" ) { map = snark::render::colour_map::temperature( 96, 96 ); }
                        else if( w[0] == "jet" ) { map = snark::render::colour_map::jet(); }
                        else if( w[0] == "magma" ) { map = snark::render::colour_map::magma(); }
                        else if( w[0] == "viridis" ) { map = snark::render::colour_map::viridis(); }
                        else if( w[0] == "twilight" ) { map = snark::render::colour_map::twilight(); }
                        else { COMMA_THROW( comma::exception, "expected colour map, got: " << s ); }
                        break;
                    case 2:
                        from_color = color_from_name( w[0] );
                        to_color = color_from_name( w[1] );
                        break;
                    default:
                        COMMA_THROW( comma::exception, "expected range (e.g. -5:20,red:blue or 3:10), or colour map name, got " << s );
                }
                for( unsigned int i = 2; i < v.size(); ++i )
                {
                    if( v[i] == "alpha" ) { alpha_by_scalar = true; }
                    else if( v[i] == "cyclic" ) { cyclic = true; }
                    else { COMMA_THROW_BRIEF( comma::exception, "expected range (e.g. -5:20,red:blue[,alpha] or 3:10), or colour map name, got " << s ); }
                }
                break;
            }
            default:
                COMMA_THROW( comma::exception, "expected range (e.g. -5:20,red:blue[,alpha] or 3:10), or colour map name, got " << s );
        }
        return map ? new by_scalar( from, to, *map, alpha_by_scalar, cyclic ) : new by_scalar( from, to, from_color, to_color, alpha_by_scalar, cyclic );
    }
    colored* c;
    try
    {
        c = new fixed( s );
    }
    catch( ... )
    {
        std::vector< std::string > v = comma::split( s, ',' );
        double from = 0;
        double to = 1;
        color_t from_color = color_from_name( "red" );
        color_t to_color = color_from_name( "yellow" );
        bool sharp = false;
        bool cyclic = false;
        bool linear = true;
        for( unsigned int i = 0; i < v.size(); ++i )
        {
            if( v[i].empty() ) { continue; }
            else if( v[i] == "sharp" ) { sharp = true; }
            else if( v[i] == "smooth" ) { sharp = false; }
            else if( v[i] == "cyclic" ) { cyclic = true; from = 1; }
            else if( v[i] == "linear" ) { linear = true; }
            else if( v[i] == "quadratic" ) { linear = false; }
            else
            {
                std::vector< std::string > w = comma::split( v[i], ':' );
                if( w[0][0] >= 'a' && w[0][0] <= 'z' )
                {
                    from_color = color_from_name( w[0] );
                    to_color = color_from_name( w[1] );
                }
                else
                {
                    from = boost::lexical_cast< double >( w[0] );
                    to = boost::lexical_cast< double >( w[ w.size() == 1 ? 0 : 1 ] );
                }
            }
        }
        c = new by_height( from, to, from_color, to_color, cyclic, linear, sharp );
    }
    return c;
}

} } } // namespace snark { namespace graphics { namespace view {
