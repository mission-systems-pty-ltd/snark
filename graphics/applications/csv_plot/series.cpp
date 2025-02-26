// Copyright (c) 2021 Vsevolod Vlaskine

#include <QtCharts/QXYSeries>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QSplineSeries>
#include <comma/base/exception.h>
#include "series.h"

namespace snark { namespace graphics { namespace plotting { namespace series {

static const char* hex_color_( const std::string& c )
{
    if( c == "red" ) { return "#FF0000"; }
    if( c == "green" ) { return "#00FF00"; }
    if( c == "blue" ) { return "#0000FF"; }
    if( c == "yellow" ) { return "#FFFF00"; }
    if( c == "cyan" ) { return "#00FFFF"; }
    if( c == "magenta" ) { return "#FF00FF"; }
    if( c == "black" ) { return "#000000"; }
    if( c == "white" ) { return "#FFFFFF"; }
    if( c == "grey" ) { return "#888888"; }
    // todo: more colours, e.g:
    // Red: #FF0000
    // Cyan: #00FFFF
    // Green: #00FF00
    // Magenta: #FF00FF
    // Blue: #0000FF
    // Orange: #FFA500
    // Purple: #800080
    // Teal: #008080
    // Dark Red: #8B0000
    // Lime: #00FF00
    // Dark Cyan: #008B8B
    // Indigo: #4B0082
    // Dark Green: #006400
    // Fuchsia: #FF00FF
    // Dodger Blue: #1E90FF
    // Dark Orange: #FF8C00
    return &c[0];
}

std::unordered_map< std::string, std::vector< std::string > > default_colourmaps()
{
    static std::unordered_map< std::string, std::vector< std::string > > m =
    {
        { "black",    { "black" } },
        { "rgb",      { "red", "green", "blue" } },
        { "cmyk",     { "cyan", "magenta", "yellow", "black" } },
        { "basic-4",  { "#FF0000","#00FFFF","#00FF00","#FF00FF" } },
        { "basic-8",  { "#FF0000","#00FFFF","#00FF00","#FF00FF","#0000FF","#FFA500","#800080","#008080" } },
        { "basic-16", { "#FF0000","#00FFFF","#00FF00","#FF00FF","#0000FF","#FFA500","#800080","#008080","#8B0000","#00FF00","#008B8B","#4B0082","#006400","#FF00FF","#1E90FF","#FF8C00" } }
        // todo: more nice maps
    };
    return m;
}

static std::unordered_map< std::string, std::vector< std::string > > _make_colourmap( const std::string& s ) // todo: put it in a proper class (series::colour or alike)
{
    static std::unordered_map< std::string, std::vector< std::string > > m = default_colourmaps();
    if( m.find( s ) == m.end() ) { m[s] = comma::split( s, ',', true ); }
    return m;
    // todo?
    // if( v.size() > 1 ) { m[s] = v; }
    // std::ifstream ifs( s );
    // COMMA_ASSERT_BRIEF( ifs.is_open(), "failed to open colour map file '" << s << "'" );
    // while()
}

static std::string _colormap_at( const std::string& s, unsigned int i )
{
    static std::unordered_map< std::string, std::vector< std::string > > colours = _make_colourmap( s );
    auto it = colours.find( s );
    return it->second[ i % it->second.size() ];
}

void config::set_next_colour() // hyper-quick and dirty for now
{
    if( color_map.empty() ) { return; }
    static unsigned int number_of_series{0};
    color_name = _colormap_at( color_map, number_of_series++ );
    color = QColor( hex_color_( color_name ) );
}

config::config( const comma::command_line_options& options )
    : color_map( options.value< std::string >( "--colors,--colours", "" ) )
    , color_name( options.value< std::string >( "--color,--colour", "" ) ) //: color_name( options.value< std::string >( "--color,--colour", _colormap_at( options.value< std::string >( "--colors,--colours", "black" ), number_of_series ) ) )
    , scroll( options.exists( "--scroll" ) )
    , shape( options.value< std::string >( "--shape,--type", "line" ) )
    , style( options.value< std::string >( "--style", "" ) )
    , title( options.value< std::string >( "--title", "" ) )
    , weight( options.value( "--weight", 1.0 ) )
{
    if( !color_name.empty() ) { color = QColor( hex_color_( color_name ) ); }
}

xy::xy( QtCharts::QXYSeries* s, const series::config& c ): series_( s ), config_( c ), updated_( false )
{
    QPen pen = config_.color ? QPen( *config_.color ) : QPen();
    pen.setWidth( config_.weight );
    series_->setPen( pen );
    series_->setName( &config_.title[0] );
    if( config_.color ) { series_->setColor( *config_.color ); }
}

void xy::clear()
{
    series_->clear();
    extents_ = std::make_pair( QPointF( std::numeric_limits< double >::max(), std::numeric_limits< double >::max() )
                             , QPointF( std::numeric_limits< double >::lowest(), std::numeric_limits< double >::lowest() ) );
    updated_ = false;
}

void xy::append( boost::posix_time::ptime, const point& p )
{
    series_->append( QPointF( *p.x, *p.y ) ); // todo: support 3d data, time series, polar data (or template stream upon those)
    if( extents_.first.x() > p.x ) { extents_.first.setX( *p.x ); }
    if( extents_.second.x() < p.x ) { extents_.second.setX( *p.x ); }
    if( extents_.first.y() > p.y ) { extents_.first.setY( *p.y ); }
    if( extents_.second.y() < p.y ) { extents_.second.setY( *p.y ); }
    updated_ = true;
}

QtCharts::QXYSeries* make_series_( const series::config& c, QtCharts::QChart* chart )
{
    if( c.shape == "line" || c.shape.empty() ) { return new QtCharts::QLineSeries( chart ); }
    if( c.shape == "spline" ) { return new QtCharts::QSplineSeries( chart ); }
    if( c.shape == "scatter" ) // todo: quick and dirty; make polymorphic
    {
        auto s = new QtCharts::QScatterSeries( chart );
        s->setMarkerSize( c.weight );
        if( c.color ) { s->setBorderColor( *c.color ); }
        return s;
    }
    COMMA_THROW( comma::exception, "csv-plot: expected stream type as shape, got: \"" << c.shape << "\"" );
};

bool xy::updated( bool reset )
{ 
    bool r = updated_;
    updated_ = updated_ && !reset;
    return r;
}

xy xy::make( const series::config& c, QtCharts::QChart* chart ) { return xy( make_series_( c, chart ), c ); }

} } } } // namespace snark { namespace graphics { namespace plotting { namespace series {
