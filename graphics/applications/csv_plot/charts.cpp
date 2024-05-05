// Copyright (c) 2021 Vsevolod Vlaskine

#include "charts.h"

namespace snark { namespace graphics { namespace plotting {

chart::config_t::config_t( const std::string& name, const std::string& t )
    : animate( true )
    , legend( false )
    , name( name )
    , scroll( false )
    , title( t.empty() ? name : t )
{
}

chart::config_t::config_t( const comma::command_line_options& options )
    : scroll( options.exists( "--scroll" ) )
{
}

QtCharts::QChart::ChartTheme chart::config_t::theme_from_string( const std::string& t )
{
    if( t.empty() || t == "light" ) { return QChart::ChartThemeLight; }
    if( t == "blue-cerulean" ) { return QChart::ChartThemeBlueCerulean; }
    if( t == "dark" ) { return QChart::ChartThemeDark; }
    if( t == "brown-sand" ) { return QChart::ChartThemeBrownSand; }
    if( t == "blue-ncs" ) { return QChart::ChartThemeBlueNcs; }
    if( t == "high-contrast" ) { return QChart::ChartThemeHighContrast; }
    if( t == "blue-icy" ) { return QChart::ChartThemeBlueIcy; }
    if( t == "qt" ) { return QChart::ChartThemeQt; }
    COMMA_THROW_BRIEF( comma::exception, "expected chart theme, got: '" << t << "'" );
}

QtCharts::QChart::ChartType chart::config_t::type_from_string( const std::string& t )
{
    if( t.empty() || t == "cartesian" ) { return QChart::ChartTypeCartesian; }
    if( t == "polar" ) { return QChart::ChartTypePolar; }
    COMMA_THROW_BRIEF( comma::exception, "expected chart type, got: '" << t << "'" );
}

chart::chart( const chart::config_t& config, QGraphicsItem *parent, Qt::WindowFlags window_flags )
    : QChart( config.get_type(), parent, window_flags )
    , _config( config )
    , _fixed_x( config.min.x && config.max.x )
    , _fixed_y( config.min.y && config.max.y )
    , _zooming( false )
{
    setTitle( &_config.title[0] );
    if( !_config.legend ) { legend()->hide(); }
    if( _config.animate ) { setAnimationOptions( QChart::SeriesAnimations ); }
    grabGesture( Qt::PanGesture );
    grabGesture( Qt::PinchGesture );
    setTheme( config.get_theme() );
}
            
xy_chart::xy_chart( const chart::config_t& config, QGraphicsItem *parent, Qt::WindowFlags window_flags )
    : chart( config, parent, window_flags )
    , _x_axis( new QValueAxis )
    , _y_axis( new QValueAxis )
{
    //setMargins( QMargins( 5, 5, 5, 5 ) ); // quick and dirty
    //QFont font( _x_axis->titleFont().family(), 1, 1 );
    _x_axis->setTitleFont( _x_axis->titleFont() ); // voodoo, this removes font boldness... whatever...
    _x_axis->setTitleText( &config.axes.x.title[0] );
    _x_axis->setTickAnchor( config.axes.x.tick.anchor );
    _x_axis->setTickInterval( config.axes.x.tick.interval );
    if( config.axes.x.tick.anchor != 0 || config.axes.x.tick.interval != 0 ) { _x_axis->setTickType( QValueAxis::TicksDynamic ); }
    _x_axis->setTickCount( config.axes.x.tick.count );
    if( !config.axes.x.label.format.empty() ) { _x_axis->setLabelFormat( &config.axes.x.label.format[0] ); }
    _x_axis->setLabelsAngle( config.axes.x.label.angle );
    if( config.axes.x.label.nice ) { _x_axis->applyNiceNumbers(); }
    double min_x = config.min.x ? *config.min.x : 0;
    double max_x = config.max.x ? *config.max.x : min_x + 10; // quick and dirty
    _x_axis->setRange( min_x, max_x );
    _y_axis->setTitleText( &config.axes.y.title[0] );
    _y_axis->setTitleFont( _y_axis->titleFont() ); // voodoo, this removes font boldness... whatever...
    _y_axis->setTickAnchor( config.axes.y.tick.anchor );
    _y_axis->setTickInterval( config.axes.y.tick.interval );
    if( config.axes.y.tick.anchor != 0 || config.axes.y.tick.interval != 0 ) { _y_axis->setTickType( QValueAxis::TicksDynamic ); }
    _y_axis->setTickCount( config.axes.y.tick.count );
    if( !config.axes.y.label.format.empty() ) { _y_axis->setLabelFormat( &config.axes.y.label.format[0] ); }
    _y_axis->setLabelsAngle( config.axes.y.label.angle );
    if( config.axes.y.label.nice ) { _y_axis->applyNiceNumbers(); }
    double min_y = config.min.y ? *config.min.y : 0;
    double max_y = config.max.y ? *config.max.y : min_y + 10; // quick and dirty
    _y_axis->setRange( min_y, max_y );
    addAxis( _x_axis, Qt::AlignBottom ); // todo? make configurable
    addAxis( _y_axis, Qt::AlignLeft ); // todo? make configurable
}

void xy_chart::push_back( plotting::series::xy* s )
{
    _series.push_back( s );
    addSeries( ( *s )() );
    ( *s )()->attachAxis( _x_axis );
    ( *s )()->attachAxis( _y_axis );
    if( s->config().scroll ) { _config.scroll = true; } // quick and dirty
}

void xy_chart::update()
{
    if( _zooming || ( _fixed_x && _fixed_y ) ) { return; }
    _extents.reset();
    for( auto s: _series )
    {
        if( !s->updated() ) { continue; }
        if( !_extents ) { _extents = std::make_pair( QPointF( std::numeric_limits< double >::max(), std::numeric_limits< double >::max() )
                                                   , QPointF( std::numeric_limits< double >::lowest(), std::numeric_limits< double >::lowest() ) ); }
        if( !_config.min.x && _extents->first.x() > s->extents().first.x() ) { _extents->first.setX( s->extents().first.x() ); }
        if( !_config.max.x && _extents->second.x() < s->extents().second.x() ) { _extents->second.setX( s->extents().second.x() ); }
        if( !_config.min.y && _extents->first.y() > s->extents().first.y() ) { _extents->first.setY( s->extents().first.y() ); }
        if( !_config.max.y && _extents->second.y() < s->extents().second.y() ) { _extents->second.setY( s->extents().second.y() ); }
    }
    if( _config.axes.x.label.nice ) { _x_axis->applyNiceNumbers(); } // am not sure it works at all
    if( _config.axes.y.label.nice ) { _y_axis->applyNiceNumbers(); } // am not sure it works at all
    if( !_extents ) { return; }
    if( !_fixed_x )
    {
        if( _config.scroll ) // todo! quick and dirty; improve
        {
            double mx = ( _extents->second.x() - _extents->first.x() ) * 0.1;
            if( !_config.min.x ) { _x_axis->setMin( _extents->first.x() - mx ); }
            if( !_config.max.x ) { _x_axis->setMax( _extents->second.x() + mx ); }
        }
        else
        {
            double mx = ( _x_axis->max() - _x_axis->min() ) * 0.1; // todo: make configurable or at least not so daft
            if( !_config.min.x && _x_axis->min() > _extents->first.x() ) { _x_axis->setMin( _extents->first.x() - mx ); }
            if( !_config.max.x && _x_axis->max() < _extents->second.x() ) { _x_axis->setMax( _extents->second.x() + mx ); }
        }
    }
    if( !_fixed_y )
    {
        if( _config.scroll ) // todo! quick and dirty; improve
        {
            double my = ( _extents->second.y() - _extents->first.y() ) * 0.1;
            if( !_config.min.y ) { _y_axis->setMin( _extents->first.y() - my ); }
            if( !_config.max.y ) { _y_axis->setMax( _extents->second.y() + my ); }
        }
        else
        {
            double my = ( _y_axis->max() - _y_axis->min() ) * 0.1; // todo: make configurable or at least not so daft
            if( !_config.min.y && _y_axis->min() > _extents->first.y() ) { _y_axis->setMin( _extents->first.y() - my ); }
            if( !_config.max.y && _y_axis->max() < _extents->second.y() ) { _y_axis->setMax( _extents->second.y() + my ); }
        }
    }
}

} } } // namespace snark { namespace graphics { namespace plotting {
