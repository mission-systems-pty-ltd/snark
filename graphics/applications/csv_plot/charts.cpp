// Copyright (c) 2021 Vsevolod Vlaskine

#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include "charts.h"

namespace snark { namespace graphics { namespace plotting {

chart::chart( const std::string& title, QGraphicsItem *parent, Qt::WindowFlags window_flags )
    : QChart( QChart::ChartTypeCartesian, parent, window_flags )
    , title_( title )
{
    setTitle( &title[0] );
    legend()->hide();
    setAnimationOptions( QChart::SeriesAnimations ); // chart->setAnimationOptions( QChart::AllAnimations ); // todo? make configurable?
}

xy_chart::xy_chart( const std::string& title, QGraphicsItem *parent, Qt::WindowFlags window_flags )
    : chart( title, parent, window_flags )
    , x_axis_( new QValueAxis )
    , y_axis_( new QValueAxis )
    , scroll_( false )
{
    addAxis( x_axis_, Qt::AlignBottom );
    addAxis( y_axis_, Qt::AlignLeft );
    x_axis_->setTickCount( 1 ); // todo!
    x_axis_->setRange( 0, 10 ); // todo!
    y_axis_->setRange( 0, 10 ); // todo!
}

void xy_chart::push_back( plotting::series::xy* s )
{
    series_.push_back( s );
    addSeries( ( *s )() );
    ( *s )()->attachAxis( x_axis_ );
    ( *s )()->attachAxis( y_axis_ );
    if( s->config().scroll ) { scroll_ = true; } // todo: quick and dirty; scroll should be chart property
}

void xy_chart::update()
{
    extents_.reset();
    for( auto s: series_ )
    {
        //if( !s->updated() ) { continue; }
        if( !extents_ ) { extents_ = std::make_pair( QPointF( std::numeric_limits< double >::max(), std::numeric_limits< double >::max() ), QPointF( std::numeric_limits< double >::min(), std::numeric_limits< double >::min() ) ); }
        if( extents_->first.x() > s->extents().first.x() ) { extents_->first.setX( s->extents().first.x() ); }
        if( extents_->second.x() < s->extents().second.x() ) { extents_->second.setX( s->extents().second.x() ); }
        if( extents_->first.y() > s->extents().first.y() ) { extents_->first.setY( s->extents().first.y() ); }
        if( extents_->second.y() < s->extents().second.y() ) { extents_->second.setY( s->extents().second.y() ); }
    }
    if( !extents_ ) { return; }
    if( scroll_ ) // todo! quick and dirty; improve
    {
        double mx = ( extents_->second.x() - extents_->first.x() ) * 0.1;
        double my = ( extents_->second.y() - extents_->first.y() ) * 0.1;
        x_axis_->setMin( extents_->first.x() - mx );
        x_axis_->setMax( extents_->second.x() + mx );
        y_axis_->setMin( extents_->first.y() - my );
        y_axis_->setMax( extents_->second.y() + my );
    }
    else
    {
        double mx = ( x_axis_->max() - x_axis_->min() ) * 0.1; // todo: make configurable or at least not so daft
        double my = ( y_axis_->max() - y_axis_->min() ) * 0.1; // todo: make configurable or at least not so daft
        if( x_axis_->min() > extents_->first.x() ) { x_axis_->setMin( extents_->first.x() - mx ); }
        if( x_axis_->max() < extents_->second.x() ) { x_axis_->setMax( extents_->second.x() + mx ); }
        if( y_axis_->min() > extents_->first.y() ) { y_axis_->setMin( extents_->first.y() - my ); }
        if( y_axis_->max() < extents_->second.y() ) { y_axis_->setMax( extents_->second.y() + my ); }
    }
}

} } } // namespace snark { namespace graphics { namespace plotting {
