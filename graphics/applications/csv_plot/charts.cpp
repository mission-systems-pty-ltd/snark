// Copyright (c) 2021 Vsevolod Vlaskine

#include <comma/math/compare.h>
#include "charts.h"

#include <iostream>

namespace snark { namespace graphics { namespace plotting {

chart::chart( float timeout, QGraphicsItem *parent, Qt::WindowFlags window_flags ): QChart( QChart::ChartTypeCartesian, parent, window_flags )
{
    QObject::connect( &timer_, &QTimer::timeout, this, &chart::update );
    timer_.setInterval( ( unsigned int )( timeout * 1000 ) );
}

chart::~chart() { shutdown(); }

void chart::start()
{
    for( unsigned int i = 0; i < streams_.size(); ++i ) { streams_[i].start(); }
    timer_.start();
}

void chart::shutdown()
{
    timer_.stop();
    for( unsigned int i = 0; i < streams_.size(); ++i ) { streams_[i].shutdown(); }
}

void chart::update()
{
    bool all_shutdown = true;
    extents_.reset();
    for( unsigned int i = 0; i < streams_.size(); ++i )
    {
        streams_[i].update();
        if( !streams_[i].is_shutdown() ) { all_shutdown = false; }
        if( streams_[i].size() > 0 )
        {
            if( !extents_ ) { extents_ = std::make_pair( QPointF( std::numeric_limits< double >::max(), std::numeric_limits< double >::max() ), QPointF( std::numeric_limits< double >::min(), std::numeric_limits< double >::min() ) ); }
            if( extents_->first.x() > streams_[i].extents().first.x() ) { extents_->first.setX( streams_[i].extents().first.x() ); }
            if( extents_->second.x() < streams_[i].extents().second.x() ) { extents_->second.setX( streams_[i].extents().second.x() ); }
            if( extents_->first.y() > streams_[i].extents().first.y() ) { extents_->first.setY( streams_[i].extents().first.y() ); }
            if( extents_->second.y() < streams_[i].extents().second.y() ) { extents_->second.setY( streams_[i].extents().second.y() ); }
        }
    }
    update_();
    if( all_shutdown ) { timer_.stop(); }
}

xy_chart::xy_chart( float timeout, QGraphicsItem *parent, Qt::WindowFlags window_flags )
    : chart( timeout, parent, window_flags )
    , x_axis_( new QValueAxis )
    , y_axis_( new QValueAxis )
{
    addAxis( x_axis_, Qt::AlignBottom );
    addAxis( y_axis_, Qt::AlignLeft );
    x_axis_->setTickCount( 1 ); // todo!
    x_axis_->setRange( 0, 10 ); // todo!
    y_axis_->setRange( -5, 10 ); // todo!
}

void xy_chart::push_back( plotting::stream* s )
{
    streams_.push_back( s );
    addSeries( s->series );
    s->series->attachAxis( x_axis_ );
    s->series->attachAxis( y_axis_ );
}

void xy_chart::update_()
{
    // todo: handle range of zero length
    // todo: add configurable margins
    // todo: handle various range policies
    // todo: fixed range
    if( extents_ )
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