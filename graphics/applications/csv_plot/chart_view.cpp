// Copyright (c) 2021 Vsevolod Vlaskine

#include "chart_view.h"
#include <QPointF>
#include <QRectF>

namespace snark { namespace graphics { namespace plotting {
            
chart_view::chart_view( QChart* chart, QWidget* parent )
    : QChartView( chart, parent )
    , mouse_click_state_( mouse_state::NONE )
    , chart_( dynamic_cast<plotting::chart*>(chart) )
    , rubber_band_box_()
{
    // setRubberBand( QChartView::RectangleRubberBand );  // rubber band: click and drag box thing
    setRubberBand( QChartView::NoRubberBand );  // rubber band: click and drag box thing
}
            
chart_view::chart_view( QWidget* parent )
    : QChartView( parent )
    , mouse_click_state_( mouse_state::NONE )
    , rubber_band_box_()
{
    // setRubberBand( QChartView::RectangleRubberBand );  // rubber band: click and drag box thing
    setRubberBand( QChartView::NoRubberBand );  // rubber band: click and drag box thing
}

void chart_view::mousePressEvent( QMouseEvent* event )
{
    switch ( event->button() )
    {
    case Qt::LeftButton:
        chart_->zooming( true );
        if ( mouse_click_state_ != mouse_state::NONE ) { break; }
        mouse_click_state_ = mouse_state::LEFT;
        rubber_band_box_.setTopLeft( event->localPos() );
        break;
    case Qt::RightButton:
        chart_->zooming( true );
        if ( mouse_click_state_ != mouse_state::NONE ) { break; }
        mouse_click_state_ = mouse_state::RIGHT;
        rubber_band_box_.setTopLeft( event->localPos() );
        break;
    case Qt::MiddleButton:
        chart_->zooming( true );
        if ( mouse_click_state_ != mouse_state::NONE ) { break; }
        mouse_click_state_ = mouse_state::MIDDLE;
        last_mouse_pos_ = event->pos();
        chart()->setAnimationOptions( QChart::NoAnimation );
        break;
    default:
        break;
    }
    QChartView::mousePressEvent( event );
}

void chart_view::mouseMoveEvent( QMouseEvent* event )
{
    if ( mouse_click_state_ == mouse_state::MIDDLE )
    {
        int dx = last_mouse_pos_.x() - event->x();
        int dy = event->y() - last_mouse_pos_.y();
        chart()->scroll( dx, dy );
        last_mouse_pos_ = event->pos();
    }
    QChartView::mouseMoveEvent( event );
}
void chart_view::mouseReleaseEvent( QMouseEvent* event )
{
    switch ( event->button() )
    {
    case Qt::LeftButton:
        if ( mouse_click_state_ == mouse_state::LEFT )
        {
            mouse_click_state_ = mouse_state::NONE;
            rubber_band_box_.setBottomRight( event->localPos() );
            chart()->zoomIn( rubber_band_box_ );
        }
        break;
    case Qt::RightButton:
        if ( mouse_click_state_ == mouse_state::RIGHT )
        {
            mouse_click_state_ = mouse_state::NONE; 
            rubber_band_box_.setBottomRight( event->localPos() );
            chart()->zoomIn( inverse_rubber_band_box() );
        }
        break;
    case Qt::MiddleButton:
        if ( mouse_click_state_ == mouse_state::MIDDLE ) { mouse_click_state_ = mouse_state::NONE; }
        chart()->setAnimationOptions( QChart::SeriesAnimations );
        break;
    default:
        break;
    }
    QChartView::mouseReleaseEvent( event );
}

QRectF chart_view::inverse_rubber_band_box()
{
    chart_->zooming( true );
    QRectF plot_area = chart()->plotArea();
    float height_ratio = plot_area.height() / rubber_band_box_.height();
    float width_ratio  = plot_area.width()  / rubber_band_box_.width();
    QPointF old_center = plot_area.center();
    if ( height_ratio > width_ratio )
    {
        plot_area.setHeight( plot_area.height() * height_ratio );
        plot_area.setWidth(  plot_area.width()  * height_ratio );
    }
    else
    {
        plot_area.setHeight( plot_area.height() * width_ratio );
        plot_area.setWidth(  plot_area.width()  * width_ratio );
    }
    plot_area.moveCenter( old_center );
    return plot_area;
}

void chart_view::rectangle_zoom( int scroll_angle, QRectF plot_area, QPoint& local_mouse_pos )
{
    chart_->zooming( true );
    float zoom_percent;
    if      ( scroll_angle > 0 ) { zoom_percent =     zoom_factor_; }
    else if ( scroll_angle < 0 ) { zoom_percent = 1 / zoom_factor_; }
    else                         { return;                          }
    plot_area.moveTo( ( 1 - zoom_percent ) * local_mouse_pos + zoom_percent * plot_area.topLeft() );
    plot_area.setSize( zoom_percent * plot_area.size() );
    chart()->zoomIn( plot_area );
}

void chart_view::basic_zoom( int scroll_angle )
{
    chart_->zooming( true );
    if      ( scroll_angle > 0 ) { chart()->zoom( 1.1 ); }
    else if ( scroll_angle < 0 ) { chart()->zoom( 0.9 ); }
}

void chart_view::wheelEvent( QWheelEvent* event )
{
    int scroll_angle = event->angleDelta().y();  // +ve if scrolling in
    if ( chart()->chartType() == QChart::ChartTypePolar )  // polar charts don't support rectangle zooming
    {
        basic_zoom( scroll_angle );
    }
    else
    {
        QRectF plot_area = chart()->plotArea();
        QPoint local_mouse_pos = QChartView::mapFromGlobal( event->globalPos() ); // deprecated, but does not work in earlier versions of qtcharts
        // QPoint local_mouse_pos = QChartView::mapFromGlobal( event->globalPosition().toPoint() );
        rectangle_zoom( scroll_angle, plot_area, local_mouse_pos );
    }
}

void chart_view::keyPressEvent( QKeyEvent* event )
{
    switch( event->key() )
    {
        case Qt::Key_Plus:  chart_->zooming( true  ); chart()->zoomIn();            break;
        case Qt::Key_Minus: chart_->zooming( true  ); chart()->zoomOut();           break;
        case Qt::Key_Left:  chart_->zooming( true  ); chart()->scroll( -50,   0 );  break;
        case Qt::Key_Right: chart_->zooming( true  ); chart()->scroll(  50,   0 );  break;
        case Qt::Key_Up:    chart_->zooming( true  ); chart()->scroll(   0,  50 );  break;
        case Qt::Key_Down:  chart_->zooming( true  ); chart()->scroll(   0, -50 );  break;
        case Qt::Key_R:     chart_->zooming( false ); chart()->zoomReset();         break;
        default:            QGraphicsView::keyPressEvent( event );                  break;
    }
}

} } } // namespace snark { namespace graphics { namespace plotting {
