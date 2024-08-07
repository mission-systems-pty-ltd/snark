// Copyright (c) 2021 Vsevolod Vlaskine

#pragma once

#include <string>
#include <vector>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtWidgets/QGesture>
#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QGraphicsView>
#include <boost/optional.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <comma/application/command_line_options.h>
#include "record.h"
#include "series.h"

// QT_BEGIN_NAMESPACE
// class QGestureEvent;
// QT_END_NAMESPACE

QT_USE_NAMESPACE
QT_CHARTS_USE_NAMESPACE

namespace snark { namespace graphics { namespace plotting {
    
class chart: public QChart
{
    Q_OBJECT
    public:
        struct axis
        { 
            struct tick
            {
                float anchor{0};
                float interval{0};
                float count{0};
            };
            struct label
            {
                std::string format;
                float angle{0};
                bool nice{false};
            };
            struct config
            {
                std::string title;
                axis::tick tick;
                axis::label label;
            };
        };

        struct axes { struct config { axis::config x; axis::config y; }; };
        
        struct config_t
        {
            bool animate;
            plotting::chart::axes::config axes;
            bool legend;
            point max; // todo? is it xychart-specific?
            point min; // todo? is it xychart-specific?
            std::string name;
            bool scroll; // todo? a better name?
            std::string title;
            std::string theme;
            std::string type{"cartesian"};
            
            config_t( const std::string& name = "", const std::string& title = "" );
            config_t( const comma::command_line_options& options );
            static config_t make( const std::string& s, const chart::config_t& defaults = chart::config_t() );
            static QtCharts::QChart::ChartTheme theme_from_string( const std::string& t );
            QtCharts::QChart::ChartTheme get_theme() const { return theme_from_string( theme ); }
            static QtCharts::QChart::ChartType type_from_string( const std::string& t );
            QtCharts::QChart::ChartType get_type() const { return type_from_string( type ); }
        };
        
        chart( const config_t& c, QGraphicsItem *parent = nullptr, Qt::WindowFlags window_flags = {} );
        virtual ~chart() {}
        virtual void push_back( plotting::series::xy* r ) = 0;
        virtual void update() = 0;
        void zooming( bool is_zooming ) { _zooming = is_zooming; }
        const std::vector< plotting::series::xy* >& series() const { return _series; }
        const config_t& config() const { return _config; }
        
    protected:
        std::vector< plotting::series::xy* > _series; // todo! implement and use series::base!
        config_t _config;
        bool _fixed_x; // quick and dirty
        bool _fixed_y; // quick and dirty
        bool _zooming;
        virtual void _update() {}
        
    private:
        std::string title_;
};

class xy_chart: public chart
{
    Q_OBJECT
    public:
        xy_chart( const chart::config_t& config, QGraphicsItem *parent = nullptr, Qt::WindowFlags window_flags = {} );
        void push_back( plotting::series::xy* s );        
        void update();
    
    private:
        boost::optional< std::pair< QPointF, QPointF > > _extents;
        QtCharts::QValueAxis* _x_axis;
        QtCharts::QValueAxis* _y_axis;
};

} } } // namespace snark { namespace graphics { namespace plotting {
