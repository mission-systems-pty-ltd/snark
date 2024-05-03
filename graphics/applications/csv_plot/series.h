// Copyright (c) 2021 Vsevolod Vlaskine

#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <QColor>
#include <QtCharts/QChart>
#include <QtCharts/QXYSeries>
#include <boost/optional.hpp>
#include <comma/application/command_line_options.h>
#include "record.h"

namespace snark { namespace graphics { namespace plotting { namespace series {

std::unordered_map< std::string, std::vector< std::string > > default_colourmaps();

struct config
{
    std::string chart;
    std::string color_map; // quick and dirty
    std::string color_name;
    boost::optional< QColor > color;
    std::string name;
    bool scroll; // todo! a better name! move to chart config
    std::string shape;
    std::string style; // todo
    std::string title;
    float weight;
    
    config(): scroll( false ), weight( 1 ) {}
    config( const comma::command_line_options& options );
    void set_next_colour(); // super-quick and dirty
};

class xy // todo? derive from base class? template on qt series type? time to decide: when introducing time series
{
    public:
        xy( QtCharts::QXYSeries* s = nullptr, const series::config& c = series::config() );
        const series::config& config() const { return config_; }
        QtCharts::QXYSeries* operator()() { return series_; }
        const std::pair< QPointF, QPointF >& extents() const { return extents_; }
        void clear();
        void append( boost::posix_time::ptime t, const point& p );
        static xy make( const series::config& c, QtCharts::QChart* chart );
        bool updated( bool reset = true );
    
    private:
        QtCharts::QXYSeries* series_;
        series::config config_;
        std::pair< QPointF, QPointF > extents_;
        bool updated_;
};
    
} } } } // namespace snark { namespace graphics { namespace plotting { namespace series {
