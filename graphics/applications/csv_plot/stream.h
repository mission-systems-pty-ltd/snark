// Copyright (c) 2021 Vsevolod Vlaskine

#pragma once

#include <deque>
#include <map>
#include <vector>
#include <boost/optional.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <QColor>
#include <QPointF>
#include <QtCharts/QChart>
#include <QtCharts/QXYSeries>
#include <comma/application/command_line_options.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/io/stream.h>
#include <comma/sync/synchronized.h>
#include "../../../graphics/block_buffer.h"
#include "charts.h"
#include "record.h"
#include "series.h"

QT_USE_NAMESPACE
QT_CHARTS_USE_NAMESPACE

namespace snark { namespace graphics { namespace plotting {

class stream // todo: if stream other than xy stream required, create stream baseclass and derive from it xy stream, xyz stream, etc
{
    public:
        struct config_t
        {
            comma::csv::options csv;
            bool pass_through;
            std::vector< plotting::series::config > series;
            comma::uint32 size;
            comma::uint32 number_of_series; // todo: a shorter name?
            bool block_by_size{false};
            config_t() : pass_through( false ), series( 1 ), size( 10000 ), number_of_series( 1 ), block_by_size(false) {}
            config_t( const comma::command_line_options& options );
            config_t( const std::string& options, const std::map< std::string, plotting::series::config >& series_configs, const config_t& defaults = config_t() );
        };
        
        std::vector< plotting::series::xy > series;
        const config_t config;
        typedef std::deque< graphics::plotting::record > records_t;
        comma::synchronized< records_t > records; // quick and dirty

        stream( const std::vector< plotting::series::xy >& s, const config_t& config );
        void start();
        bool is_shutdown() const;
        bool is_stdin() const;
        void shutdown();
        bool update();
        unsigned int size() const { return size_; }
        static plotting::stream* make( const plotting::stream::config_t& config, const std::map< std::string, snark::graphics::plotting::chart* >& charts );

    protected:
        bool is_shutdown_;
        bool is_stdin_;
        comma::io::istream is_;
        comma::csv::input_stream< graphics::plotting::record > istream_;
        boost::scoped_ptr< comma::csv::passed< graphics::plotting::record > > passed_; // boost::optional< comma::csv::passed< graphics::plotting::record > > passed_; // todo: move semantics
        boost::scoped_ptr< boost::thread > thread_;
        comma::uint64 count_;
        bool has_x_;
        void read_();
        struct buffers_t_
        {
            block_buffer< plotting::record > records;
            buffers_t_( comma::uint32 size );
            void add( const record& p );
            bool changed() const;
            void mark_seen();
        };
        buffers_t_ buffers_;
        unsigned int size_;
};
    
} } } // namespace snark { namespace graphics { namespace plotting {
