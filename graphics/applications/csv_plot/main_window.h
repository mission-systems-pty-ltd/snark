// Copyright (c) 2021 Vsevolod Vlaskine

#pragma once

#include <functional>
#include <map>
#include <string>
#include <QApplication>
#include <QMainWindow>
#include <QShortcut>
#include <QTimer>
#include "charts.h"
#include "chart_view.h"
#include "stream.h"

QT_USE_NAMESPACE

namespace snark { namespace graphics { namespace plotting {

class main_window: public QMainWindow
{
    Q_OBJECT
    public:
        typedef std::map< std::string, snark::graphics::plotting::chart* > charts_t;
        main_window( const QApplication& application
                   , const std::vector< snark::graphics::plotting::stream::config_t >& stream_configs
                   , std::map< std::string, snark::graphics::plotting::chart::config_t > chart_configs
                   , const std::string& layout
                   , float timeout );
        virtual ~main_window();
        void start();
        void shutdown();
        const charts_t& charts() const { return charts_; }
        const boost::ptr_vector< plotting::stream >& streams() const { return streams_; }
        const std::string& pass_through_stream_name() const { return pass_through_stream_name_; }

    public slots:
        void update();
        void print_window_geometry() const;
        void screenshot() const;
        
    private:
        const QApplication& _application;
        QTimer timer_;
        bool verbose_;
        boost::ptr_vector< plotting::stream > streams_;
        charts_t charts_;
        std::string pass_through_stream_name_;
        QShortcut _escape;
        QShortcut _print_window_geometry;
        QShortcut _screenshot;

        void closeEvent( QCloseEvent* event );
};

} } } // namespace snark { namespace graphics { namespace plotting {
