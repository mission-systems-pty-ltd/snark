#include <map>
#include <functional>
#include <QGridLayout>
#include <QTabWidget>
#include <comma/csv/options.h>
#include <comma/csv/traits.h>
#include <comma/name_value/map.h>
#include <comma/string/string.h>
#include "main_window.h"

namespace snark { namespace graphics { namespace plotting {

static QWidget* make_widget_( const std::string& l, main_window::charts_t& charts )
{
    comma::name_value::map m( l, "shape" );
    std::string shape = m.value< std::string >( "shape" );
    if( shape == "grid" )
    {
        auto cols = m.optional< unsigned int >( "cols" );
        auto rows = m.optional< unsigned int >( "rows" );
        if( !cols && !rows ) { cols = 1; }
        if( !cols ) { cols = charts.size() / *rows + 1; }
        if( !rows ) { rows = charts.size() / *cols + 1; }
        if( *cols * *rows < charts.size() ) { COMMA_THROW( comma::exception, "csv-plot: expected grid of size at least " << charts.size() << "; got: grid " << *cols << "x" << *rows ); }
        QGridLayout* grid = new QGridLayout;
        grid->setContentsMargins( 0, 0, 0, 0 );
        unsigned int row = 0;
        unsigned int col = 0;
        for( auto c: charts )
        {
            auto v = new snark::graphics::plotting::chart_view( c.second );
            v->setRenderHint( QPainter::Antialiasing );
            v->setContentsMargins( 0, 0, 0, 0 );
            grid->addWidget( v, row, col, 1, 1 );
            if( ++col == cols ) { col = 0; ++row; }
        }
        auto w = new QWidget;
        w->setLayout( grid );
        w->setContentsMargins( 0, 0, 0, 0 );
        return w;
    }
    else if( shape == "tabs" )
    {
        auto w = new QTabWidget;
        w->setContentsMargins( 0, 0, 0, 0 );
        for( auto c: charts )
        {
            auto* v = new snark::graphics::plotting::chart_view( c.second );
            v->setRenderHint( QPainter::Antialiasing );
            v->setContentsMargins( 0, 0, 0, 0 );
            w->addTab( v, &c.second->config().title[0] );
        }
        return w;
    }
    else if( shape == "stacked" ) { COMMA_THROW( comma::exception, "csv-plot: --layout=stacked: todo" ); }
    else if( shape == "windows" ) { COMMA_THROW( comma::exception, "csv-plot: --layout=windows: todo" ); }
    COMMA_THROW( comma::exception, "csv-plot: expected layout; got: '" << shape << "'" );
}

main_window::main_window( const std::vector< snark::graphics::plotting::stream::config_t >& stream_configs
                        , std::map< std::string, snark::graphics::plotting::chart::config_t > chart_configs
                        , const std::string& layout
                        , float timeout )
    : _escape( QKeySequence( Qt::Key_Escape ), this, SLOT( close() ) )
    , _print_window_geometry( QKeySequence( tr( "Ctrl+G" ) ), this, SLOT( print_window_geometry() ) )
{
    for( const auto& c: stream_configs )
    {
        for( const auto& s: c.series ) // quick and dirty
        {
            auto i = chart_configs.insert( std::make_pair( s.chart, plotting::chart::config_t( s.chart ) ) );
            if( i.first->second.title.empty() ) { i.first->second.title = s.title; } // todo: quick and dirty; move to a chart method
            if( s.scroll ) { i.first->second.scroll = true; } // todo: quick and dirty; move to a chart method
        }
    }
    for( const auto& c: chart_configs ) { charts_[ c.first ] = new plotting::xy_chart( plotting::chart::config_t( c.second ) ); }
    for( const auto& c: stream_configs ) // todo: multiple series from a stream could go to different charts
    { 
        auto s = plotting::stream::make( c, charts_ );
        if( s->config.pass_through )
        {
            if( !pass_through_stream_name_.empty() ) { COMMA_THROW( comma::exception, "csv-plot: expected pass-through only for one stream; got at least two: '" << pass_through_stream_name_ << "' and then '" << s->config.csv.filename << "'" ); }
            pass_through_stream_name_ = s->config.csv.filename;
        }
        streams_.push_back( s );
        for( auto& t: s->series ) { charts_[ t.config().chart ]->push_back( &t ); } // quick and dirty; todo? move to stream::make()? (then change series vector in stream to ptr_vector)
    }
    setCentralWidget( make_widget_( layout, charts_ ) );
    //addAction( new snark::graphics::qaction( "Pring Window Geometry", std::bind( &MainWindow::_print_window_geometry, this ), "Ctrl+G" ) );
    // addAction( new snark::graphics::qaction( "print window geometry"
    //                                        , [&]()
    //                                          {
    //                                             auto g = this->geometry();
    //                                             std::cerr << g.left() << "," << g.top() << "," << g.width() << "," << g.height() << std::endl;
    //                                          }
    //                                        , "Ctrl+G" ) );
    QObject::connect( &timer_, &QTimer::timeout, this, &main_window::update );
    timer_.setInterval( ( unsigned int )( timeout * 1000 ) );
}

main_window::~main_window()
{
    shutdown();
}

void main_window::closeEvent( QCloseEvent* event )
{
    timer_.stop(); // todo: call shutdown instead
    // todo: stop threads!
}

void main_window::print_window_geometry() const
{
    auto g = geometry();
    std::cerr << g.left() << "," << g.top() << "," << g.width() << "," << g.height() << std::endl;
}

void main_window::start()
{
    for( unsigned int i = 0; i < streams_.size(); ++i ) { streams_[i].start(); }
    timer_.start();
}

void main_window::shutdown()
{
    timer_.stop();
    // todo! stop threads properly (they block on read() if no more input)
    for( unsigned int i = 0; i < streams_.size(); ++i ) { streams_[i].shutdown(); }
}

void main_window::update()
{
    bool all_shutdown = true;
    for( auto& s: streams_ )
    {
        s.update();
        if( !s.is_shutdown() ) { all_shutdown = false; }
    }
    for( auto& c: charts_ ) { c.second->update(); }
    if( all_shutdown ) { timer_.stop(); }
}

} } } // namespace snark { namespace graphics { namespace plotting {
