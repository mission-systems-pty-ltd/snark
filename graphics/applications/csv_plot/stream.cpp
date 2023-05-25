// Copyright (c) 2021 Vsevolod Vlaskine

#include <map>
#include <vector>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <QtCharts/QChart>
#include <comma/base/exception.h>
#include <comma/name_value/parser.h>
#include <comma/string/string.h>
#include "stream.h"
#include "traits.h"

namespace snark { namespace graphics { namespace plotting {

plotting::stream* stream::make( const plotting::stream::config_t& config, const std::map< std::string, snark::graphics::plotting::chart* >& charts )
{
    std::vector< plotting::series::xy > series;
    for( const auto& c: config.series ) { series.push_back( plotting::series::xy::make( c, charts.find( c.chart )->second ) ); }
    return new plotting::stream( series, config );
}

static std::string fields_from_aliases_( const std::string& fields )
{
    auto v = comma::split( fields, ',', true );
    for( auto& f: v ) { if( f == "x" || f == "y" || f == "z" ) { f = "series[0]/" + f; } }
    return comma::join( v, ',' );
}

static std::size_t estimated_number_of_records( const comma::csv::options& csv ) // todo? move to a more generic place?
{
    boost::filesystem::path f( csv.filename );
    for( ; boost::filesystem::is_symlink( f ); boost::filesystem::read_symlink( f ) );
    if( !boost::filesystem::is_regular_file( f ) ) { return 0; }
    if( csv.binary() ) { return boost::filesystem::file_size( f ) / csv.format().size(); }
    std::ifstream ifs( f.string(), std::ios::binary );
    COMMA_ASSERT_BRIEF( ifs.is_open(), "failed to open '" << f.string() << "'" );
    std::vector< char > v( 65536 );
    std::size_t n{0};
    while( ifs.good() && !ifs.eof() )
    {
        ifs.read( &v[0], v.size() );
        if( ifs.gcount() <= 0 ) { return n; }
        const char* end = &v[0] + ifs.gcount();
        for( const char* p = &v[0]; p < end; ++p ) { if( *p == '\n' ) { ++n; } }
    }
    return n;
}

stream::config_t::config_t( const comma::command_line_options& options )
    : csv( options, "series[0]/x,series[0]/y" )
    , pass_through( options.exists( "--pass-through,--pass" ) )
    , size( options.exists( "--size,-s,--tail" )
          ? options.value< std::size_t >( "--size,-s,--tail" )
          : estimated_number_of_records( csv ) )
    , number_of_series( options.value( "--number-of-series,-n", 1 ) )
{
    csv.fields = fields_from_aliases_( csv.fields );
    const auto& o = plotting::series::config( options ); // todo! fix optional reuse of x in all series!
    series.resize( number_of_series );
    for( auto& s: series ) { s = o; }
}

stream::config_t::config_t( const std::string& options, const std::map< std::string, plotting::series::config >& series_configs, const stream::config_t& defaults )
{
    std::vector< std::string > g;
    std::map< unsigned int, std::string > s;
    const auto& v = comma::split( options, ';' );
    for( const auto& o: v ) // quick and dirty
    {
        try
        {
            if( o.substr( 0, 7 ) == "series[" ) { s[ boost::lexical_cast< unsigned int >( o.substr( 7, o.find_first_of( ']' ) - 7 ) ) ] = o.substr( o.find_first_of( '=' ) + 1 ); }
            else { g.push_back( o ); }
        }
        catch( std::exception& ex )
        {
            COMMA_THROW( comma::exception, "invalid stream options '" << options << "': " << ex.what() );
        }
    }
    *this = comma::name_value::parser( "filename", ';', '=', false ).get( comma::join( g, ';' ), defaults ); // quick and dirty; todo: unhack visiting
    csv.fields = fields_from_aliases_( csv.fields );
    if( size == 0 ) { size = estimated_number_of_records( csv ); }
    if( size == 0 ) { size = 10000; }
    plotting::record sample = plotting::record::sample( csv.fields, number_of_series ); // quick and dirty
    number_of_series = sample.series.size(); // quick and dirty
    series.resize( sample.series.size() );
    auto series_defaults = series[0];
    for( unsigned int i = 0; i < series.size(); ++i )
    {
        series[i] = s.find( i ) == s.end() ? series_defaults : comma::name_value::parser( '|', ':', false ).get( s[i], series_defaults );
        auto it = series_configs.find( series[i].name );
        if( it != series_configs.end() ) { series[i] = s.find( i ) == s.end() ? it->second : comma::name_value::parser( '|', ':', false ).get( s[i], it->second ); } // uber-quick and dirty
    }
}

stream::stream( const std::vector< plotting::series::xy >& s, const config_t& config )
    : series( s )
    , config( config )
    , is_shutdown_( false )
    , is_stdin_( config.csv.filename == "-" )
    , is_( config.csv.filename, config.csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii, comma::io::mode::non_blocking )
    , istream_( *is_, config.csv, plotting::record::sample( config.csv.fields, config.number_of_series ) )
    , count_( 0 )
    , has_x_( config.csv.fields.empty() || config.csv.has_field( "x" ) || config.csv.has_field( "series[0]/x" ) || config.csv.has_field( "series[0]" ) ) // todo: quick and dirty, improve
    , buffers_( config.size )
    , size_( 0 )
{
    if( config.pass_through ) { passed_.reset( new comma::csv::passed< graphics::plotting::record >( istream_, std::cout, config.csv.flush ) ); }
}

stream::buffers_t_::buffers_t_( comma::uint32 size ) : records( size ) {}

void stream::buffers_t_::add( const record& p ) { records.add( p, p.block ); }

bool stream::buffers_t_::changed() const { return records.changed(); }

void stream::buffers_t_::mark_seen() { records.mark_seen(); }

void stream::start() { thread_.reset( new boost::thread( boost::bind( &graphics::plotting::stream::read_, boost::ref( *this ) ) ) ); } // todo: replace with std::thread

bool stream::is_shutdown() const { return is_shutdown_; }

bool stream::is_stdin() const { return is_stdin_; }

void stream::shutdown()
{
    is_shutdown_ = true;
    if( thread_ ) { thread_->join(); }
    if( !is_stdin_ ) { is_.close(); }
}

void stream::read_()
{
    while( !is_shutdown_ && ( istream_.ready() || ( is_->good() && !is_->eof() ) ) )
    {
        const record* p = istream_.read();
        if( !p ) { break; }
        if( passed_ ) { passed_->write(); }
        record q = *p;
        if( !has_x_ ) { q.series[0].x = config.block_by_size ? count_ % config.size : count_; } // todo! fix optional reuse of x in all series!
        if( config.block_by_size ) { q.block = count_ / config.size; }
        ++count_;
        comma::synchronized< records_t >::scoped_transaction( records )->push_back( q );
    }
    is_shutdown_ = true;
}

bool stream::update()
{
    records_t p;
    {
        comma::synchronized< records_t >::scoped_transaction t( records );
        p = *t; // todo! quick and dirty, potentially massive copy; watch performance!
        t->clear();
    }
    if( p.empty() ) { return false; }
    for( auto& e: p ) { buffers_.add( e ); }
    bool changed = buffers_.changed();
    static_assert( sizeof( qreal ) == 8 );
    size_ = buffers_.records.size();
    auto append = [&]( plotting::series::xy& s, unsigned int i, unsigned int j ) { s.append( buffers_.records.values()[i].t, buffers_.records.values()[i].series[j] ); }; // todo: support 3d data, time series, polar data
    if( buffers_.changed() )
    {
        for( unsigned int j = 0; j < series.size(); ++j )
        { 
            series[j].clear();
            for( unsigned int i = buffers_.records.begin(); i < buffers_.records.size(); ++i ) { append( series[j], i, j ); }
            for( unsigned int i = 0; i < buffers_.records.begin(); ++i ) { append( series[j], i, j ); }
        }
    }
    buffers_.mark_seen();
    return changed;
}

} } } // namespace snark { namespace graphics { namespace plotting {
