// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2026 Vsevolod Vlaskine
// All rights reserved.

/// @author vsevolod vlaskine

#include <iostream>
#include <memory>
#include <comma/application/command_line_options.h>
#include <comma/base/exception.h>
#include <comma/base/none.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include "../../visiting/eigen.h"
#include "../../visiting/traits.h"
#include "../filter/linear_kalman_filter.h"

void usage( bool verbose )
{
    std::cerr << R"(
read measurement on stdin, apply kalman filter, append state, output to stdout

currently, only linear kalman filter uniform across dimensions is plugged in

usage
    cat measurements.csv | math-kalman-filter <options> > measurements.states.csv

options
    --measurement-dimensions,--measurement-size=<n>
    --measurement-noise=<value>
    --process-noise=<value>
    --state-dimensions,--state-size=[<n>]; optional if --state-from-measurement
    --step,-dt=[<dt>]; use <dt> as a fixed step
    --step-max,--max-step=[<max_dt>]; if exceeded, reset filter
info options
    --input-fields
    --output-fields
    --output-format

examples
    basics
        csv-random make --type f,f,f --range -1,1 \
            | csv-paste line-number - \
            | csv-eval --fields i,rx,ry,rz 'x=10*cos(i*pi/180)+rx;y=10*sin(i*pi/180)+ry;z=i/50+rz' \
            | csv-shuffle --fields ,,,,x,y,z -e \
            | math-kalman-filter --measurement-size 3 \
                                 --fields measurement \
                                 --measurement-noise 1 \
                                 --process-noise 0.00001 \
                                 --state-from-measurement \
                                 --dt 1 \
            | head -n5000 \
            > filtered.csv
        view-points 'filtered.csv;fields=x,y,z;weight=3;colour=yellow' \
                    'filtered.csv;fields=,,,x,y,z;weight=3;colour=green;shape=lines' \
                    <( echo 0 )';fields=x;label=0,0,0;weight=10'
)" << std::endl;
    exit( 0 );
}

static unsigned int measurement_dimensions{0};
static unsigned int state_dimensions{0};

namespace snark { namespace math { namespace applications { namespace kalman_filtering {

template < typename T >
struct input
{
    T t{};
    Eigen::VectorXd measurement;
    
    input() : measurement( Eigen::VectorXd::Zero( measurement_dimensions ) ) {}
    input( unsigned int d ) : measurement( Eigen::VectorXd::Zero( d ) ) {}
};

struct output
{
    Eigen::VectorXd state;
    
    output() : state( Eigen::VectorXd::Zero( state_dimensions ) ) {}
    output( unsigned int d ) : state( Eigen::VectorXd::Zero( d ) ) {}
};

template < typename T > struct input_traits { static double diff( T a, T b ) { return b - a; } };
template <> struct input_traits< boost::posix_time::ptime > { static double diff( boost::posix_time::ptime a, boost::posix_time::ptime b ) { return 1e-6 * ( b - a ).total_microseconds(); } };

} } } } // namespace snark { namespace math { namespace applications { namespace kalman_filtering {

namespace comma { namespace visiting {

template < typename T > struct traits< snark::math::applications::kalman_filtering::input< T > >
{
    template < typename K, typename V > static void visit( const K&, snark::math::applications::kalman_filtering::input< T >& p, V& v )
    {
        v.apply( "t", p.t );
        static std::vector< double > m( measurement_dimensions ); // quick and dirty for now
        std::memcpy( reinterpret_cast< char* >( &m[0] ), reinterpret_cast< const char* >( &p.measurement[0] ), measurement_dimensions * sizeof( double ) );
        v.apply( "measurement", m );
        std::memcpy( reinterpret_cast< char* >( &p.measurement[0] ), reinterpret_cast< const char* >( &m[0] ), measurement_dimensions * sizeof( double ) );
    }
    template < typename K, typename V > static void visit( const K&, const snark::math::applications::kalman_filtering::input< T >& p, V& v )
    {
        v.apply( "t", p.t );
        static std::vector< double > m( measurement_dimensions ); // todo: quick and dirty for now; fix visiting Eigen::VectorXd!
        std::memcpy( reinterpret_cast< char* >( &m[0] ), reinterpret_cast< const char* >( &p.measurement[0] ), measurement_dimensions * sizeof( double ) );
        v.apply( "measurement", m );
    }
};

template <> struct traits< snark::math::applications::kalman_filtering::output >
{
    template < typename K, typename V > static void visit( const K&, const snark::math::applications::kalman_filtering::output& p, V& v )
    {
        static std::vector< double > s( state_dimensions ); // todo: quick and dirty for now; fix visiting Eigen::VectorXd!
        std::memcpy( reinterpret_cast< char* >( &s[0] ), reinterpret_cast< const char* >( &p.state[0] ), state_dimensions * sizeof( double ) );
        v.apply( "state", s );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace math { namespace applications { namespace kalman_filtering {

namespace linear {

template < typename T > static int run( const comma::command_line_options& options )
{
    using input_t = input< T >;
    measurement_dimensions = options.value< unsigned int >( "--measurement-dimensions,--measurement-size" );
    if( options.exists( "--input-fields" ) ) { std::cout << comma::join( comma::csv::names< input_t >( true, input_t( measurement_dimensions ) ), ',' ) << std::endl; return 0; }
    bool state_from_measurement = !options.exists( "--state-dimensions,--state-size" );
    state_dimensions = options.value< unsigned int >( "--state-dimensions,--state-size", measurement_dimensions * 2 );
    COMMA_ASSERT_BRIEF( state_dimensions == measurement_dimensions * 2, "currently, only state-from-measurement mapping is implemented, thus for measurement dimensions " << measurement_dimensions << " expected state dimensions" << ( measurement_dimensions * 2 ) << "; got: " << state_dimensions << " (remove --state-dimensions option to allow default behaviour)" );
    if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< output >( true, output( state_dimensions ) ), ',' ) << std::endl; return 0; }
    if( options.exists( "--output-format" ) ) { std::cout << comma::csv::format::value< output >( output( state_dimensions ) ) << std::endl; return 0; }
    comma::csv::options csv( options );
    double process_noise = options.value< double >( "--process-noise" );
    double measurement_noise = options.value< double >( "--measurement-noise" );
    auto dt = options.optional< double >( "--step,--dt" );
    COMMA_ASSERT_BRIEF( dt || csv.fields.empty() || csv.has_field( "t" ), "expected either --dt or t field; got neither" );
    COMMA_ASSERT_BRIEF( !dt || ( !csv.fields.empty() && !csv.has_field( "t" ) ), "expected either --dt or t field; got both" );
    std::string initial_state_string = options.value< std::string >( "--initial-state,--state", "" );
    auto v = initial_state_string.empty() ? std::vector< double >( state_dimensions, 0. ) : comma::split_as< double >( initial_state_string, ',' );
    Eigen::VectorXd initial_state( state_dimensions );
    for( unsigned int i = 0; i < state_dimensions; ++i ) { initial_state[i] = v[i]; }
    auto max_step = options.optional< double >( "--max-step" );
    comma::csv::input_stream< input_t > istream( std::cin, csv, input_t( measurement_dimensions ) );
    comma::csv::options output_csv;
    if( csv.binary() ) { output_csv.format( comma::csv::format::value( output( state_dimensions ) ) ); }
    comma::csv::output_stream< output > ostream( std::cout, output_csv, output( state_dimensions ) );
    std::unique_ptr< linear_kalman_filter > f;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero( measurement_dimensions, state_dimensions ); // hyper-quick and dirty for now
    for( unsigned int i = 0; i < measurement_dimensions; ++i ) { H( i, i ) = 1; }
    if( !initial_state_string.empty() || !state_from_measurement )
    {
        f = std::make_unique< linear_kalman_filter >( state_dimensions, measurement_dimensions, process_noise, measurement_noise );
        f->measurement_matrix( H );
        f->state( initial_state );
    }
    boost::optional< T > last{comma::silent_none< T >()};
    output o;
    while( istream.ready() || std::cin.good() )
    {
        const auto* p = istream.read();
        if( !p ) { break; }
        auto d = dt ? *dt : last ? input_traits< T >::diff( *last, p->t ) : 0.;
        last = p->t;
        if( ( max_step && d > *max_step ) || !f )
        {
            if( f ) { o.state = f->state(); }
            f = std::make_unique< linear_kalman_filter >( state_dimensions, measurement_dimensions, process_noise, measurement_noise );
            f->measurement_matrix( H );
            if( state_from_measurement )
            {
                o.state = f->state();
                for( unsigned int i = 0; i < measurement_dimensions; ++i ) { o.state[i] = state_from_measurement ? p->measurement[i] : 0.; }
            }
            else
            {
                o.state = initial_state;
            }
            f->state( o.state );
        }
        else
        {
            o.state = f->update( p->measurement, d );
        }
        comma::csv::append( istream, ostream, o );
        if( csv.flush ) { ostream.flush(); }
    }
    return 0;
}

} // namespace linear {

} } } } // namespace snark { namespace math { namespace applications { namespace kalman_filtering {

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--time-as-number" ) ) { return snark::math::applications::kalman_filtering::linear::run< double >( options ); } // todo: quick and dirty; a better option name; also, if binary, deduce timestamp type from format
        return snark::math::applications::kalman_filtering::linear::run< boost::posix_time::ptime >( options );
    }
    catch( std::exception& ex ) { comma::say() << ex.what() << std::endl; }
    catch( ... ) { comma::say() << "unknown exception" << std::endl; }
    return 1;
}
