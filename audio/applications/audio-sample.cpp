// Copyright (c) 2011 The University of Sydney
// Copyright (c) 2022 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <cmath>
#include <iostream>
#include <vector>
// #include <boost/date_time/posix_time/posix_time.hpp>
#include <tbb/parallel_for.h> 
#include <comma/base/none.h>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/io/select.h>
#include <comma/visiting/traits.h>
#include "../../visiting/traits.h"

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "todo" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: audio-sample <options> > sample.pcm" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: --help --verbose for more help" << std::endl;
    std::cerr << "    --amplitude=[<value>]: if amplitude field absent, use this amplitude for all the samples" << std::endl;
    std::cerr << "    --amplitude-profile=[<profile>]: amplitude profile as relative duration-amplitude pairs" << std::endl;
    std::cerr << "                                     e.g: --amplitude-profile=0.1,1;0.2,0.5;0.6,0.5 for 2-second duration means" << std::endl;
    std::cerr << "                                          attack : 0.2 second from 0 to desired amplitude" << std::endl;
    std::cerr << "                                          decay  : 0.2-0.4 seconds max amplitude to 0.5 amplitude" << std::endl;
    std::cerr << "                                          sustain: 0.4-1.2 seconds at 0.5 amplitude" << std::endl;
    std::cerr << "                                          release: 1.2-2 seconds from 0.5 amplitude to zero" << std::endl;
    std::cerr << "    --anticlick; smoothen clicks; lame, but helps a bit; todo: hanning or alike" << std::endl;
    std::cerr << "    --duration=[<seconds>]: if duration field absent, use this duration for all the samples" << std::endl;
    std::cerr << "    --flush: flush after each record (won't run in parallel, so, can be slow; todo: speed up)" << std::endl;
    std::cerr << "    --frequency=[<frequency>]: if frequency field absent, use this frequency for all the samples" << std::endl;
    std::cerr << "    --input-fields: print input fields and exit" << std::endl;
    std::cerr << "    --rate=[<value>]: samples per second" << std::endl;
    std::cerr << "    --verbose,-v: more output" << std::endl;
    std::cerr << std::endl << "csv options" << std::endl << comma::csv::options::usage( verbose ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    play from stdin" << std::endl;
    std::cerr << "        echo -e 440,15000,2,0\\\\n880,10000,2,0\\\\n1760,12000,2,0 | audio-sample -r 64000 | csv-to-bin d | csv-cast d w --force | play -t raw -r 64k -e signed -b 16 -c 1 -" << std::endl;
    std::cerr << "        seq 440 10 1050 | csv-paste - line-number | audio-sample -r 64000 --amplitude=20000 --duration=0.1 --fields=frequency,block | csv-to-bin d | csv-cast d w --force | play -t raw -r 64k -e signed -b 16 -c 1 -" << std::endl;
    std::cerr << "    play from file" << std::endl;
    std::cerr << "        echo -e 440,15000,2,0\\\\n880,10000,2,0\\\\n1760,12000,2,0 | audio-sample -r 64000 | csv-to-bin d | csv-cast d w --force > test.64000.w.raw" << std::endl;
    std::cerr << "        sox -r 64k -e signed -b 16 -c 1 test.64000.w.raw test.64000.w.wav" << std::endl;
    std::cerr << "        play test.64000.w.wav" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

struct input
{
    struct point // quick and dirty; reuse from a library
    {
        double x{0};
        double y{0};
    };

    //boost::posix_time::ptime time;
    double frequency{0};
    double amplitude{0};
    double duration{0};
    comma::uint32 block{0};
    std::vector< point > amplitude_profile; // todo: validate profile: x monotonous, x and y between 0 and 1

    unsigned int segment( double t )
    {
        double x{ t / duration };
        unsigned int i = 0;
        for( ; i < amplitude_profile.size() && x >= amplitude_profile[i].x; ++i );
        return i;
    }

    double amplitude_factor( double t, unsigned int i ) // todo! use exponential decay
    {
        const auto& p = i == 0 ? point{0, 0} : amplitude_profile[ i - 1 ]; 
        const auto& q = i < amplitude_profile.size() ? amplitude_profile[i] : point{1, 0};
        return p.y + ( q.y - p.y ) / ( q.x - p.x ) * ( t / duration - p.x ); // todo: calculate coefficients once - or tabulate
    }

    double amplitude_factor( double t ) { return amplitude_profile.empty() ? 1. : amplitude_factor( t, segment( t ) ); } // todo! watch performance; e.g. use std::map
};

namespace comma { namespace visiting {

template <> struct traits< input::point >
{
    template < typename K, typename V > static void visit( const K&, input::point& n, V& v )
    {
        v.apply( "x", n.x );
        v.apply( "y", n.y );
    }

    template < typename K, typename V > static void visit( const K&, const input::point& n, V& v )
    {
        v.apply( "x", n.x );
        v.apply( "y", n.y );
    }
};

template <> struct traits< input >
{
    template < typename K, typename V > static void visit( const K&, input& n, V& v )
    {
        //v.apply( "time", n.time );
        v.apply( "frequency", n.frequency );
        v.apply( "amplitude", n.amplitude );
        v.apply( "duration", n.duration );
        v.apply( "block", n.block );
        v.apply( "amplitude-profile", n.amplitude_profile );
    }

    template < typename K, typename V > static void visit( const K&, const input& n, V& v )
    {
        //v.apply( "time", n.time );
        v.apply( "frequency", n.frequency );
        v.apply( "amplitude", n.amplitude );
        v.apply( "duration", n.duration );
        v.apply( "block", n.block );
        v.apply( "amplitude-profile", n.amplitude_profile );
    }
};

} } // namespace comma { namespace visiting {

struct value
{
    double frequency;
    double phase;

    value( double frequency = 0, double phase = 0 ): frequency( frequency ), phase( phase - int( phase ) ) {}
};

static std::string _replace( const std::string& s, char in, char out ) // quick and dirty
{
    std::string t = s;
    for( auto& c: t ) { c = c == in ? out : c; }
    return t;
} 

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--input-fields" ) ) { std::cout << comma::join( comma::csv::names< input >( false ), ',' ) << std::endl; return 0; }
        bool verbose = options.exists( "--verbose,-v" );
        unsigned int rate = options.value< unsigned int >( "--rate,-r" );
        comma::csv::options csv( options );
        std::istringstream amplitude_profile( _replace( options.value< std::string >( "--amplitude-profile", "" ), ';', '\n' ) );
        input default_input{ options.value( "--frequency", 0.0 )
                           , options.value( "--amplitude", 0.0 )
                           , options.value( "--duration", 0.0 )
                           , 0
                           , comma::csv::read_as< std::vector< input::point > >( amplitude_profile ) };
        COMMA_ASSERT_BRIEF( !options.exists( "--attack" ), "--attack removed; use --amplitude-profile" );
        bool anticlick = options.exists( "--anticlick" );
        bool realtime = options.exists( "--realtime" );
        COMMA_ASSERT_BRIEF( !realtime, "--realtime: todo" );
        comma::io::select select;
        if( realtime ) { select.read().add( 0 ); }
        comma::csv::input_stream< input > istream( std::cin, csv, default_input );
        boost::optional< input > last = comma::silent_none< input >();
        std::vector< input > v;
        std::vector< value > previous;
        unsigned int count = 0;
        auto phase = []( const std::vector< value >& previous, double frequency ) -> double
        {
            const value* v = &previous[0];
            for( const auto& p: previous ) { if( std::abs( p.frequency - frequency ) < std::abs( v->frequency - frequency ) ) { v = &p; } }
            return v->phase;
        };
        while( std::cin.good() )
        {
            const input* p = istream.read();
            if( !p || ( !v.empty() && v.back().block != p->block ) )
            {
                std::vector< double > start( v.size(), 0 );
                std::vector< double > finish( v.size(), std::numeric_limits< double >::max() );
                if( anticlick ) { for( unsigned int i = 0; i < finish.size(); ++i ) { finish[i] = start[i] + static_cast< unsigned int >( ( v[i].duration - start[i] ) * v[i].frequency ) / v[i].frequency; } }
                double step = 1.0 / rate;
                if( previous.empty() )
                {
                    previous.resize( v.size() );
                    for( unsigned int i = 0; i < v.size(); ++i ) { previous[i].frequency = v[i].frequency; }
                }
                std::vector< double > phases( v.size() );
                for( unsigned int i = 0; i < v.size(); phases[i] = phase( previous, v[i].frequency ), ++i );
                std::uint64_t steps = rate * 60; // quick and dirty; todo? parametrise?
                if( csv.flush )
                {
                    for( double t = 0; t < v[0].duration; t += step )
                    {
                        double factor = v[0].amplitude_factor( t );
                        double a = 0;
                        for( unsigned int i = 0; i < v.size(); ++i )
                        {
                            if( t > start[i] && t < finish[i] ) { a += v[i].amplitude * factor * std::sin( M_PI * 2 * ( phases[i] + v[i].frequency * t ) ); }
                        }
                        if( csv.binary() ) { std::cout.write( reinterpret_cast< const char* >( &a ), sizeof( double ) ); } else { std::cout << a << std::endl; }
                        if( csv.flush ) { std::cout.flush(); }
                    }
                }
                else
                {
                    for( double t0 = 0; t0 < v[0].duration; t0 += step * steps )
                    {
                        unsigned int size = t0 + step * steps > v[0].duration ? ( v[0].duration - t0 ) / step : steps; 
                        std::vector< double > amplitudes( size, 0 );
                        tbb::parallel_for( tbb::blocked_range< std::size_t >( 0, size ), [&]( const tbb::blocked_range< std::size_t >& r )
                        {
                            double t = t0;
                            double factor = v[0].amplitude_factor( t );
                            for( unsigned int j = r.begin(); j < r.end(); ++j, t += step )
                            {
                                for( unsigned int i = 0; i < v.size(); ++i ) // tbb::parallel_for here does not really speed it up
                                {
                                    if( t > start[i] && t < finish[i] ) { amplitudes[j] += v[i].amplitude * factor * std::sin( M_PI * 2 * ( phases[i] + v[i].frequency * t ) ); }
                                }
                            }
                        } );
                        if( csv.binary() ) { std::cout.write( reinterpret_cast< const char* >( &amplitudes[0] ), amplitudes.size() * sizeof( double ) ); }
                        else { for( double a: amplitudes ) { std::cout << a << std::endl; } }
                        if( csv.flush ) { std::cout.flush(); }
                    }
                }
                previous.resize( v.size() );
                for( unsigned int i = 0; i < v.size(); ++i ) { previous[i] = value( v[i].frequency, phases[i] + v[i].frequency * v[0].duration ); }
                v.clear();
                if( verbose && ++count % 100 == 0 ) { std::cerr << "audio-sample: processed " << count << " blocks" << std::endl; }
            }
            if( !p ) { break; }
            if( !v.empty() && !comma::math::equal( v.back().duration, p->duration ) ) { std::cerr << "audio-sample: expected consistent duration across a block, got " << v.back().duration << " and " << p->duration << " in block " << p->block << std::endl; return 1; }
            v.push_back( *p );
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "audio-sample: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "audio-sample: unknown exception" << std::endl; }
    return 1;
}
