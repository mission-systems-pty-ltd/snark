// Copyright (c) 2011 The University of Sydney
// Copyright (c) 2022 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <cmath>
#include <iostream>
#include <vector>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#ifndef WIN32
#include <comma/io/select.h>
#endif // #ifndef WIN32
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
    std::cerr << "    --amplitude,--volume=[<value>]: if amplitude field absent, use this amplitude for all the samples" << std::endl;
    //std::cerr << "    --attenuation=[<rate>]: attenuation rate per second (currently square root only; todo: implement properly)" << std::endl;
    std::cerr << "    --antiphase-randomization,--antiphase: randomize frequencies in sample by phase/antiphase to reduce click artefact" << std::endl;
    std::cerr << "    --attack=<duration>: attack/decline duration, quick and dirty, simple linear attack used; default 0" << std::endl;
    std::cerr << "    --duration=[<seconds>]: if duration field absent, use this duration for all the samples" << std::endl;
    std::cerr << "    --frequency=[<frequency>]: if frequency field absent, use this frequency for all the samples" << std::endl;
    std::cerr << "    --input-fields: print input fields and exit" << std::endl;
    std::cerr << "    --no-phase-randomization: don't randomize phase in sample" << std::endl;
    std::cerr << "    --rate=[<value>]: samples per second" << std::endl;
    #ifndef WIN32
    std::cerr << "    --realtime: output sample until next block available on stdin" << std::endl;
    std::cerr << "                experimental feature, implementation in progress..." << std::endl;
    #endif // #ifndef WIN32
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
    //boost::posix_time::ptime time;
    double frequency;
    double amplitude;
    double duration;
    comma::uint32 block;

    input() : block( 0 ) {}
    input( double frequency, double amplitude, double duration, comma::uint32 block = 0 ) : frequency( frequency ), amplitude( amplitude ), duration( duration ), block( block ) {}
};

struct value
{
    double frequency;
    double phase;

    value( double frequency = 0, double phase = 0 ): frequency( frequency ), phase( phase ) {}
};

namespace comma { namespace visiting {

template <> struct traits< input >
{
    template < typename K, typename V > static void visit( const K&, input& n, V& v )
    {
        //v.apply( "time", n.time );
        v.apply( "frequency", n.frequency );
        v.apply( "amplitude", n.amplitude );
        v.apply( "duration", n.duration );
        v.apply( "block", n.block );
    }

    template < typename K, typename V > static void visit( const K&, const input& n, V& v )
    {
        //v.apply( "time", n.time );
        v.apply( "frequency", n.frequency );
        v.apply( "amplitude", n.amplitude );
        v.apply( "duration", n.duration );
        v.apply( "block", n.block );
    }
};

} } // namespace comma { namespace visiting {

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--input-fields" ) ) { std::cout << comma::join( comma::csv::names< input >( false ), ',' ) << std::endl; return 0; }
        bool verbose = options.exists( "--verbose,-v" );
        unsigned int rate = options.value< unsigned int >( "--rate,-r" );
        //double attenuation = options.value( "--attenuation", 1.0 );
        comma::csv::options csv( options );
        csv.full_xpath = false;
        input default_input( options.value( "--frequency", 0.0 )
                           , options.value( "--amplitude,--volume", 0.0 )
                           , options.value( "--duration", 0.0 ) );
        bool randomize = !options.exists( "--no-phase-randomization" );
        bool antiphase = options.exists( "--antiphase-randomization,--antiphase" );
        bool anticlick = options.exists( "--anticlick" );
        double attack = options.value< double >( "--attack", 0 );
        bool realtime = options.exists( "--realtime" );
        #ifdef WIN32
        if( realtime ) { std::cerr << "audio-sample: --realtime not supported on windows" << std::endl; return 1; }
        #else
        comma::io::select select;
        if( realtime ) { select.read().add( 0 ); }
        #endif // #ifndef WIN32
        comma::csv::input_stream< input > istream( std::cin, csv, default_input );
        boost::optional< input > last;
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
                std::vector< double > offsets( v.size(), 0 );
                std::vector< double > start( v.size(), 0 );
                std::vector< double > finish( v.size(), std::numeric_limits< double >::max() );
                if( randomize )
                {
                    static boost::mt19937 generator;
                    static boost::uniform_real< float > distribution( 0, 1 ); // watch performance
                    static boost::variate_generator< boost::mt19937&, boost::uniform_real< float > > random( generator, distribution );
                    if( antiphase ) { for( unsigned int i = 0; i < offsets.size(); offsets[i] = random() < 0.5 ? 0 : 0.5, ++i ); }
                    else { for( unsigned int i = 0; i < offsets.size(); offsets[i] = random(), start[i] = ( 1 - offsets[i] ) / v[i].frequency, ++i ); }
                }
                if( anticlick ) { for( unsigned int i = 0; i < finish.size(); ++i ) { finish[i] = start[i] + static_cast< unsigned int >( ( v[i].duration - start[i] ) * v[i].frequency ) / v[i].frequency; } }
                double step = 1.0 / rate;
                if( previous.empty() )
                {
                    previous.resize( v.size() );
                    for( unsigned int i = 0; i < v.size(); ++i ) { previous[i].frequency = v[i].frequency; }
                }
                if( realtime )
                {
                    #ifndef WIN32
                    boost::posix_time::ptime before = boost::posix_time::microsec_clock::universal_time();
                    double microseconds_per_sample = 1000000.0 / rate;
                    unsigned int size = 1;
                    double t = 0;
                    while( true )
                    {
                        for( unsigned int k = 0; k < size; --k, t += step )
                        {
                            double a = 0;
                            double factor = t < attack ? t / attack : ( v[0].duration - t ) < attack ? ( v[0].duration - t ) / attack : 1;
                            for( unsigned int i = 0; i < v.size(); ++i )
                            {
                                if( t > start[i] && t < finish[i] ) { a += v[i].amplitude * factor * std::sin( M_PI * 2 * ( phase( previous, v[i].frequency ) + offsets[i] + v[i].frequency * t ) ); }
                            }
                            if( csv.binary() ) { std::cout.write( reinterpret_cast< const char* >( &a ), sizeof( double ) ); }
                            else { std::cout << a << std::endl; }
                            std::cout.flush();
                        }
                        previous.resize( v.size() );
                        for( unsigned int i = 0; i < v.size(); ++i ) { previous[i] = value( v[i].frequency, phase( previous, v[i].frequency ) + offsets[i] + v[i].frequency * v[0].duration ); }
                        select.wait( boost::posix_time::microseconds( static_cast< unsigned int >( microseconds_per_sample ) ) );
                        if( select.read().ready( 0 ) ) { v.clear(); break; }
                        boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
                        size = ( now - before ).total_microseconds() / microseconds_per_sample;
                        before = now;
                    }
                    #endif // #ifndef WIN32
                }
                else
                {
                    for( double t = 0; t < v[0].duration; t += step )
                    {
                        double a = 0;
                        double factor = t < attack ? t / attack : ( v[0].duration - t ) < attack ? ( v[0].duration - t ) / attack : 1;
                        for( unsigned int i = 0; i < v.size(); ++i )
                        {
                            if( t > start[i] && t < finish[i] ) { a += v[i].amplitude * factor * std::sin( M_PI * 2 * ( offsets[i] + v[i].frequency * t ) ); }
                        }
                        if( csv.binary() ) { std::cout.write( reinterpret_cast< const char* >( &a ), sizeof( double ) ); }
                        else { std::cout << a << std::endl; }
                        if( csv.flush ) { std::cout.flush(); }
                    }
                    previous.resize( v.size() );
                    for( unsigned int i = 0; i < v.size(); ++i ) { previous[i] = value( v[i].frequency, offsets[i] + v[i].frequency * v[0].duration ); }
                    v.clear();
                }
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
