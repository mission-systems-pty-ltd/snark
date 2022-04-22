// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


/// @author vsevolod vlaskine

#ifdef WIN32
#include <stdio.h>
#include <fcntl.h>
#include <io.h>
#endif
#include <deque>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <vector>
#include <tbb/version.h>
#if TBB_VERSION_MAJOR < 2021
#include <tbb/pipeline.h>
#else
#include <tbb/parallel_pipeline.h>
#endif
#include <boost/array.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <comma/sync/synchronized.h>
#include <comma/visiting/traits.h>
#include "../../math/interval.h"
#include "../../tbb/bursty_reader.h"
#include "../../tbb/types.h"

#ifdef PROFILE
#include <google/profiler.h>
#endif

enum{ background_t, foreground_t, forebackground_t, unknown_t};
enum{ no_transition, rising, falling };

static void usage()
{
    std::cerr << std::endl;
    std::cerr << "partition 2d scan into foreground-background" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat points.csv | points-foreground-partitions [<options>] > partitions.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "output: each point is tagged with a parition identity and a foreground flag each of type ui"<<std::endl;
    std::cerr << "        if --output-all is not specified only foreground points are output"<<std::endl;
    std::cerr << std::endl;
    std::cerr << "foreground flag:"<<std::endl;
    std::cerr << "        "<<background_t<<": background"<<std::endl;
    std::cerr << "        "<<foreground_t<<": foreground"<<std::endl;
    std::cerr << "        "<<forebackground_t<<": foreground or background"<<std::endl;
    std::cerr << "        "<<unknown_t<<": scan ends"<<std::endl;
    std::cerr << "<options>" << std::endl;
    std::cerr << "    partitioning options:" << std::endl;
    std::cerr << "        --foreground-threshold: distance threshold for a partition to be classified as foreground; default: 1"<<std::endl;
    std::cerr << "        --min-points-per-partition <n>: min number of points in a partition; default: 1" << std::endl;
    std::cerr << "    data flow options:" << std::endl;
    std::cerr << "        --discard,-d: if present, partition as many points as possible, discard the rest" << std::endl;
    std::cerr << "        --output-all: output all points, even non-foreground ones" << std::endl;
    std::cerr << "        --verbose, -v: output progress info" << std::endl;
    std::cerr << std::endl;
    std::cerr << "<fields>" << std::endl;
    std::cerr << "    required fields: r,b,e" << std::endl;
    std::cerr << "    default: \"r,b,e\"" << std::endl;
    std::cerr << "    block: data block id, if present, accumulate and partition each data block separately" << std::endl;
    std::cerr << "           if absent, read until the end of file/stream and then partition" << std::endl;
    std::cerr << comma::csv::options::usage() << std::endl;
    std::cerr << std::endl;
    exit( -1 );
}

static bool verbose;
static std::size_t min_points_per_partition = 1;
static double foreground_threshold = 1; // 1 metre
static comma::csv::options csv;
static bool discard;
static bool output_all;


struct input_t
{
    Eigen::Vector3d point;
    comma::uint32 block;
    comma::uint32 id;
    comma::uint32 foreground;

    input_t() : point( 0, 0, 0 ), id( 0 ), foreground( 0 ) {}
};

namespace comma { namespace visiting {

template <> struct traits< Eigen::Vector3d >
{
    template < typename K, typename V > static void visit( const K&, Eigen::Vector3d& p, V& v )
    {
        v.apply( "r", p(0) );
        v.apply( "b", p(1) );
        v.apply( "e", p(2) );
    }

    template < typename K, typename V > static void visit( const K&, const Eigen::Vector3d& p, V& v )
    {
        v.apply( "r", p(0) );
        v.apply( "b", p(1) );
        v.apply( "e", p(2) );
    }
};

template <> struct traits< input_t >
{
    template < typename K, typename V > static void visit( const K&, input_t& p, V& v )
    {
        v.apply( "point", p.point );
        v.apply( "block", p.block );
    }

    template < typename K, typename V > static void visit( const K&, const input_t& p, V& v )
    {
        v.apply( "point", p.point );
        v.apply( "block", p.block );
    }
};

} } // namespace ark { namespace visiting {

struct block_t // quick and dirty, no optimization for now
{
    typedef std::pair< input_t, std::string > pair_t;
    typedef std::deque< pair_t > pairs_t;

    boost::scoped_ptr< pairs_t > points;
    comma::uint32 id;
    volatile bool empty;

    block_t() : id( 0 ), empty( true ) {}
    void clear() { points.reset(); empty = true; }
};

static comma::signal_flag is_shutdown;
static boost::scoped_ptr< snark::tbb::bursty_reader< block_t* > > bursty_reader;

static block_t* read_block_impl_( ::tbb::flow_control* flow = nullptr )
{
    static boost::array< block_t, 3 > blocks;
    static boost::optional< block_t::pair_t > last;
    static comma::uint32 block_id = 0;
    block_t::pairs_t* points = new block_t::pairs_t;
    while( true ) // quick and dirty, only if --discard
    {
        static comma::csv::input_stream< input_t > istream( std::cin, csv );
        points->clear();
        while( true )
        {
            if( last )
            {
                block_id = last->first.block;
                points->push_back( *last );
                last.reset();
            }
            if( is_shutdown || std::cout.bad() || std::cin.bad() || std::cin.eof() )
            {
                if( bursty_reader ) { bursty_reader->stop(); } // quick and dirty, it sucks...
                if( flow ) { flow->stop(); }
                break;
            }
            const input_t* p = istream.read();
            if( !p ) { break; }
            std::string line;
            if( csv.binary() )
            {
                line.resize( csv.format().size() );
                ::memcpy( &line[0], istream.binary().last(), csv.format().size() );
            }
            else
            {
                line = comma::join( istream.ascii().last(), csv.delimiter );
            }
            last = std::make_pair( *p, line );
            if( p->block != block_id ) { break; }
        }
        for( unsigned int i = 0; i < blocks.size(); ++i )
        {
            if( !blocks[i].empty ) { continue; }
            blocks[i].clear();
            blocks[i].id = block_id;
            blocks[i].points.reset( points );
            blocks[i].empty = false;
            return &blocks[i];
        }
    }
}

static block_t* read_block_( ::tbb::flow_control& flow ) { return read_block_impl_( &flow ); }
static block_t* read_block_bursty_() { return read_block_impl_(); }

static void write_block_( block_t* block )
{
    if( !block ) { return; } // quick and dirty for now, only if --discard
    for( std::size_t i = 0; i < block->points->size(); ++i )
    {
        const block_t::pair_t& p = block->points->operator[]( i );
        if( ( p.first.foreground != foreground_t ) && !output_all ) { continue; }
        comma::uint32 id = p.first.id;
        comma::uint32 foreground = p.first.foreground;
        std::cout.write( &p.second[0], p.second.size() );
        if( csv.binary() )
        {
            std::cout.write( reinterpret_cast< const char* >( &id ), sizeof( comma::uint32 ) );
            std::cout.write( reinterpret_cast< const char* >( &foreground ), sizeof( comma::uint32 ) );
        }
        else
        {
            std::cout << csv.delimiter << id << csv.delimiter << foreground << std::endl;
        }
    }
    std::cout.flush();
    block->clear();
}

static block_t* partition_( block_t* block )
{

    if( !block ) { return nullptr; } // quick and dirty for now, only if --discard
    if( block->points->empty() ) { return block; }
    snark::math::closed_interval< double, 3 > extents;
    for( std::size_t i = 0; i < block->points->size(); ++i ) { extents.set_hull( block->points->operator[](i).first.point ); }

    //foreground partition here
    block->points->at(0).first.foreground = no_transition;
    std::size_t last_transition = 0;
    comma::uint32 id = 0;
    comma::uint32 foreground;

    for( std::size_t i = 1; i < block->points->size(); i++ )
    {
        if( block->points->at(i).first.point(0) - block->points->at(i-1).first.point(0) > foreground_threshold )
        {
            block->points->at(i).first.foreground = rising;
        }
        else if( block->points->at(i).first.point(0) - block->points->at(i-1).first.point(0) < -foreground_threshold )
        {
            block->points->at(i).first.foreground = falling;
        }
        else
        {
            block->points->at(i).first.foreground = no_transition;
        }

        comma::uint32 foreground_current = block->points->at(i).first.foreground;
        comma::uint32 foreground_last = block->points->at(last_transition).first.foreground;

        if( ( foreground_current == falling || foreground_current == rising ) && ( foreground_last == no_transition ) )
        {
            foreground = unknown_t;
        }
        else if( ( foreground_current == rising && foreground_last == rising ) || ( foreground_current == falling && foreground_last == falling ) )
        {
            foreground = forebackground_t;
        }
        else if( foreground_current == falling && foreground_last == rising )
        {
            foreground = background_t;
        }
        else if( foreground_current == rising && foreground_last == falling )
        {
            if( i - last_transition >= min_points_per_partition )
            {
                foreground = foreground_t;
            }
            else
            {
                foreground = forebackground_t;
            }
        }
        else if( i == ( block->points->size() - 1 ) )
        {
            // is this the last point?
            block->points->at(i).first.foreground = unknown_t;
            if( foreground_current != no_transition )
            {
                block->points->at(i).first.id = id+1;
            }
            else
            {
                block->points->at(i).first.id = id;
            }
            foreground = unknown_t;
        }
        else
        {
            // do nothing
            continue;
        }

        for( std::size_t j = last_transition; j < i; j++ )
        {
            block->points->at(j).first.foreground = foreground;
            block->points->at(j).first.id = id;
        }
        id++;
        last_transition = i;
    }

    return block;
}

int main( int ac, char** av )
{
    try
    {
        #ifdef WIN32
        _setmode( _fileno( stdout ), _O_BINARY );
        #endif
        comma::command_line_options options( ac, av );
        if( options.exists( "--help,-h" ) ) { usage(); }
        csv = comma::csv::options( options, "r,b,e" );
        csv.full_xpath = false;
        foreground_threshold = options.value( "--foreground-threshold", 1.0 );
        min_points_per_partition = options.value( "--min-points-per-partition", 1u );
        verbose = options.exists( "--verbose,-v" );
        discard = options.exists( "--discard,-d" );
        output_all = options.exists( "--output-all" );
        snark::tbb::filter< block_t*, block_t* >::type partition_filter( snark::tbb::filter_mode::serial_in_order, &partition_ );
        snark::tbb::filter< block_t*, void >::type write_filter( snark::tbb::filter_mode::serial_in_order, &write_block_ );
        #ifdef PROFILE
        ProfilerStart( "points-foreground-partitions.prof" ); {
        #endif
        if( discard )
        {
            bursty_reader.reset( new snark::tbb::bursty_reader< block_t* >( &read_block_bursty_ ) );
            snark::tbb::filter< void, void >::type filters = bursty_reader->filter() & partition_filter & write_filter;
            ::tbb::parallel_pipeline( 3, filters ); // while( bursty_reader->wait() ) { ::tbb::parallel_pipeline( 3, filters ); }
            bursty_reader->join();
        }
        else
        {
            snark::tbb::filter< void, block_t* >::type read_filter( snark::tbb::filter_mode::serial_in_order, &read_block_ );
            snark::tbb::filter< void, void >::type filters = read_filter & partition_filter & write_filter;
            ::tbb::parallel_pipeline( 3, filters );
        }
        #ifdef PROFILE
        ProfilerStop(); }
        #endif
        if( is_shutdown ) { std::cerr << "points-foreground-partitions: caught signal" << std::endl; }
        else { std::cerr << "points-foreground-partitions: end of stream" << std::endl; }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "points-foreground-partitions: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-foreground-partitions: unknown exception" << std::endl; }
    return 1;
}
