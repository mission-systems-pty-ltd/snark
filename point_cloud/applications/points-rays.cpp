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

/// @author james underwood
/// @author vsevolod vlaskine

#ifdef WIN32
#include <stdio.h>
#include <fcntl.h>
#include <io.h>
#endif
#include <cmath>
#include <string.h>
#include <fstream>
#include <map>
#include <vector>
#include <boost/bind/bind.hpp>
#include <boost/optional.hpp>
#include <tbb/parallel_for.h>
#include <comma/application/command_line_options.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/math/compare.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include "../../math/range_bearing_elevation.h"
#include "../../math/spherical_geometry/great_circle.h"
#include "../../point_cloud/voxel_map.h"
#include "../../visiting/traits.h"
//#include <google/profiler.h>

void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "ray-tracing operations" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat points.csv | points-rays <operation> [<options>] > points.with-distance.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: help; --help --verbose: more help" << std::endl;
    std::cerr << "    --angle-threshold,-a=<value>: angular radius in radians" << std::endl;
    std::cerr << "    --range-threshold,-r=<value>: default=0; range threshold in meters" << std::endl;
    std::cerr << "    --input-fields; print input fields for an operation and exit" << std::endl;
    std::cerr << "    --output-fields; print output fields for an operation and exit" << std::endl;
    std::cerr << "    --output-format; print output format for an operation and exit" << std::endl;
    std::cerr << "    --threads=<number>; default=8; number of threads for parallel processing" << std::endl;
    std::cerr << "    --verbose,-v: more debug output" << std::endl;
    std::cerr << std::endl;
    std::cerr << "csv options" << std::endl;
    if( verbose ) { std::cerr << std::endl << comma::csv::options::usage() << std::endl; } else { std::cerr << "    run --help --verbose for more..." << std::endl; }
    std::cerr << std::endl;
    std::cerr << "operations" << std::endl;
    std::cerr << "    trace: for each point on stdin output the point on the ray with the shortest range" << std::endl;
    std::cerr << "        fields: r,b,e[,block]: range, bearing, elevation; default: r,b,e" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

typedef snark::range_bearing_elevation point_t;

namespace snark { namespace points_rays { namespace trace {

struct input_t
{
    typedef std::pair< snark::points_rays::trace::input_t, std::vector< char > > pair_t;
    
    point_t point;
    comma::uint32 block;
    const pair_t* traced;
    
    input_t() : block( 0 ), traced( NULL ) {}
};

} } } // namespace snark { namespace points_rays { namespace trace {

namespace comma { namespace visiting {

template <> struct traits< snark::points_rays::trace::input_t >
{
    template< typename K, typename V > static void visit( const K&, const snark::points_rays::trace::input_t& t, V& v )
    {
        v.apply( "point", t.point );
        v.apply( "block", t.block );
    }
    
    template< typename K, typename V > static void visit( const K&, snark::points_rays::trace::input_t& t, V& v )
    {
        v.apply( "point", t.point );
        v.apply( "block", t.block );
    }
};
    
} } // namespace comma { namespace visiting {

namespace snark { namespace points_rays { namespace trace {

static double distance( const point_t& p, const point_t& q ) // todo: quick and dirty, watch performance
{
    return snark::spherical::great_circle::arc( snark::spherical::coordinates( p.bearing_elevation() ), snark::spherical::coordinates( q.bearing_elevation() ) ).angle();
}

typedef std::pair< snark::points_rays::trace::input_t, std::vector< char > > pair_t;

// struct cell
// {
//     std::vector< pair_t* > points; // quick and dirty
//     
//     const pair_t* trace( const point_t& p, double threshold ) const
//     {
//         const pair_t* min = NULL;
//         for( std::size_t i = 0; i < points.size(); ++i )
//         {
//             if( distance( p, points[i]->first.point ) > threshold ) { continue; }
//             if( !min || points[i]->first.point.range() < min->first.point.range() ) { min = points[i]; }
//         }
//         return min;
//     }
// };

struct cell
{
    typedef std::map< double, pair_t* > map_t;
    map_t points; // quick and dirty
    
    const pair_t* trace( const point_t& p, double threshold ) const
    {
        for( map_t::const_iterator it = points.begin(); it != points.end(); ++it )
        {
            if( distance( p, it->second->first.point ) < threshold && it->second->first.point.range() < p.range() ) { return it->second; }
        }
        return nullptr;
    }
};

typedef snark::voxel_map< cell, 2 > grid_t;

static void trace_points( const tbb::blocked_range< std::size_t >& range, std::deque< pair_t >& records, const grid_t& grid, double threshold )
{
    for( std::size_t i = range.begin(); i < range.end(); ++i )
    {
        grid_t::const_iterator it = grid.at( grid_t::point_type( records[i].first.point.bearing(), records[i].first.point.elevation() ) );
        if( it == grid.end() )
        { 
            records[i].first.traced = &records[i];
        }
        else
        {
            records[i].first.traced = it->second.trace( records[i].first.point, threshold );
            if( !records[i].first.traced ) { records[i].first.traced = &records[i]; }
        }
    }
}

} } } // namespace snark { namespace points_rays { namespace trace {

// namespace snark { namespace points_rays { namespace trace_fast {
// 
// const pair_t* traced;
//     
// typedef snark::voxel_map< cell, 3 > grid_t;
// 
// } } } // namespace snark { namespace points_rays { namespace trace_fast {

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        bool verbose = options.exists( "--verbose,-v" );
        std::vector< std::string > unnamed = options.unnamed( "--verbose,-v", "-.*" );
        if( unnamed.empty() ) { std::cerr << "points-rays: please specify operation" << std::endl; return 1; }
        const std::string& operation = unnamed[0];
        if( operation == "trace" )
        {
            if( options.exists( "--input-fields" ) ) { std::cerr << comma::join( comma::csv::names< snark::points_rays::trace::input_t >( false ), ',' ) << std::endl; return 0; }
            if( options.exists( "--output-fields" ) ) { std::cerr << comma::join( comma::csv::names< snark::points_rays::trace::input_t >( false ), ',' ) << std::endl; return 0; }
            comma::csv::options csv( options, "range,bearing,elevation" );
            csv.full_xpath = false;
            std::vector< std::string > v = comma::split( csv.fields, ',' );
            for( unsigned int i = 0; i < v.size(); ++i )
            {
                if( v[i] == "r" ) { v[i] = "range"; }
                else if( v[i] == "b" ) { v[i] = "bearing"; }
                else if( v[i] == "e" ) { v[i] = "elevation"; }
            }
            csv.fields = comma::join( v, ',' );
            csv.full_xpath = false;
            double threshold = options.value< double >( "--angle-threshold,-a" );
            comma::csv::input_stream< snark::points_rays::trace::input_t > istream( std::cin, csv );
            snark::voxel_map< int, 2 >::point_type resolution = snark::points_rays::trace::grid_t::point_type( threshold, threshold );
            boost::optional< snark::points_rays::trace::pair_t > last;
            while( istream.ready() || std::cin.good() )
            {
                snark::points_rays::trace::grid_t grid( resolution );
                if( verbose ) { std::cerr << "points-rays: block" << ( last ? boost::lexical_cast< std::string >( last->first.block ) : std::string() ) << ": loading..." << std::endl; }
                std::deque< snark::points_rays::trace::pair_t > records;
                if( last ) { records.push_back( *last ); }
                while( istream.ready() || std::cin.good() )
                {
                    const snark::points_rays::trace::input_t* p = istream.read();
                    if( !p ) { break; }
                    if( !last ) { last = snark::points_rays::trace::pair_t(); }
                    last->first = *p;
                    if( csv.binary() ) // quick and dirty
                    {
                        static unsigned int s = istream.binary().binary().format().size();
                        last->second.resize( s );
                        ::memcpy( &last->second[0], istream.binary().last(), s );
                    }
                    else
                    {
                        std::string s = comma::join( istream.ascii().last(), csv.delimiter );
                        last->second.resize( s.size() );
                        ::memcpy( &last->second[0], &s[0], s.size() );
                    }
                    if( !records.empty() && records.back().first.block != p->block ) { break; }
                    records.push_back( *last ); // todo: quick and dirty; watch performance
                    for( int i = -1; i < 2; ++i )
                    {
                        for( int j = -1; j < 2; ++j )
                        {
                            double bearing = records.back().first.point.bearing() + threshold * i;
                            if( bearing < -M_PI ) { bearing += ( M_PI * 2 ); }
                            else if( bearing >= M_PI ) { bearing -= ( M_PI * 2 ); }
                            double elevation = records.back().first.point.elevation() + threshold * j;
                            snark::points_rays::trace::grid_t::iterator it = grid.touch_at( snark::points_rays::trace::grid_t::point_type( bearing, elevation ) );
                            it->second.points.insert( std::make_pair( records.back().first.point.range(), &records.back() ) ); //it->second.points.push_back( &records.back() );
                        }
                    }
                }
                if( records.empty() ) { break; }
                if( verbose ) { std::cerr << "points-rays: block " << records[0].first.block << ": loaded " << records.size() << " points in a grid of size " << grid.size() << " voxels" << std::endl; }
                if( verbose ) { std::cerr << "points-rays: block " << records[0].first.block << ": tracing..." << std::endl; }
                unsigned int threads = options.value( "--threads", 8 );
                tbb::parallel_for( tbb::blocked_range< std::size_t >( 0, records.size(), records.size() / threads ), boost::bind( &snark::points_rays::trace::trace_points, boost::placeholders::_1, boost::ref( records ), boost::cref( grid ), threshold ) );
                if( verbose ) { std::cerr << "points-rays: block " << records[0].first.block << ": outputting..." << std::endl; }
                for( std::size_t i = 0; i < records.size(); ++i )
                {
                    std::cout.write( &records[i].second[0], records[i].second.size() );
                    if( !csv.binary() ) { std::cout << csv.delimiter; }
                    std::cout.write( &records[i].first.traced->second[0], records[i].first.traced->second.size() );
                    if( !csv.binary() ) { std::cout << std::endl; }
                }
                if( csv.flush ) { std::cout.flush(); }
                if( verbose ) { std::cerr << "points-rays: block " << records[0].first.block << ": done" << std::endl; }
            }
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "points-rays: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-rays: unknown exception" << std::endl; }
    return 1;
}
