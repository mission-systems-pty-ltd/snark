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
#include <boost/array.hpp>
#include <boost/optional.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/math/compare.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include "../../math/range_bearing_elevation.h"
#include "../../point_cloud/voxel_map.h"
#include "../../visiting/traits.h"
//#include <google/profiler.h>

void usage( bool long_help = false )
{
    std::cerr << std::endl;
    std::cerr << "a quick-n-dirty application; the interface is way too rigid" << std::endl;
    std::cerr << "load a point cloud in polar from file; for each point on stdin output it, if it is blocked by a point in the reference point cloud, appending the blocking point" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat points.csv | points-detect-change reference_points.csv [<options>] > points.marked-as-blocked.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --angle-threshold,-a=<value>: angular radius in radians" << std::endl;
    std::cerr << "    --help: print this help; more help: --help --verbose" << std::endl;
    std::cerr << "    --range-threshold,-r=<value>: if present, output only the points" << std::endl;
    std::cerr << "                                  that have reference points nearer than range + range-threshold" << std::endl;
    std::cerr << "    --verbose,-v: more debug output" << std::endl;
    std::cerr << std::endl;
    std::cerr << "fields: r,b,e: range, bearing, elevation; default: r,b,e" << std::endl;
    if( long_help )
    {
        std::cerr << std::endl;
        std::cerr << comma::csv::options::usage() << std::endl;
    }
    std::cerr << std::endl;
    exit( 0 );
}

typedef snark::range_bearing_elevation point_t;

//std::ostream& operator<<( std::ostream& os, const point_t& p ) { os << p.range() << "," << p.bearing() << "," << p.elevation(); return os; }

static double abs_bearing_distance_( double m, double b ) // quick and dirty
{
    double m1 = m < 0 ? m + ( M_PI * 2 ) : m;
    double b1 = b < 0 ? b + ( M_PI * 2 ) : b;
    return std::abs( m1 - b1 );
}

static bool bearing_between_( double b, double min, double max )
{
    return comma::math::equal( abs_bearing_distance_( b, min ) + abs_bearing_distance_( b, max ), abs_bearing_distance_( min, max ) ); // quick and dirty: watch performance
}

static double bearing_min_( double m, double b )
{
    double m1 = m < 0 ? m + ( M_PI * 2 ) : m;
    double b1 = b < 0 ? b + ( M_PI * 2 ) : b;
    return comma::math::less( m1, b1 ) ? m : b;
}

static double bearing_max_( double m, double b )
{
    double m1 = m < 0 ? m + ( M_PI * 2 ) : m;
    double b1 = b < 0 ? b + ( M_PI * 2 ) : b;
    return comma::math::less( b1, m1 ) ? m : b;
}

static bool verbose;
snark::voxel_map< int, 2 >::point_type resolution;

struct cell
{
    struct entry
    {
        point_t point;
        comma::uint64 index;
        entry() {}
        entry( const point_t& point, comma::uint64 index ) : point( point ), index( index ) {}
    };

    std::vector< entry > points;

//     typedef snark::voxel_map< cell, 2 > grid_t;
//
//     enum { factor = 4 };
//
//     boost::array< boost::array< std::vector< entry >, 3 * factor >, 3 * factor > grid;
//
//     static grid_t::Index index( double b, double e )
//     {
//         grid_t::point_t p( b, e );
//         static grid_t::point_t finer_resolution = resolution / factor; // quick and dirty
//         const grid_t::Index& ri = grid_t::index_of( p, resolution );
//         const grid_t::Index& fi = grid_t::index_of( p, finer_resolution );
//         grid_t::Index i;
//         i[0] = fi[0] - ri[0] * factor;
//         i[1] = fi[1] - ri[1] * factor;
//         return i;
//     }
//
//     void add_to_grid( const entry& p )
//     {
//         const grid_t::Index& i = index( p.point.bearing, p.point.elevation );
//         std::vector< entry >& v = grid[i[0]][i[1]];
//         if( v.size() == v.capacity() ) { v.reserve( 512 ); } // quick and dirty
//         v.push_back( p );
//     }
//
//     const entry* trace_in_grid( const point_t& p, double threshold, boost::optional< double > range_threshold ) const
//     {
//         static const double threshold_square = threshold * threshold; // static: quick and dirty
//         const entry* entry = NULL;
//         boost::optional< point_t > min; // todo: quick and dirty, fix point_tRBE and use extents
//         boost::optional< point_t > max; // todo: quick and dirty, fix point_tRBE and use extents
//         const grid_t::Index& begin = index( p.bearing - threshold, p.elevation - threshold );
//         const grid_t::Index& end = index( p.bearing + threshold, p.elevation + threshold );
//         for( unsigned int b = begin[0]; b < ( unsigned int )( end[0] ); ++b )
//         {
//             for( unsigned int e = begin[1]; e < ( unsigned int )( end[1] ); ++b )
//             {
//                 const std::vector< entry >& v = grid[b][e];
//                 for( std::size_t i = 0; i < v.size(); ++i )
//                 {
//                     double db = abs_bearing_distance_( p.bearing, v[i].point.bearing );
//                     double de = p.elevation - v[i].point.elevation;
//                     if( ( db * db + de * de ) > threshold_square ) { continue; }
//                     if( range_threshold && v[i].point.range < ( p.range + *range_threshold ) ) { return NULL; }
//                     if( !min || v[i].point.range < min->range ) { entry = &v[i]; }
//                     if( min ) // todo: quick and dirty, fix point_tRBE and use extents
//                     {
//                         min->range = std::min( min->range, v[i].point.range );
//                         min->bearing = bearing_min_( min->bearing, v[i].point.bearing );
//                         min->elevation = std::min( min->elevation, v[i].point.elevation );
//                         max->range = std::max( max->range, v[i].point.range );
//                         max->bearing = bearing_max_( max->bearing, v[i].point.bearing );
//                         max->elevation = std::max( max->elevation, v[i].point.elevation );
//                     }
//                     else
//                     {
//                         min = max = v[i].point;
//                     }
//                 }
//             }
//         }
//         return    !min
//                || !bearing_between_( p.bearing, min->bearing, max->bearing )
//                || comma::math::less( p.elevation, min->elevation )
//                || !comma::math::less( p.elevation, max->elevation ) ? NULL : entry;
//     }

    void add( const entry& p )
    {
        if( points.size() == points.capacity() ) { points.reserve( 2048 ); } // quick and dirty
        points.push_back( p );
    }

    const entry* trace( const point_t& p, double threshold, boost::optional< double > range_threshold ) const
    {
        const entry* e = NULL;
        static const double threshold_square = threshold * threshold; // static: quick and dirty
        boost::optional< point_t > min;
        boost::optional< point_t > max;
        for( std::size_t i = 0; i < points.size(); ++i )
        {
            double db = abs_bearing_distance_( p.bearing(), points[i].point.bearing() );
            double de = p.elevation() - points[i].point.elevation();
            if( ( db * db + de * de ) > threshold_square ) { continue; }
            if( range_threshold && points[i].point.range() < ( p.range() + *range_threshold ) ) { return NULL; }
            if( min ) // todo: quick and dirty, fix point_tRBE and use extents
            {
                if( points[i].point.range() < min->range() )
                {
                    min->range( points[i].point.range() );
                    e = &points[i];
                }
                min->bearing( bearing_min_( min->bearing(), points[i].point.bearing() ) );
                min->elevation( std::min( min->elevation(), points[i].point.elevation() ) );
                max->bearing( bearing_max_( max->bearing(), points[i].point.bearing() ) );
                max->elevation( std::max( max->elevation(), points[i].point.elevation() ) );
            }
            else
            {
                e = &points[i];
                min = max = points[i].point;
            }
        }
        return    !min
               || !bearing_between_( p.bearing(), min->bearing(), max->bearing() )
               || !comma::math::less( min->elevation(), p.elevation() )
               || !comma::math::less( p.elevation(), max->elevation() ) ? NULL : e;
    }
};

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        verbose = options.exists( "--verbose,-v" );
        comma::csv::options csv( options, "range,bearing,elevation" );
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
        boost::optional< double > range_threshold = options.optional< double >( "--range-threshold,-r" );
        std::vector< std::string > unnamed = options.unnamed( "--verbose,-v", "-.*" );
        if( unnamed.empty() ) { std::cerr << "points-detect-change: please specify file with the reference point cloud" << std::endl; return 1; }
        if( unnamed.size() > 1 ) { std::cerr << "points-detect-change: expected file with the reference point cloud, got: " << comma::join( unnamed, ' ' ) << std::endl; return 1; }
        #ifdef WIN32
            std::ios::openmode mode = 0;
            if( csv.binary() )
            {
                mode |= std::ios::binary;
                _setmode( _fileno( stdin ), _O_BINARY );
                _setmode( _fileno( stdout ), _O_BINARY );
            }
            std::ifstream ifs( unnamed[0].c_str(), mode );
        #else
            std::ifstream ifs( unnamed[0].c_str() );
        #endif
        if( !ifs.is_open() ) { std::cerr << "points-detect-change: failed to open \"" << unnamed[0] << "\"" << std::endl; return 1; }
        comma::csv::input_stream< point_t > ifstream( ifs, csv );
        typedef snark::voxel_map< cell, 2 > grid_t;
        resolution = grid_t::point_type( threshold, threshold );
        grid_t grid( resolution );
        if( verbose ) { std::cerr << "points-detect-change: loading reference point cloud..." << std::endl; }
        comma::signal_flag is_shutdown;
        comma::uint64 index = 0;
        //{ ProfilerStart( "points-detect-change.prof" );
        std::deque< std::vector< char > > buffers;
        while( ifs.good() && !ifs.eof() && !is_shutdown )
        {
            const point_t* p = ifstream.read();
            if( !p ) { break; }
            cell::entry entry( *p, index );
            for( int i = -1; i < 2; ++i )
            {
                for( int j = -1; j < 2; ++j )
                {
                    double bearing = p->bearing() + threshold * i;
                    if( bearing < -M_PI ) { bearing += ( M_PI * 2 ); }
                    else if( bearing >= M_PI ) { bearing -= ( M_PI * 2 ); }
                    double elevation = p->elevation() + threshold * j;
                    grid_t::iterator it = grid.touch_at( grid_t::point_type( bearing, elevation ) );
                    it->second.add( entry ); //it->second.add_to_grid( entry );
                }
            }
            buffers.push_back( std::vector< char >() ); // todo: quick and dirty; use memory map instead?
            if( csv.binary() )
            {
                static unsigned int s = ifstream.binary().binary().format().size();
                buffers.back().resize( s );
                ::memcpy( &buffers.back()[0], ifstream.binary().last(), s );
            }
            else
            {
                std::string s = comma::join( ifstream.ascii().last(), csv.delimiter );
                buffers.back().resize( s.size() );
                ::memcpy( &buffers.back()[0], &s[0], s.size() );
            }
            ++index;
        }
        if( verbose ) { std::cerr << "points-detect-change: loaded reference point cloud: " << index << " points in a grid of size " << grid.size() << " voxels" << std::endl; }
        comma::csv::input_stream< point_t > istream( std::cin, csv );
        while( std::cin.good() && !std::cin.eof() && !is_shutdown )
        {
            const point_t* p = istream.read();
            if( !p ) { break; }
            grid_t::const_iterator it = grid.at( grid_t::point_type( p->bearing(), p->elevation() ) );
            if( it == grid.end() ) { continue; }
            const cell::entry* q = it->second.trace( *p, threshold, range_threshold );
            if( !q ) { continue; }
            if( csv.binary() )
            {
                static unsigned int is = istream.binary().binary().format().size();
                std::cout.write( istream.binary().last(), is );
                static unsigned int fs = ifstream.binary().binary().format().size();
                std::cout.write( &buffers[q->index][0], fs );
            }
            else
            {
                std::cout << comma::join( istream.ascii().last(), csv.delimiter )
                          << csv.delimiter
                          << std::string( &buffers[q->index][0], buffers[q->index].size() ) << std::endl;
            }
        }
        //} ProfilerStop();
        if( is_shutdown ) { std::cerr << "points-detect-change: caught signal" << std::endl; return 1; }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "points-detect-change: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-detect-change: unknown exception" << std::endl; }
    return 1;
}
