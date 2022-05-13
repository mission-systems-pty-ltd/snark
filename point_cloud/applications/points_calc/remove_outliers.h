// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2017 The University of Sydney
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

#pragma once
#include <iostream>
#include <vector>
#include <deque>
#include <comma/csv/options.h>
#include "../../../visiting/eigen.h"

namespace remove_outliers {

struct point
{
    Eigen::Vector3d coordinates;
    unsigned block;
    point() : coordinates(Eigen::Vector3d::Zero()),block(0) {}
};

struct record
{
    remove_outliers::point point;
    std::string line;
    bool rejected;
    
    record() : rejected( false ) {}
    record( const remove_outliers::point& p, const std::string& line ) : point( p ), line( line ), rejected( false ) {}
};

struct app
{
    comma::csv::options csv;
    bool verbose;
    std::deque< remove_outliers::record > records;
    snark::math::closed_interval< double, 3 > extents;
    app(const comma::csv::options& csv,bool verbose=false) : csv(csv), verbose(verbose) {}
    void process(const Eigen::Vector3d& resolution,bool no_antialiasing,unsigned size)
    {
        if( verbose ) { std::cerr << "points-calc: loading " << records.size() << " points into grid..." << std::endl; }
        typedef std::vector< remove_outliers::record* > voxel_t; // todo: is vector a good container? use deque
        typedef snark::voxel_map< voxel_t, 3 > grid_t;
        grid_t grid( extents.min(), resolution );
        for( std::size_t i = 0; i < records.size(); ++i ) { ( grid.touch_at( records[i].point.coordinates ) )->second.push_back( &records[i] ); }
        if( verbose ) { std::cerr << "points-calc: removing outliers..." << std::endl; }
        for( grid_t::iterator it = grid.begin(); it != grid.end(); ++it )
        {
            bool rejected = true;
            if( no_antialiasing )
            {
                rejected = it->second.size() < size;
            }
            else
            {
                grid_t::index_type i;
                for( i[0] = it->first[0] - 1; i[0] < it->first[0] + 2 && rejected; ++i[0] )
                {
                    for( i[1] = it->first[1] - 1; i[1] < it->first[1] + 2 && rejected; ++i[1] )
                    {
                        for( i[2] = it->first[2] - 1; i[2] < it->first[2] + 2 && rejected; ++i[2] )
                        {
                            grid_t::iterator git = grid.find( i );
                            rejected = git == grid.end() || git->second.size() < size;
                        }
                    }
                }
            }
            if( rejected ) { for( std::size_t i = 0; i < it->second.size(); ++i ) { it->second[i]->rejected = true; } }
        }
    }
    void write_output(bool discard)
    {
        if( verbose ) { std::cerr << "points-calc: outputting..." << std::endl; }
        std::string endl = csv.binary() ? "" : "\n";
        std::string delimiter = csv.binary() ? "" : std::string( 1, csv.delimiter );
        if(discard)
        {
            for( std::size_t i = 0; i < records.size(); ++i )
            {
                if(!records[i].rejected)
                {
                    std::cout.write( &records[i].line[0], records[i].line.size() );
                    std::cout.write( &endl[0], endl.size() );
                    if( csv.flush ) { std::cout.flush(); }
                }
            }
        }
        else
        {
            char zero=csv.binary() ? 0 : '0';
            char one=csv.binary() ? 1 : '1';
            for( std::size_t i = 0; i < records.size(); ++i )
            {
                char valid = records[i].rejected ? zero : one;
                std::cout.write( &records[i].line[0], records[i].line.size() );
                std::cout.write( &delimiter[0], delimiter.size() );
                std::cout.write( &valid, 1 );
                std::cout.write( &endl[0], endl.size() );
                if( csv.flush ) { std::cout.flush(); }
            }
        }
    }
};

} // namespace remove_outliers {

namespace comma { namespace visiting {

template <> struct traits< remove_outliers::point >
{
    template< typename K, typename V > static void visit( const K&, const remove_outliers::point& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "block", t.block );
    }
    
    template< typename K, typename V > static void visit( const K&, remove_outliers::point& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "block", t.block );
    }
};

} } // namespace comma { namespace visiting {
    
