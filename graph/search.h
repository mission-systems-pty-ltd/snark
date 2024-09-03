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

/// @author jason gan
/// @author vsevolod vlaskine

#pragma once

#include <queue>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/optional.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <comma/base/exception.h>
#include <comma/math/compare.h>

namespace snark {

/// forward search
template < typename G, typename D, typename A, typename O, typename V >
void forward_search( G& graph, const D& start, const A& advance, const O& objective_function, const V& valid );

/// pull the best path from the graph
/// @return best path descriptors
template < typename Graph >
std::vector< typename boost::graph_traits< Graph >::vertex_descriptor > best_path( const Graph& graph
                                                                                 , const typename boost::graph_traits< Graph >::vertex_descriptor& start
                                                                                 , const typename boost::graph_traits< Graph >::vertex_descriptor& goal );

template < typename Graph >
inline std::vector< typename boost::graph_traits< Graph >::vertex_descriptor > best_path( const Graph& graph
                                                                                        , const typename boost::graph_traits< Graph >::vertex_descriptor& start
                                                                                        , const typename boost::graph_traits< Graph >::vertex_descriptor& goal )
{
    typedef boost::graph_traits< Graph > traits_t;
    #ifdef BOOST_GRAPH_NO_BUNDLED_PROPERTIES
    typedef typename Graph::vertex_property_type node_t;
    #else // BOOST_GRAPH_NO_BUNDLED_PROPERTIES
    typedef typename Graph::vertex_bundled node_t;
    #endif // BOOST_GRAPH_NO_BUNDLED_PROPERTIES
    std::vector< typename traits_t::vertex_descriptor > best;
    COMMA_ASSERT( graph.m_vertices.find( goal ) != graph.m_vertices.end(), "goal vertex not found in the graph" );
    best.push_back( goal );
    for( typename traits_t::vertex_descriptor target = goal; target != start; )
    {
        const node_t& node = graph[target];
        if( !node.best_parent ) { return std::vector< typename traits_t::vertex_descriptor >(); }
        std::pair< typename traits_t::in_edge_iterator, typename traits_t::in_edge_iterator > it;
        for( it = boost::in_edges( target, graph ); it.first != it.second; ++it.first )
        {
            target = boost::source( *it.first, graph );
            if( graph[target].id == *node.best_parent ) { best.push_back( target ); break; } // todo: quick and dirty, may be suboptimal
        }
        COMMA_ASSERT( it.first != it.second, "node with " << graph[target].id << " has best parent with id " << *node.best_parent << ", but the edge from the best parent to the node not found" );
    }
    std::reverse( best.begin(), best.end() );
    return best;
}

template < typename G, typename D, typename A, typename O, typename V >
inline void forward_search( G& graph, const boost::unordered_set< D >& start, const A& advance, const O& objective_function, const V& valid )
{
    typedef boost::graph_traits< G > traits_t;
    #ifdef BOOST_GRAPH_NO_BUNDLED_PROPERTIES
    typedef typename G::vertex_property_type node_t;
    #else // BOOST_GRAPH_NO_BUNDLED_PROPERTIES
    typedef typename G::vertex_bundled node_t;
    #endif // BOOST_GRAPH_NO_BUNDLED_PROPERTIES
    typedef boost::unordered_set< D > set_t;
    typedef D vertex_desc;
    typedef std::multimap< double, vertex_desc > map_t;
    typedef boost::unordered_map< vertex_desc, typename map_t::iterator > vertex_map_t;
    vertex_map_t vertex_map;
    map_t vertex_queue;
    for( typename set_t::const_iterator it = start.begin(); it != start.end(); ++it )
    {
        vertex_map[ *it ] = vertex_queue.insert( std::make_pair( -objective_function( graph[ *it ].value ), *it ) );
    }
    while( !vertex_queue.empty() )
    {
        D v = vertex_queue.begin()->second;
        vertex_queue.erase( vertex_queue.begin() );
        vertex_map.erase( v );
        const node_t& source = graph[v];
        if( !valid( source.value ) ) { continue; }
        typedef std::pair< typename traits_t::out_edge_iterator, typename traits_t::out_edge_iterator > edge_iterator;
        for( edge_iterator out_edges = boost::out_edges( v, graph ); out_edges.first != out_edges.second; ++out_edges.first )
        {
            vertex_desc target_desc = boost::target( *out_edges.first, graph ); // extract vertex at the other end of edge
            node_t& target = graph[target_desc];
            if( source.best_parent && target.id == *( source.best_parent ) ) { continue; } 
            if( target.best_parent && objective_function( source.value ) < objective_function( target.value ) ) { continue; } // objective can only decrease or (in special cases equal) in forward prop
            const boost::optional< typename node_t::value_type >& node = advance( source.value, target.value, graph[ *out_edges.first ] );
            if( !node || !valid( *node ) ) { continue; }
            double objective = objective_function( *node );            
            if( target.best_parent )
            {
                if( *( target.best_parent ) == source.id && comma::math::equal( objective, objective_function( target.value ) ) ) { continue; }
                if( objective <= objective_function( target.value ) ) { continue; }
            }
            target.value = *node;
            target.best_parent = source.id;
            typename vertex_map_t::iterator it = vertex_map.find( target_desc );
            if( it != vertex_map.end() ) { vertex_queue.erase( it->second ); }
            vertex_map[target_desc] = vertex_queue.insert( std::make_pair( -objective, target_desc ) );
        }
    }
}

template < typename G, typename D, typename A, typename O, typename V >
inline void forward_search( G& graph, const D& start, const A& advance, const O& objective_function, const V& valid )
{
    boost::unordered_set< D > s;
    s.insert( start );
    forward_search( graph, s, advance, objective_function, valid );
}

} // namespace snark {
