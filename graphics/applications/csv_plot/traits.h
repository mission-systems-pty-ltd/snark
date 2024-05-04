// Copyright (c) 2021 Vsevolod Vlaskine

/// @author Vsevolod Vlaskine

#pragma once

#include <cmath>
#include <comma/csv/traits.h>
#include <comma/visiting/traits.h>
#include "charts.h"
#include "record.h"
#include "series.h"
#include "stream.h"

namespace comma { namespace visiting {

template <> struct traits< snark::graphics::plotting::point >
{
    template< typename K, typename V > static void visit( const K&, snark::graphics::plotting::point& t, V& v )
    {
        if( t.x ) { v.apply( "x", *t.x ); } // todo? what should be the behaviour in comma::csv::from_ascii on optional?
        if( t.y ) { v.apply( "y", *t.y ); } // todo? what should be the behaviour in comma::csv::from_ascii on optional?
        if( t.z ) { v.apply( "z", *t.z ); } // todo? what should be the behaviour in comma::csv::from_ascii on optional?
    }
    template< typename K, typename V > static void visit( const K&, const snark::graphics::plotting::point& t, V& v )
    {
        if( t.x ) { v.apply( "x", *t.x ); } // todo? what should be the behaviour in comma::csv::from_ascii on optional?
        if( t.y ) { v.apply( "y", *t.y ); } // todo? what should be the behaviour in comma::csv::from_ascii on optional?
        if( t.z ) { v.apply( "z", *t.z ); } // todo? what should be the behaviour in comma::csv::from_ascii on optional?
    }
};

template <> struct traits< snark::graphics::plotting::record >
{
    template< typename K, typename V > static void visit( const K& k, snark::graphics::plotting::record& t, V& v )
    {
        v.apply( "t", t.t );
        v.apply( "series", t.series );
        v.apply( "block", t.block );
        for( unsigned int i = 1; i < t.series.size(); ++i ) // todo: lots of copying, watch performance; hacky for traits in libraries, but ok for applications where traits have limited use
        {
            if( !t.series[i].x ) { t.series[i].x = *t.series[0].x; }
            if( !t.series[i].y ) { t.series[i].y = *t.series[0].y; }
            if( !t.series[i].z ) { t.series[i].z = *t.series[0].z; }
        }
    }
    template< typename K, typename V > static void visit( const K& k, const snark::graphics::plotting::record& t, V& v )
    {
        v.apply( "t", t.t );
        v.apply( "series", t.series );
        v.apply( "block", t.block );
    }
};

template <> struct traits< snark::graphics::plotting::series::config >
{
    template< typename K, typename V > static void visit( const K&, snark::graphics::plotting::series::config& t, V& v )
    {
        v.apply( "chart", t.chart );
        v.apply( "color", t.color_name );
        if( !t.color_name.empty() ) { t.color = QColor( &t.color_name[0] ); }
        v.apply( "name", t.name );
        v.apply( "scroll", t.scroll );
        v.apply( "shape", t.shape );
        v.apply( "style", t.style );
        v.apply( "title", t.title );
        v.apply( "weight", t.weight );
    }
    template< typename K, typename V > static void visit( const K&, const snark::graphics::plotting::series::config& t, V& v )
    {
        if( t.color ) { v.apply( "color", std::string( t.color->name() ) ); }
        v.apply( "name", t.name );
        v.apply( "scroll", t.scroll );
        v.apply( "shape", t.shape );
        v.apply( "style", t.style );
        v.apply( "title", t.title );
        v.apply( "weight", t.weight );
    }
};

template <> struct traits< snark::graphics::plotting::stream::config_t >
{
    template< typename K, typename V > static void visit( const K&, snark::graphics::plotting::stream::config_t& t, V& v )
    {
        v.apply( "csv", t.csv );
        v.apply( "pass-through", t.pass_through );
        v.apply( "size", t.size );
        v.apply( "number-of-series", t.number_of_series );
        v.apply( "block-by-size", t.block_by_size );
        v.apply( "blocking", t.blocking );
        if( t.series.size() < t.number_of_series ) { t.series.resize( t.number_of_series ); } // quick and dirty
        v.apply( "series", t.series[0] );
        for( unsigned int i = 1; i < t.series.size(); ++i ) { t.series[i] = t.series[0]; } // todo: options per series
    }
};

template <> struct traits< snark::graphics::plotting::chart::axis::tick >
{
    template< typename K, typename V > static void visit( const K&, snark::graphics::plotting::chart::axis::tick& t, V& v )
    {
        v.apply( "anchor", t.anchor );
        v.apply( "interval", t.interval );
        v.apply( "count", t.count );
    }
};

template <> struct traits< snark::graphics::plotting::chart::axis::label >
{
    template< typename K, typename V > static void visit( const K&, snark::graphics::plotting::chart::axis::label& t, V& v )
    {
        v.apply( "format", t.format );
        v.apply( "angle", t.angle );
    }
};

template <> struct traits< snark::graphics::plotting::chart::axis::config >
{
    template< typename K, typename V > static void visit( const K&, snark::graphics::plotting::chart::axis::config& t, V& v )
    { 
        v.apply( "title", t.title );
        v.apply( "tick", t.tick );
        v.apply( "label", t.label );
    }
};

template <> struct traits< snark::graphics::plotting::chart::axes::config >
{
    template< typename K, typename V > static void visit( const K&, snark::graphics::plotting::chart::axes::config& t, V& v )
    {
        v.apply( "x", t.x );
        v.apply( "y", t.y );
    }
};

template <> struct traits< snark::graphics::plotting::chart::config_t >
{
    static void set_to_nan( boost::optional< double >& d ) { if( !d ) { d = std::nan( "" ); } } // todo: shame, shame, shame; fix csv visiting of optional
    static void reset_if_nan( boost::optional< double >& d ) { if( d && std::isnan( *d ) ) { d.reset(); } } // todo: shame, shame, shame; fix csv visiting of optional
    static void set_to_nan( snark::graphics::plotting::point& p ) { set_to_nan( p.x ); set_to_nan( p.y ); set_to_nan( p.z ); } // todo: shame, shame, shame; fix csv visiting of optional
    static void reset_if_nan( snark::graphics::plotting::point& p ) { reset_if_nan( p.x ); reset_if_nan( p.y ); reset_if_nan( p.z ); } // todo: shame, shame, shame; fix csv visiting of optional
    
    template< typename K, typename V > static void visit( const K&, snark::graphics::plotting::chart::config_t& t, V& v )
    {
        v.apply( "animate", t.animate );
        v.apply( "axes", t.axes );
        v.apply( "legend", t.legend );
        set_to_nan( t.max ); // shame
        v.apply( "max", t.max );
        reset_if_nan( t.max ); // shame
        set_to_nan( t.min ); // shame
        v.apply( "min", t.min );
        reset_if_nan( t.min ); // shame
        v.apply( "name", t.name );
        v.apply( "scroll", t.scroll );
        v.apply( "title", t.title );
        v.apply( "theme", t.theme );
    }
};

} } // namespace comma { namespace visiting {
