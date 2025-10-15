// Copyright (c) 2021 Mission Systems Pty Ltd

#pragma once

#include "types.h"
#include <comma/visiting/traits.h>

namespace comma { namespace visiting {

template <> struct traits< snark::innovusion::frame_t >
{
    template< typename Key, class Visitor >
    static void visit( const Key&, snark::innovusion::frame_t& t, Visitor& v )
    {
        v.apply( "idx", t.idx );
        v.apply( "sub_idx", t.sub_idx );
        v.apply( "sub_seq", t.sub_seq );
        v.apply( "ts_us_start", t.ts_us_start );
        v.apply( "ts_us_end", t.ts_us_end );
        v.apply( "points_number", t.points_number );
        v.apply( "conf_level", t.conf_level );
        v.apply( "timestamp_sync", t.timestamp_sync );
    }

    template< typename Key, class Visitor >
    static void visit( const Key&, const snark::innovusion::frame_t& t, Visitor& v )
    {
        v.apply( "idx", t.idx );
        v.apply( "sub_idx", t.sub_idx );
        v.apply( "sub_seq", t.sub_seq );
        v.apply( "ts_us_start", t.ts_us_start );
        v.apply( "ts_us_end", t.ts_us_end );
        v.apply( "points_number", t.points_number );
        v.apply( "conf_level", t.conf_level );
        v.apply( "timestamp_sync", t.timestamp_sync );
    }
};

template <> struct traits< snark::innovusion::point_t >
{
    template< typename Key, class Visitor >
    static void visit( const Key&, snark::innovusion::point_t& t, Visitor& v )
    {
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
        v.apply( "range", t.radius );
        v.apply( "ts_100us", t.ts_100us );
        v.apply( "value", t.value );
        v.apply( "flags", t.flags );
        v.apply( "channel", t.channel );
        v.apply( "scan_id", t.scan_id );
        v.apply( "scan_idx", t.scan_idx );
    }

    template< typename Key, class Visitor >
    static void visit( const Key&, const snark::innovusion::point_t& t, Visitor& v )
    {
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
        v.apply( "range", t.radius );
        v.apply( "ts_100us", t.ts_100us );
        v.apply( "value", t.value );
        v.apply( "flags", t.flags );
        v.apply( "channel", t.channel );
        v.apply( "scan_id", t.scan_id );
        v.apply( "scan_idx", t.scan_idx );
    }
};

template <> struct traits< snark::innovusion::output_data_t >
{
    template< typename Key, class Visitor >
    static void visit( const Key&, snark::innovusion::output_data_t& t, Visitor& v )
    {
        v.apply( "t", t.t );
        v.apply( "block", t.block );
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
        v.apply( "range", t.radius );
        v.apply( "value", t.value );
    }

    template< typename Key, class Visitor >
    static void visit( const Key&, const snark::innovusion::output_data_t& t, Visitor& v )
    {
        v.apply( "t", t.t );
        v.apply( "block", t.block );
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
        v.apply( "range", t.radius );
        v.apply( "value", t.value );
    }
};

template <> struct traits< snark::innovusion::output_data_full_t >
{
    template< typename Key, class Visitor >
    static void visit( const Key&, snark::innovusion::output_data_full_t& t, Visitor& v )
    {
        v.apply( "t", t.t );
        v.apply( "block", t.block );
        v.apply( "frame", t.frame );
        v.apply( "point", t.point );
    }

    template< typename Key, class Visitor >
    static void visit( const Key&, const snark::innovusion::output_data_full_t& t, Visitor& v )
    {
        v.apply( "t", t.t );
        v.apply( "block", t.block );
        v.apply( "frame", t.frame );
        v.apply( "point", t.point );
    }
};

template< typename T > struct traits< snark::innovusion::checksummed< T > >
{
    template< typename K, typename V > static void visit( const K&, snark::innovusion::checksummed< T >& p, V& v )
    {
        v.apply( "data", p.data );
        v.apply( "crc", p.crc );
    }

    template< typename K, typename V > static void visit( const K&, const snark::innovusion::checksummed< T >& p, V& v )
    {
        v.apply( "data", p.data );
        v.apply( "crc", p.crc );
    }
};

} } // namespace comma { namespace visiting {
