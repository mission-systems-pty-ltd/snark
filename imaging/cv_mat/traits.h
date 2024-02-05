// Copyright (c) 2011 The University of Sydney

/// @author vsevolod vlaskine

#pragma once

#include <opencv2/features2d/features2d.hpp>
#include <comma/visiting/traits.h>
#include "serialization.h"

namespace comma { namespace visiting {

template < typename T > struct traits< cv::Point_< T > >
{
    template < typename Key, class Visitor > static void visit( const Key&, const cv::Point_< T >& p, Visitor& v )
    {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, cv::Point_< T >& p, Visitor& v )
    {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
    }
};

template <> struct traits< cv::KeyPoint >
{
    template < typename Key, class Visitor > static void visit( const Key&, const cv::KeyPoint& p, Visitor& v )
    {
        v.apply( "point", p.pt );
        v.apply( "size", p.size );
        v.apply( "angle", p.angle );
        v.apply( "response", p.response );
        v.apply( "octave", p.octave );
        v.apply( "class_id", p.class_id );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, cv::KeyPoint& p, Visitor& v )
    {
        v.apply( "point", p.pt );
        v.apply( "size", p.size );
        v.apply( "angle", p.angle );
        v.apply( "response", p.response );
        v.apply( "octave", p.octave );
        v.apply( "class_id", p.class_id );
    }
};

template <> struct traits< snark::cv_mat::serialization::header >
{
    template < typename K, typename V >
    static void visit( const K&, snark::cv_mat::serialization::header& h, V& v )
    {
        v.apply( "t", h.timestamp );
        v.apply( "rows", h.rows );
        v.apply( "cols", h.cols );
        v.apply( "type", h.type );
        v.apply( "size", h.size );
    }

    template < typename K, typename V >
    static void visit( const K&, const snark::cv_mat::serialization::header& h, V& v )
    {
        v.apply( "t", h.timestamp );
        v.apply( "rows", h.rows );
        v.apply( "cols", h.cols );
        v.apply( "type", h.type );
        v.apply( "size", h.size );
    }
};

template <> struct traits< snark::cv_mat::serialization::options >
{
    template < typename K, typename V >
    static void visit( const K&, snark::cv_mat::serialization::options& h, V& v )
    {
        v.apply( "fields", h.fields );
        std::string s = h.format.string();
        v.apply( "binary", s );
        h.format = comma::csv::format(s);
        v.apply( "rows", h.rows );
        v.apply( "cols", h.cols );
        v.apply( "type", h.type );
        v.apply( "no-header", h.no_header );
        v.apply( "header-only", h.header_only );
        v.apply( "timestamp", h.timestamp );
    }

    template < typename K, typename V >
    static void visit( const K&, const snark::cv_mat::serialization::options& h, V& v )
    {
        v.apply( "fields", h.fields );
        v.apply( "binary", h.format.string() );
        v.apply( "rows", h.rows );
        v.apply( "cols", h.cols );
        v.apply( "type", h.type );
        v.apply( "no-header", h.no_header );
        v.apply( "header-only", h.header_only );
        v.apply( "timestamp", h.timestamp );
    }
};

} } // namespace comma { namespace visiting {
