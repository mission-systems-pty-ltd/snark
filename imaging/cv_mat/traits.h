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

} } // namespace comma { namespace visiting {
