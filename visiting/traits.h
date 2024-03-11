// Copyright (c) 2011 The University of Sydney

#pragma once

#include <comma/visiting/apply.h>
#include <comma/visiting/visit.h>
#include "../math/interval.h"
#include "../math/polynomial.h"
#include "../math/pose.h"
#include "../math/position.h"
#include "../math/range_bearing_elevation.h"
#include "../math/frame_transforms.h"
#include "../math/roll_pitch_yaw.h"
#include "eigen.h"

namespace comma { namespace visiting {

template <> struct traits< snark::range_bearing_elevation >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::range_bearing_elevation& p, Visitor& v )
    {
        v.apply( "range", p.range() );
        v.apply( "bearing", p.bearing() );
        v.apply( "elevation", p.elevation() );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, snark::range_bearing_elevation& p, Visitor& v )
    {
        double r = p.r();
        double b = p.b();
        double e = p.e();
        v.apply( "range", r );
        v.apply( "bearing", b );
        v.apply( "elevation", e );
        p = snark::range_bearing_elevation( r, b, e );
    }
};

template <> struct traits< snark::bearing_elevation >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::bearing_elevation& p, Visitor& v )
    {
        v.apply( "bearing", p.bearing() );
        v.apply( "elevation", p.elevation() );
    }

    template < typename Key, class Visitor > static void visit( const Key&, snark::bearing_elevation& p, Visitor& v )
    {
        double b = p.b();
        double e = p.e();
        v.apply( "bearing", b );
        v.apply( "elevation", e );
        p = snark::bearing_elevation( b, e );
    }
};

template <> struct traits< snark::roll_pitch_yaw >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::roll_pitch_yaw& p, Visitor& v )
    {
        v.apply( "roll", p.roll() );
        v.apply( "pitch", p.pitch() );
        v.apply( "yaw", p.yaw() );
    }

    template < typename Key, class Visitor > static void visit( const Key&, snark::roll_pitch_yaw& t, Visitor& v )
    {
        double r = t.roll();
        double p = t.pitch();
        double y = t.yaw();
        v.apply( "roll", r );
        v.apply( "pitch", p );
        v.apply( "yaw", y );
        t = snark::roll_pitch_yaw( r, p, y );
    }
};

template < typename T, unsigned int D > struct traits< snark::math::closed_interval< T, D > >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::math::closed_interval< T, D >& p, Visitor& v )
    {
        v.apply( "min", p ? p.min() : Eigen::Matrix< T, D, 1 >() ); // quick and dirty
        v.apply( "max", p ? p.max() : Eigen::Matrix< T, D, 1 >() ); // quick and dirty
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, snark::math::closed_interval< T, D >& p, Visitor& v )
    {
        Eigen::Matrix< T, D, 1 > min;
        if( p ) { min = p.min(); } // quick and dirty
        v.apply( "min", min );
        Eigen::Matrix< T, D, 1 > max;
        if( p ) { max = p.max(); } // quick and dirty
        v.apply( "max", max );
        p = snark::math::closed_interval< T, D >( min, max );
    }
};

template <> struct traits< snark::frame_transforms::tr_transform >
{
    template< typename K, typename V > static void visit( const K& k, snark::frame_transforms::tr_transform& t, V& v )
    {
        v.apply( "translation", t.translation );
        v.apply( "rotation", t.rotation );
    }

    template< typename K, typename V > static void visit( const K& k, const snark::frame_transforms::tr_transform& t, V& v )
    {
        v.apply( "translation", t.translation );
        v.apply( "rotation", t.rotation );
    }
};

template <> struct traits< snark::pose >
{
    template< typename K, typename V > static void visit( const K& k, snark::pose& t, V& v )
    {
        v.apply( "translation", t.translation );
        v.apply( "rotation", t.rotation );
    }

    template< typename K, typename V > static void visit( const K& k, const snark::pose& t, V& v )
    {
        v.apply( "translation", t.translation );
        v.apply( "rotation", t.rotation );
    }
};

template <> struct traits< snark::position >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::position& p, Visitor& v )
    {
        v.apply( "coordinates", p.coordinates );
        v.apply( "orientation", p.orientation );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::position& p, Visitor& v )
    {
        v.apply( "coordinates", p.coordinates );
        v.apply( "orientation", p.orientation );
    }
};

template < typename T, unsigned int Dim, unsigned int Degree > struct traits< snark::polynomial< T, Dim, Degree > >
{
    template< typename K, typename V > static void visit( const K& k, snark::polynomial< T, Dim, Degree >& t, V& v )
    {
        v.apply( "coef", t.coef );
    }

    template< typename K, typename V > static void visit( const K& k, const snark::polynomial< T, Dim, Degree >& t, V& v )
    {
        v.apply( "coef", t.coef );
    }
};

} } // namespace comma { namespace visiting {
