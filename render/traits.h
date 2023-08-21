// Copyright (c) 2011 The University of Sydney

#pragma once

#include <comma/visiting/traits.h>
#include "colour.h"

namespace comma { namespace visiting {

template < typename T > struct traits< snark::render::colour< T > >
{
    /// const visiting
    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::render::colour< T >& p, Visitor& v )
    {
        v.apply( "r", p.red() );
        v.apply( "g", p.green() );
        v.apply( "b", p.blue() );
        v.apply( "a", p.alpha() );
    }

    /// visiting
    template < typename Key, class Visitor >
    static void visit( Key, snark::render::colour< T >& p, Visitor& v )
    {
        T r = p.red();
        T g = p.green();
        T b = p.blue();
        T a = p.alpha();
        v.apply( "r", r );
        v.apply( "g", g );
        v.apply( "b", b );
        v.apply( "a", a );
        p = snark::render::colour< T >( r, g, b, a );
    }
};

} } // namespace comma { namespace visiting {
