// Copyright (c) 2023 Vsevolod Vlaskine
// All rights reserved.

#pragma once

#include <comma/visiting/traits.h>
#include "tree.h"

namespace comma { namespace visiting {

template < typename Offset > struct traits< snark::frames::tree::properties< Offset > >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::frames::tree::properties< Offset >& p, Visitor& v )
    {
        v.apply( "file", p.file );
        v.apply( "path", p.path );
        v.apply( "offset", p.offset );
    }

    template < typename Key, class Visitor > static void visit( const Key&, const snark::frames::tree::properties< Offset >& p, Visitor& v )
    {
        v.apply( "file", p.file );
        v.apply( "path", p.path );
        v.apply( "offset", p.offset );
    }
};

} } // namespace comma { namespace visiting {
