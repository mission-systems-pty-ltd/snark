// Copyright (c) 2023 Vsevolod Vlaskine
// All rights reserved.

#include <fstream>
#include <comma/base/exception.h>
#include <comma/name_value/serialize.h>
#include "traits.h"
#include "tree.h"

namespace snark { namespace frames {

template < typename Offset >
tree& tree::make( std::istream& is, const comma::xpath& path, tree& t )
{
    boost::property_tree::ptree p;
    comma::property_tree::from_unknown( is, p );
    auto q = comma::property_tree::get_tree( p, path, true );
    COMMA_ASSERT( q, "failed to get subtree at '" << path.to_string() << "'" );
    

    // todo: traverse tree
    // todo: traverse tree
    // todo: parameterize on offset field
    // todo: populate offset



    return t;
}

template < typename Offset >
tree& tree::make( const std::string& filename, const comma::xpath& path, tree& t )
{
    std::ifstream ifs( filename );
    COMMA_ASSERT( ifs.is_open(), "failed to open: \"" << filename << "\"" );
    return make< Offset >( ifs, path, t );
}

} } // namespace snark { namespace frames {
