// Copyright (c) 2023 Vsevolod Vlaskine
// All rights reserved.

#include <fstream>
#include <comma/base/exception.h>
#include <comma/name_value/serialize.h>
#include "tree.h"

namespace snark { namespace frames {

tree& tree::make( std::istream& is, const comma::xpath& path, bool offset_as_csv, tree& t )
{
    boost::property_tree::ptree p;
    comma::property_tree::from_unknown( is, p );
    auto q = comma::property_tree::get_tree( p, path, true );
    COMMA_ASSERT( q, "failed to get subtree at '" << path.to_string() << "'" );
    

    // todo: traverse tree
    // todo: handle file
    // todo: handle path
    // todo: populate offset field
    //       - handle non-csv
    //       - handle csv


    return t;
}

tree& tree::make( const std::string& filename, const comma::xpath& path, bool offset_as_csv, tree& t )
{
    std::ifstream ifs( filename );
    COMMA_ASSERT( ifs.is_open(), "failed to open: \"" << filename << "\"" );
    return make( ifs, path, offset_as_csv, t );
}

} } // namespace snark { namespace frames {
