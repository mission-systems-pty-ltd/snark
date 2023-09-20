// Copyright (c) 2023 Vsevolod Vlaskine
// All rights reserved.

#include <fstream>
#include <comma/base/exception.h>
#include <comma/name_value/serialize.h>
#include <comma/visiting/traits.h>
#include "../../visiting/traits.h"
#include "tree.h"

// namespace comma { namespace visiting {

// template <>
// struct traits< ::Eigen::Matrix< T, Rows, Columns > >
// {
//     enum { rows = Rows, columns = Columns, size = rows * columns };

//     static const ::Eigen::Matrix< T, Rows, Columns >& zero()
//     {
//         static ::Eigen::Matrix< T, Rows, Columns > z = ::Eigen::Matrix< T, Rows, Columns >::Zero();
//         return z;
//     }

//     static const ::Eigen::Matrix< T, Rows, Columns >& indentity()
//     {
//         static ::Eigen::Matrix< T, Rows, Columns > i = ::Eigen::Matrix< T, Rows, Columns >::Identity();
//         return i;
//     }
// };

// } } // namespace comma { namespace visiting {

namespace snark { namespace frames {

tree& tree::make( std::istream& is, const comma::xpath& path, offset_format::values format, tree& t )
{
    boost::property_tree::ptree p;
    comma::property_tree::from_unknown( is, p );
    auto q = path.elements.empty() ? p : comma::property_tree::get_tree( p, path, true ); // todo? should not need path...empty() check
    COMMA_ASSERT( q, "failed to get subtree at '" << path.to_string() << "'" );
    
    

    // todo: traverse tree
    // todo: handle file
    // todo: handle path
    // todo: populate offset field
    //       - handle non-csv
    //       - handle csv

    t._format = format;
    t._tree = *q;
    return t;
}

tree& tree::make( const std::string& filename, const comma::xpath& path, offset_format::values format, tree& t )
{
    if( filename == "-" ) { return make( std::cin, path, format, t ); }
    std::ifstream ifs( filename );
    COMMA_ASSERT( ifs.is_open(), "failed to open: \"" << filename << "\"" );
    return make( ifs, path, format, t );
}

std::vector< position > tree::operator()( const comma::xpath& path, const std::string& name ) const
{
    std::vector< position > v( path.elements.size() );
    for( auto p = path; !p.elements.empty(); p = p.head() )
    {
        auto o = comma::property_tree::as< position >( _tree, p / name, true, true );
        if( o ) { v[ v.size() - p.elements.size() ] = *o; }
    }
    return v;
}

} } // namespace snark { namespace frames {
