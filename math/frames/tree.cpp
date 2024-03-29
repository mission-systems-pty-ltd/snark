// Copyright (c) 2023 Vsevolod Vlaskine
// All rights reserved.

#include <fstream>
#include <comma/base/exception.h>
#include <comma/base/none.h>
#include <comma/csv/ascii.h>
#include <comma/name_value/serialize.h>
#include <comma/visiting/traits.h>
#include "../pose.h"
#include "../../visiting/traits.h"
#include "tree.h"

namespace snark { namespace frames { namespace impl {

struct pose_flat // quick and dirty
{
    double x{0};
    double y{0};
    double z{0};
    double roll{0};
    double pitch{0};
    double yaw{0};

    operator pose() const { return pose( {x, y, z}, {roll, pitch, yaw} ); }
};

} } } // namespace snark { namespace frames { namespace impl {

namespace comma { namespace visiting {

template <> struct traits< snark::frames::impl::pose_flat >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::frames::impl::pose_flat& p, Visitor& v )
    {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
        v.apply( "z", p.z );
        v.apply( "roll", p.roll );
        v.apply( "pitch", p.pitch );
        v.apply( "yaw", p.yaw );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::frames::impl::pose_flat& p, Visitor& v )
    {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
        v.apply( "z", p.z );
        v.apply( "roll", p.roll );
        v.apply( "pitch", p.pitch );
        v.apply( "yaw", p.yaw );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace frames {

static boost::optional< pose > _pose( const boost::property_tree::ptree& t, const comma::xpath& path, tree::frame_format::values format )
{
    boost::optional< pose > p = comma::silent_none< pose >();
    switch( format )
    {
        case tree::frame_format::flat:
        {
            auto f = comma::property_tree::as< impl::pose_flat >( t, path, true, true );
            if( f ) { p = *f; }
            break;
        }
        case tree::frame_format::nested:
        {
            p = comma::property_tree::as< pose >( t, path, true, true );
            break;
        }
        case tree::frame_format::csv:
        {
            static comma::csv::ascii< pose > ascii( "x,y,z,roll,pitch,yaw", ',', false );
            auto s = comma::property_tree::get( t, path, true );
            if( s ) { p = ascii.get( s ); }
            break;
        }
    }
    return p;
}

tree::frame_format::values tree::frame_format::from_string( const std::string& s )
{
    if( s == "nested" || s.empty() ) { return tree::frame_format::nested; }
    if( s == "flat" ) { return tree::frame_format::flat; }
    if( s == "csv" ) { return tree::frame_format::csv; }
    COMMA_THROW( comma::exception, "expected 'nested', 'flat', or 'csv'; got: '" << s << "'" );
}

tree& tree::_make( std::istream& is, const comma::xpath& path, frame_format::values format, tree& t )
{
    boost::property_tree::ptree p;
    comma::property_tree::from_unknown( is, p );
    auto q = path.elements.empty() ? p : comma::property_tree::get_tree( p, path, true ); // todo? should not need path...empty() check
    COMMA_ASSERT( q, "failed to get subtree at '" << path.to_string() << "'" );
    
    // todo: traverse tree
    // todo: handle file
    // todo: handle path

    t._format = format;
    t._tree = *q;
    return t;
}

tree& tree::_make( const std::string& filename, const comma::xpath& path, frame_format::values format, tree& t )
{
    if( filename == "-" ) { return _make( std::cin, path, format, t ); }
    std::ifstream ifs( filename );
    COMMA_ASSERT( ifs.is_open(), "failed to open: \"" << filename << "\"" );
    return _make( ifs, path, format, t );
}

std::vector< pose > tree::operator()( const comma::xpath& path, const std::string& name ) const
{
    std::vector< pose > v( path.elements.size() );
    for( auto p = path; !p.elements.empty(); p = p.head() )
    {
        auto o = _pose( _tree, p / name, _format );
        if( o ) { v[ v.size() - p.elements.size() ] = *o; }
    }
    return v;
}

} } // namespace snark { namespace frames {
