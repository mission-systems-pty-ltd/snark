// Copyright (c) 2018 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <array>
#include <cmath>
#include <functional>
#include <sstream>
#include <comma/csv/stream.h>
#include <comma/math/compare.h>
#include "../../../math/geometry/polygon.h"
#include "../../../math/geometry/traits.h"
#include "triangles.h"

namespace snark { namespace points_calc { namespace triangles {

namespace area {

std::string traits::usage() { return "    triangles-area: read triangles on stdin, append area\n"; }

typedef snark::triangle input;

struct output
{
    double area;
    output( double a = 0 ): area( a ) {}
};

} // namespace area

namespace discretise {

std::string traits::usage()
{
    std::ostringstream oss;
    oss
        << "    triangles-discretise: read triangles on stdin, append/output points that reasonably cover the triangle\n"
        << "                          (currently pretty quick and dirty)\n"
        << "        fields\n"
        << "            default input: \"corners\"\n"
        << "            default output: \"x,y,z,internal\"\n"
        << "        options\n"
        << "            --output-points-only,--points-only,--discard-input; output points, don't output input triangles\n"
        << "            --output-internal-only,--internal-only; output only points inside triangles\n"
        << "            --output-sides-only,--sides-only; output points only on the sides of triangles\n"
        << "            --radius,-r=<meters>; max distance between the points\n"
        << "            --tolerance,-t=[<meters>]; min distance between the points, todo\n"
        << std::endl
        << "        examples\n"
        << "            echo 0,0,0,0,0,10,0,10,0 \\\n"
        << "                | points-calc triangles-discretise --fields corners \\\n"
        << "                | view-points --fields ,,,,,,,,,x,y,z,id\n"
        << "            echo 0,0,0,0,0,10,0,10,0 \\\n"
        << "                | points-calc triangles-discretise --fields corners --discard-input \\\n"
        << "                | view-points --fields x,y,z,id\n"
        << std::endl;
    return oss.str();    
}

struct input
{
    snark::triangle triangle;
    comma::uint32 id;
    input() : id( 0 ) {}
};

struct output
{
    Eigen::Vector3d point;
    bool internal;
    comma::uint32 id;
    output( const Eigen::Vector3d& point = Eigen::Vector3d::Zero(), bool internal = false, comma::uint32 id = 0 ): point( point ), internal( internal ), id( id ) {}
};

} // namespace discretise {

} } } // namespace snark { namespace points_calc { namespace triangles {

namespace comma { namespace visiting {

template <> struct traits< snark::points_calc::triangles::discretise::input >
{
    template < typename K, typename V > static void visit( const K& k, snark::points_calc::triangles::discretise::input& t, V& v )
    {
        traits< snark::triangle >::visit( k, t.triangle, v );
        v.apply( "id", t.id );
    }
    
    template < typename K, typename V > static void visit( const K& k, const snark::points_calc::triangles::discretise::input& t, V& v )
    {
        traits< snark::triangle >::visit( k, t.triangle, v );
        v.apply( "id", t.id );
    }
};

template <> struct traits< snark::points_calc::triangles::discretise::output >
{
    template < typename K, typename V > static void visit( const K& k, snark::points_calc::triangles::discretise::output& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t.point, v );
        v.apply( "internal", t.internal );
        v.apply( "id", t.id );
    }
    
    template < typename K, typename V > static void visit( const K& k, const snark::points_calc::triangles::discretise::output& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t.point, v );
        v.apply( "internal", t.internal );
        v.apply( "id", t.id );
    }
};

template <> struct traits< snark::points_calc::triangles::area::output >
{
    template < typename K, typename V > static void visit( const K& k, snark::points_calc::triangles::area::output& t, V& v )
    {
        v.apply( "area", t.area );
    }
    
    template < typename K, typename V > static void visit( const K& k, const snark::points_calc::triangles::area::output& t, V& v )
    {
        v.apply( "area", t.area );
    }
};

} }

namespace snark { namespace points_calc { namespace triangles {

namespace area {

std::string traits::input_fields() { return comma::join( comma::csv::names< input >( true ), ',' ); }

std::string traits::input_format() { return comma::csv::format::value< input >(); }
    
std::string traits::output_fields() { return comma::join( comma::csv::names< output >( false ), ',' ); }

std::string traits::output_format() { return comma::csv::format::value< output >(); }

int traits::run( const comma::command_line_options& options )
{
    comma::csv::options csv( options );
    comma::csv::options output_csv;
    output_csv.flush = csv.flush;
    if( csv.binary() ) { output_csv.format( comma::csv::format::value< output >() ); }
    comma::csv::input_stream< input > istream( std::cin, csv );
    comma::csv::output_stream < output > ostream( std::cout, output_csv );
    comma::csv::tied< input, output > tied( istream, ostream );
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const input* r = istream.read();
        if( !r ) { break; }
        tied.append( output( r->area() ) );
    }
    return 0;
}

} // namespace area {
    
namespace discretise {

std::string traits::input_fields() { return comma::join( comma::csv::names< input >( true ), ',' ); }

std::string traits::input_format() { return comma::csv::format::value< input >(); }
    
std::string traits::output_fields() { return comma::join( comma::csv::names< output >( false ), ',' ); }

std::string traits::output_format() { return comma::csv::format::value< output >(); }

int traits::run( const comma::command_line_options& options )
{
    double radius = options.value< double >( "--radius,-r" );
    auto tolerance = options.optional< double >( "--tolerance,-t" );
    if( tolerance ) { std::cerr << "points-calc: triangles-discretise: --tolerance: todo" << std::endl; return 1; }
    comma::csv::options csv( options );
    if( csv.fields.empty() ) { csv.fields = "corners"; }
    comma::csv::options output_csv;
    output_csv.flush = csv.flush;
    output_csv.full_xpath = false;
    bool discard_input = options.exists( "--output-points-only,--points-only,--discard-input" );
    bool output_sides = !options.exists( "--output-internal-only,--internal-only" );
    bool output_internal = !options.exists( "--output-sides-only,--sides-only" );
    if( csv.has_field( "id" ) )
    {
        if( csv.binary() ) { output_csv.format( "3d,b,ui" ); }
    }
    else
    {
        output_csv.fields = "x,y,z,internal";
        if( csv.binary() ) { output_csv.format( "3d,b" ); }
    }
    comma::csv::input_stream< input > istream( std::cin, csv );
    comma::csv::output_stream < output > ostream( std::cout, output_csv );
    comma::csv::tied< input, output > tied( istream, ostream );
    std::function< void( const output& ) > do_output = [&]( const output& o ) { tied.append( o ); };
    if( discard_input ) { do_output = [&]( const output& o ) { ostream.write( o ); }; }
    auto discretise_line = [&]( const Eigen::Vector3d& begin, const Eigen::Vector3d& end, bool internal, comma::uint32 id )
    {
        const auto& line = end - begin;
        const auto& norm = line.norm();
        const auto& step = line * ( radius / norm );
        for( unsigned int k = 0; radius * k < norm; ++k ) { do_output( output( begin + step * k, internal, id ) ); }
    };
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const input* r = istream.read();
        if( !r ) { break; }
        if( output_sides )
        {
            for( unsigned int i = 0; i < 3; ++i )
            {
                discretise_line( r->triangle.corners[i], r->triangle.corners[ i == 2 ? 0 : i + 1 ], false, r->id );
            }
        }
        if( output_internal )
        {
            const auto& a = r->triangle.corners[1] - r->triangle.corners[0]; // todo? wasteful to calculate sided twice? watch performance
            const auto& b = r->triangle.corners[2] - r->triangle.corners[0]; // todo? wasteful to calculate sided twice? watch performance
            const auto& c = r->triangle.corners[2] - r->triangle.corners[1]; // todo? wasteful to calculate sided twice? watch performance
            double n = std::abs( b.dot( a.cross( a.cross( b ) ).normalized() ) ); // quick and dirty
            double bn = b.norm();
            double cn = c.norm();
            static double s = radius * std::sqrt( 3 ) / 2;
            double bs = ( bn / n ) * s;
            auto bd = b / bn * bs;
            double cs = ( cn / n ) * s;
            auto cd = c / cn * cs;
            for( unsigned int k = 1; bs * k < bn; ++k )
            {
                discretise_line( r->triangle.corners[0] + bd * k, r->triangle.corners[1] + cd * k, true, r->id );
            }
        }
    }
    return 0;
}

} // namespace discretise {
    
} } } // namespace snark { namespace points_calc { namespace triangles { 
