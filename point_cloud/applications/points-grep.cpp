// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/// @authors abdallah kassir, vsevolod vlaskine

#include <iostream>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/strategies/agnostic/point_in_poly_winding.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/io/stream.h>
#include <comma/name_value/parser.h>
#include "../../math/roll_pitch_yaw.h"
#include "../../math/rotation_matrix.h"
#include "../../math/geometry/polygon.h"
#include "../../math/geometry/polytope.h"
#include "../../math/geometry/traits.h"
#include "../../math/applications/frame.h"
#include "../../visiting/eigen.h"
#include "../../visiting/traits.h"
#include "points_grep/n_sphere.h"

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "filter points from dynamic objects represented by a position and orientation stream" << std::endl;
    std::cerr << "stdin: stream of points to filter" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat points.csv | points-grep <what> [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "<what>" << std::endl;
    std::cerr << "     box: rectangular box" << std::endl;
    std::cerr << "         options" << std::endl;
    std::cerr << "             --begin,--origin=<x>,<y>,<z>: lower left back corner of the box" << std::endl;
    std::cerr << "             --end=<x>,<y>,<z>: upper right front corner of the box" << std::endl;
    std::cerr << "             --center,--centre=<x>,<y>,<z>: centre of the box; default: 0,0,0" << std::endl;
    std::cerr << "             --inflate-by=<x>,<y>,<z>: add it to the box size, a convenience option" << std::endl;
    std::cerr << "             --size=<x>,<y>,<z>: size of the box" << std::endl;
    std::cerr << "             any two of the options above are sufficient to specify a box" << std::endl;
    std::cerr << std::endl;
    std::cerr << "     polygons: take input 2d points or line segments, filter them based upon whether they are fully contained in or fully outside a set of 2d polygons" << std::endl;
    std::cerr << "         options" << std::endl;
//     std::cerr << "             --exclude-boundary: exclude points on the boundary for both permissive and restrictive polygons; the only mode currently implemented" << std::endl;
    std::cerr << "             Due to limitations of Boost 1.53, points on boundary are not excluded from restrictive polygons, while lines are excluded." << std::endl;
    std::cerr << "             --fields" << std::endl;
    std::cerr << "                 x,y: input is points" << std::endl;
    std::cerr << "                 first,second,first/x,first/y,second/x,second/y: if any of these fields present, input is line segments" << std::endl;
    std::cerr << "                 default: x,y" << std::endl;
    std::cerr << "             --output-all,--all: instead of filtering input records, append a boolean pass/fail field for every polygon in --polygons=" << std::endl;
    std::cerr << "             --or: polygons become OR filters, output 2d point or line segment if it is allowed by any polygon in the polygon set. Default: AND filters" << std::endl;
    std::cerr << "             --polygon-fields: show polygon fields and exit, see --polygons" << std::endl;
    std::cerr << "             --polygon-format: show polygon format and exit, see --polygons" << std::endl;
    std::cerr << "             --polygons=<filename>[;<csv options>]: polygon points specified in clockwise order" << std::endl;
    std::cerr << "                 default fields: x,y" << std::endl;
    std::cerr << "                 polygons: defined by boundary points identified by id field, default id: 0, both clockwise and anti-clockwise direction accepted" << std::endl;
    std::cerr << "                 restrictive field: control wether a polygon act as fully 'contained in' or 'fully outside' filter, default: 0 for 'contained in'" << std::endl;
    std::cerr << std::endl;
    std::cerr << "     polytope,planes: arbitrary polytope that can be specified either as a set of planes or a set of plane normals and distances from 0,0,0" << std::endl;
    std::cerr << "         options" << std::endl;
    std::cerr << "             --inflate-by=<radius>: expand prism by the given value, a convenience option" << std::endl;
    std::cerr << "             --normals=<filename>[;<csv options>]: normals specifying a polytope" << std::endl;
    std::cerr << "             --normals-fields: output normals fields and exit" << std::endl;
    std::cerr << "             --normals-format: output normals format and exit" << std::endl;
    std::cerr << "             --normals-size,--number-of-planes,--number-of-normals: if normals fields given on stdin, specify the number of normals" << std::endl;
    std::cerr << "             --planes=<filename>[;<csv options>]: planes specifying a polytope" << std::endl;
    std::cerr << "             --planes-fields: output planes fields and exit" << std::endl;
    std::cerr << "             --planes-format: output planes format and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "     prism" << std::endl;
    std::cerr << "         options" << std::endl;
    std::cerr << "             --axis=<x>,<y>,<z>,<length>: prism axis vector; default: 0,0,1,0" << std::endl;
    std::cerr << "             --corners=<filename>[;<csv options>]: corners of prism base" << std::endl;
    std::cerr << "             --corners-fields: output corners fields and exit" << std::endl;
    std::cerr << "             --corners-format: output corners format and exit" << std::endl;
    std::cerr << "             --corners-size,--number-of-corners: if corners fields given on stdin, specify the number of corners" << std::endl;
    std::cerr << "             --inflate-box-by=<radius>: expand prism by the given value, a convenience option" << std::endl;
    std::cerr << std::endl;
    std::cerr << "     points: take points, output those within radius of filter points" << std::endl;
    std::cerr << "         options" << std::endl;
    std::cerr << "             --radius=[<n>]: filtering radius if not provided in fields (default: 1e-12)" << std::endl;
    std::cerr << "             --filter=<filename>[;<csv_options>]: filter points" << std::endl;
    std::cerr << "                                                  default fields: x,y,z" << std::endl;
    std::cerr << "                                                  optional fields: radius" << std::endl;
    std::cerr << "             --filter-fields: show --filter fields" << std::endl;
    std::cerr << "             --filter-format: show --filter format" << std::endl;
    std::cerr << "             --output-all,--all: output all input with matching flag(s) appended for each filter point rather than filtering input" << std::endl;
    std::cerr << "             --or: match with 'or' operation, default: 'and'" << std::endl;
    std::cerr << "             --not-matching,--negate: negate matching" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --input-fields: print input fields and exit; default input fields: x,y,z" << std::endl;
    std::cerr << "    --input-format: print input format and exit" << std::endl;
    std::cerr << "    --not-matching,--negate: output points outside of the given shape (i.e. inverted output logic)" << std::endl;
    std::cerr << "    --output-all,--all: output all input points, append 1, if point is inside the shape, 0 otherwise" << std::endl;
    std::cerr << "    --output-fields: if --output-all given, print appended output fields and exit" << std::endl;
    std::cerr << "    --output-format: if --output-all given, print appended output format and exit" << std::endl;
    std::cerr << "    --point=<x>,<y>,<z>: default point coordinates; default: 0,0,0" << std::endl;
    std::cerr << "                         useful e.g. when we need to filter a fixed point by polytope position" << std::endl;
    std::cerr << "    --position=<x>,<y>,<z>,<roll>,<pitch>,<yaw>: default filter shape position" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples: points-grep -h -v" << std::endl;
    std::cerr << std::endl;
    if( verbose ) {
    std::cerr << comma::csv::options::usage() << std::endl << std::endl;
    std::cerr << "    make a sample dataset" << std::endl;
    std::cerr << "        for i in $( seq -5 0.1 5 ) ; do for j in $( seq -5 0.1 5 ) ; do for k in $( seq -5 0.1 5 ) ; do echo $i,$j,$k ; done ; done ; done > cube.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    filter by a box; view result" << std::endl;
    std::cerr << "        cat cube.csv | points-grep box --size=1,2,3 --position=1,2,3,0.5,0.6,0.7 > filtered.csv" << std::endl;
    std::cerr << "        view-points \"cube.csv;colour=grey;hide\" \"filtered.csv;colour=red\" <( echo 0,0,0,0:0:0 ; echo 1,2,3,1:2:3 )\";colour=green;weight=10;fields=x,y,z,label\"" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    filter by two planes; view result" << std::endl;
    std::cerr << "        cat cube.csv | points-grep planes --normals <( echo 1,1,1,0.5 ; echo -1,1,1,0.5 ) > filtered.csv" << std::endl;
    std::cerr << "        view-points \"cube.csv;colour=grey;hide\" \"filtered.csv;colour=red\" <( echo 0,0,0,0:0:0 )\";colour=green;weight=10;fields=x,y,z,label\"" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    filter by a polytope; view result" << std::endl;
    std::cerr << "        cat cube.csv | points-grep planes --normals <( echo 0,0,-1,0 ; echo 0,-1,0,0 ; echo -1,0,0,0 ; echo 1,1,1,3 ) > filtered.csv" << std::endl;
    std::cerr << "        view-points \"cube.csv;colour=grey;hide\" \"filtered.csv;colour=red\" <( echo 0,0,0,0:0:0 )\";colour=green;weight=10;fields=x,y,z,label\"" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    polygons" << std::endl;
    std::cerr << "      testing for points inside of polygon" << std::endl;
    std::cerr << "        ( echo 0,5; echo 5,5; echo 5,0; echo 0,5 ) | csv-paste - value=1 >polygons.csv" << std::endl;
    std::cerr << "        ( for i in $( seq 0 0.2 6 ) ; do for j in $( seq 0 0.2 6 ) ; do echo $i,$j ; done ; done ) | points-grep polygons --polygons polygons.csv --all >results.csv" << std::endl;
    std::cerr << "        visualisation:" << std::endl;
    std::cerr << "           view-points 'results.csv;fields=x,y,scalar' 'polygons.csv;shape=loop;fields=x,y;colour=yellow'" << std::endl;
    std::cerr << "      testing for points outside of polygon" << std::endl;
    std::cerr << "        ( for i in $( seq 0 0.2 6 ) ; do for j in $( seq 0 0.2 6 ) ; do echo $i,$j ; done ; done ) | points-grep polygons --polygons <( csv-paste polygons.csv value=1 )';fields=x,y,id,restrictive' --all >results.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "      making a concave polygon" << std::endl;
    std::cerr << "        ( echo 0,5; echo 5,5; echo 0,0; echo 5,-5; echo 0,-5; echo -5,0 ) | csv-paste - value=1 >polygons.csv" << std::endl;
    std::cerr << "      testing of line totally contained in a concave polygon" << std::endl;
    std::cerr << "        ( echo -2,0,-1,2,inside; echo 1,2,1,-2,partial; echo 2,0,4,0,outside) | points-grep polygons --fields first,second --polygons polygons.csv --all | tee results.csv" << std::endl;
    std::cerr << "        visualisation:" << std::endl;
    std::cerr << "           view-points 'results.csv;fields=first/x,first/y,second/x,second/y,label,id;shape=line' 'polygons.csv;colour=grey;weight=10;fields=x,y;shape=loop'" << std::endl;
    std::cerr << "      testing of line totally fully outside the same concave polygon" << std::endl;
    std::cerr << "        ( echo -2,0,-1,2,inside; echo 1,2,1,-2,partial; echo 2,0,4,0,outside) | points-grep polygons --fields first,second --polygons <( csv-paste polygons.csv value=1 )';fields=x,y,id,restrictive' --all | tee results.csv" << std::endl;
    std::cerr << "        visualisation:" << std::endl;
    std::cerr << "           view-points 'results.csv;fields=first/x,first/y,second/x,second/y,label,id;shape=line' 'polygons.csv;colour=grey;weight=10;fields=x,y;shape=loop'" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    points" << std::endl;
    std::cerr << "        output points within radius of any filter points" << std::endl;
    std::cerr << "            cat cube.csv | points-grep points --filter <( echo 0,0,0,1; echo 1.73,1.73,1.73,2; )';fields=x,y,z,radius' --or | view-points 'cube.csv;colour=grey;hide' '-;colour=-1:3.7,cyan:magenta;weight=5'" << std::endl;
    std::cerr << "        output points within radius of all filter points" << std::endl;
    std::cerr << "            cat cube.csv | points-grep points --filter <( echo 0,0,0; echo 0.5,0.5,0.5; ) --radius 1 | view-points 'cube.csv;colour=grey;hide' '-;colour=-0.5:1,cyan:magenta;weight=5'" << std::endl;
    std::cerr << "        output points not within radius of any filter points" << std::endl;
    std::cerr << "            cat cube.csv | points-grep points --filter <( echo -3,3,-3,5; echo -3,-2,-3,3 )';fields=x,y,z,radius' --or --negate | view-points 'cube.csv;colour=grey;hide' '-;colour=-5:5,cyan:magenta;weight=2'" << std::endl;
    std::cerr << "        filter 2d points inside a circle of radius 5" << std::endl;
    std::cerr << "            cat cube.csv | grep ',0.0$' | points-grep points --filter <( echo 5,0 )';fields=x,y' --radius 5 | view-points 'cube.csv;colour=grey;hide' '-;weight=5'" << std::endl;
    std::cerr << std::endl;
    }
    exit( 0 );
}

static bool verbose;
static bool matching;

namespace snark { namespace operations {

struct flags_t
{
    flags_t( comma::uint32 size, bool state=false ) : flags( size, state ) {}
    std::vector< bool > flags;
};

} } // namespace snark { namespace operations {

namespace snark { namespace operations { namespace polygons {

struct polygon_point : public Eigen::Vector2d {
    comma::uint32 id;   // default is 0
    bool restrictive;

    polygon_point() : id(0), restrictive(false) {}
};

} } } // namespace snark { namespace operations { namespace polygons {

namespace snark { namespace operations { namespace polytopes {

struct normal
{
    Eigen::Vector3d coordinates;
    double distance;

    normal( const Eigen::Vector3d& coordinates = Eigen::Vector3d::Zero(), double distance = 0 ) : coordinates( coordinates ), distance( distance ) {}
};

struct position
{
    Eigen::Vector3d coordinates;
    snark::roll_pitch_yaw orientation;

    position() : coordinates( Eigen::Vector3d::Zero() ) {}
};

namespace species {

struct polytope
{
    struct filter : public position // quick and dirty
    {
        std::vector< normal > normals;

        filter( const position& p = position() ) : position( p ) {}
    };
};

struct prism
{
    struct filter : public position // quick and dirty
    {
        normal axis;
        std::vector< Eigen::Vector3d > corners;

        filter( const position& p = position() ) : position( p ) {}
    };
};

struct box
{
    typedef position filter;
};

} // namespace species {

template < typename S >
struct input_t : public Eigen::Vector3d // quick and dirty
{
    typename S::filter filter;

    input_t( const Eigen::Vector3d& p = Eigen::Vector3d::Zero(), const typename S::filter& filter = typename S::filter() ) : Eigen::Vector3d( p ), filter( filter ) {}
};

struct output_t
{
    bool included;

    output_t( bool included = false ) : included( included ) {}
};

} } } // namespace snark { namespace operations { namespace polytopes {

namespace snark { namespace operations { namespace points {

struct sphere
{
    Eigen::Vector3d center;
    boost::optional< double > radius;
    sphere() : center( Eigen::Vector3d::Zero() ) {}
};

} } } // namespace snark { namespace operations { namespace points {

namespace comma { namespace visiting {

template <> struct traits< snark::operations::flags_t >
{
    template < typename K, typename V > static void visit( const K& k, snark::operations::flags_t& t, V& v ) { v.apply( "flags", t.flags ); }
    template < typename K, typename V > static void visit( const K& k, const snark::operations::flags_t& t, V& v ) { v.apply( "flags", t.flags ); }
};

template <> struct traits< snark::operations::polygons::polygon_point >
{
    template < typename K, typename V > static void visit( const K& k, snark::operations::polygons::polygon_point& t, V& v )
    {
        traits< Eigen::Vector2d >::visit( k, t, v );
        v.apply( "id", t.id );
        v.apply( "restrictive", t.restrictive );
    }

    template < typename K, typename V > static void visit( const K& k, const snark::operations::polygons::polygon_point& t, V& v )
    {
        traits< Eigen::Vector2d >::visit( k, t, v );
        v.apply( "id", t.id );
        v.apply( "restrictive", t.restrictive );
    }
};

template <> struct traits< snark::operations::polytopes::position >
{
    template < typename K, typename V > static void visit( const K& k, snark::operations::polytopes::position& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "orientation", t.orientation );
    }

    template < typename K, typename V > static void visit( const K& k, const snark::operations::polytopes::position& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "orientation", t.orientation );
    }
};

template <> struct traits< snark::operations::polytopes::normal >
{
    template < typename K, typename V > static void visit( const K& k, snark::operations::polytopes::normal& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "distance", t.distance );
    }

    template < typename K, typename V > static void visit( const K& k, const snark::operations::polytopes::normal& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "distance", t.distance );
    }
};

template <> struct traits< snark::operations::polytopes::species::polytope::filter >
{
    template < typename K, typename V > static void visit( const K& k, snark::operations::polytopes::species::polytope::filter& t, V& v )
    {
        traits< snark::operations::polytopes::position >::visit( k, t, v );
        v.apply( "normals", t.normals );
    }

    template < typename K, typename V > static void visit( const K& k, const snark::operations::polytopes::species::polytope::filter& t, V& v )
    {
        traits< snark::operations::polytopes::position >::visit( k, t, v );
        v.apply( "normals", t.normals );
    }
};

template <> struct traits< snark::operations::polytopes::species::prism::filter >
{
    template < typename K, typename V > static void visit( const K& k, snark::operations::polytopes::species::prism::filter& t, V& v )
    {
        traits< snark::operations::polytopes::position >::visit( k, t, v );
        v.apply( "axis", t.axis );
        v.apply( "corners", t.corners );
    }

    template < typename K, typename V > static void visit( const K& k, const snark::operations::polytopes::species::prism::filter& t, V& v )
    {
        traits< snark::operations::polytopes::position >::visit( k, t, v );
        v.apply( "axis", t.axis );
        v.apply( "corners", t.corners );
    }
};

template < typename F > struct traits< snark::operations::polytopes::input_t< F > >
{
    template < typename K, typename V > static void visit( const K& k, snark::operations::polytopes::input_t< F >& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t, v );
        v.apply( "filter", t.filter );
    }

    template < typename K, typename V > static void visit( const K& k, const snark::operations::polytopes::input_t< F >& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t, v );
        v.apply( "filter", t.filter );
    }
};

template <> struct traits< snark::operations::polytopes::output_t >
{
    template < typename K, typename V > static void visit( const K& k, const snark::operations::polytopes::output_t& t, V& v )
    {
        v.apply( "included", t.included );
    }
};

template <> struct traits< snark::operations::points::sphere >
{
    template < typename K, typename V > static void visit( const K& k, snark::operations::points::sphere& t, V& v )
    {
        v.apply( "", t.center );
        v.apply( "radius", t.radius );
    }

    template < typename K, typename V > static void visit( const K& k, const snark::operations::points::sphere& t, V& v )
    {
        v.apply( "", t.center );
        v.apply( "radius", t.radius );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace operations { namespace polytopes {

static snark::geometry::convex_polytope transform( const snark::geometry::convex_polytope& polytope, const position& position ) { return polytope.transformed( position.coordinates, position.orientation ); }

template < typename Species, typename Genus = snark::geometry::convex_polytope > struct shape_traits;

template < typename Species > struct shape_traits< Species, snark::geometry::convex_polytope >
{
    static snark::geometry::convex_polytope make( const typename Species::filter& f, double inflate_by = 0 ) // todo: quick and dirty, watch performance
    {
        Eigen::MatrixXd normals( f.normals.size(), 3 );
        Eigen::VectorXd distances( f.normals.size() );
        for( unsigned int i = 0; i < f.normals.size(); ++i )
        {
            normals.row( i ) = f.normals[i].coordinates.normalized().transpose();
            distances( i ) = f.normals[i].distance + inflate_by;
        }
        return snark::geometry::convex_polytope( normals, distances );
    }

    static void set_default_filter( typename Species::filter& f, const comma::command_line_options& options, const comma::csv::options& csv )
    {
        bool has_normals = csv.has_some_of_fields( "filter,filter/normals" );
        if( options.exists( "--normals,--planes" ) )
        {
            if( !has_normals ) { return; }
            std::cerr << "points-grep: polytope: expected either --planes or --normals, or filter/normals in input fields, got both" << std::endl;
            exit( 1 );
        }
        if( !has_normals ) { std::cerr << "points-grep: polytope: please specify --planes, or --normals, or filter/normals in input fields" << std::endl; exit( 1 ); }
        f.normals.resize( options.value< unsigned int >( "--normals-size,--number-of-planes,--number-of-normals" ) );
    }
};

template <> struct shape_traits< species::box, snark::geometry::convex_polytope >
{
    static snark::geometry::convex_polytope make( const typename species::box::filter&, double ) { std::cerr << "points-grep: box: making box from stream: not implemented" << std::endl; exit( 1 ); }

    static void set_default_filter( typename species::box::filter& f, const comma::command_line_options& options, const comma::csv::options& csv ) { return; }
};

template <> struct shape_traits< species::prism, snark::geometry::convex_polytope >
{
    static snark::geometry::convex_polytope make( const typename species::prism::filter& f, double inflate_by = 0 )
    {
        if( f.corners.size() < 3 ) { std::cerr << "points-grep: prism: expected at least 3 corners, got: " << f.corners.size() << std::endl; exit( 1 ); }
        unsigned int size = f.corners.size() + 2;
        Eigen::MatrixXd normals( size, 3 );
        Eigen::VectorXd distances( size );
        for( unsigned int i = 1; i < f.corners.size(); ++i )
        {
            const Eigen::Vector3d& n = f.axis.coordinates.cross( f.corners[i] - f.corners[ i - 1 ] ).normalized();
            normals.row( i ) = n.transpose();
            distances( i ) = n.dot( f.corners[i] );
        }
        const Eigen::Vector3d& n = f.axis.coordinates.cross( f.corners[0] - f.corners.back() ).normalized();
        normals.row( 0 ) = n.transpose();
        distances( 0 ) = n.dot( f.corners[0] );
        const Eigen::Vector3d& m = ( ( f.corners[2] - f.corners[1] ).cross( f.corners[0] - f.corners[1] ) ).normalized();
        normals.row( f.corners.size() ) = m.transpose();
        distances( f.corners.size() ) = m.dot( f.corners[0] );
        normals.row( f.corners.size() + 1 ) = -m.transpose();
        distances( f.corners.size() + 1 ) = -m.dot( f.corners[0] + f.axis.coordinates );
        for( unsigned int i = 0; i < distances.size(); ++i ) { distances[i] += inflate_by; }
        return snark::geometry::convex_polytope( normals, distances );
    }

    static void set_default_filter( typename species::prism::filter& f, const comma::command_line_options& options, const comma::csv::options& csv )
    {
        bool has_corners = csv.has_some_of_fields( "filter,filter/corners" );
        if( options.exists( "--corners" ) )
        {
            if( !has_corners ) { return; }
            std::cerr << "points-grep: prism: expected either --corners or filter/corners in input fields, got both" << std::endl;
            exit( 1 );
        }
        if( !has_corners ) { std::cerr << "points-grep: prism: please specify --corners, or filter/corners in input fields" << std::endl; exit( 1 ); }
        f.corners.resize( options.value< unsigned int >( "--corners-size,--number-of-corners" ) );
        f.axis = comma::csv::ascii< normal >().get( options.value< std::string >( "--axis", "0,0,1,0" ) );
    }
};

template < typename Species, typename Genus > static int run( const boost::optional< Genus >& shape, const comma::command_line_options& options )
{
    comma::csv::options csv( options );
    csv.full_xpath = true;
    if( csv.fields.empty() ) { csv.fields = "x,y,z"; }
    std::vector< std::string > fields = comma::split( csv.fields, ',' );
    for( unsigned int i = 0; i < fields.size(); ++i )
    {
        if( fields[i] == "filter/x" ) { fields[i] = "filter/coordinates/x"; }
        else if( fields[i] == "filter/y" ) { fields[i] = "filter/coordinates/y"; }
        else if( fields[i] == "filter/z" ) { fields[i] = "filter/coordinates/z"; }
        else if( fields[i] == "filter/roll" ) { fields[i] = "filter/orientation/roll"; }
        else if( fields[i] == "filter/pitch" ) { fields[i] = "filter/orientation/pitch"; }
        else if( fields[i] == "filter/yaw" ) { fields[i] = "filter/orientation/yaw"; }
    }
    csv.fields = comma::join( fields, ',' );
    input_t< Species > default_input( comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--point", "0,0,0" ) )
                                    , typename Species::filter( comma::csv::ascii< position >().get( options.value< std::string >( "--position", "0,0,0,0,0,0" ) ) ) );
    shape_traits< Species, Genus >::set_default_filter( default_input.filter, options, csv );
    bool output_all = options.exists( "--output-all,--all" );
    double inflate_by = options.value( "--inflate-by", 0.0 );
    boost::optional< Genus > transformed;
    if( shape && !csv.has_some_of_fields( "filter,filter/coordinates,filter/coordinates/x,filter/coordinates/y,filter/coordinates/z,filter/orientation,filter/orientation/roll,filter/orientation/pitch,filter/orientation/yaw" ) ) { transformed = transform( *shape, default_input.filter ); }
    comma::csv::input_stream< input_t< Species > > istream( std::cin, csv, default_input );
    comma::csv::output_stream< output_t > ostream( std::cout, csv.binary() );
    comma::csv::passed< input_t< Species > > passed( istream, std::cout );
    comma::csv::tied< input_t< Species >, output_t > tied( istream, ostream );
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const input_t< Species >* p = istream.read();
        if( !p ) { break; }
        bool has = transformed ? transformed->has( *p ) : transform( shape ? *shape : shape_traits< Species, Genus >::make( p->filter, inflate_by ), p->filter ).has( *p );
        bool keep = has == matching;
        if( output_all ) { tied.append( output_t( keep ) ); } else if( keep ) { passed.write(); }
        if( csv.flush ) { std::cout.flush(); }
    }
    return 0;
}

template < typename Species, typename Genus > int run( const Genus& shape, const comma::command_line_options& options ) { return run< Species >( boost::optional< Genus >( shape ), options ); }

} } } // namespace snark { namespace operations { namespace polytopes {

namespace snark { namespace operations { namespace polygons {

typedef boost::geometry::model::d2::point_xy< double > point_t;
typedef boost::geometry::model::linestring< point_t > line_t;
typedef line_t boundary_t;

struct polygon_t
{
    polygon_t() : restrictive(false) {}
    polygon_t(const line_t& ring, bool is_restrictive) : boundary(ring), restrictive(is_restrictive) {
        boost::geometry::append( polygon, boundary );
        // if the user did not close the loop, we need to do it. Use first point as last point
        if( boundary.front().x() != boundary.back().x() || boundary.front().y() != boundary.back().y() ) { boundary.push_back( boundary.front() ); }
    }

    bool within( const point_t& g ) const { return boost::geometry::within( g, polygon ); }

    bool outside( const point_t& g ) const { return !boost::geometry::within( g, polygon );  }

    bool within( const line_t& g ) const { return boost::geometry::within( g[0], polygon ) && boost::geometry::within( g[1], polygon ) && !boost::geometry::intersects( g, boundary ); }

    bool outside( const line_t& g ) const  { return !boost::geometry::within( g[0], polygon ) && !boost::geometry::within( g[1], polygon ) && !boost::geometry::intersects( g, boundary ); }

    template < typename T > bool allowed( const T& g ) const { return restrictive ? outside( g ) : within( g ); }

    bool is_restrictive() const { return restrictive; }
private:
    boost::geometry::model::polygon< point_t, true, false > polygon;
    boost::geometry::model::linestring< point_t > boundary;
    bool restrictive;
};

// returns a list of polygons, boundary_t is its ring boundary
std::vector< polygon_t > read_polygons(comma::command_line_options& options)
{
    const std::string& polygons_file = options.value< std::string >( "--polygons", "" );
    comma::csv::options filter_csv = comma::name_value::parser( "filename" ).get< comma::csv::options >( polygons_file );
    filter_csv.full_xpath = true;
    if( filter_csv.fields.empty() ) { filter_csv.fields = "x,y"; }
    comma::io::istream is( filter_csv.filename, filter_csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii );
    comma::csv::input_stream< polygon_point > polystream( *is, filter_csv );

    bool or_option = options.exists("--or");

    std::vector< polygon_t > polygons;
    boundary_t ring;    // clockwise, or counter-clockwise, it does not seem to matter
    bool is_restrictive = false;
    comma::uint32 current_id = 0;
    while( polystream.ready() || ( is->good() && !is->eof() ) )
    {
        const polygon_point* p = polystream.read();
        if( !p ) { break; }
        if( ring.empty() || current_id == p->id ) { ring.push_back( { p->x(), p->y() } ); }
        else
        {
            polygons.emplace_back( ring, is_restrictive );
            ring.clear();
            ring.push_back( { p->x(), p->y() } );

            if( or_option && polygons.back().is_restrictive() ) { std::cerr << "points-grep: 'warning' --or option found with restrictive polygon, use permissive polygon(s) only" << std::endl; }
        }
        is_restrictive = p->restrictive;
        current_id = p->id;
    }
    if( !ring.empty() )
    {
        polygons.emplace_back( ring, is_restrictive );
        if( or_option && polygons.back().is_restrictive() ) { std::cerr << "points-grep: 'warning' --or option found with restrictive polygon, use permissive polygon(s) only" << std::endl; }
    }
    if( verbose ) { std::cerr << "points-grep: total number of polygons: " << polygons.size() << std::endl; }
    return polygons;
}

static point_t make_geometry( const Eigen::Vector2d& v ) { return { v.x(), v.y() }; }

static line_t make_geometry( const std::pair< Eigen::Vector2d, Eigen::Vector2d >& v )
{
    line_t line;
    line.push_back( make_geometry( v.first ) ); // todo: watch performance
    line.push_back( make_geometry( v.second ) ); // todo: watch performance
    return line;
    //return { make_geometry( v.first ), make_geometry( v.second ) }; // does not compile with boost.1.58
}

template < typename T > static int run( const comma::csv::options& csv, const std::vector< polygon_t >& polygons, bool output_all, bool or_mode )
{
    comma::csv::input_stream< T > istream( std::cin, csv );
    flags_t outputs( polygons.size(), false );
    comma::csv::output_stream< flags_t > ostream( std::cout, csv.binary(), true, false, outputs );
    comma::csv::tied< T, flags_t > tied( istream, ostream );
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const T* p = istream.read();
        if( !p ) { break; }
        const auto& g = make_geometry( *p );
        bool all_passed = true;
        bool all_failed = true;
        for( std::size_t i=0; i<polygons.size(); ++i )
        {
            outputs.flags[i] = polygons[i].allowed( g );
            if( or_mode ) { if ( outputs.flags[i] == true ) { all_failed=false; break; }  }
            else { if( !output_all && outputs.flags[i] == false ) { all_passed = false; break; } }
        }

        if( !output_all )
        {
            if( !or_mode && !all_passed ) { continue; } // filtered out
            if( or_mode && all_failed ) { continue; } // filtered out
            //passthrough
            if( istream.is_binary()) { std::cout.write( istream.binary().last(), istream.binary().size() ); }
            else { std::cout << comma::join( istream.ascii().last(), istream.ascii().ascii().delimiter() )<< std::endl; }
        }
        else { tied.append( outputs ); }
    }
    return 0;
}

} } } // namespace snark { namespace operations { namespace polygons {

namespace snark { namespace operations { namespace points {

static std::vector< snark::geometry::n_sphere > read_filter( const comma::command_line_options& options )
{
    const std::string& file = options.value< std::string >( "--filter", "" );
    const double radius = options.value< double >( "--radius", 1e-12 );
    comma::csv::options csv = comma::name_value::parser( "filename" ).get< comma::csv::options >( file );
    csv.full_xpath = true;
    if( csv.fields.empty() ) { csv.fields = "x,y,z"; }
    comma::io::istream is( csv.filename, csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii );
    comma::csv::input_stream< sphere > input( *is, csv );
    std::vector< snark::geometry::n_sphere > filter;
    while( input.ready() || ( is->good() && !is->eof() ) )
    {
        const sphere* s = input.read();
        if( !s ) { break; }
        if( s->radius && *s->radius <= 0 && verbose ) { std::cerr << "points-grep points: warning: filter point: " << s->center.x() << ',' << s->center.y() << ',' << s->center.z() << ": has non-positve radius: " << *s->radius << std::endl; }
        filter.push_back( snark::geometry::n_sphere( s->center, s->radius ? *s->radius : radius ) );
    }
    if( verbose ) { std::cerr << "points-grep points: filter size: " << filter.size() << std::endl; }
    return filter;
}

static int run( const comma::command_line_options& options )
{
    const bool output_all = options.exists( "--output-all,--all" );
    const bool or_mode = options.exists( "--or" );
    const bool negate = options.exists( "--not-matching,--negate" );
    const bool require_all_matches = or_mode == negate;
    const auto& filter = snark::operations::points::read_filter( options );
    if( filter.empty() ) { std::cerr << "points-grep points: warning: empty --filter" << std::endl; }
    flags_t output( filter.size(), false );
    if( ( options.exists( "--output-fields" ) || options.exists( "--output-format" ) ) && !output_all ) { std::cerr <<  "points-grep points: warning: without --output-all output is filtered input" << std::endl; return 1; }
    if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< snark::operations::flags_t >( true, output ), ',' ) << std::endl; return 0; }
    if( options.exists( "--output-format" ) ) { std::cout << comma::csv::format::value< snark::operations::flags_t >( output ) << std::endl; return 0; }
    comma::csv::options csv( options );
    csv.full_xpath = true;
    comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv );
    comma::csv::output_stream< flags_t > ostream( std::cout, csv.binary(), true, false, output );
    comma::csv::tied< Eigen::Vector3d, flags_t > tied( istream, ostream );
    while( istream.ready() || std::cin.good() )
    {
        const Eigen::Vector3d* p = istream.read();
        if( !p ) { break; }
        unsigned int matches = 0;
        for( std::size_t i = 0; i < filter.size(); ++i )
        {
            output.flags[i] = filter[i].contains( *p ) != negate;
            if( output.flags[i] ) { ++matches; if( !output_all && !require_all_matches ) { break; } }
        }
        if( output_all ) { tied.append( output ); continue; }
        if( !matches || ( require_all_matches && matches != filter.size() ) ) { continue; }
        if( istream.is_binary() ) { std::cout.write( istream.binary().last(), istream.binary().size() ); }
        else { std::cout << comma::join( istream.ascii().last(), istream.ascii().ascii().delimiter() )<< std::endl; }
    }
    return 0;
}

} } } // namespace snark { namespace operations { namespace points {

int main( int argc, char** argv )
{
    namespace species = snark::operations::polytopes::species;
    namespace polytopes = snark::operations::polytopes;
    namespace polygons = snark::operations::polygons;
    try
    {
        comma::command_line_options options( argc, argv, usage );
        verbose = options.exists( "--verbose,-v" );
        bool output_all = options.exists( "--output-all,--all" );
        matching = !options.exists( "--not-matching,--negate" );
        const std::vector< std::string >& unnamed = options.unnamed("--output-all,--all,--or,--not-matching,--negate,--verbose,-v,--flush","-.*");
        if(!unnamed.size()) { std::cerr << "points-grep: expected filter name, got nothing"<< std::endl; return 1; }
        std::string what = unnamed[0];
        if( what == "polytope" || what == "planes" || what == "prism" || what == "box" )
        {
            if( options.exists( "--input-fields" ) ) { std::cerr << comma::join( comma::csv::names< polytopes::input_t< species::polytope > >( true ), ',' ) << std::endl; return 0; }
            if( options.exists( "--input-format" ) ) { std::cerr << comma::csv::format::value< polytopes::input_t< species::polytope > >() << std::endl; return 0; }
            if( options.exists( "--output-fields" ) ) { if( output_all ) { std::cerr << comma::join( comma::csv::names< polytopes::output_t >( true ), ',' ) << std::endl; } return 0; }
            if( options.exists( "--output-format" ) ) { if( output_all ) { std::cerr << comma::csv::format::value< polytopes::output_t >() << std::endl; } return 0; }
        }

        if( what == "polytope" || what == "planes" )
        {
            if( options.exists( "--normals-fields" ) ) { std::cerr << comma::join( comma::csv::names< polytopes::normal >( true ), ',' ) << std::endl; return 0; }
            if( options.exists( "--normals-format" ) ) { std::cerr << comma::csv::format::value< polytopes::normal >() << std::endl; return 0; }
            if( options.exists( "--planes-fields" ) ) { std::cerr << comma::join( comma::csv::names< snark::triangle >( true ), ',' ) << std::endl; return 0; }
            if( options.exists( "--planes-format" ) ) { std::cerr << comma::csv::format::value< snark::triangle >() << std::endl; return 0; }

            options.assert_mutually_exclusive( "--normals,--planes" );
            const std::string& normals_input = options.value< std::string >( "--normals", "" );
            const std::string& planes_input = options.value< std::string >( "--planes", "" );
            double inflate_by = options.value( "--inflate-by", 0.0 );

            if( !normals_input.empty() )
            {
                comma::csv::options filter_csv = comma::name_value::parser( "filename" ).get< comma::csv::options >( normals_input );
                filter_csv.full_xpath = true;
                comma::io::istream is( filter_csv.filename, filter_csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii );
                comma::csv::input_stream< polytopes::normal > istream( *is, filter_csv );
                snark::operations::polytopes::species::polytope::filter f;
                while( istream.ready() || ( is->good() && !is->eof() ) )
                {
                    const polytopes::normal* p = istream.read();
                    if( !p ) { break; }
                    f.normals.push_back( *p );
                }
                return polytopes::run< species::polytope >( polytopes::shape_traits< species::polytope >::make( f, inflate_by ), options );
            }
            else if( !planes_input.empty() )
            {
                comma::csv::options filter_csv = comma::name_value::parser( "filename" ).get< comma::csv::options >( planes_input );
                filter_csv.full_xpath = true;
                comma::io::istream is( filter_csv.filename, filter_csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii );
                comma::csv::input_stream< snark::triangle > istream( *is, filter_csv );
                std::vector< snark::triangle > planes;
                while( istream.ready() || ( is->good() && !is->eof() ) )
                {
                    const snark::triangle* p = istream.read();
                    if( !p ) { break; }
                    planes.push_back( *p );
                }
                Eigen::MatrixXd normals( planes.size(), 3 );
                Eigen::VectorXd distances( planes.size() );
                for( unsigned int i = 0; i < planes.size(); ++i )
                {
                    const Eigen::Vector3d& normal = ( planes[i].corners[1] - planes[i].corners[0] ).cross( planes[i].corners[2] - planes[i].corners[0] ).normalized();
                    normals.row( i ) = normal;
                    distances[i] = normal.dot( planes[i].corners[0] + normal * inflate_by );
                }
                return snark::operations::polytopes::run< species::polytope >( snark::geometry::convex_polytope( normals, distances ), options );
            }
            return snark::operations::polytopes::run< species::polytope >( boost::optional< snark::geometry::convex_polytope >(), options );
        }
        else if( what == "polygons" )
        {
            if( options.exists( "--input-fields" ) ) { std::cerr << "points-grep polygons: --input-fields not implemented, since it is ambiguous; see --help instead" << std::endl; return 1; }
            if( options.exists( "--output-fields" ) ) { std::cerr << "points-grep polygons: --output-fields not implemented, since it is hard to define; see --help instead" << std::endl; return 1; }
            if( options.exists( "--output-format" ) ) { std::cerr << "points-grep polygons: --output-format not implemented, since it is hard to define; see --help instead" << std::endl; return 1; }
            if( options.exists("--polygon-fields") ) { std::cout << comma::join( comma::csv::names< polygons::polygon_point >(), ',' ) << std::endl; return 0; }
            if( options.exists("--polygon-format") ) { std::cout << comma::csv::format::value< polygons::polygon_point >() << std::endl; return 0; }
            options.assert_mutually_exclusive("--or,--output-all,--all");
//             if( !options.exists( "--exclude-boundary" ) ) { std::cout << "points-grep polygons: please specify --exclude-boundary (the only mode currently implemented)" << std::endl; return 1; }
            const auto& polygons = snark::operations::polygons::read_polygons(options);
            if( polygons.empty() ) { std::cerr << "points-grep: please specify at least one polygon in --polygons=" << std::endl; return 1; }
            comma::csv::options csv(options);
            csv.full_xpath = true;
            bool is_line_mode = csv.has_some_of_fields( "first,second,first/x,first/y,second/x,second/y" );
            return is_line_mode
                 ? snark::operations::polygons::run< std::pair< Eigen::Vector2d, Eigen::Vector2d > >( csv, polygons, options.exists("--output-all,--all"), options.exists("--or") )
                 : snark::operations::polygons::run< Eigen::Vector2d >( csv, polygons, options.exists("--output-all,--all"), options.exists("--or") );
        }
        else if( what == "prism" )
        {
            if( options.exists( "--corners-fields" ) ) { std::cerr << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl; return 0; }
            if( options.exists( "--corners-format" ) ) { std::cerr << comma::csv::format::value< Eigen::Vector3d >() << std::endl; return 0; }

            const std::string& corners_input = options.value< std::string >( "--corners", "" );
            if( corners_input.empty() ) { return snark::operations::polytopes::run< species::prism >( boost::optional< snark::geometry::convex_polytope >(), options ); }
            comma::csv::options filter_csv = comma::name_value::parser( "filename" ).get< comma::csv::options >( corners_input );
            comma::io::istream is( filter_csv.filename, filter_csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii );
            comma::csv::input_stream< Eigen::Vector3d > istream( *is, filter_csv );
            species::prism::filter f;
            while( istream.ready() || ( is->good() && !is->eof() ) )
            {
                const Eigen::Vector3d* p = istream.read();
                if( !p ) { break; }
                f.corners.push_back( *p );
            }
            f.axis = comma::csv::ascii< snark::operations::polytopes::normal >().get( options.value< std::string >( "--axis", "0,0,1,0" ) );
            return snark::operations::polytopes::run< species::prism >( snark::operations::polytopes::shape_traits< species::prism >::make( f, options.value( "--inflate-by", 0.0 ) ), options );
        }
        else if( what == "box" )
        {
            boost::optional< Eigen::Vector3d > origin;
            boost::optional< Eigen::Vector3d > end;
            boost::optional< Eigen::Vector3d > size;
            Eigen::Vector3d centre = comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--center,--centre", "0,0,0" ) );
            if( options.exists( "--origin,--begin" ) ) { origin = comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--origin,--begin" ) ); }
            if( options.exists( "--end" ) ) { end = comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--end" ) ); }
            if( options.exists( "--size" ) ) { size = comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--size" ) ); }
            if( !origin )
            {
                if( end && size ) { origin = *end - *size; }
                else if( size ) { origin = centre - *size / 2; }
                else { std::cerr << "points-grep: box: got --origin; please specify --end or --size" << std::endl; return 1; }
            }
            if( !end )
            {
                if( origin && size ) { end = *origin + *size; }
                else if( size ) { end = centre + *size / 2; }
                else { std::cerr << "points-grep: box: got --end; please specify --origin or --size" << std::endl; return 1; }
            }
            centre = ( *origin + *end ) / 2 ;
            Eigen::Vector3d inflate_by = comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--inflate-box-by", "0,0,0" ) );
            Eigen::Vector3d radius = ( *end - *origin ) / 2 + inflate_by;
            Eigen::MatrixXd normals( 6, 3 );
            normals <<  0,  0,  1,
                        0,  0, -1,
                        0,  1,  0,
                        0, -1,  0,
                        1,  0,  0,
                       -1,  0,  0;
            Eigen::VectorXd distances( 6 );
            distances << radius.x(), radius.x(), radius.y(), radius.y(), radius.z(), radius.z();
            return snark::operations::polytopes::run< species::box >( snark::geometry::convex_polytope( normals, distances ), options );
        }
        else if( what == "points" )
        {
            if( options.exists( "--input-fields" ) ) { std::cerr << comma::join( comma::csv::names< Eigen::Vector3d >(), ',' ) << std::endl; return 0; }
            if( options.exists( "--input-format" ) ) { std::cerr << comma::csv::format::value< Eigen::Vector3d >() << std::endl; return 0; }
            if( options.exists( "--filter-fields" ) ) { std::cout << "x,y,z" << std::endl; return 0; }
            if( options.exists( "--filter-format" ) ) { std::cout << "d,d,d" << std::endl; return 0; }
            options.assert_mutually_exclusive("--or,--output-all,--all");
            return snark::operations::points::run( options );
        }
        std::cerr << "points-grep: expected filter name, got: \"" << what << "\"" << std::endl;
        return 1;
    }
    catch( std::exception& ex ) { std::cerr << "points-grep: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-grep: unknown exception" << std::endl; }
    return 1;
}
