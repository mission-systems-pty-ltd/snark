// Copyright (c) 2011 The University of Sydney
// Copyright (c) 2020 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <cmath>
#include <deque>
#include <functional>
#include <iostream>
#include <type_traits>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <boost/optional.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <comma/math/compare.h>
#include "../../math/rotation_matrix.h"
#include "../../math/traits.h"
#include "../voxel_grid.h"
#include "../voxel_map.h"
#include "../../visiting/eigen.h"
#include "../../visiting/traits.h"
#include "points-calc/frame.h"
#include "points-calc/life.h"
#include "points-calc/lines_nearest.h"
#include "points-calc/plane_intersection.h"
#include "points-calc/plane_intersection_with_trajectory.h"
#include "points-calc/project.h"
#include "points-calc/remove_outliers.h"
#include "points-calc/thin.h"
#include "points-calc/triangles.h"
#include "points-calc/vector_calc.h"
#include "points-calc/visit.h"

static comma::csv::options csv;
static bool verbose;
std::string app_name="points-calc";
typedef std::pair< Eigen::Vector3d, Eigen::Vector3d > point_pair_t;
static comma::csv::ascii< Eigen::Vector3d > ascii;

struct point_with_block
{
    Eigen::Vector3d coordinates;
    comma::uint32 block;
    point_with_block() : coordinates( Eigen::Vector3d::Zero() ), block( 0 ) {}
};

struct point_with_direction : public Eigen::Vector3d // todo: a bit of trash bin... decouple trajectory-partition operation into a set of operations?
{
    Eigen::Vector3d direction;
    point_with_direction( const Eigen::Vector3d v = Eigen::Vector3d::Zero(), const Eigen::Vector3d d = Eigen::Vector3d::Zero() ): Eigen::Vector3d( v ), direction( d ) {}
};

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "take coordinates from stdin, perform calculations, and output result to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage" << std::endl;
    std::cerr << "    points-calc <operation> [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage examples" << std::endl;
    std::cerr << "    cat points.csv | points-calc distance > results.csv" << std::endl;
    std::cerr << "    echo -e \"0\\n1\\n3\\n6\" | points-calc distance --fields x --next" << std::endl;
    std::cerr << "    cat points.csv | points-calc trajectory-cumulative-distance > results.csv" << std::endl;
    std::cerr << "    cat points.csv | points-calc trajectory-discretise --step <step> > results.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations" << std::endl;
    std::cerr << "    angle-axis" << std::endl;
    std::cerr << "    distance" << std::endl;
    std::cerr << "    discretise,discretize" << std::endl;
    std::cerr << "    find-outliers" << std::endl;
    std::cerr << "    frame-integrate, integrate-frame (deprecated)" << std::endl;
    std::cerr << "    life" << std::endl;
    std::cerr << "    lines-nearest" << std::endl;
    std::cerr << "    local-min" << std::endl;
    std::cerr << "    local-max" << std::endl;
    std::cerr << "    nearest-min,min-in-radius,min-in-range" << std::endl;
    std::cerr << "    nearest-max,max-in-radius,max-in-range" << std::endl;
    std::cerr << "    nearest-point,nearest-any" << std::endl;
    std::cerr << "    nearest" << std::endl;
    std::cerr << "    percentile-in-radius,percentile-in-range,percentile" << std::endl;
    std::cerr << "    plane-intersection" << std::endl;
    std::cerr << "    plane-intersection-with-trajectory" << std::endl;
    std::cerr << "    project-onto-line" << std::endl;
    std::cerr << "    project-onto-plane" << std::endl;
    std::cerr << "    thin" << std::endl;
    std::cerr << "    trajectory-chord,chord" << std::endl;
    std::cerr << "    trajectory-cumulative-distance,cumulative-distance" << std::endl;
    std::cerr << "    trajectory-cumulative-discretise,trajectory-cumulative-discretize,cumulative-discretise,cumulative-discretize,sample" << std::endl;
    std::cerr << "    trajectory-discretize,trajectory-discretise,discretize,discretise" << std::endl;
    std::cerr << "    trajectory-partition, trajectory-partitions (deprecated)" << std::endl;
    std::cerr << "    trajectory-thin" << std::endl;
    std::cerr << "    triangles-area" << std::endl;
    std::cerr << "    triangles-discretise,triangles-discretize" << std::endl;
    std::cerr << "    visit" << std::endl;
    vector_calc::usage_list_operations();
    std::cerr << std::endl;
    std::cerr << "general options" << std::endl;
    std::cerr << "    usage: points-calc <option>" << std::endl;
    std::cerr << "    --help,-h: show help, --verbose for more detailed help" << std::endl;
    std::cerr << "    --verbose,-v: show detailed messages" << std::endl;
    std::cerr << std::endl;
    std::cerr << "common operation options" << std::endl;
    std::cerr << "    usage: points-calc <operation> <option>" << std::endl;
    std::cerr << "    --input-fields: print input field names and exit" << std::endl;
    std::cerr << "    --input-format: print input format and exit" << std::endl;
    std::cerr << "    --output-fields: print output field names and exit" << std::endl;
    std::cerr << "    --output-format: print output format and exit" << std::endl;
    std::cerr << std::endl;
    if( verbose ) { std::cerr << "csv options:" << std::endl << comma::csv::options::usage() << std::endl; }
    std::cerr << "operation details" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    angle-axis" << std::endl;
    std::cerr << "        angle-axis at the origin between subsequent points, or if input is pairs, between" << std::endl;
    std::cerr << "        the points of the same record. For subsequent points the first value" << std::endl;
    std::cerr << "        (or last if --next is given) is invalid as we don't yet have a pair of" << std::endl;
    std::cerr << "        points." << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << "                      " << comma::join( comma::csv::names< point_pair_t >( true ), ',' ) << " ( not with --relative )" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options: " << std::endl;
    std::cerr << "            --ignore-repeats: used with --relative, ignore repeating points" << std::endl;
    std::cerr << "            --next: subsequent points only and mutually exclusive with --relative" << std::endl;
    std::cerr << "                    angle-axis to next point is appended" << std::endl;
    std::cerr << "                    (default: angle-axis to previous point is appended)" << std::endl;
    std::cerr << "            --relative: at each point, calculate the angle-axis between the previous" << std::endl;
    std::cerr << "                     and next point. First and last are assigned 0 angle as they have" << std::endl;
    std::cerr << "                     no previous and next point respectively" << std::endl;
    std::cerr << "                    (default: angle-axis is at the origin)" << std::endl;
    std::cerr << "            --reverse,--supplementary: used with --relative, output the supplementary" << std::endl;
    std::cerr << "                    angle to the result of --relative operation" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    distance" << std::endl;
    std::cerr << "        distance between subsequent points or, if input is pairs, between the" << std::endl;
    std::cerr << "        points of the same record" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << "                      " << comma::join( comma::csv::names< point_pair_t >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options: " << std::endl;
    std::cerr << "            --next: for subsequent points only, distance to next point" << std::endl;
    std::cerr << "                    is appended" << std::endl;
    std::cerr << "                    (default: distance to previous point is appended)" << std::endl;
    std::cerr << "                    fake zero is appended to the final point (since there is" << std::endl;
    std::cerr << "                    no next point)" << std::endl;
    std::cerr << "            --propagate: if the current point is the same as previous," << std::endl;
    std::cerr << "                    output distance from one step before" << std::endl;
    std::cerr << "                    e.g: ( echo 0,0,0 ; echo 0,0,1 ; echo 0,0,1 ) \\" << std::endl;
    std::cerr << "                             | points-calc distance --propagate" << std::endl;
    std::cerr << "                         0,0,0,0" << std::endl;
    std::cerr << "                         0,0,1,1" << std::endl;
    std::cerr << "                         0,0,1,1" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    find-outliers" << std::endl;
    std::cerr << "        find points in low density areas currently quick and dirty, may have" << std::endl;
    std::cerr << "        aliasing problems unless --no-antialiasing defined, will not remove" << std::endl;
    std::cerr << "        points on the border of a denser cloud" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: " << comma::join( comma::csv::names< remove_outliers::point >( false ), ',' ) << "; default: x,y,z"<<std::endl;
    std::cerr << std::endl;
    std::cerr << "        output: input line with appended 0 for outliers, 1 for non-outliers" << std::endl;
    std::cerr << "                (unless --discard is specified)" << std::endl;
    std::cerr << "                binary output: flag as unsigned byte (ub)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --resolution=<resolution>: size of the voxel to remove outliers" << std::endl;
    std::cerr << "            --min-number-of-points-per-voxel,--size=<number>:" << std::endl;
    std::cerr << "                    min number of points for a voxel to keep" << std::endl;
    std::cerr << "            --no-antialiasing: don't check neighbour voxels, which is faster," << std::endl;
    std::cerr << "                    but may remove points in borderline voxels" << std::endl;
    std::cerr << "            --discard: discards records that are outliers; output: same as input" << std::endl;
    std::cerr << std::endl;
    std::cerr << snark::points_calc::frame::integrate::traits::usage() << std::endl;
    std::cerr << snark::points_calc::life::traits::usage() << std::endl;
    std::cerr << snark::points_calc::lines_nearest::traits::usage() << std::endl;
    std::cerr << "    local-min, local-max: deprecated, use nearest-min, nearest-max;" << std::endl;
    std::cerr << "        output local minimums or maximums inside of given radius" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: x,y,z,id,scalar; default x,y,z,scalar" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        output fields: <input line>,reference_id,distance," << std::endl;
    std::cerr << "                where reference_id is nearest point id" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --radius=<metres>: radius of the local region to search" << std::endl;
    std::cerr << "            --trace: if a points's reference point is not local extremum," << std::endl;
    std::cerr << "                     replace it with its reference (todo: debug)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    nearest-point, nearest-any" << std::endl;
    std::cerr << "        for each point, output the single nearest point id (if any) inside the" << std::endl;
    std::cerr << "        given radius" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: x,y,z,id; default x,y,z" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        output fields: <input line>,reference_id,distance," << std::endl;
    std::cerr << "                       where reference_id is nearest point id" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --output-full-record,--full-record,--full:" << std::endl;
    std::cerr << "                    instead of reference_id and distance, output full extremum" << std::endl;
    std::cerr << "                    record, discard points with no extremum nearby" << std::endl;
    std::cerr << "            --radius=<metres>: radius of the local region to search" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    nearest-min, min-in-radius, min-in-range" << std::endl;
    std::cerr << "    nearest-max, max-in-radius, max-in-range" << std::endl;
    std::cerr << "        for each point, output nearest minimums or maximums inside of given" << std::endl;
    std::cerr << "        radius" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: x,y,z,scalar,id; default: x,y,z,id" << std::endl;
//     std::cerr << "            if mask field is specified it only considers points with a mask value of 1, ignoring points with mask value of 0"<< std::endl;
    std::cerr << std::endl;
    std::cerr << "        output fields: <input line>,reference_id,distance," << std::endl;
    std::cerr << "                       where reference_id is nearest min or max id" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --fast: consider the enclosing and adjacent voxels as the search region" << std::endl;
    std::cerr << "            --output-extremums-only,--output-extremums,--extremums:" << std::endl;
    std::cerr << "                    output only the extremum records associated with points" << std::endl;
    std::cerr << "            --output-full-record,--full-record,--full:" << std::endl;
    std::cerr << "                    instead of reference_id and distance, output full extremum" << std::endl;
    std::cerr << "                    record, discard points with no extremum nearby" << std::endl;
    std::cerr << "            --radius=<metres>: radius of the local region to search" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        example: get local height maxima in the radius of 5 metres:" << std::endl;
    std::cerr << "            cat xyz.csv | points-calc local-max --fields=x,y,scalar --radius=5" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    percentile-in-radius, percentile-in-range, percentile" << std::endl;
    std::cerr << "        for each point, append the point at the given percentile in a given radius," << std::endl;
    std::cerr << "        based on scalar field" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: x,y,z,scalar; default: x,y,z,scalar" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        output fields: <input line>,<reference_line>" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --fast: consider the enclosing and adjacent voxels as the search region" << std::endl;
    std::cerr << "            --min-count=[<count>]: output only points which has atleast this number of neighbours" << std::endl;
    std::cerr << "            --percentile=<percentile>: percentile value in [0,1]" << std::endl;
    std::cerr << "            --radius=<metres>: radius of the local region to search" << std::endl;
    std::cerr << "            --output-percentile-record-only,--output-percentile: output only the percentile record once," << std::endl;
    std::cerr << "                                                                 even if there are several input points records" << std::endl;
    std::cerr << "                                                                 referring to the same percentile record" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        example:" << std::endl;
    std::cerr << "            cat xyz.csv | points-calc percentile-in-radius --fields=x,y,scalar --percentile=0.3 --radius=1" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    nearest" << std::endl;
    std::cerr << "        find point nearest to a given point" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options:" << std::endl;
    std::cerr << "            --point,--to=<x>,<y>,<z>" << std::endl;
    std::cerr << std::endl;
    std::cerr << snark::points_calc::project::onto_line::traits::usage() << std::endl;
    std::cerr << snark::points_calc::project::onto_plane::traits::usage() << std::endl;
    std::cerr << snark::points_calc::plane_intersection::traits::usage() << std::endl;
    std::cerr << snark::points_calc::plane_intersection_with_trajectory::traits::usage() << std::endl;
    std::cerr << snark::points_calc::thin::traits::usage() << std::endl;
    std::cerr << snark::points_calc::triangles::area::traits::usage() << std::endl;
    std::cerr << snark::points_calc::triangles::discretise::traits::usage() << std::endl;
    std::cerr << "    trajectory calculations" << std::endl;
    std::cerr << "        the following operations perform calculations on trajectories." << std::endl;
    std::cerr << "        a trajectory is an ordered sequence of points" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        trajectory-chord, chord" << std::endl;
    std::cerr << "            determine chords along a trajectory" << std::endl;
    std::cerr << std::endl;
    std::cerr << "            input fields: x,y,z; default x,y,z" << std::endl;
    std::cerr << "                    if id is not given it is generated" << std::endl;
    std::cerr << std::endl;
    std::cerr << "            output fields: <input line>,<chord line>" << std::endl;
    std::cerr << "                    where <chord line> is the input line that is the opposite" << std::endl;
    std::cerr << "                                       end of the chord" << std::endl;
    std::cerr << "              -or- <input line> for --filter option" << std::endl;
    std::cerr << std::endl;
    std::cerr << "            options:" << std::endl;
    std::cerr << "                --arc-ratio=<n>: ratio of the length of the arc to the chord" << std::endl;
    std::cerr << "                                 the furthest point that does not exceed this" << std::endl;
    std::cerr << "                                 ratio is chosen" << std::endl;
    std::cerr << "                --arc-maximum,--arc-max=<metres>: maximum distance along the" << std::endl;
    std::cerr << "                                 arc to next point" << std::endl;
    std::cerr << "                --filter: output only connected points, starting with first" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        trajectory-cumulative-distance, cumulative-distance" << std::endl;
    std::cerr << "            cumulative distance between subsequent points" << std::endl;
    std::cerr << "            if 'block' field present, calculate distance block-wise" << std::endl;
    std::cerr << std::endl;
    std::cerr << "            options" << std::endl;
    std::cerr << "                --differential,--diff: if present, input point represents" << std::endl;
    std::cerr << "                                 difference with the previous point," << std::endl;
    std::cerr << "                                 not the absolute coordinates" << std::endl;
    std::cerr << std::endl;
    std::cerr << "            input fields: " << comma::join( comma::csv::names< point_with_block >( false ), ',' ) << std::endl;
    std::cerr << "            default input fields: " << comma::join( comma::csv::names< Eigen::Vector3d >( false ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "        trajectory-cumulative-discretise, trajectory-cumulative-discretize, cumulative-discretise, cumulative-discretize, sample" << std::endl;
    std::cerr << "            read input data and discretise intervals with --step along the whole" << std::endl;
    std::cerr << "            trajectory" << std::endl;
    std::cerr << std::endl;
    std::cerr << "            input fields: " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "            options" << std::endl;
    std::cerr << "                --step=<step>: linear step of discretisation" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        trajectory-discretise, trajectory-discretize, discretise, discretize" << std::endl;
    std::cerr << "            read input data and discretise intervals between adjacent points" << std::endl;
    std::cerr << "            with --step; skip discretised points that are closer to the end of" << std::endl;
    std::cerr << "            the interval than --tolerance" << std::endl;
    std::cerr << std::endl;
    std::cerr << "            input fields: " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "            options" << std::endl;
    std::cerr << "                --step=<step>: linear step of discretisation" << std::endl;
    std::cerr << "                --tolerance=<tolerance>: tolerance; default: " << std::numeric_limits< double >::min() << std::endl;
    std::cerr << std::endl;
    std::cerr << "        trajectory-distance" << std::endl;
    std::cerr << "            alias for distance operation" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        trajectory-partition" << std::endl;
    std::cerr << "            read input, append partition ids based on a rule or heuristic (see options below)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "            input fields: " << comma::join( comma::csv::names< point_with_direction >( true ), ',' ) << std::endl;
    std::cerr << "                          default: x,y,z" << std::endl;
    std::cerr << std::endl;
    std::cerr << "            options" << std::endl;
    std::cerr << "                --how=<how>; default=by-direction; how to partition" << std::endl;
    std::cerr << "                    by-direction,direction; input is trajectory points and direction vectors for them" << std::endl;
    std::cerr << "                                            take first point, for each next point, if threshold exceeded, start new partition" << std::endl;
    std::cerr << "                        options" << std::endl;
    std::cerr << "                            --threshold,--angle-threshold=<radians>; default=1.57079632679; whenever angle is greater than threshold, start new partition" << std::endl;
    std::cerr << "                            --tolerance,--distance-tolerance,--distance-threshold=<meters>; if present, even if angle threshold reached, do not start new partition until trajectory moves farther than a given distance" << std::endl;
    std::cerr << "                --input-fields; print input fields and exit" << std::endl;
    std::cerr << "                --input-format; print input format and exit" << std::endl;
    std::cerr << "                --output-fields; print output fields and exit" << std::endl;
    std::cerr << "                --output-format; print output format and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        trajectory-thin (replaces deprecated thin --linear)" << std::endl;
    std::cerr << "            read points of a trajectory and thin them down" << std::endl;
    std::cerr << "            input fields: " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << "            options:" << std::endl;
    std::cerr << "                --do-not-accumulate: do not accumulate distance travelled from the last point, just measure distance to the last point directly" << std::endl;
    std::cerr << "                --resolution=<distance>: minimum distance between points" << std::endl;
    std::cerr << std::endl;
    std::cerr << snark::points_calc::triangles::area::traits::usage() << std::endl;
    std::cerr << std::endl;
    std::cerr << snark::points_calc::triangles::discretise::traits::usage() << std::endl;
    std::cerr << std::endl;
    std::cerr << snark::points_calc::visit::traits::usage() << std::endl;
    std::cerr << std::endl;
    vector_calc::usage();
    exit( 0 );
}

static int calculate_distance( bool cumulative, bool propagate = false, bool differential = false )
{
    if( csv.fields.empty() ) { csv.fields = "x,y,z"; }
    csv.full_xpath = false;
    comma::csv::input_stream< point_with_block > istream( std::cin, csv );
    boost::optional< point_with_block > last;
    double distance = 0;
    double previous_norm = 0;
    if( cumulative && propagate ) { std::cerr << "points-calc: cumulative distance and propagate are mutually exclusive" << std::endl; exit( 1 ); }
    if( !cumulative && differential ) { std::cerr << "points-calc: differential distance calculation is implemented only for trajectory-cumulative-distance" << std::endl; exit( 1 ); }
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const point_with_block* p = istream.read();
        if( !p ) { break; }
        double norm = differential ? p->coordinates.norm() : last ? ( p->coordinates - last->coordinates ).norm() : 0;
        if( propagate ) { if( comma::math::equal( norm, 0 ) ) { norm = previous_norm; } else { previous_norm = norm; } }
        distance = cumulative ? ( !last || p->block == last->block ? distance + norm : 0 ) : norm;
        last = *p;
        if( csv.binary() )
        {
            std::cout.write( istream.binary().last(), istream.binary().binary().format().size() );
            std::cout.write( reinterpret_cast< const char* >( &distance ), sizeof( double ) );
            if( csv.flush ) { std::cout.flush(); }
        }
        else
        {
            std::cout << comma::join( istream.ascii().last(), csv.delimiter ) << csv.delimiter << distance << std::endl;
        }
    }
    return 0;
}

static int calculate_distance_next( bool propagate )
{
    comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv, Eigen::Vector3d::Zero() );
    boost::optional< Eigen::Vector3d > last;
    double previous_norm = 0;
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const Eigen::Vector3d* p = istream.read();
        if( !p ) { break; }
        if( last )
        {
            double distance = ( *p - *last ).norm();
            if( propagate ) { if( comma::math::equal( distance, 0 ) ) { distance = previous_norm; } else { previous_norm = distance; } }
            if( csv.binary() )
            {
                std::cout.write( reinterpret_cast< const char* >( &distance ), sizeof( double ) );
                if( csv.flush ) { std::cout.flush(); }
            }
            else { std::cout << csv.delimiter << distance << std::endl; }
        }
        if( csv.binary() )
        {
            std::cout.write( istream.binary().last(), istream.binary().binary().format().size() );
            if( csv.flush ) { std::cout.flush(); }
        }
        else { std::cout << comma::join( istream.ascii().last(), csv.delimiter ); }
        last = *p;
    }
    if( last )
    {
        double fake_final_distance = 0;
        if( csv.binary() )
        {
            std::cout.write( reinterpret_cast< const char* >( &fake_final_distance ), sizeof( double ) );
            if( csv.flush ) { std::cout.flush(); }
        }
        else { std::cout << csv.delimiter << fake_final_distance << std::endl; }
    }
    return 0;
}

static int calculate_distance_for_pairs( bool propagate )
{
    comma::csv::input_stream< point_pair_t > istream( std::cin, csv, point_pair_t( Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() ) );
    double previous_norm = 0;
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const point_pair_t* p = istream.read();
        if( !p ) { break; }
        double norm = ( p->first - p->second ).norm();
        if( propagate ) { if( comma::math::equal( norm, 0 ) ) { norm = previous_norm; } else { previous_norm = norm; } }
        if( csv.binary() )
        {
            std::cout.write( istream.binary().last(), istream.binary().binary().format().size() );
            std::cout.write( reinterpret_cast< const char* >( &norm ), sizeof( double ) );
            if( csv.flush ) { std::cout.flush(); }
        }
        else
        {
            std::cout << comma::join( istream.ascii().last(), csv.delimiter ) << csv.delimiter << norm << std::endl;
        }
    }
    return 0;
}

namespace percentile_in_radius {

struct input_t
{
    Eigen::Vector3d coordinates;
    double scalar;
    comma::uint32 block;

    input_t() : coordinates( Eigen::Vector3d::Zero() ), scalar( 0.0 ), block( 0U ) {}
};

struct record_t
{
    input_t input;
    std::string line;
    record_t* reference_record;

    record_t( input_t const& in, comma::csv::input_stream< input_t > const& istrm )
        : input( in )
        , line()
        , reference_record( nullptr )
    {
        if( csv.binary() )
        {
            line.resize( csv.format().size() );
            ::memcpy( &line[ 0 ], istrm.binary().last(), csv.format().size() );
        }
        else
        {
            line = comma::join( istrm.ascii().last(), csv.delimiter );
        }
    }

    void output( bool const percentile_only = false ) const
    {
        if( reference_record )
        {
            if( csv.binary() )
            {
                if( !percentile_only ) { std::cout.write( &line[0], line.size() ); }
                std::cout.write( &reference_record->line[0], reference_record->line.size() );
                std::cout.flush();
            }
            else
            {
                if( !percentile_only ) { std::cout << line << csv.delimiter; }
                std::cout << reference_record->line << std::endl;
            }
        }
    }
};

struct map_value
{
    record_t* record;
    comma::uint32 in_range;

    map_value( record_t* rec = nullptr, comma::uint16 in_range = 1U ): record( rec ), in_range( 1 ) {}
};

static void process( std::deque< record_t >& que
                                          , snark::math::closed_interval< double, 3 > const& extents
                                          , Eigen::Vector3d const&  resolution
                                          , double const percentile
                                          , double const radius
                                          , size_t const min_count
                                          , bool const fast )
{
    if( verbose ) { std::cerr << "points-calc: loading " << que.size() << " points into grid..." << std::endl; }

    typedef std::multimap< double, record_t* > voxel_t;
    typedef snark::voxel_map< voxel_t, 3 > grid_t;

    grid_t grid( extents.min(), resolution );

    for( std::size_t reci = 0; reci < que.size(); reci++ )
    {
        ( grid.touch_at( que[ reci ].input.coordinates ) )->second.emplace( std::make_pair( que[ reci ].input.scalar, &que[ reci ] ) );
    }
    if( verbose ) { std::cerr << "points-calc: grid size " << grid.size() << std::endl; }

    for( grid_t::iterator vi = grid.begin(); vi != grid.end(); ++vi )
    {
        std::multimap< double, map_value > points_in_range;
        grid_t::index_type adj_vx;

        for( adj_vx[0] = vi->first[0] - 1; adj_vx[0] < vi->first[0] + 2; ++adj_vx[0] )
        {
            for( adj_vx[1] = vi->first[1] - 1; adj_vx[1] < vi->first[1] + 2; ++adj_vx[1] )
            {
                for( adj_vx[2] = vi->first[2] - 1; adj_vx[2] < vi->first[2] + 2; ++adj_vx[2] )
                {

                    grid_t::iterator adj_vi = grid.find( adj_vx );
                    if( adj_vi == grid.end() ) { continue; }

                    for( auto adj_pi = adj_vi->second.cbegin(); adj_pi != adj_vi->second.cend(); adj_pi++ )
                    {
                        points_in_range.emplace( std::make_pair( adj_pi->second->input.scalar, map_value( adj_pi->second ) ) );
                    }
                }
            }
        }
        if( fast )
        {
            if( min_count <= points_in_range.size() )
            {
                auto percentile_index = ( size_t )std::ceil( percentile * points_in_range.size() ) - 1;
                auto percentilei = points_in_range.cbegin();
                auto percentilex = size_t( 0U );
                for( ; percentilex < percentile_index; percentilex++, percentilei++ );

                for( auto pi = vi->second.begin(); pi != vi->second.end(); pi++ ) { pi->second->reference_record = percentilei->second.record; }
            }
        }
        else
        {
            for( auto pi = vi->second.begin(); pi != vi->second.end(); pi++ )
            {
                size_t range_size = 0;
                for( auto adj_pi = points_in_range.begin(); adj_pi != points_in_range.end(); adj_pi++ )
                {
                    if(( pi->second->input.coordinates - adj_pi->second.record->input.coordinates ).norm() <= radius )
                    {
                        adj_pi->second.in_range =  1U;
                        range_size++;
                    }
                    else { adj_pi->second.in_range =  0U; }
                }
                if( min_count <= range_size )
                {
                    auto percentile_index = ( size_t )std::ceil( percentile * range_size ) - 1;
                    auto percentilei = points_in_range.cbegin();
                    auto percentilex = size_t( 0U );

                    while(( percentilex += percentilei->second.in_range ) <= percentile_index ) { percentilei++; }
                    pi->second->reference_record = percentilei->second.record;
                }
            }
        }
    }
}

static void output( std::deque< record_t > const& que, bool const percentile_only )
{
    if( percentile_only )
    {
        std::unordered_set< record_t const* > unique_rec;
        for( auto ri = que.cbegin(); ri != que.cend(); ri++ )
        {
            if( ri->reference_record )
            {
                if( unique_rec.find( ri->reference_record ) == unique_rec.cend() )
                {
                    unique_rec.insert( ri->reference_record );
                    ri->output( percentile_only );
                }
            }
        }
    }
    else { for( auto const& ii: que ) { ii.output(); } }
}

static void execute( double const radius, double const percentile, size_t const min_count, bool const force, bool const percentile_only )
{
    if( csv.fields.empty() ) { csv.fields = "x,y,z,scalar"; }
    csv.full_xpath = false;
    comma::csv::options output_csv( csv );
    std::deque< record_t > que;
    comma::csv::input_stream< input_t > istrm( std::cin, csv );
    auto extents = snark::math::closed_interval< double, 3 >();
    auto const resolution = Eigen::Vector3d( radius, radius, radius );
    comma::uint32 block = 0U;
    while( istrm.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        auto in = istrm.read();
        if( !in || block != in->block )
        {
            if( !que.empty() )
            {
                process( que, extents, resolution, percentile, radius, min_count, force );
                output( que, percentile_only );
                extents = snark::math::closed_interval< double, 3 >();
                que.clear();
            }
            if( !in ) { break; }
            block = in->block;
        }
        que.emplace_back( *in, istrm );
        extents.set_hull( in->coordinates );
    }
}

} // namespace percentile_in_radius {

namespace angle_axis_relative {

struct record
{
    Eigen::Vector3d coordinates;
    std::string line;
    Eigen::AngleAxis< double > angle_axis;

    record( Eigen::Vector3d const& c, comma::csv::input_stream< Eigen::Vector3d >const& istrm )
        : coordinates( c )
        , line()
        , angle_axis( 0, Eigen::Vector3d( 0, 0, 0 ) )
    {
        if( csv.binary() )
        {
            line.resize( csv.format().size() );
            ::memcpy( &line[ 0 ], istrm.binary().last(), csv.format().size() );
        }
        else
        {
            line = comma::join( istrm.ascii().last(), csv.delimiter );
        }
    }

    void calculate_angle_axis( Eigen::Vector3d const& prev_coordinates, Eigen::Vector3d const& next_coordinates, bool const supplementary )
    {
        auto const dist1 = ( coordinates - prev_coordinates );
        auto const dist2 = ( next_coordinates - coordinates );
        if( comma::math::less( 1e-6, dist1.norm() ) && comma::math::less( 1e-6, dist2.norm() ))
        {
            angle_axis = supplementary
                ? Eigen::Quaternion< double >::FromTwoVectors( dist2, -dist1 )
                : Eigen::Quaternion< double >::FromTwoVectors( dist1, dist2 );

            //std::cerr << std::setprecision( 16 ) << "input:" << std::endl
            //    << prev_coordinates.x() << ',' << prev_coordinates.y() << ',' << prev_coordinates.z() << std::endl
            //    << coordinates.x() << ',' << coordinates.y() << ',' << coordinates.z() << std::endl
            //    << next_coordinates.x() << ',' << next_coordinates.y() << ',' << next_coordinates.z() << std::endl
            //    << "output:" << std::endl
            //    << angle_axis.angle() << ',' << angle_axis.axis().x() << ',' << angle_axis.axis().y() << ',' << angle_axis.axis().z() << std::endl;
        }
    }

    void output( comma::csv::output_stream< Eigen::AngleAxis< double > >& ostrm )
    {
        ostrm.append( line, angle_axis );
        if( csv.binary() && csv.flush ) { std::cout.flush(); }
    }
};

static void execute( bool const supplementary, bool const ignore_repeats )
{
    if( csv.fields.empty() ) { csv.fields = "x,y,z"; }
    csv.full_xpath = false;

    comma::csv::options output_csv( csv );
    output_csv.fields = comma::join( comma::csv::names< Eigen::AngleAxis< double > >( false ), ',' );
    if( output_csv.binary() ) { output_csv.format( comma::csv::format::value< Eigen::AngleAxis< double > >() ); }

    std::deque< record > que;
    comma::csv::input_stream< Eigen::Vector3d > istrm( std::cin, csv, Eigen::Vector3d::Zero() );
    comma::csv::output_stream< Eigen::AngleAxis< double > > ostrm( std::cout, output_csv );

    auto unique_points = 0U;
    while( istrm.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const Eigen::Vector3d* p = istrm.read();
        if( !p ) { break; }

        if( que.empty() || !ignore_repeats || ( *p - que.back().coordinates ).norm() >= 1e-6 ) { unique_points++; }
        que.emplace_back( *p, istrm );

        if( 3U == unique_points )
        {
            auto relevant_index = que.size() - 2U;
            que[ relevant_index ].calculate_angle_axis( que[ 0 ].coordinates, que.back().coordinates, supplementary );
            while( 2 < que.size() ) { que.front().output( ostrm ); que.pop_front(); }
            unique_points = 2;
        }
    }
    for( auto ii = 0U; ii < que.size(); ii++ ) { que[ ii ].output( ostrm ); }
}

}

static void angle_axis()
{
    comma::csv::options output_csv( csv );
    output_csv.fields = comma::join( comma::csv::names< Eigen::AngleAxis< double > >( false ), ',' );
    if( output_csv.binary() ) { output_csv.format( comma::csv::format::value< Eigen::AngleAxis< double > >() ); }
    comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv, Eigen::Vector3d::Zero() );
    comma::csv::output_stream< Eigen::AngleAxis< double > > ostream( std::cout, output_csv );
    comma::csv::tied< Eigen::Vector3d, Eigen::AngleAxis< double > > tied( istream, ostream );

    boost::optional< Eigen::Vector3d > last;
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const Eigen::Vector3d* p = istream.read();
        if( !p ) { break; }
        Eigen::AngleAxis< double > angle_axis;
        if( last ) { angle_axis = Eigen::Quaternion< double >::FromTwoVectors( *last, *p ); }
        else { angle_axis = Eigen::AngleAxis< double >( 0, Eigen::Vector3d( 0, 0, 0 )); }
        last = *p;

        tied.append( angle_axis );
    }
}

static void angle_axis_next()
{
    comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv );
    boost::optional< Eigen::Vector3d > last;
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const Eigen::Vector3d* p = istream.read();
        if( !p ) { break; }
        if( last )
        {
            Eigen::AngleAxis< double > angle_axis( Eigen::Quaternion< double >::FromTwoVectors( *last, *p ));
            if( csv.binary() )
            {
                std::cout.write( reinterpret_cast< const char* >( &angle_axis.angle() ), sizeof( double ));
                std::cout.write( reinterpret_cast< const char* >( &angle_axis.axis() ), sizeof( double ) * 3 );
                if( csv.flush ) { std::cout.flush(); }
            }
            else
            {
                std::cout << csv.delimiter << angle_axis.angle()
                          << csv.delimiter << comma::join( angle_axis.axis(), csv.delimiter )
                          << std::endl;
            }
        }
        if( csv.binary() )
        {
            std::cout.write( istream.binary().last(), istream.binary().binary().format().size() );
            if( csv.flush ) { std::cout.flush(); }
        }
        else { std::cout << comma::join( istream.ascii().last(), csv.delimiter ); }
        last = *p;
    }
    if( last )
    {
        Eigen::AngleAxis< double > angle_axis( 0, Eigen::Vector3d( 0, 0, 0 ));
        if( csv.binary() )
        {
            std::cout.write( reinterpret_cast< const char* >( &angle_axis.angle() ), sizeof( double ));
            std::cout.write( reinterpret_cast< const char* >( &angle_axis.axis() ), sizeof( double ) * 3 );
            if( csv.flush ) { std::cout.flush(); }
        }
        else
        {
            std::cout << csv.delimiter << angle_axis.angle()
                      << csv.delimiter << comma::join( angle_axis.axis(), csv.delimiter )
                      << std::endl;
        }
    }
}

static void angle_axis_for_pairs()
{
    comma::csv::options output_csv( csv );
    output_csv.fields = comma::join( comma::csv::names< Eigen::AngleAxis< double > >( false ), ',' );
    if( output_csv.binary() ) { output_csv.format( comma::csv::format::value< Eigen::AngleAxis< double > >() ); }
    comma::csv::input_stream< point_pair_t > istream( std::cin, csv, point_pair_t( Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() ) );
    comma::csv::output_stream< Eigen::AngleAxis< double > > ostream( std::cout, output_csv );
    comma::csv::tied< point_pair_t, Eigen::AngleAxis< double > > tied( istream, ostream );
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const point_pair_t* p = istream.read();
        if( !p ) { break; }
        Eigen::AngleAxis< double > angle_axis( Eigen::Quaternion< double >::FromTwoVectors( p->first, p->second ));
        tied.append( angle_axis );
    }
}

static void trajectory_discretise( double step, double tolerance )
{
    if( csv.has_some_of_fields( "first,first/x,first/y,first/z,second,second/x,second/y,second/z" ) )
    {
        comma::csv::input_stream< std::pair< Eigen::Vector3d, Eigen::Vector3d > > istream( std::cin, csv );
        comma::csv::options output_csv;
        if( csv.binary() ) { output_csv.format( "3d" ); }
        output_csv.flush = csv.flush;
        comma::csv::output_stream< Eigen::Vector3d > ostream( std::cout, output_csv );
        while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
        {
            const std::pair< Eigen::Vector3d, Eigen::Vector3d >* p = istream.read();
            if( !p ) { break; }
            Eigen::ParametrizedLine< double, 3 > line = Eigen::ParametrizedLine< double, 3 >::Through( p->first, p->second );
            double norm = ( p->first - p->second ).norm();
            unsigned int size = norm / step;
            if( ( line.pointAt( step * size ) - p->second ).norm() > tolerance ) { ++size; }
            for( unsigned int i = 0; i < size; ++i ) { comma::csv::append( istream, ostream, line.pointAt( step * i ) ); }
            comma::csv::append( istream, ostream, p->second );
        }
    }
    else
    {
        static_assert( sizeof( Eigen::Vector3d ) == sizeof( double ) * 3, "expected eigen::vector3d of size 3 * 8" );
        comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv, Eigen::Vector3d::Zero() );
        boost::optional< Eigen::Vector3d > previous_point;
        comma::csv::options output_csv;
        if( csv.binary() ) { output_csv.format( "3d" ); }
        output_csv.flush = csv.flush;
        comma::csv::output_stream< Eigen::Vector3d > ostream( std::cout, output_csv );
        std::string previous_record;
        while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
        {
            const Eigen::Vector3d* current_point = istream.read();
            if( !current_point ) { break; }
            if( previous_point )
            {
                ostream.append( previous_record, *previous_point );
                double distance = ( *previous_point - *current_point ).norm();
                if( comma::math::less( step, distance ) )
                {
                    Eigen::ParametrizedLine< double, 3 > line = Eigen::ParametrizedLine< double, 3 >::Through( *previous_point, *current_point );
                    for( double t = step; comma::math::less( t + tolerance, distance ); t += step ) { ostream.append( previous_record, line.pointAt( t ) ); }
                }
            }
            previous_point.reset( *current_point );
            previous_record = istream.last();
        }
        if( previous_point ) { ostream.append( previous_record, *previous_point ); }

//         while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
//         {
//             const Eigen::Vector3d* current_point = istream.read();
//             if( !current_point ) { break; }
//             if( previous_point )
//             {
//                 comma::csv::append( istream, ostream, *previous_point );
//                 double distance = ( *previous_point - *current_point ).norm();
//                 if( comma::math::less( step, distance ) )
//                 {
//                     Eigen::ParametrizedLine< double, 3 > line = Eigen::ParametrizedLine< double, 3 >::Through( *previous_point, *current_point );
//                     for( double t = step; comma::math::less( t + tolerance, distance ); t += step )
//                     {
//                         Eigen::Vector3d point = line.pointAt( t );
//                         comma::csv::append( istream, ostream, point );
//                     }
//                 }
//             }
//             previous_point.reset( *current_point );
//         }
//         if( previous_point ) { comma::csv::append( istream, ostream, *previous_point ); }
    }
}

static int trajectory_cumulative_discretise( const comma::command_line_options& options )
{
    double step = options.value< double >( "--step" );
    if( step <= 0 ) { std::cerr << "points-calc: expected positive step, got " << step << std::endl; return 1; }
    comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv, Eigen::Vector3d::Zero() );
    comma::csv::options output_csv;
    if( csv.binary() ) { output_csv.format( "3d" ); }
    output_csv.flush = csv.flush;
    comma::csv::output_stream< Eigen::Vector3d > ostream( std::cout, output_csv );
    comma::signal_flag is_shutdown;
    const Eigen::Vector3d* p = istream.read();
    if( !p ) { return 0; }
    Eigen::Vector3d previous_point = *p;
    comma::csv::append( istream, ostream, previous_point );
    double length = 0;
    double stepped = step;
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const Eigen::Vector3d* current_point = istream.read();
        if( !current_point ) { break; }
        double end = length + ( previous_point - *current_point ).norm();
        Eigen::ParametrizedLine< double, 3 > line = Eigen::ParametrizedLine< double, 3 >::Through( previous_point, *current_point );
        for( ; !is_shutdown && comma::math::less( stepped, end ); stepped += step ) { comma::csv::append( istream, ostream, line.pointAt( stepped - length ) ); }
        previous_point = *current_point;
        length = end;
    }
    if( !comma::math::equal( length, stepped ) ) { comma::csv::append( istream, ostream, previous_point ); }
    return 0;
}

namespace trajectory_chord_operation {

struct record
{
    Eigen::Vector3d coordinates;
    std::string line;
    record* reference_record;

    record() : reference_record( nullptr ) {}
    record( const Eigen::Vector3d& c, const std::string& line )
        : coordinates( c ), line( line ), reference_record( nullptr ) {}
};

static record* target_record = nullptr;

void output( const record& record, bool filter )
{
    if( filter )
    {
        if( &record == target_record )
        {
            std::cout.write( &record.line[0], record.line.size() );
            if( !csv.binary() ) { std::cout << std::endl; }
            if( csv.flush ) { std::cout.flush(); }
            target_record = record.reference_record;
        }
    }
    else
    {
        if( csv.binary() )
        {
            std::cout.write( &record.line[0], record.line.size() );
            std::cout.write( &record.reference_record->line[0], record.reference_record->line.size() );
        }
        else
        {
            std::cout << record.line << csv.delimiter << record.reference_record->line << std::endl;
        }
    }
}

} // namespace trajectory_chord_operation {

namespace local_operation {

struct point
{
    Eigen::Vector3d coordinates;
    double scalar;
    comma::uint32 id;
    comma::uint32 block;
//     uint8_t mask;

    point() : coordinates( 0, 0, 0 ), scalar( 0 ), id( 0 ), block( 0 ) {}
    point( const Eigen::Vector3d& coordinates, double scalar ) : coordinates( coordinates ), scalar( scalar ), id( 0 ), block( 0 ) {}
};

struct output // quick and dirty
{
    comma::uint32 id;
    double distance;
    output() : id( 0 ), distance( 0 ) {}
    output( comma::uint32 id, double distance ) : id( id ), distance( distance ) {}
};

struct record // todo: it's a mess; remove code duplication: all information can be extracted from reference_record (see nearest for usage)
{
    local_operation::point point;
    std::string line;
    bool is_extremum;
    comma::uint32 reference_id;
    double distance;
    record* reference_record;

    static comma::uint32 invalid_id;

    record() : is_extremum( true ), reference_id( invalid_id ), distance( std::numeric_limits< double >::max() ), reference_record( NULL ) {}
    record( const local_operation::point& p, const std::string& line ) : point( p ), line( line ), is_extremum( true ), reference_id( invalid_id ), distance( std::numeric_limits< double >::max() ), reference_record( NULL ) {}
    local_operation::output output( bool force = true ) const { return !force && reference_id == invalid_id ? local_operation::output( invalid_id, 0 ) : local_operation::output( reference_record->point.id, ( reference_record->point.coordinates - point.coordinates ).norm() ); }
};

comma::uint32 record::invalid_id = std::numeric_limits< comma::uint32 >::max();

static void evaluate_local_extremum( record* i, record* j, double radius, double sign )
{
    if( i == j || !i->is_extremum ) { return; }
    if( ( i->point.coordinates - j->point.coordinates ).squaredNorm() > radius * radius ) { return; }
    i->is_extremum = !comma::math::less( ( i->point.scalar - j->point.scalar ) * sign, 0 );
    if( i->is_extremum ) { j->is_extremum = false; }
}

static void update_nearest_extremum( record* i, record* j, double radius, bool trace )
{
    if( i->is_extremum )
    {
        if( i->reference_id != record::invalid_id ) { return; }
        i->reference_id = i->point.id;
        i->distance = 0;
        i->reference_record = i; // todo: dump reference_id, reference_record is enough
        return;
    }
    record* k = j;
    for( ; trace && k->reference_record != k && !k->reference_record->is_extremum; k = k->reference_record );
    if( !trace && !k->is_extremum ) { return; }
    double norm = ( i->point.coordinates - k->point.coordinates ).norm();
    if( norm > radius ) { return; }
    if( i->reference_id != record::invalid_id && i->distance < norm ) { return; }
    i->reference_id = k->point.id;
    i->distance = norm;
    i->reference_record = k; // todo: dump reference_id, reference_record is enough
}

static void update_nearest( record* i, record* j, double radius, double sign, bool any )
{
    if( i == j ) { return; }
    if( any ) // quick and dirty
    {
        double norm = ( i->point.coordinates - j->point.coordinates ).norm();
        if( norm > radius ) { return; }
        if( i != i->reference_record && ( i->reference_record->point.coordinates - i->point.coordinates ).norm() < norm ) { return; }
        i->reference_record = j;
    }
    else
    {
        if( comma::math::less( ( j->point.scalar - i->reference_record->point.scalar ) * sign, 0 ) ) { return; }
        double norm = ( i->point.coordinates - j->point.coordinates ).norm();
        if( norm > radius ) { return; }
        if(    comma::math::equal( ( i->reference_record->point.scalar - j->point.scalar ) * sign, 0 )
            && ( i->reference_record->point.coordinates - i->point.coordinates ).norm() < norm ) { return; }
        i->reference_record = j;
    }
}

static void process_nearest_extremum_block( std::deque< local_operation::record >& records
                                          , snark::math::closed_interval< double, 3 > const&  extents
                                          , Eigen::Vector3d const&  resolution
                                          , double const sign
                                          , double const radius
                                          , bool const any
                                          , bool const fast )
{
    if( verbose ) { std::cerr << "points-calc: loading " << records.size() << " points into grid..." << std::endl; }
    typedef std::vector< local_operation::record* > voxel_t; // todo: is vector a good container? use deque
    typedef snark::voxel_map< voxel_t, 3 > grid_t;

    grid_t grid( extents.min(), resolution );
    for( std::size_t i = 0; i < records.size(); ++i )
    {
//         if(records[i].point.mask )
        ( grid.touch_at( records[i].point.coordinates ) )->second.push_back( &records[i] );
    }
    for( grid_t::iterator it = grid.begin(); it != grid.end(); ++it )
    {
        grid_t::index_type i;
        if( fast )
        {
            std::unordered_set< local_operation::record* > extremums;
            for( i[0] = it->first[0] - 1; i[0] < it->first[0] + 2; ++i[0] )
            {
                for( i[1] = it->first[1] - 1; i[1] < it->first[1] + 2; ++i[1] )
                {
                    for( i[2] = it->first[2] - 1; i[2] < it->first[2] + 2; ++i[2] )
                    {
                        grid_t::iterator git = grid.find( i );
                        if( git == grid.end() ) { continue; }
                        for( std::size_t k = 0; k < git->second.size(); ++k )
                        {
                            if( !extremums.empty() )
                            {
                                if( comma::math::less(( git->second[k]->point.scalar - (*extremums.begin())->point.scalar ) * sign, 0 )) { continue; }
                                if( !comma::math::equal(( git->second[k]->point.scalar - (*extremums.begin())->point.scalar ) * sign, 0 )) { extremums.clear(); }
                            }
                            extremums.insert( git->second[k] );
                        }
                        for( std::size_t n = 0; n < it->second.size(); ++n )
                        {
                            it->second[n]->reference_record = nullptr;
                            for( auto exi = extremums.cbegin(); exi != extremums.cend(); exi++ )
                            {
                                if( !it->second[n]->reference_record || comma::math::less
                                        ( ( (*exi)->point.coordinates - it->second[n]->point.coordinates ).norm()
                                        , ( it->second[n]->reference_record->point.coordinates - it->second[n]->point.coordinates ).norm() ) )
                                {
                                    it->second[n]->reference_record = *exi;
                                }
                            }
                        }
                    }
                }
            }
        }
        else
        {
            for( i[0] = it->first[0] - 1; i[0] < it->first[0] + 2; ++i[0] )
            {
                for( i[1] = it->first[1] - 1; i[1] < it->first[1] + 2; ++i[1] )
                {
                    for( i[2] = it->first[2] - 1; i[2] < it->first[2] + 2; ++i[2] )
                    {
                        grid_t::iterator git = grid.find( i );
                        if( git == grid.end() ) { continue; }
                        for( std::size_t n = 0; n < it->second.size(); ++n )
                        {
                            for( std::size_t k = 0; k < git->second.size(); ++k )
                            {
                                local_operation::update_nearest( it->second[n], git->second[k], radius, sign, any );
                            }
                        }
                    }
                }
            }
        }
    }
}

static void output_nearest_extremum_block( const std::deque< local_operation::record >& records
                                         , comma::csv::options const& output_csv
                                         , std::string const& endl
                                         , std::string const& delimiter
                                         , bool output_full_record
                                         , bool output_extremums_only )
{
    if( output_full_record )
    {
        unsigned int discarded = 0;
        for( std::size_t i = 0; i < records.size(); ++i )
        {
            if( !records[i].reference_record ) { ++discarded; continue; }
            std::cout.write( &records[i].line[0], records[i].line.size() );
            std::cout.write( &delimiter[0], delimiter.size() );
            std::cout.write( &records[i].reference_record->line[0], records[i].reference_record->line.size() );
            std::cout.write( &endl[0], endl.size() );
            if( csv.flush ) { std::cout.flush(); }
        }
        if( verbose ) { std::cerr << "points-calc: discarded " << discarded << " point(s) of " << records.size() << std::endl; }
        return;
    }
    if( output_extremums_only )
    {
        std::unordered_set< comma::uint32 > ids;
        for( std::size_t i = 0; i < records.size(); ++i )
        {
            if( !records[i].reference_record ) { continue; }
            if( ids.find( records[i].reference_record->point.id ) != ids.end() ) { continue; }
            ids.insert( records[i].reference_record->point.id );
            std::cout.write( &records[i].reference_record->line[0], records[i].reference_record->line.size() );
            std::cout.write( &endl[0], endl.size() );
            if( csv.flush ) { std::cout.flush(); }
        }
        return;
    }
    comma::csv::output_stream< local_operation::output > ostream( std::cout, output_csv );
    for( std::size_t i = 0; i < records.size(); ++i )
    {
        std::cout.write( &records[i].line[0], records[i].line.size() );
        std::cout.write( &delimiter[0], delimiter.size() );
        ostream.write( records[i].output() );
    }
}

} // namespace local_operation {

namespace snark { namespace points_calc { namespace trajectory_partition {

typedef point_with_direction input;

struct output { comma::uint32 id; output( comma::uint32 id = 0 ): id( id ) {} };

} } } // namespace snark { namespace points_calc { namespace trajectory_partition {

namespace comma { namespace visiting {

template <> struct traits< point_with_direction >
{
    template< typename K, typename V > static void visit( const K& k, const point_with_direction& t, V& v )
    {
        comma::visiting::traits< Eigen::Vector3d >::visit( k, static_cast< const Eigen::Vector3d& >( t ), v );
        v.apply( "direction", t.direction );
    }

    template< typename K, typename V > static void visit( const K& k, point_with_direction& t, V& v )
    {
        comma::visiting::traits< Eigen::Vector3d >::visit( k, static_cast< Eigen::Vector3d& >( t ), v );
        v.apply( "direction", t.direction );
    }
};

template <> struct traits< snark::points_calc::trajectory_partition::output >
{
    template< typename K, typename V > static void visit( const K&, const snark::points_calc::trajectory_partition::output& t, V& v ) { v.apply( "id", t.id ); }
};

template <> struct traits< point_with_block >
{
    template< typename K, typename V > static void visit( const K&, const point_with_block& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "block", t.block );
    }

    template< typename K, typename V > static void visit( const K&, point_with_block& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "block", t.block );
    }
};

template <> struct traits< local_operation::point >
{
    template< typename K, typename V > static void visit( const K&, const local_operation::point& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "scalar", t.scalar );
        v.apply( "id", t.id );
        v.apply( "block", t.block );
//         v.apply( "mask", t.mask );
    }

    template< typename K, typename V > static void visit( const K&, local_operation::point& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "scalar", t.scalar );
        v.apply( "id", t.id );
        v.apply( "block", t.block );
//         v.apply( "mask", t.mask );
    }
};

template <> struct traits< local_operation::output >
{
    template< typename K, typename V > static void visit( const K&, const local_operation::output& t, V& v )
    {
        v.apply( "id", t.id );
        v.apply( "distance", t.distance );
    }
};

template <> struct traits< percentile_in_radius::input_t >
{
    template< typename K, typename V > static void visit( const K&, const percentile_in_radius::input_t& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "scalar", t.scalar );
        v.apply( "block", t.block );
    }

    template< typename K, typename V > static void visit( const K&, percentile_in_radius::input_t& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "scalar", t.scalar );
        v.apply( "block", t.block );
    }
};

} } // namespace comma { namespace visiting {

template < typename Traits > static int run( const comma::command_line_options& options )
{
    if( options.exists( "--input-fields" ) ) { std::cout << Traits::input_fields() << std::endl; return 0; }
    if( options.exists( "--input-format" ) ) { std::cout << Traits::input_format() << std::endl; return 0; }
    if( options.exists( "--output-fields" ) ) { std::cout << Traits::output_fields() << std::endl; return 0; }
    if( options.exists( "--output-format" ) ) { std::cout << Traits::output_format() << std::endl; return 0; }
    return Traits::run( options );
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        verbose = options.exists( "--verbose,-v" );
        csv = comma::csv::options( options );
        csv.full_xpath = true;
        ascii = comma::csv::ascii< Eigen::Vector3d >( "x,y,z", csv.delimiter );
        const std::vector< std::string >& operations = options.unnamed( "--at-fields,--do-not-accumulate,--id-filter,--id-pass,--in-place,--emplace,--from,--to,--differential,--diff,--verbose,-v,--trace,--no-antialiasing,--next,--unit,--output-full-record,--full-record,--full,--flush,--with-trajectory,--trajectory,--linear,--input-fields,--input-format,--output-fields,--output-format,--filter,--fast,--min-count,--output-percentile-only,--output-percentile,--output-points-only,--points-only,--discard-input,--output-sides-only,--sides-only,--output-internal-only,--internal-only", "-.*" );
        if( operations.size() != 1 ) { std::cerr << "points-calc: expected one operation, got " << operations.size() << ": " << comma::join( operations, ' ' ) << std::endl; return 1; }
        const std::string& operation = operations[0];
        if( operation == "frame-integrate" || operation == "integrate-frame" ) { return run< snark::points_calc::frame::integrate::traits >( options ); }
        if( operation == "life" ) { return run< snark::points_calc::life::traits >( options ); }
        if( operation == "lines-nearest" ) { return run< snark::points_calc::lines_nearest::traits >( options ); }
        if( operation == "project-onto-line" ) { return run< snark::points_calc::project::onto_line::traits >( options ); }
        if( operation == "project-onto-plane" ) { return run< snark::points_calc::project::onto_plane::traits >( options ); }
        if( operation == "plane-intersection-with-trajectory" ) { return run< snark::points_calc::plane_intersection_with_trajectory::traits >( options ); }
        if( vector_calc::has_operation( operation ) ) { vector_calc::process(operation, options, csv); return 0; }
        if( operation == "plane-intersection" ) { snark::points_calc::plane_intersection::traits::process(options, csv); return 0; }
        if( operation == "triangles-area" ) { return run< snark::points_calc::triangles::area::traits >( options ); }
        if( operation == "triangles-discretise" || operation == "triangles-discretize" ) { return run< snark::points_calc::triangles::discretise::traits >( options ); }
        if( operation == "visit" ) { return run< snark::points_calc::visit::traits >( options ); }
        if( operation == "distance" || operation == "trajectory-distance" )
        {
            if( options.exists("--output-fields" )){ std::cout << "distance" << std::endl; return 0; }
            if( options.exists("--output-format" )){ std::cout << "d" << std::endl; return 0; }
            bool propagate = options.exists( "--propagate" );
            return csv.has_some_of_fields( "first,first/x,first/y,first/z,second,second/x,second/y,second/z" )
                   ? calculate_distance_for_pairs( propagate )
                   : options.exists( "--next" )
                   ? calculate_distance_next( propagate )
                   : calculate_distance( false, propagate );
        }
        if( operation == "trajectory-cumulative-distance" || operation == "cumulative-distance" )
        {
            if( options.exists("--input-fields" ) ) { std::cout << comma::join( comma::csv::names< point_with_block >( false ), ',' ) << std::endl; return 0; }
            if( options.exists("--input-format" ) ) { std::cout << comma::csv::format::value< point_with_block >() << std::endl; return 0; }
            if( options.exists("--output-fields" ) ) { std::cout << "distance" << std::endl; return 0; }
            if( options.exists("--output-format" ) ) { std::cout << "d" << std::endl; return 0; }
            return calculate_distance( true, false, options.exists( "--differential,--diff" ) );
        }
        if( operation == "percentile-in-radius" || operation == "percentile-in-range" || operation == "percentile" )
        {
            auto const force = options.exists( "--fast" );
            auto const percentile_only = options.exists( "--output-percentile-only,--output-percentile" );
            auto const radius = options.value< double >( "--radius" );
            auto const min_count = options.value< size_t >( "--min-count", 0U );
            auto percentile = options.value< double >( "--percentile" );
            if( comma::math::less( percentile, 0.0 ) || comma::math::less( 1.0, percentile ) ) { COMMA_THROW( comma::exception, "percentile must be in [0,1], got: " << percentile ); }
            if( comma::math::equal( percentile, 0.0 ) ) { percentile = std::numeric_limits< double >::min(); }
            percentile_in_radius::execute( radius, percentile, min_count, force, percentile_only );
            return 0;
        }
        if( operation == "angle-axis" )
        {
            if( options.exists("--output-fields" )){ std::cout << comma::join( comma::csv::names< Eigen::AngleAxis< double > >( false ), ',' ) << std::endl; return 0; }
            if( options.exists("--output-format" )){ std::cout << comma::csv::format::value< Eigen::AngleAxis< double > >() << std::endl; return 0; }
            if( options.exists( "--relative" ) )
            {
                angle_axis_relative::execute( options.exists( "--reverse,--supplementary" ), options.exists( "--ignore-repeats" ) );
            }
            else
            {
                if(    csv.has_field( "first" )   || csv.has_field( "second" )
                    || csv.has_field( "first/x" ) || csv.has_field( "second/x" )
                    || csv.has_field( "first/y" ) || csv.has_field( "second/y" )
                    || csv.has_field( "first/z" ) || csv.has_field( "second/z" ) )
                {
                    angle_axis_for_pairs();
                    return 0;
                }
                if( options.exists( "--next" ) ) { angle_axis_next(); } else { angle_axis(); }
            }
            return 0;
        }
        if( operation == "nearest" )
        {
            Eigen::Vector3d point = comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--point,--to" ) );
            comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv, Eigen::Vector3d::Zero() );
            std::string record;
            double min_distance = std::numeric_limits< double >::max();
            while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const Eigen::Vector3d* p = istream.read();
                if( !p ) { break; }
                double d = ( *p - point ).norm();
                if( d >= min_distance ) { continue; }
                min_distance = d;
                record = csv.binary() ? std::string( istream.binary().last(), csv.format().size() ) : comma::join( istream.ascii().last(), csv.delimiter );
            }
            if( record.empty() ) { return 0; }
            std::cout.write( &record[0], record.size() );
            if( csv.binary() ) { std::cout.write( reinterpret_cast< const char* >( &min_distance ), sizeof( double ) ); }
            else { std::cout << csv.delimiter << min_distance << std::endl; }
            if( csv.flush ) { std::cout.flush(); }
            return 0;
        }
        if( operation == "trajectory-thin" )
        {
            double resolution = options.value< double >( "--resolution" );
            comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv, Eigen::Vector3d::Zero() );
            comma::csv::passed< Eigen::Vector3d > passed( istream, std::cout );
            const Eigen::Vector3d* p = istream.read();
            if( !p ) { return 0; }
            boost::optional< Eigen::Vector3d > last = *p;
            passed.write();
            if( options.exists( "--do-not-accumulate" ) )
            {
                while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
                {
                    const Eigen::Vector3d* p = istream.read();
                    if( !p ) { break; }
                    if( ( *p - *last ).norm() < resolution ) { continue; }
                    last = *p;
                    passed.write();
                }
                return 0;
            }
            double distance = 0;
            while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const Eigen::Vector3d* p = istream.read();
                if( !p ) { break; }
                distance += ( *p - *last ).norm();
                last = *p;
                if( distance < resolution ) { continue; }
                distance = 0;
                passed.write();
            }
            return 0;
        }
        if( operation == "thin" ) { return run< snark::points_calc::thin::traits >( options ); }
        if( operation == "trajectory-discretise" || operation == "trajectory-discretize" || operation == "discretise" || operation == "discretize" )
        {
            double step = options.value< double >( "--step" );
            if( step <= 0 ) { std::cerr << "points-calc: expected positive step, got " << step << std::endl; return 1; }
            // the last discretised point can be very close to the end of the interval, in which case the last two points can be identical in the output since ascii.put uses 12 digits by default
            // setting --tolerance=1e-12 will not allow the last discretised point to be too close to the end of the interval and therefore the output will have two distinct points at the end
            double tolerance = options.value( "--tolerance", std::numeric_limits< double >::min() );
            if( tolerance < 0 ) { std::cerr << "points-calc: expected non-negative tolerance, got " << tolerance << std::endl; return 1; }
            trajectory_discretise( step, tolerance );
            return 0;
        }
        if( operation == "trajectory-cumulative-discretise" || operation == "trajectory-cumulative-discretize" || operation == "cumulative-discretise" || operation == "cumulative-discretize" || operation == "sample" ) { return trajectory_cumulative_discretise( options ); }
        if( operation == "local-max" || operation == "local-min" ) // todo: if( operation == "local-calc" ? )
        {
            double sign = operation == "local-max" ? 1 : -1;
            if( csv.fields.empty() ) { csv.fields = "x,y,z,scalar"; }
            csv.full_xpath = false;
            bool has_id = csv.has_field( "id" );
            comma::csv::input_stream< local_operation::point > istream( std::cin, csv );
            std::deque< local_operation::record > records;
            double radius = options.value< double >( "--radius" );
            bool trace = options.exists( "--trace" );
            Eigen::Vector3d resolution( radius, radius, radius );
            snark::math::closed_interval< double, 3 > extents;
            comma::uint32 id = 0;
            if( verbose ) { std::cerr << "points-calc: reading input points..." << std::endl; }
            while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const local_operation::point* p = istream.read();
                if( !p ) { break; }
                std::string line;
                if( csv.binary() ) // quick and dirty
                {
                    line.resize( csv.format().size() );
                    ::memcpy( &line[0], istream.binary().last(), csv.format().size() );
                }
                else
                {
                    line = comma::join( istream.ascii().last(), csv.delimiter );
                }
                local_operation::point q = *p;
                if( !has_id ) { q.id = id++; }
                records.push_back( local_operation::record( q, line ) );
                records.back().reference_record = &records.back();
                extents.set_hull( p->coordinates );
            }
            if( verbose ) { std::cerr << "points-calc: loading " << records.size() << " points into grid..." << std::endl; }
            typedef std::vector< local_operation::record* > voxel_t; // todo: is vector a good container? use deque
            typedef snark::voxel_map< voxel_t, 3 > grid_t;
            grid_t grid( extents.min(), resolution );
            for( std::size_t i = 0; i < records.size(); ++i ) { ( grid.touch_at( records[i].point.coordinates ) )->second.push_back( &records[i] ); }
            if( verbose ) { std::cerr << "points-calc: searching for local extrema..." << std::endl; }
            for( grid_t::iterator it = grid.begin(); it != grid.end(); ++it )
            {
                grid_t::index_type i;
                for( i[0] = it->first[0] - 1; i[0] < it->first[0] + 2; ++i[0] )
                {
                    for( i[1] = it->first[1] - 1; i[1] < it->first[1] + 2; ++i[1] )
                    {
                        for( i[2] = it->first[2] - 1; i[2] < it->first[2] + 2; ++i[2] )
                        {
                            grid_t::iterator git = grid.find( i );
                            if( git == grid.end() ) { continue; }
                            for( voxel_t::iterator vit = it->second.begin(); vit != it->second.end(); ++vit )
                            {
                                for( std::size_t k = 0; k < git->second.size() && ( *vit )->is_extremum; ++k )
                                {
                                    local_operation::evaluate_local_extremum( *vit, git->second[k], radius, sign );
                                }
                            }
                        }
                    }
                }
            }
            #ifdef WIN32
            _setmode( _fileno( stdout ), _O_BINARY );
            #endif
            if( verbose ) { std::cerr << "points-calc: filling extrema grid..." << std::endl; }
            grid_t extrema( extents.min(), resolution );
            for( std::size_t i = 0; i < records.size(); ++i )
            {
                if( records[i].is_extremum )
                {
                    ( extrema.touch_at( records[i].point.coordinates ) )->second.push_back( &records[i] );
                }
                else
                {
                    records[i].reference_id = local_operation::record::invalid_id; // quick and dirty for now
//                     if( records[i].reference_id == local_operation::record::invalid_id ) { continue; }
//                     while( records[i].reference_record->point.id != records[i].reference_record->reference_record->point.id )
//                     {
//                         records[i].reference_record = records[i].reference_record->reference_record;
//                     }
//                     records[i].reference_id = records[i].reference_record->point.id;
                }
            }
            if( verbose ) { std::cerr << "points-calc: calculating distances to " << extrema.size() << " local extrema..." << std::endl; }
            for( grid_t::iterator it = grid.begin(); it != grid.end(); ++it )
            {
                grid_t::index_type i;
                for( i[0] = it->first[0] - 1; i[0] < it->first[0] + 2; ++i[0] )
                {
                    for( i[1] = it->first[1] - 1; i[1] < it->first[1] + 2; ++i[1] )
                    {
                        for( i[2] = it->first[2] - 1; i[2] < it->first[2] + 2; ++i[2] )
                        {
                            grid_t::iterator git = extrema.find( i );
                            if( git == extrema.end() ) { continue; }
                            for( std::size_t n = 0; n < it->second.size(); ++n )
                            {
                                for( std::size_t k = 0; k < git->second.size(); ++k )
                                {
                                    local_operation::update_nearest_extremum( it->second[n], git->second[k], radius, trace );
                                }
                            }
                        }
                    }
                }
            }
            if( trace )
            {
                if( verbose ) { std::cerr << "points-calc: tracing extrema..." << std::endl; }
                for( std::size_t i = 0; i < records.size(); ++i )
                {
                    if( records[i].reference_id == local_operation::record::invalid_id ) { continue; }
                    while( records[i].reference_record->point.id != records[i].reference_record->reference_record->point.id )
                    {
                        records[i].reference_record = records[i].reference_record->reference_record;
                    }
                }
            }


            // todo: move output to a function


            if( verbose ) { std::cerr << "points-calc: " << operation << ": outputting..." << std::endl; }
            std::string endl;
            std::string delimiter;
            comma::csv::options output_csv;
            if( csv.binary() )
            {
                output_csv.format( "ui,d" );
            }
            else
            {
                std::ostringstream oss;
                oss << std::endl;
                endl = oss.str();
                delimiter = std::string( 1, csv.delimiter );
            }
            output_csv.flush = csv.flush;
            if( options.exists( "--output-full-record,--full-record,--full" ) )
            {
                unsigned int discarded = 0;
                for( std::size_t i = 0; i < records.size(); ++i )
                {
                    if( !records[i].reference_record ) { ++discarded; continue; }
                    std::cout.write( &records[i].line[0], records[i].line.size() );
                    std::cout.write( &delimiter[0], delimiter.size() );
                    std::cout.write( &records[i].reference_record->line[0], records[i].reference_record->line.size() );
                    std::cout.write( &endl[0], endl.size() );
                    if( csv.flush ) { std::cout.flush(); }
                }
                if( verbose ) { std::cerr << "points-calc: " << operation << ": discarded " << discarded << " point(s) of " << records.size() << std::endl; }
            }
            else
            {
                comma::csv::output_stream< local_operation::output > ostream( std::cout, output_csv );
                for( std::size_t i = 0; i < records.size(); ++i )
                {
                    std::cout.write( &records[i].line[0], records[i].line.size() );
                    std::cout.write( &delimiter[0], delimiter.size() );
                    ostream.write( records[i].output( false ) );
                }
            }
            if( verbose ) { std::cerr << "points-calc: " << operation << ": done!" << std::endl; }
            return 0;
        }
        if( operation == "nearest-max" || operation == "max-in-range" || operation == "max-in-radius"
                || operation == "nearest-min" || operation == "min-in-radius" || operation == "min-in-range"
                || operation == "nearest-any" || operation == "nearest-point" )
        {
            std::deque< local_operation::record > records;
            double sign = operation == "nearest-max" ? 1 : -1;
            bool any = operation == "nearest-any" || operation == "nearest-point";
            if( csv.fields.empty() ) { csv.fields = any ? "x,y,z" : "x,y,z,scalar"; }
            csv.full_xpath = false;
            bool has_id = csv.has_field( "id" );
            comma::csv::input_stream< local_operation::point > istream( std::cin, csv );
            double radius = options.value< double >( "--radius" );
            Eigen::Vector3d resolution( radius, radius, radius );
            snark::math::closed_interval< double, 3 > extents;
            comma::uint32 id = 0;
            comma::uint32 block = 0;
            if( verbose ) { std::cerr << "points-calc " << operation << ": reading input points of block: " << block << std::endl; }
#ifdef WIN32
            _setmode( _fileno( stdout ), _O_BINARY );
#endif
            options.assert_mutually_exclusive( "--output-full-record,--full-record,--full", "--output-extremums-only,--output-extremums,--extremums" );
            options.assert_mutually_exclusive( "--nearest-any,--nearest-point", "--fast" );
            bool const output_full_record = options.exists( "--output-full-record,--full-record,--full" );
            bool output_extremums_only = options.exists( "--output-extremums-only,--output-extremums,--extremums" );
            bool const fast = options.exists( "--fast" );
            std::string endl;
            std::string delimiter;
            comma::csv::options output_csv;
            if( csv.binary() )
            {
                output_csv.format( "ui,d" );
            }
            else
            {
                std::ostringstream oss;
                oss << std::endl;
                endl = oss.str();
                delimiter = std::string( 1, csv.delimiter );
            }
            output_csv.flush = csv.flush;
            while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const local_operation::point* p = istream.read();
                if ( !p || block != p->block )
                {
                    if( !records.empty() )
                    {
                        if( verbose ) { std::cerr << "points-calc " << operation << ": processing points of block: " << block << std::endl; }
                        local_operation::process_nearest_extremum_block( records, extents, resolution, sign, radius, any, fast );
                        if( verbose ) { std::cerr << "points-calc " << operation << ": outputting points of block: " << block << std::endl; }
                        output_nearest_extremum_block( records, output_csv, endl, delimiter, output_full_record, output_extremums_only );
                        extents = snark::math::closed_interval< double, 3 >();
                        records.clear();
                    }
                    if( verbose ) { std::cerr << "points-calc " << operation << ": reading input points of block: " << block << std::endl; }
                    if( !p ) { break; }
                    block = p->block;
                }
                std::string line;
                if( csv.binary() ) // quick and dirty
                {
                    line.resize( csv.format().size() );
                    ::memcpy( &line[0], istream.binary().last(), csv.format().size() );
                }
                else
                {
                    line = comma::join( istream.ascii().last(), csv.delimiter );
                }
                local_operation::point q = *p;
                if( !has_id ) { q.id = id++; }
                records.push_back( local_operation::record( q, line ) );
                records.back().reference_record = &records.back();
                extents.set_hull( p->coordinates );
            }
            return 0;
        }
        if( operation == "find-outliers" )
        {
            if( csv.fields.empty() ) { csv.fields = "x,y,z"; }
            csv.full_xpath = false;
            unsigned int size = options.value< unsigned int >( "--min-number-of-points-per-voxel,--size" );
            double r = options.value< double >( "--resolution" );
            Eigen::Vector3d resolution( r, r, r );
            bool no_antialiasing = options.exists( "--no-antialiasing" );
            bool discard=options.exists("--discard");
            comma::csv::input_stream< remove_outliers::point > istream( std::cin, csv );
            remove_outliers::app app(csv,verbose);
            unsigned block=0;
            #ifdef WIN32
            _setmode( _fileno( stdout ), _O_BINARY );
            #endif
            if( verbose ) { std::cerr << "points-calc: reading input points..." << std::endl; }
            while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const remove_outliers::point* p = istream.read();
                if ( !p || block != p->block )
                {
                    app.process(resolution,no_antialiasing,size);
                    app.write_output(discard);
                    app.extents=snark::math::closed_interval< double, 3 >();
                    app.records.clear();
                }
                if( !p ) { break; }
                block = p->block;
                std::string line;
                if( csv.binary() ) // quick and dirty
                {
                    line.resize( csv.format().size() );
                    ::memcpy( &line[0], istream.binary().last(), csv.format().size() );
                }
                else
                {
                    line = comma::join( istream.ascii().last(), csv.delimiter );
                }
                app.records.push_back( remove_outliers::record( *p, line ) );
                app.extents.set_hull( p->coordinates );
            }
            if( verbose ) { std::cerr << "points-calc: done!" << std::endl; }
            return 0;
        }
        if( operation == "trajectory-chord" || operation == "chord" )
        {
            csv.full_xpath = false;
            if( csv.fields.empty() ) { csv.fields = comma::join( comma::csv::names< Eigen::Vector3d >( false ), ',' ); }

            if( options.exists( "--input-fields" )) { std::cout << comma::join( comma::csv::names< Eigen::Vector3d >( false ), ',' ) << std::endl; return 0; }
            if( options.exists( "--input-format" )) { std::cout << comma::csv::format::value< Eigen::Vector3d >() << std::endl; return 0; }

            comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv, Eigen::Vector3d::Zero() );

            std::deque< trajectory_chord_operation::record > records;
            double arc_ratio = options.value< double >( "--arc-ratio" );
            double arc_maximum = options.value< double >( "--arc-maximum,--arc-max" );
            bool filter = options.exists( "--filter" );

            comma::verbose << "reading input points..." << std::endl;
            while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const Eigen::Vector3d* p = istream.read();
                if( !p ) { break; }
                std::string line;
                if( csv.binary() ) // quick and dirty
                {
                    line.resize( csv.format().size() );
                    ::memcpy( &line[0], istream.binary().last(), csv.format().size() );
                }
                else
                {
                    line = comma::join( istream.ascii().last(), csv.delimiter );
                }
                records.push_back( trajectory_chord_operation::record( *p, line ));
                records.back().reference_record = &records.back();
                if( !trajectory_chord_operation::target_record ) { trajectory_chord_operation::target_record = &records.back(); }

                double cumulative_distance = 0;
                Eigen::Vector3d* prev_coordinates = NULL;
                // look backwards along the path from the point we've just added
                // and record this as the target point for any point close enough
                for( auto it = records.rbegin(); it != records.rend(); ++it )
                {
                    if( prev_coordinates )
                    {
                        double straight_line_distance = ( records.back().coordinates - it->coordinates ).norm();
                        cumulative_distance += ( it->coordinates - *prev_coordinates ).norm();
                        if( cumulative_distance <= straight_line_distance * arc_ratio &&
                            cumulative_distance <= arc_maximum )
                        {
                            it->reference_record = &records.back();
                        }
                    }
                    prev_coordinates = &it->coordinates;
                }

                // any points not updated by the above now have their correct target point,
                // so we can output them (and take them off the queue)-
                unsigned int records_processed = 0;
                for( auto it = records.begin(); it != records.end(); ++it )
                {
                    if( it->reference_record == &records.back() ) { break; }
                    trajectory_chord_operation::output( *it, filter );
                    records_processed++;
                }
                for( unsigned int i = 0; i < records_processed; i++ ) { records.pop_front(); }
            }

            // having read the last point and updated everything, output the remainder of the queue
            for( auto it = records.begin(); it != records.end(); ++it )
            {
                // skip any record that points to itself (only the last record)
                if( &*it == it->reference_record ) { break; }
                trajectory_chord_operation::output( *it, filter );
            }
            return 0;
        }
        if( operation == "trajectory-partition" || operation == "trajectory-partitions" )
        {
            if( operation == "trajectory-partitions" ) { std::cerr << "points-calc: DEPRECATION WARNING: trajectory-partitions (plural) is deprecated; use trajectory-partition (as verb)" << std::endl; }
            if( options.exists( "--input-fields" )) { std::cout << comma::join( comma::csv::names< point_with_direction >( true ), ',' ) << std::endl; return 0; }
            if( options.exists( "--input-format" )) { std::cout << comma::csv::format::value< point_with_direction >() << std::endl; return 0; }
            if( options.exists( "--output-fields" )) { std::cout << "id" << std::endl; return 0; }
            if( options.exists( "--output-format" )) { std::cout << "ui" << std::endl; return 0; }
            csv.full_xpath = true;
            if( csv.fields.empty() ) { csv.fields = "x,y,z"; }
            comma::csv::input_stream< point_with_direction > istream( std::cin, csv );
            comma::csv::options output_csv; // quick and dirty
            if( csv.binary() ) { output_csv.format( "ui" ); }
            comma::csv::output_stream< snark::points_calc::trajectory_partition::output > ostream( std::cout, output_csv );
            comma::csv::tied< point_with_direction, snark::points_calc::trajectory_partition::output > tied( istream, ostream );
            unsigned int id = 0;
            std::function< bool( const point_with_direction& ) > is_new_partition;
            std::string how = options.value< std::string >( "--how", "by-direction" );
            if( how == "by-direction" || how == "direction" ) // by-direction currently is the only policy
            {
                if( !csv.has_paths( "direction" ) ) { std::cerr << "points-calc: trajectory-partition --how=by-distance: please specify direction fields"; return 1; } // todo? if direction field not present, calculate direction from trajectory
                is_new_partition = [&]( const point_with_direction& v )->bool
                                   {
                                       static double threshold = options.value( "--threshold,--angle-threshold", M_PI / 2 );
                                       static boost::optional< double > distance_tolerance = options.optional< double >( "--tolerance,--distance-tolerance,--distance-threshold" );
                                       static boost::optional< Eigen::Vector3d > partition_direction;
                                       static boost::optional< Eigen::Vector3d > previous; // static boost::optional< Eigen::Vector3d > first_point_over_threshold;
                                       static double distance = 0;
                                       if( !partition_direction ) { partition_direction = v.direction; return false; }
                                       double angle = std::abs( Eigen::AngleAxis< double >( Eigen::Quaternion< double >::FromTwoVectors( *partition_direction, v.direction ) ).angle() );
                                       if( std::abs( angle ) < threshold ) { previous.reset(); distance=0; return false; } // todo? apply distance tolerance to 'good' angles, too? also, i don't think angle can be negative
                                       if( distance_tolerance )
                                       {
                                           if( !previous ) { previous = v; distance = 0; return false; }
                                           distance += ( v - *previous ).norm();
                                           previous = v;
                                           if( distance < distance_tolerance ) { return false; }
                                       }
                                       previous.reset();
                                       distance = 0;
                                       partition_direction = v.direction;
                                       return true;
                                   };
            }
            else
            {
                std::cerr << "points-calc: trajectory-partition: expected --how, got: '" << how << "'" << std::endl;
                return 1;
            }
            while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const point_with_direction* p = istream.read();
                if( !p ) { break; }
                if( is_new_partition( *p ) ) { ++id; }
                tied.append( snark::points_calc::trajectory_partition::output( id ) );
            }
            return 0;
        }
        std::cerr << "points-calc: expected operation, got: \"" << operation << "\"" << std::endl;
        return 1;
    }
    catch( std::exception& ex ) { std::cerr << "points-calc: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-calc: unknown exception" << std::endl; }
    return 1;
}
