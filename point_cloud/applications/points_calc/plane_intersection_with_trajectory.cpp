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

#include <comma/csv/stream.h>
#include <comma/math/compare.h>
#include "../../../visiting/eigen.h"
#include "plane_intersection_with_trajectory.h"
#include "traits.h"
#include "types.h"

namespace snark { namespace points_calc { namespace plane_intersection_with_trajectory {

struct input : public Eigen::Vector3d
{
    points_calc::plane plane;
    comma::uint32 block;
    
    input() : Eigen::Vector3d( Eigen::Vector3d::Zero() ), block( 0 ) {}
    input( const Eigen::Vector3d& v, const points_calc::plane& p, comma::uint32 block = 0 ) : Eigen::Vector3d( v ), plane( p ), block( block ) {}
};

struct output : public Eigen::Vector3d
{
    comma::int32 direction;
    output() : Eigen::Vector3d( Eigen::Vector3d::Zero() ), direction( 0 ) {}
    output( const Eigen::Vector3d& v, comma::int32 direction ) : Eigen::Vector3d( v ), direction( direction ) {}
};

} } } // namespace snark { namespace points_calc { namespace plane_intersection_with_trajectory {

namespace comma { namespace visiting {

template <> struct traits< snark::points_calc::plane_intersection_with_trajectory::input >
{
    template< typename K, typename V > static void visit( const K& k, snark::points_calc::plane_intersection_with_trajectory::input& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t, v );
        v.apply( "plane", t.plane );
        v.apply( "block", t.block );
    }
    
    template< typename K, typename V > static void visit( const K& k, const snark::points_calc::plane_intersection_with_trajectory::input& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t, v );
        v.apply( "plane", t.plane );
        v.apply( "block", t.block );
    }
};

template <> struct traits< snark::points_calc::plane_intersection_with_trajectory::output >
{
    template< typename K, typename V > static void visit( const K& k, const snark::points_calc::plane_intersection_with_trajectory::output& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t, v );
        v.apply( "direction", t.direction );
    }
};
 
} } // namespace comma { namespace visiting {

namespace snark { namespace points_calc { namespace plane_intersection_with_trajectory {

std::string traits::input_fields() { return comma::join( comma::csv::names< input >( true ), ',' ); }

std::string traits::input_format() { return comma::csv::format::value< input >(); }
    
std::string traits::output_fields() { return comma::join( comma::csv::names< output >( true ), ',' ); }

std::string traits::output_format() { return comma::csv::format::value< output >(); }
 
std::string traits::usage()
{
    std::ostringstream oss;
    oss << "    plane-intersection-with-trajectory" << std::endl
        << "        input points are a trajectory; for each intersection with the trajectory" << std::endl
        << "        output the input records for adjacent points between which intersection" << std::endl
        << "        is, the intersection point, and the direction of intersection (-1,0,+1)," << std::endl
        << "        where 0 indicates that both adjacent points are on the plane," << std::endl
        << "        +1 if the trajectory's direction is the same as normal of the plane," << std::endl
        << "        and -1 otherwise" << std::endl
        << std::endl
        << "        fields:" << std::endl
        << "            if block field is present, output plane intersections only for" << std::endl
        << "            trajectory points belonging to the same block" << std::endl
        << std::endl
        << "        options:" << std::endl
        << "            --plane-point,--point=<x>,<y>,<z>: point on the plane" << std::endl
        << "            --plane-normal,--normal=<x>,<y>,<z>: plane normal" << std::endl
        << "            --threshold=<distance>: any separation between contiguous points of" << std::endl
        << "                    trajectory greater than threshold will be treated as a gap" << std::endl
        << "                    in the trajectory (no intersections will lie in the gaps)" << std::endl
        << "            --v=<x>,<y>,<z>: default trajectory point" << std::endl;
    return oss.str();
}

static Eigen::Vector3d vector_( const std::string& option, const comma::command_line_options& options )
{
    static const comma::csv::ascii< Eigen::Vector3d > ascii;
    return ascii.get( options.value< std::string >( option, "0,0,0" ) );
}

int traits::run( const comma::command_line_options& options )
{
    comma::csv::options csv( options );
    if( csv.fields.empty() ) { csv.fields = "x,y,z"; }
    comma::csv::input_stream< input > istream( std::cin, csv, input( vector_( "--v", options ), points_calc::plane( vector_( "--plane-normal,--normal", options ), vector_( "--plane-point,--point", options ) ) ) );
    comma::csv::options output_csv;
    output_csv.delimiter = csv.delimiter;
    if( csv.binary() ) { output_csv.format( output_format() ); }
    comma::csv::output_stream< output > ostream( std::cout, output_csv );
    boost::optional< double > threshold = options.optional< double >( "--threshold" );
    input last;
    std::string last_record;
    while( istream.ready() || std::cin.good() )
    {
        const input* p = istream.read();
        if( !p ) { break; }
        if( !last_record.empty() )
        {
            if( last.block == p->block && ( !threshold || ( *p - last ).norm() <= *threshold ) )
            {
                double dot = ( *p - p->plane.point ).dot( p->plane.normal );
                double last_dot = ( last - p->plane.point ).dot( p->plane.normal );
                if( dot * last_dot <= 0 ) // points on the different sides of the plane
                {
                    output o(   comma::math::equal( dot, 0 ) && comma::math::equal( last_dot, 0 )
                            ? static_cast< Eigen::Vector3d >( last )
                            : Eigen::ParametrizedLine< double, 3 >::Through( last, *p ).intersectionPoint( Eigen::Hyperplane< double, 3 >( p->plane.normal.normalized(), p->plane.point ) )
                            , !comma::math::equal( last_dot, 0 )
                            ? ( last_dot < 0 ? 1 : -1 )
                            : !comma::math::equal( dot, 0 ) ? ( dot > 0 ? 1 : -1 ) : 0 );
                    if( csv.binary() )
                    {
                        std::cout.write( &last_record[0], output_csv.format().size() );
                        std::cout.write( istream.binary().last(), output_csv.format().size() );
                        const std::vector< char >& v = ostream.binary().binary().put( o ); // quick and dirty; watch performance
                        std::cout.write( &v[0], v.size() );
                    }
                    else
                    {
                        std::cout << last_record
                                << csv.delimiter << comma::join( istream.ascii().last(), csv.delimiter )
                                << csv.delimiter << ostream.ascii().ascii().put( o ) << std::endl;
                    }
                    if( csv.flush ) { std::cout.flush(); }
                }
            }
        }
        last = *p;
        if( csv.binary() ) { last_record.resize( csv.format().size() ); ::memcpy( &last_record[0], istream.binary().last(), csv.format().size() ); }
        else { last_record = comma::join( istream.ascii().last(), csv.delimiter ); }
    }
    return 0;
}

} } } // namespace snark { namespace points_calc { namespace project { namespace onto_line {
