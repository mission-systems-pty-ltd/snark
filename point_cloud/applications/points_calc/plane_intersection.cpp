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
#include "plane_intersection.h"

struct plane_intersection
{
    typedef Eigen::Vector3d output_t;
    typedef Eigen::Vector3d vector;
    //updates csv with default fields if --plane option specified
    static void process(const comma::command_line_options& options, comma::csv::options csv);
    static inline vector zero() { return Eigen::Vector3d::Zero(); }
    struct line_t
    {
        std::pair<vector, vector> points;
        line_t() : points(zero(), zero()) { }
        vector direction() const { return points.second - points.first; }
    };
    struct plane_t
    {
        vector point;
        vector normal;
        plane_t() : point(zero()), normal(zero()) { }
        plane_t(const plane_t& rhs) : point(rhs.point), normal(rhs.normal) { }
    };
    struct input_t
    {
        plane_t plane;
        line_t line;
        input_t() { }
        input_t(const plane_t& p):plane(p) { }
        input_t(const plane_t& p, const line_t& l):plane(p),line(l) { }
    };
    static void run( const comma::csv::options& csv_opt, const plane_intersection::input_t& default_value, const comma::command_line_options& options );
    struct condition;
};

namespace comma { namespace visiting {

template <> struct traits< plane_intersection::line_t >
{
    template< typename K, typename V > static void visit( const K& k, plane_intersection::line_t& t, V& v )
    {
        traits<std::pair<Eigen::Vector3d, Eigen::Vector3d> >::visit(k, t.points, v);
    }
    template< typename K, typename V > static void visit( const K& k, const plane_intersection::line_t& t, V& v )
    {
        traits<std::pair<Eigen::Vector3d, Eigen::Vector3d> >::visit(k, t.points, v);
    }
};

template <> struct traits< plane_intersection::plane_t >
{
    template< typename K, typename V > static void visit( const K& k, plane_intersection::plane_t & t, V& v )
    {
        v.apply( "point", t.point);
        v.apply( "normal", t.normal);
    }
    template< typename K, typename V > static void visit( const K& k, const plane_intersection::plane_t & t, V& v )
    {
        v.apply( "point", t.point);
        v.apply( "normal", t.normal);
    }
};

template <> struct traits< plane_intersection::input_t >
{
    template< typename K, typename V > static void visit( const K& k, plane_intersection::input_t& t, V& v )
    {
        traits<plane_intersection::line_t>::visit(k, t.line, v);
        v.apply( "plane", t.plane);
    }
    template< typename K, typename V > static void visit( const K& k, const plane_intersection::input_t& t, V& v )
    {
        traits<plane_intersection::line_t>::visit(k, t.line, v);
        v.apply( "plane", t.plane);
    }
};


} } // namespace comma { namespace visiting {

// todo:
//     trajectory
//         input/output fields

void plane_intersection::process(const comma::command_line_options& options, comma::csv::options csv)
{
    if(options.exists( "--input-fields" ) ) { std::cout<<comma::join( comma::csv::names<plane_intersection::input_t>(true), ',' ) << std::endl; return; }
    if(options.exists( "--output-fields" ) ) { std::cout<<comma::join( comma::csv::names<plane_intersection::output_t>(false), ',' ) << std::endl; return; }
    if(options.exists( "--output-format" ) ) { std::cout<<comma::csv::format::value<plane_intersection::output_t>() << std::endl; return; }
    boost::optional<std::string> plane_option=options.optional<std::string>("--plane");
    plane_t default_plane=comma::csv::ascii<plane_t>().get(options.optional<std::string>("--plane"));
    line_t default_line=comma::csv::ascii<line_t>().get(options.optional<std::string>("--line"));
    if(plane_option && csv.fields.empty()) { csv.fields=comma::join( comma::csv::names<plane_intersection::line_t>(true), ','); }
    run(csv, plane_intersection::input_t( default_plane, default_line ), options );
}

class plane_intersection::condition
{
    public:
        condition( const comma::command_line_options& options ) : closed( false, false )
        {
            // todo: parse something like points-calc plane-intersection --condition="(inf,first]&[second,inf)" 
        }
        
        bool operator()( const std::pair< Eigen::Vector3d, Eigen::Vector3d >& line, const Eigen::Vector3d& p ) const
        {
            if( !( sign.first || sign.second ) ) { return true; } // always true for now until the todo item above is implemented
            const Eigen::Vector3d& d = line.second - line.first;
            bool result = included_( d, line.first, p, sign.first, closed.first );
            if( is_and == !result ) { return result; }
            return included_( d, line.second, p, sign.first, closed.first );
        }
        
    private:
        std::pair< bool, bool > closed;
        std::pair< boost::optional< int >, boost::optional< int > > sign;
        bool is_and;
        bool included_( const Eigen::Vector3d& d, const Eigen::Vector3d& v, const Eigen::Vector3d& p, const boost::optional< int >& sign, bool closed ) const
        {
            if( !sign ) { return true; }
            const Eigen::Vector3d& e = p - v;
            double s = d.dot( e ) * *sign;
            return closed ? !comma::math::less( s, 0 ) : comma::math::less( 0, s ); // todo: implement optional epsilon, otherwise closed interval is dodgy
        }
};

void plane_intersection::run( const comma::csv::options& csv_opt, const plane_intersection::input_t& default_value, const comma::command_line_options& options )
{
    bool discard_collinear = options.exists( "--discard-collinear" );
    plane_intersection::condition condition( options );
    comma::csv::input_stream<plane_intersection::input_t > is( std::cin, csv_opt, default_value);
    comma::csv::output_stream<plane_intersection::output_t> os( std::cout, csv_opt.binary(), false, csv_opt.flush );
    comma::csv::tied<plane_intersection::input_t ,plane_intersection::output_t> tied( is, os );
    static const Eigen::Vector3d infinity( std::numeric_limits< double >::infinity(), std::numeric_limits< double >::infinity(), std::numeric_limits< double >::infinity() );
    while( is.ready() || std::cin.good() )
    {
        const plane_intersection::input_t* rec=is.read();
        if( !rec ) { break; }
        double u = ( rec->plane.point - rec->line.points.first ).dot( rec->plane.normal );
        double d = rec->line.direction().dot( rec->plane.normal );
        if( comma::math::equal( u, 0 ) ) // if both u and d are 0 then line is on the plane and we just output first point
        { 
            if( condition( rec->line.points, rec->line.points.first ) ) { tied.append( rec->line.points.first ); }
            continue;
        }
        if( comma::math::equal( d, 0 ) )
        { 
            if( !discard_collinear ) { tied.append( infinity ); }
            continue;
        }
        const Eigen::Vector3d& intersection = ( u / d ) * rec->line.direction() + rec->line.points.first;
        if( condition( rec->line.points, intersection ) ) { tied.append( intersection ); }
    }
}

namespace snark { namespace points_calc { namespace plane_intersection {

std::string traits::usage()
{
    std::ostringstream oss;
    oss
        << "    plane-intersection" << std::endl
        << "        read points and plane from stdin, output intersection of the ray with" << std::endl
        << "        the plane" << std::endl
        << std::endl
        << "        input fields: " << comma::join( comma::csv::names< ::plane_intersection::input_t >(true), ',' ) << std::endl
        << "            first and second are two points on the line; the plane passes" << std::endl
        << "            through the plane/point and is perpendicular to the direction" << std::endl
        << "            vector plane/normal" << std::endl
        << std::endl;
    oss
        << "        options:" << std::endl
        << "            --discard-collinear: don't output records when line is in parallel" << std::endl
        << "                    with plane (when not sepcified it outputs <inf,inf,inf>)" << std::endl
        << "            --plane=<" << comma::join( comma::csv::names< ::plane_intersection::plane_t >(true), ',') <<  ">:" << std::endl
        << "                default values for plane" << std::endl
        << "                if --plane specified, default fields for input are:" << std::endl
        << "                    " << comma::join( comma::csv::names< ::plane_intersection::line_t >(true), ',') <<  std::endl
        << "            --line=<" << comma::join( comma::csv::names< ::plane_intersection::line_t >(true), ',') <<  ">:" << std::endl
        << "                default values for line" << std::endl
        << "            --input-fields: print default input field names" << std::endl
        << std::endl;
    oss
        << "        example:" << std::endl
        << "            to find intersection of a line going from 0,0,0 to 1,1,1" << std::endl
        << "            and a plane at 0,0,4 perpendicular to z axis:" << std::endl
        << "                echo 1,1,1,4 | points-calc plane-intersection \\" << std::endl
        << "                               --plane=0,0,0,0,0,1 --fields=second,plane/point/z" << std::endl
        << "            outputs: 1,1,1,4,4,4,4" << std::endl;
    return oss.str();
}

void traits::process( const comma::command_line_options& options, const comma::csv::options& csv ) { ::plane_intersection::process( options, csv ); }

} } } // namespace snark { namespace points_calc { namespace plane_intersection {
