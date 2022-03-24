// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney
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

/// @author dave jennings

#include <cmath>
#include <comma/application/verbose.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include "../../math/applications/frame.h"

typedef std::pair< Eigen::Vector3d, Eigen::Vector3d > point_pair_t;

struct input_points
{
    point_pair_t points;
    comma::uint32 block;

    input_points() : block( 0 ) {}
};

struct output_t
{
    comma::uint32 block;
    snark::applications::position position;
    double scale;
    double error;

    output_t() : block( 0 ), scale( 1 ), error( 0 ) {}
    output_t( comma::uint32 block
            , const snark::applications::position& position
            , double scale
            , double error )
        : block( block )
        , position( position )
        , scale( scale )
        , error( error )
    {}
};

namespace comma { namespace visiting {

template <> struct traits< input_points >
{
    template < typename K, typename V > static void visit( const K&, input_points& p, V& v )
    {
        v.apply( "block", p.block );
        v.apply( "first", p.points.first );
        v.apply( "second", p.points.second );
    }

    template < typename K, typename V > static void visit( const K&, const input_points& p, V& v )
    {
        v.apply( "block", p.block );
        v.apply( "first", p.points.first );
        v.apply( "second", p.points.second );
    }
};

template <> struct traits< output_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, output_t& p, Visitor& v )
    {
        v.apply( "block", p.block );
        v.apply( "x", p.position.coordinates.x() );
        v.apply( "y", p.position.coordinates.y() );
        v.apply( "z", p.position.coordinates.z() );
        v.apply( "roll", p.position.orientation.x() );
        v.apply( "pitch", p.position.orientation.y() );
        v.apply( "yaw", p.position.orientation.z() );
        v.apply( "scale", p.scale );
        v.apply( "error", p.error );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const output_t& p, Visitor& v )
    {
        v.apply( "block", p.block );
        v.apply( "x", p.position.coordinates.x() );
        v.apply( "y", p.position.coordinates.y() );
        v.apply( "z", p.position.coordinates.z() );
        v.apply( "roll", p.position.orientation.x() );
        v.apply( "pitch", p.position.orientation.y() );
        v.apply( "yaw", p.position.orientation.z() );
        v.apply( "scale", p.scale );
        v.apply( "error", p.error );
    }
};

} } // namespace comma { namespace visiting {

static const std::string default_fields = comma::join( comma::csv::names< point_pair_t >(), ',' );
static const std::string standard_output_fields = "x,y,z,roll,pitch,yaw,scale";
static const std::string standard_output_format = "d,d,d,d,d,d,d";

static void bash_completion( unsigned int const ac, char const * const * av )
{
    static const char* completion_options =
        " --help -h"
        " --verbose -v"
        " --input-fields"
        " --output-fields"
        " --output-format"
        " --output-error"
        " --initial-error"
        " --with-scaling"
        ;

    std::cout << completion_options << std::endl;
    exit( 0 );
}

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "give the transform that will align a point cloud with a reference point cloud" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: " << comma::verbose.app_name() << " [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "default input fields: " << default_fields << std::endl;
    std::cerr << std::endl;
    std::cerr << "The input data is assumed to consist of matched points from the reference" << std::endl;
    std::cerr << "point cloud and the point cloud to be aligned. The output is a transform" << std::endl;
    std::cerr << "in the form " << standard_output_fields << " (angles in radians) that can be given to" << std::endl;
    std::cerr << "points-frame to align the data." << std::endl;
    std::cerr << std::endl;
    if( verbose )
    {
        std::cerr << "csv stream options: " << std::endl;
        std::cerr << comma::csv::options::usage() << std::endl;
        std::cerr << std::endl;
    }
    std::cerr << "options: " << std::endl;
    std::cerr << "    --help,-h:       show this help; --help --verbose for more help" << std::endl;
    std::cerr << "    --verbose,-v:    more output" << std::endl;
    std::cerr << "    --initial-error: don't run alignment, just output initial error" << std::endl;
    std::cerr << "    --input-fields:  print input fields to stdout and exit" << std::endl;
    std::cerr << "    --output-fields: print output fields to stdout and exit" << std::endl;
    std::cerr << "    --output-format: print output format to stdout and exit" << std::endl;
    std::cerr << "    --output-error:  include the error estimate in output" << std::endl;
    std::cerr << "    --with-scaling:  allow Umeyama algorithm to use scaling" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples: " << std::endl;
    std::cerr << "    -- output the " << standard_output_fields << " transform --" << std::endl;
    std::cerr << "    cat combined-points.csv | points-align" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    -- align the point cloud to the reference point cloud --" << std::endl;
    std::cerr << "    frame=$( cat combined-points.csv | points-align )" << std::endl;
    std::cerr << "    cat points.csv | points-frame --from $frame --fields=x,y,z" << std::endl;
    std::cerr << std::endl;
    if( verbose )
    {
        std::cerr << "algorithm: " << std::endl;
        std::cerr << "    " << comma::verbose.app_name() << " uses the Umeyama algorithm as implemented by Eigen::umeyama()." << std::endl;
        std::cerr << std::endl;
        std::cerr << "    \"Least-squares estimation of transformation parameters between two point" << std::endl;
        std::cerr << "    patterns\", Shinji Umeyama, PAMI 1991, DOI: 10.1109/34.88573" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    See http://eigen.tuxfamily.org/dox/group__Geometry__Module.html" << std::endl;
        std::cerr << "    for implementation details." << std::endl;
        std::cerr << std::endl;
        std::cerr << "    Note that the --with-scaling option sets the with_scaling argument of" << std::endl;
        std::cerr << "    Eigen::umeyama() to true. By default we set it to false." << std::endl;
        std::cerr << std::endl;
    }
    exit( 0 );
}

std::string get_output_fields( comma::csv::options csv_options, bool output_error )
{
    std::string output_fields( standard_output_fields );
    if( output_error ) { output_fields += ",error"; }
    if( csv_options.has_field( "block" )) { output_fields = output_fields + ",block"; }
    return output_fields;
}

std::string get_output_format( comma::csv::options csv_options, bool output_error )
{
    std::string output_format( standard_output_format );
    if( output_error ) { output_format += ",d"; }
    if( csv_options.has_field( "block" )) { output_format = output_format + ",ui"; }
    return output_format;
}

double error( Eigen::MatrixXd source
            , Eigen::MatrixXd target
            , Eigen::Matrix4d estimate=Eigen::Matrix4d::Identity() )
{
    // Convert source and target to a form that allows multiplication by estimate.
    // Change each vector from size 3 to size 4 and store 1 in the fourth row.
    source.conservativeResize( source.rows()+1, Eigen::NoChange );
    source.row( source.rows()-1 ) = Eigen::MatrixXd::Constant( 1, source.cols(), 1 );

    target.conservativeResize( target.rows()+1, Eigen::NoChange );
    target.row( target.rows()-1 ) = Eigen::MatrixXd::Constant( 1, target.cols(), 1 );

    return ( estimate * source - target ).norm() / target.norm();
}

void output_transform( Eigen::MatrixXd source
                     , Eigen::MatrixXd target
                     , bool with_scaling
                     , comma::uint32 block
                     , comma::csv::options output_csv )
{
    comma::verbose << "loaded " << target.cols() << " pairs of points" << std::endl;
    comma::verbose << "computing Umeyama estimate " << ( with_scaling ? "with" : "without" ) << " scaling" << std::endl;

    Eigen::Matrix4d estimate = Eigen::umeyama( source, target, with_scaling );

    comma::verbose << "umeyama estimate\n" << estimate << std::endl;

    Eigen::Matrix3d rotation( estimate.block< 3, 3 >( 0, 0 ));
    Eigen::Vector3d translation( estimate.block< 3, 1 >( 0, 3 ));
    double scale = 1.0f;

    if( with_scaling )
    {
        scale = cbrt( rotation.determinant() );
        rotation /= scale;
    }

    Eigen::Vector3d orientation( snark::rotation_matrix::roll_pitch_yaw( rotation ));

    comma::verbose << "rotation matrix\n" << rotation << std::endl;
    comma::verbose << "translation ( " << comma::join( translation, ',' ) << " )" << std::endl;
    comma::verbose << "orientation (rad) ( " << comma::join( orientation, ',' ) << " )" << std::endl;
    comma::verbose << "orientation (deg) ( " << comma::join( orientation * 180 / M_PI, ',' ) << " )" << std::endl;
    comma::verbose << "scale=" << scale << std::endl;

    comma::csv::output_stream< output_t > ostream( std::cout, output_csv );
    ostream.write( output_t( block
                           , snark::applications::position( translation, orientation )
                           , scale
                           , error( source, target, estimate )));
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--bash-completion" ) ) { bash_completion( ac, av ); }
        comma::csv::options csv( options );
        csv.full_xpath = true;
        if( csv.fields.empty() ) { csv.fields = default_fields; }
        comma::verbose << "csv.fields=" << csv.fields << std::endl;
        bool output_error = options.exists( "--output-error" );
        bool initial_error = options.exists( "--initial-error" );

        std::string output_fields = get_output_fields( csv, output_error );
        std::string output_format = get_output_format( csv, output_error );

        if( options.exists( "--input-fields" )) { std::cout << comma::join( comma::csv::names< input_points >(), ',' ) << std::endl; return 0; }
        if( options.exists( "--output-format" )) { std::cout << output_format << std::endl; return 0; }
        if( options.exists( "--output-fields" )) { std::cout << output_fields << std::endl; return 0; }

        comma::csv::options output_csv;
        output_csv.fields = output_fields;
        comma::verbose << "output fields=" << output_fields << std::endl;

        Eigen::MatrixXd source( 3, 0 );
        Eigen::MatrixXd target( 3, 0 );
        unsigned int block = 0;
        unsigned int discarded_records = 0;
        bool with_scaling = options.exists( "--with-scaling" );

        comma::csv::input_stream< input_points > istream( std::cin, csv );
        while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
        {
            const input_points* p = istream.read();
            if( !p ) { break; }

            // Discard any records that have a NaN value
            bool discard = false;
            for( int i = 0; i <=2; i++ )
            {
                discard = ( std::isnan( p->points.first(i) ) || std::isnan( p->points.second(i) ));
                if( discard ) { ++discarded_records; break; }
            }
            if( discard ) { continue; }
            if( p->block != block )
            {
                if( initial_error ) { std::cout << error( source, target ) << std::endl; }
                else { output_transform( source, target, with_scaling, block, output_csv ); }
                source.resize( Eigen::NoChange, 0 );
                target.resize( Eigen::NoChange, 0 );
                block = p->block;
            }

            target.conservativeResize( Eigen::NoChange, target.cols()+1 );
            source.conservativeResize( Eigen::NoChange, source.cols()+1 );

            for( int i = 0; i <=2; i++ )
            {
                target(i, target.cols()-1) = p->points.first(i);
                source(i, source.cols()-1) = p->points.second(i);
            }
        }
        if( discarded_records > 0 ) { std::cerr << comma::verbose.app_name() << ": discarded " << discarded_records << " out of " << ( target.cols() + discarded_records ) << " records" << std::endl; }
        if( initial_error ) { std::cout << error( source, target ) << std::endl; }
        else { output_transform( source, target, with_scaling, block, output_csv ); }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": unknown exception" << std::endl; }
    return 1;
}
