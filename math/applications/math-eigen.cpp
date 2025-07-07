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

/// @author vsevolod vlaskine

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <iostream>
#include <vector>
#include <boost/scoped_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <comma/application/command_line_options.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include "../../visiting/traits.h"
#include "../rotation_matrix.h"

void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "simple wrapper for eigen library operations" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat sample.csv | math-eigen <operation> [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "<operation>: eigen, rotation" << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations" << std::endl;
    std::cerr << "    eigen (default): calculate eigen vector and eigen values on a sample" << std::endl;
    std::cerr << "        fields" << std::endl;
    std::cerr << "            block: block number; output eigen vectors and eigen values for each" << std::endl;
    std::cerr << "                   contiguous block of samples with the same block id" << std::endl;
    std::cerr << "            data: sample data" << std::endl;
    std::cerr << "            default: data" << std::endl;
    std::cerr << "        output" << std::endl;
    std::cerr << "            default output fields" << std::endl;
    std::cerr << "                one eigen vector per line; if block field present: vector,value,block" << std::endl;
    std::cerr << "                                           if no block field present: vector,value" << std::endl;
    std::cerr << "            binary format: 32-bit unsigned integer for block, doubles for other output fields" << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --normalize,-n: output normalized eigen values" << std::endl;
    std::cerr << "            --rsort,--descending: output eigen vectors and values in descending order of eigen values" << std::endl;
    std::cerr << "            --sort,-s,--ascending: output eigen vectors and values in ascending order of eigen values" << std::endl;
    std::cerr << "            --single-line-output,--single-line,--single: output eigen vectors and eigen values all as one line:" << std::endl;
    std::cerr << "                                                         vector[0],vector[1],...,value[0],value[1],...,block" << std::endl;
    std::cerr << "            --size: a hint of number of elements in the data vector, ignored, if data indices" << std::endl;
    std::cerr << "                    specified, e.g. data[0],data[1],data[2]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    fit plane: fit plane to dataset, output its mean and normal" << std::endl;
    std::cerr << "        fields" << std::endl;
    std::cerr << "            block: block number; fit plane for each contiguous block of samples with the same block id" << std::endl;
    std::cerr << "            data: sample data" << std::endl;
    std::cerr << "            default: data" << std::endl;
    std::cerr << "        output" << std::endl;
    std::cerr << "            output fields: mean,normal[,block]" << std::endl;
    std::cerr << "            binary format: 32-bit unsigned integer for block, doubles for other output fields" << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --size: a hint of number of elements in the data vector, ignored, if data indices" << std::endl;
    std::cerr << "                    specified, e.g. data[0],data[1],data[2]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    rotation: convert rotation from representation to another, append the result to stdin data, output to stdout" << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --from=<what>: input representation; default euler" << std::endl;
    std::cerr << "            --to=<what>: output representation; default euler" << std::endl;
    std::cerr << "                <what>" << std::endl;
    std::cerr << "                    euler;rpy;roll,pitch,yaw: euler angles, i.e. roll, pitch, yaw" << std::endl;
    std::cerr << "                    axis-angle,angle-axis: angle-axis" << std::endl;
    std::cerr << "                    axis-angle-scaled: rotation axis with the norm equal to rotation angle" << std::endl;
    std::cerr << "                    quaternion: quaternion" << std::endl;
    std::cerr << "                    rotation-matrix,matrix: rotation matrix" << std::endl;
    std::cerr << "            --input-fields: print input fields to stdout and exit" << std::endl;
    std::cerr << "            --output-fields: print output fields to stdout and exit" << std::endl;
    std::cerr << "            --output-format: print binary output format to stdout and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: show this help; --help --verbose: more help" << std::endl;
    if( verbose ) { std::cerr << std::endl << "csv options" << std::endl << comma::csv::options::usage() << std::endl; }
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    eigen" << std::endl;
    std::cerr << "        cat sample.csv | math-eigen" << std::endl;
    std::cerr << "        cat sample.csv | math-eigen --fields=block,data" << std::endl;
    std::cerr << "        cat sample.csv | math-eigen --fields=block,,,data --size=4" << std::endl;
    std::cerr << "        cat sample.csv | math-eigen --fields=block,,data[0],,data[1],,data[2],,data[3]" << std::endl;
    std::cerr << "        cat sample.bin | math-eigen --fields=block,data --binary=ui,6d" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

static boost::optional< unsigned int > size;

namespace snark { namespace eigen {

struct input_t
{
    std::vector< double > data;
    comma::uint32 block;
    
    input_t() : data( *size ), block( 0 ) {}
};

struct output_t
{
    std::vector< double > vector;
    double value;
    comma::uint32 block;
    
    output_t() : vector( *size ), value( 0 ), block( 0 ) {}
};

struct single_line_output_t
{
    std::vector< double > vectors; // quick and dirty
    std::vector< double > values;
    comma::uint32 block;
    
    single_line_output_t(): vectors( *size * *size ), values( *size ), block( 0 ) {}
};

struct fit_plane_output_t
{
    std::vector< double > mean;
    std::vector< double > normal;
    comma::uint32 block;
    
    fit_plane_output_t(): mean( *size ), normal( *size ), block( 0 ) {}
};

} } // namespace snark { namespace eigen {

namespace comma { namespace visiting {

template <> struct traits< snark::eigen::input_t >
{
    template < typename K, typename V > static void visit( const K&, snark::eigen::input_t& p, V& v )
    {
        v.apply( "data", p.data );
        v.apply( "block", p.block );
    }

    template < typename K, typename V > static void visit( const K&, const snark::eigen::input_t& p, V& v )
    {
        v.apply( "data", p.data );
        v.apply( "block", p.block );
    }
};

template <> struct traits< snark::eigen::output_t >
{
    template < typename K, typename V > static void visit( const K&, const snark::eigen::output_t& p, V& v )
    {
        v.apply( "vector", p.vector );
        v.apply( "value", p.value );
        v.apply( "block", p.block );
    }
};

template <> struct traits< snark::eigen::single_line_output_t >
{
    template < typename K, typename V > static void visit( const K&, const snark::eigen::single_line_output_t& p, V& v )
    {
        v.apply( "vectors", p.vectors );
        v.apply( "values", p.values );
        v.apply( "block", p.block );
    }
};

template <> struct traits< snark::eigen::fit_plane_output_t >
{
    template < typename K, typename V > static void visit( const K&, const snark::eigen::fit_plane_output_t& p, V& v )
    {
        v.apply( "mean", p.mean );
        v.apply( "normal", p.normal );
        v.apply( "block", p.block );
    }
};

} } // namespace comma { namespace visiting {

namespace rotation {

template < typename T > struct traits;

template <> struct traits< snark::roll_pitch_yaw >
{
    static snark::roll_pitch_yaw zero() { return snark::roll_pitch_yaw(); }
    static snark::roll_pitch_yaw get( const snark::rotation_matrix& m ) { return m.roll_pitch_yaw(); }
};

template < typename T > struct traits< Eigen::AngleAxis< T > >
{
    static Eigen::AngleAxis< T > zero() { Eigen::AngleAxis< T > a( 0, Eigen::Matrix< T, 3, 1 >::Zero() ); return a; }
    static Eigen::AngleAxis< T > get( const snark::rotation_matrix& m ) { return Eigen::AngleAxis< T >( m.rotation() ); }
};

template <> struct traits< Eigen::Vector3d >
{
    static Eigen::Vector3d zero() { return Eigen::Vector3d::Zero(); }
    static Eigen::Vector3d get( const snark::rotation_matrix& m ) { const Eigen::AngleAxis< double > a( m.rotation() ); return a.axis() * a.angle(); }
};

template < typename T > struct traits< Eigen::Quaternion< T > >
{
    static Eigen::Quaternion< T > zero() { Eigen::Quaternion< T > q( 0, 0, 0, 0 ); return q; }
    static Eigen::Quaternion< T > get( const snark::rotation_matrix& m ) { return m.quaternion(); }
};

template <> struct traits< Eigen::Matrix< double, 3, 3 > >
{
    static Eigen::Matrix< double, 3, 3 > zero() { return Eigen::Matrix< double, 3, 3 >::Zero(); }
    static Eigen::Matrix< double, 3, 3 > get( const snark::rotation_matrix& m ) { return Eigen::Matrix< double, 3, 3 >( m.rotation() ); }
};
    
template < typename From, typename To >
static int run( const comma::command_line_options& options )
{
    if( options.exists( "--input-fields" ) ) { std::cout << comma::join( comma::csv::names< From >( true, rotation::traits< From >::zero() ), ',' ) << std::endl; return 0; }
    if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< To >( true, rotation::traits< To >::zero() ), ',' ) << std::endl; return 0; }
    if( options.exists( "--output-format" ) ) { std::cout << comma::csv::format::value< To >( rotation::traits< To >::zero() ) << std::endl; return 0; }
    comma::csv::options csv( options );
    csv.full_xpath = true;
    comma::csv::input_stream< From > istream( std::cin, csv, rotation::traits< From >::zero() );
    comma::csv::options output_csv;
    if( csv.binary() ) { output_csv.format( comma::csv::format::value< To >( rotation::traits< To >::zero() ) ); }
    output_csv.full_xpath = true;
    comma::csv::output_stream< To > ostream( std::cout, output_csv, rotation::traits< To >::zero() );
    while( istream.ready() || std::cin.good() )
    {
        const From* p = istream.read();
        if( !p ) { break; }
        comma::csv::append( istream, ostream, rotation::traits< To >::get( snark::rotation_matrix( *p ) ) );
        if( csv.flush ) { ostream.flush(); }
    }
    return 0;
}
    
template < typename From >
static int run( const comma::command_line_options& options, const std::string& to )
{
    if( to == "euler" || to == "rpy" || to == "roll,pitch,yaw" ) { return rotation::run< From, snark::roll_pitch_yaw >( options ); }
    if( to == "angle-axis" || to == "axis-angle" ) { return rotation::run< From, Eigen::AngleAxis< double > >( options ); }
    if( to == "axis-angle-scaled" ) { return rotation::run< From, Eigen::Vector3d >( options ); }
    if( to == "quaternion" ) { return rotation::run< From, Eigen::Quaternion< double > >( options ); }
    if( to == "rotation-matrix" || to == "matrix" ) { return rotation::run< From, Eigen::Matrix< double, 3, 3 > >( options ); }
    std::cerr << "math-eigen: rotation: expected valid value for --to; got --to=\"" << to << "\"" << std::endl;
    return 1;
}

} // namespace rotation {

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        const std::vector< std::string >& unnamed = options.unnamed( "--flush,--normalize,-n,--sort,-s,--ascending,--rsort,--descending,--single-line-output,--single-line,--single,--verbose,-v" );
        std::string operation = unnamed.empty() ? std::string( "eigen" ) : unnamed[0];
        if( operation == "eigen" || operation == "fit-plane" )
        {
            comma::csv::options csv( options );
            size = options.optional< unsigned int >( "--size" );
            if( csv.fields.empty() ) { csv.fields = "data"; }
            std::string first;
            if( !size )
            {
                const std::vector< std::string >& fields = comma::split( csv.fields, ',' );
                if( csv.has_field( "data" ) )
                {
                    unsigned int count;
                    if( csv.binary() )
                    {
                        count = csv.format().count();
                    }
                    else
                    {
                        while( std::cin.good() && first.empty() ) { std::getline( std::cin, first ); }
                        count = comma::split( first, csv.delimiter ).size(); // quick and dirty, wasteful
                    }
                    size = count - fields.size() + 1;
                }
                else
                {
                    unsigned int max = 0;
                    for( unsigned int i = 0; i < fields.size(); ++i )
                    {
                        if( fields[i].substr( 0, 5 ) == "data[" && *fields[i].rbegin() == ']' ) { unsigned int k = boost::lexical_cast< unsigned int >( fields[i].substr( 5, fields[i].size() - 6 ) ) + 1; if( k > max ) { max = k; } }
                    }
                    if( max == 0 ) { std::cerr << "math-eigen: please specify valid data fields" << std::endl; return 1; }
                    size = max;
                }
            }
            
            bool has_block = csv.has_field( "block" );
            std::deque< snark::eigen::input_t > buffer;
            if( !first.empty() ) { buffer.push_back( comma::csv::ascii< snark::eigen::input_t >( csv ).get( first ) ); }
            comma::csv::input_stream< snark::eigen::input_t > istream( std::cin, csv );
            std::vector< unsigned int > indices( *size ); // quick and dirty
            for( unsigned int i = 0; i < indices.size(); indices[i] = i, ++i );
            if( operation == "eigen" )
            {
                bool single_line_output = options.exists( "--single-line-output,--single-line,--single" );
                bool normalize = options.exists( "--normalize,-n" );
                bool sort = options.exists( "--sort,--ascending,-s,--descending,--rsort" );
                bool ascending = sort && !options.exists( "--descending,--rsort" );
                comma::csv::options output_csv;
                output_csv.fields = single_line_output ? has_block ? "vectors,values,block" : "vectors,values"
                                                       : has_block ? "vector,value,block" : "vector,value";
                if( csv.binary() )
                {
                    std::string s = boost::lexical_cast< std::string >( single_line_output ? *size * ( *size + 1 ) : ( *size + 1 ) );
                    output_csv.format( has_block ? s + "d,ui" : s + "d" );
                }
                output_csv.flush = csv.flush;
                output_csv.precision = csv.precision;
                boost::scoped_ptr< comma::csv::output_stream< snark::eigen::output_t > > ostream;
                boost::scoped_ptr< comma::csv::output_stream< snark::eigen::single_line_output_t > > single_line_ostream;
                if( single_line_output ) { single_line_ostream.reset( new comma::csv::output_stream< snark::eigen::single_line_output_t >( std::cout, output_csv ) ); }
                else { ostream.reset( new comma::csv::output_stream< snark::eigen::output_t >( std::cout, output_csv ) ); }
                while( true )
                {
                    const snark::eigen::input_t* p = istream.read();
                    if( !p || ( !buffer.empty() && buffer.front().block != p->block ) )
                    {
                        typedef Eigen::Matrix< double, -1, -1, Eigen::RowMajor > matrix_t;
                        matrix_t sample( buffer.size(), *size );
                        for( std::size_t i = 0; i < buffer.size(); ++i ) { ::memcpy( &sample( i, 0 ), &buffer[i].data[0], *size * sizeof( double ) ); } // dodgy?
                        //if( buffer.size() == 1 ) { std::cerr << "math-eigen: on block " << buffer.front().block << ": expected block with at least two entries, got only one" << std::endl; return 1; }
                        matrix_t covariance = sample.adjoint() * sample;
                        covariance = covariance / ( sample.rows() - 1 );                    
                        Eigen::SelfAdjointEigenSolver< matrix_t > solver( covariance );
                        Eigen::VectorXd values = solver.eigenvalues();
                        if( normalize ) { values = values / solver.eigenvalues().sum(); }
                        const matrix_t& vectors = solver.eigenvectors().transpose();
                        if( sort )
                        {
                            std::map< double, unsigned int > m; // quick and dirty, watch performance
                            for( unsigned int i = 0; i < indices.size(); m[ ascending ? values[i] : -values[i] ] = i, ++i );
                            unsigned int i = 0;
                            for( std::map< double, unsigned int >::const_iterator it = m.begin(); it != m.end(); indices[i] = it->second, ++it, ++i );
                        }
                        if( single_line_output )
                        {
                            snark::eigen::single_line_output_t output;
                            output.block = buffer.front().block;
                            if( sort )
                            {
                                for( unsigned int i = 0; i < indices.size(); ++i )
                                {
                                    output.values[i] = values[ indices[i] ];
                                    ::memcpy( &output.vectors[ i * *size ], &vectors( indices[i], 0 ), *size * sizeof( double ) ); // quick and dirty
                                }
                            }
                            else
                            {
                                ::memcpy( &output.vectors[0], &vectors( 0, 0 ), *size * *size * sizeof( double ) ); // quick and dirty
                                ::memcpy( &output.values[0], &values[0], *size * sizeof( double ) );
                            }
                            single_line_ostream->write( output );
                            if( csv.flush ) { single_line_ostream->flush(); }
                        }
                        else
                        {
                            for( std::size_t i = 0; i < *size; ++i )
                            {
                                snark::eigen::output_t output;
                                output.block = buffer.front().block;
                                ::memcpy( &output.vector[0], &vectors( indices[i], 0 ), *size * sizeof( double ) );
                                output.value = values[ indices[i] ];
                                ostream->write( output );
                                if( csv.flush ) { ostream->flush(); }
                            }
                        }
                        buffer.clear();
                    }
                    if( !p ) { break; }
                    buffer.push_back( *p );
                }
                return 0;
            }
            if( operation == "fit-plane" ) // see http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
            {
                comma::csv::options output_csv;
                output_csv.fields = has_block ? "mean,normal,block" : "mean,normal";
                if( csv.binary() )
                {
                    std::string s = boost::lexical_cast< std::string >( *size );
                    output_csv.format( has_block ? s + "d," + s + "d,ui" : s + "d," + s + "d" );
                }
                output_csv.flush = csv.flush;
                std::cout.precision( csv.precision );
                comma::csv::output_stream< snark::eigen::fit_plane_output_t > ostream( std::cout, output_csv );
                while( true )
                {
                    const snark::eigen::input_t* p = istream.read();
                    if( !p || ( !buffer.empty() && buffer.front().block != p->block ) )
                    {
                        typedef Eigen::Matrix< double, -1, -1 > matrix_t;
                        matrix_t sample( *size, buffer.size() );
                        for( std::size_t i = 0; i < buffer.size(); ++i ) // todo: watch performance, cache misses; use tbb::parallel_for?
                        {
                            for( std::size_t j = 0; j < *size; ++j ) { sample.row( j )[i] = buffer[i].data[j]; }
                        }
                        Eigen::VectorXd mean( *size );
                        for( std::size_t i = 0; i < *size; ++i )
                        {
                            mean( i ) = sample.row( i ).mean();
                            sample.row( i ).array() -= mean( i );
                        }
                        Eigen::VectorXd normal = sample.jacobiSvd( Eigen::ComputeThinU | Eigen::ComputeThinV ).matrixU().rightCols< 1 >();
                        snark::eigen::fit_plane_output_t output;
                        ::memcpy( &output.mean[0], &mean[0], *size * sizeof( double ) ); // quick and dirty
                        ::memcpy( &output.normal[0], &normal[0], *size * sizeof( double ) ); // quick and dirty
                        output.block = buffer.front().block;
                        ostream.write( output );
                        buffer.clear();
                    }
                    if( !p ) { break; }
                    buffer.push_back( *p );
                }
                return 0;
            }
                
        }
        if( operation == "rotation" )
        {
            std::string from = options.value< std::string >( "--from", "euler" );
            std::string to = options.value< std::string >( "--to", "euler" );
            if( from == "euler" || from == "rpy" || from == "roll,pitch,yaw" ) { return rotation::run< snark::roll_pitch_yaw >( options, to ); }
            if( from == "angle-axis" || from == "axis-angle" ) { return rotation::run< Eigen::AngleAxis< double > >( options, to ); }
            if( from == "axis-angle-scaled" ) { return rotation::run< Eigen::Vector3d >( options, to ); }
            if( from == "quaternion" ) { return rotation::run< Eigen::Quaternion< double > >( options, to ); }
            if( from == "rotation-matrix" || from == "matrix" ) { return rotation::run< Eigen::Matrix< double, 3, 3 > >( options, to ); }
            std::cerr << "math-eigen: rotation: expected valid value for --from; got --from=\"" << from << "\"" << std::endl;
            return 1;
        }
        std::cerr << "math-eigen: expected operation, got \"" << operation << "\"" << std::endl;
    }
    catch( std::exception& ex ) { std::cerr << "math-eigen: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "math-eigen: unknown exception" << std::endl; }
    return 1;
}
