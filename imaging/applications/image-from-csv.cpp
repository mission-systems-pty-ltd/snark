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

#include <comma/application/command_line_options.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/name_value/parser.h>
#include <comma/csv/options.h>
#include "../cv_mat/filters.h"
#include "../cv_mat/serialization.h"
#include "../cv_mat/traits.h"
#include "../cv_mat/type_traits.h"
    
struct input_t
{
    boost::posix_time::ptime t;
    double x;
    double y;
    std::vector< double > channels;
    comma::uint32 block;
    
    input_t() : x( 0 ), y( 0 ), block( 0 ) {}
};

namespace comma { namespace visiting {

template <> struct traits< input_t >
{
    template < typename K, typename V > static void visit( const K&, input_t& r, V& v )
    {
        v.apply( "t", r.t );
        v.apply( "x", r.x );
        v.apply( "y", r.y );
        v.apply( "channels", r.channels );
        v.apply( "block", r.block );
    }
    
    template < typename K, typename V > static void visit( const K&, const input_t& r, V& v )
    {
        v.apply( "t", r.t );
        v.apply( "x", r.x );
        v.apply( "y", r.y );
        v.apply( "channels", r.channels );
        v.apply( "block", r.block );
    }
};

} } // namespace comma { namespace visiting {

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "read pixel coordinates and values, output images in the format readable by cv-cat" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: image-from-csv <options>" << std::endl
              << std::endl
              << "options" << std::endl
              << "    --help,-h: show help; --help --verbose: more help" << std::endl
              << "    --background=<colour>; e.g. --background=0, --background=0,-1,-1, etc; default: zeroes" << std::endl
              << "    --from,--begin,--origin=[<x>,<y>]: offset pixel coordinates by a given offset; default: 0,0" << std::endl
              << "    --number-of-blocks,--block-count=[<count>]; if --output-on-missing-blocks, expected number of input blocks" << std::endl
              << "    --output: output options, same as --input for image-from-csv or cv-cat (see --help --verbose)" << std::endl
              << "    --output-on-missing-blocks: output empty images on missing input blocks; input blocks expected ordered" << std::endl
              << "    --output-on-empty-input,--output-on-empty: output empty image on empty input" << std::endl
              << "    --timestamp=<how>: which image timestamp to output" << std::endl
              << "          <how>" << std::endl
              << "              first: timestamp of the first point of a block" << std::endl
              << "              last: timestamp of the last point of a block" << std::endl
              << "              max: maximum timestamp of a block" << std::endl
              << "              mean,average: average timestamp across all points of a block" << std::endl
              << "              middle: average of the first and last timestamp of a block" << std::endl
              << "              min: minimum timestamp of a block" << std::endl
              << "              default: first" << std::endl
              << "    --verbose,-v: more output" << std::endl
              << std::endl
              << "fields: t,x,y,r,g,b,block or t,x,y,grey,block" << std::endl
              << "    t: image timestamp, optional" << std::endl
              << "    x,y: pixel coordinates, if double, will get rounded to the nearest integer" << std::endl
              << "    r,g,b: red, green, blue values" << std::endl
              << "    grey: greyscale value" << std::endl
              << "    channels[0],channels[1],channels[2]: blue, green, red values; notice that the order is bgr" << std::endl
              << "                                         if only channels[0] given, it is the same as specifying grey field" << std::endl
              << "    block: image number, optional" << std::endl
              << std::endl;
    if( verbose )
    {
        std::cerr<< snark::cv_mat::serialization::options::usage() << std::endl << std::endl;
        std::cerr<< "input stream csv options" << std::endl << comma::csv::options::usage() << std::endl << std::endl;
    }
    std::cerr << "example" << std::endl;
    std::cerr << "     cat pixels.csv | image-from-csv --fields x,y,grey --output=\"rows=1200;cols=1000;type=ub\" | cv-cat --output no-header \"encode=png\" > test.bmp" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

static void set_pixel( cv::Mat& m, const input_t& v, const std::pair< double, double >& offset ) // quick and dirty; reimplement as templates
{
    int x = std::floor( v.x + 0.5 - offset.first );
    int y = std::floor( v.y + 0.5 - offset.second );
    snark::cv_mat::set( m, y, x, v.channels );
    return;
}

class timestamping
{
public:
    timestamping( const std::string& s ) : how_( from_string_( s ) ), count_( 0 ) {}
    
    void reset() { t_.reset(); count_ = 0; }
    
    boost::posix_time::ptime value() const { return t_ ? *t_ : boost::posix_time::not_a_date_time; }
    
    void update( const boost::posix_time::ptime& t, bool commit = false )
    {
        //std::cerr << "--> a: t_: " << ( t_ ? boost::posix_time::to_iso_string( *t_ ) : "none" ) << " t: " << boost::posix_time::to_iso_string( t ) << " commit: " << commit << std::endl;
        switch( how_ )
        {
            case first:
                if( !t_ ) { t_ = t; }
                break;
            case last:
                if( commit ) { t_ = t; }
                break;
            case max:
                if( !t_ ) { t_ = t; }
                if( t_->is_not_a_date_time() ) { break; }
                if( t.is_not_a_date_time() ) { t_ = boost::posix_time::not_a_date_time; } else if( *t_ < t ) { t_ = t; }
                break;
            case mean:
                if( t.is_special() || t.is_not_a_date_time() ) { break; }
                if( t_ && t_->is_not_a_date_time() ) { break; }
                if( t_ ) { ++count_; t_ = *t_ + ( t - *t_ ) / count_; }
                else { count_ = 1; t_ = t; }
                break;
            case middle:
                if( !t_ ) { t_ = t; }
                if( !commit ) { break; }
                if( t.is_special() || t.is_not_a_date_time() ) { t_ = boost::posix_time::not_a_date_time; }
                if( !t_->is_not_a_date_time() ) { t_ = *t_ + ( t - *t_ ) / 2; }
                break;
            case min:
                if( !t_ ) { t_ = t; }
                if( t_->is_not_a_date_time() ) { break; }
                if( t.is_not_a_date_time() ) { t_ = boost::posix_time::not_a_date_time; } else if( *t_ > t ) { t_ = t; }
                break;
        }
        //std::cerr << "--> b: t_: " << ( t_ ? boost::posix_time::to_iso_string( *t_ ) : "none" ) << std::endl << std::endl;
    }
    
private:
    enum values_ { first, last, max, mean, middle, min };
    values_ from_string_( const std::string& s )
    {
        if( s == "first" ) { return first; }
        if( s == "last" ) { return last; }
        if( s == "max" ) { return max; }
        if( s == "mean" || s == "average" ) { return mean; }
        if( s == "middle" ) { return middle; }
        if( s == "min" ) { return min; }
        std::cerr << "image-from-csv: expected timestamping method, got: \"" << s << "\"" << std::endl;
        exit( 1 );
    }
    values_ how_;
    boost::optional< boost::posix_time::ptime > t_;
    unsigned int count_;
};

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options csv( options );
        if( csv.fields.empty() ) { std::cerr << "image-from-csv: please specify --fields" << std::endl; return 1; }
        std::vector< std::string > v = comma::split( csv.fields, ',' );
        input_t sample;
        bool is_greyscale = true;
        bool has_alpha = false;
        for( unsigned int i = 0; i < v.size(); ++i ) // quick and dirty, somewhat silly
        {
            if( v[i] == "grey" ) { v[i] = "channels[0]"; }
            else if( v[i] == "b" ) { v[i] = "channels[0]"; is_greyscale = false; }
            else if( v[i] == "g" || v[i] == "channels[1]" ) { v[i] = "channels[1]"; is_greyscale = false; }
            else if( v[i] == "r" || v[i] == "channels[2]" ) { v[i] = "channels[2]"; is_greyscale = false; }
            else if( v[i] == "a" || v[i] == "channels[3]" ) { v[i] = "channels[3]"; is_greyscale = false; has_alpha = true; }
            else if( v[i] == "channels" ) { std::cerr << "image-from-csv: please specify channels fields explicitly, e.g. as \"channels[0],channels[1]\", or \"r,g\"" << std::endl; return 1; }
        }
        csv.fields = comma::join( v, ',' );
        std::string offset_string = options.value< std::string >( "--from,--begin,--origin", "0,0" );
        bool output_on_empty_input = options.exists( "--output-on-empty-input,--output-on-empty" );
        bool output_on_missing_blocks = options.exists( "--output-on-missing-blocks" );
        auto number_of_blocks = options.optional<unsigned int>("--number-of-blocks,--block-count");
        const std::vector< std::string >& w = comma::split( offset_string, ',' );
        if( w.size() != 2 ) { std::cerr << "image-from-csv: --from: expected <x>,<y>; got: \"" << offset_string << "\"" << std::endl; return 1; }
        std::pair< double, double > offset( boost::lexical_cast< double >( w[0] ), boost::lexical_cast< double >( w[1] ) ); // todo: quick and dirty; use better types
        if( is_greyscale && has_alpha ) { std::cerr << "image-from-csv: warning: found alpha channel for a greyscale image; not implemented; ignored" << std::endl; }
        sample.channels.resize( is_greyscale ? 1 : has_alpha ? 4 : 3 );
        snark::cv_mat::serialization::options output_options = comma::name_value::parser( ';', '=' ).get< snark::cv_mat::serialization::options >( options.value<std::string>("--output" ) );
        snark::cv_mat::serialization output( output_options ); // todo: check whether output type matches fields
        comma::csv::input_stream< input_t > is( std::cin, csv, sample );
        boost::optional< input_t > last;
        int type = output_options.get_header().type;
        timestamping t( options.value< std::string >( "--timestamp", "first" ) );
        cv::Mat background = cv::Mat::zeros( output_options.rows, output_options.cols, type );
        if( options.exists( "--background" ) )
        {
            const auto& v = comma::split( options.value< std::string >( "--background" ), ',' );
            if( int( v.size() ) != background.channels() ) { std::cerr << "image-from-csv: expected --background for " << background.channels() << "; got: '" << options.value< std::string >( "--background" ) << "'" << std::endl; return 1; }
            std::vector< cv::Mat > channels( background.channels() );
            for( int i = 0; i < background.channels(); ++i )
            {
                channels[i] = cv::Mat::zeros( output_options.rows, output_options.cols, snark::cv_mat::single_channel_type( background.type() ) );
                try { channels[i].setTo( boost::lexical_cast< float >( v[i] ) ); }
                catch( std::exception& ex ) { std::cerr << "image-from-csv: --background: invalid value: '" << v[i] << "' (" << ex.what() << ")" << std::endl; return 1; }
                catch( ... ) { std::cerr << "image-from-csv: --background: invalid value: '" << v[i] << "'" << std::endl; return 1; }
            }
            cv::merge( channels, background );
        }
        std::pair< boost::posix_time::ptime, cv::Mat > pair;
        while( is.ready() || std::cin.good() )
        {
            const input_t* p = is.read();
            bool block_done = !p || ( last && p->block != last->block );
            if( last ) { t.update( last->t, block_done ); }
            if( !last || block_done )
            {
                if( last )
                {
                    pair.first = t.value();
                    t.reset();
                    output.write( std::cout, pair );
                    std::cout.flush();
                }
                background.copyTo( pair.second );
                if( output_on_missing_blocks )
                {
                    int gap;
                    if( p ) { gap = last ? p->block - last->block - 1 : p->block; } 
                    else if ( number_of_blocks ) { gap = last ? *number_of_blocks - last->block - 1: *number_of_blocks; } 
                    else { gap = 0; }
                    if( gap < 0 ) { std::cerr << "image-from-csv: expected incrementing block numbers, got: " << p->block << " after " << last->block << std::endl; exit( 1 ); }
                    if( number_of_blocks && p && p->block >= *number_of_blocks ) { std::cerr << "image-from-csv: expecting block number less than number-of-blocks (" << *number_of_blocks << "), got: " << p->block << std::endl; exit( 1 ); }
                    for( int i = 0; i < gap; ++i ) { output.write( std::cout, pair ); }
                    std::cout.flush();
                }
            }
            if( !p ) { break; }
            set_pixel( pair.second, *p, offset );
            last = *p;
        }
        if( output_on_empty_input && !output_on_missing_blocks && !last ) { output.write( std::cout, pair ); }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "image-from-csv: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "image-from-csv: unknown exception" << std::endl; }
    return 1;
}
