// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// Copyright (c) 2024 Vsevolod Vlaskine
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

#include <limits>
#include <boost/optional.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <comma/application/command_line_options.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/name_value/parser.h>
#include <comma/csv/options.h>
#include "../cv_mat/filters.h"
#include "../cv_mat/serialization.h"
#include "../cv_mat/traits.h"
#include "../cv_mat/type_traits.h"

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "read pixel coordinates and values, output images in the format readable by cv-cat" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: image-from-csv <options>" << std::endl
              << std::endl
              << "options" << std::endl
              << "    --help,-h: show help; --help --verbose: more help" << std::endl
              << "    --autoscale=[<properties>]; make sure all pixel coordinates fit into the image after applying offset" << std::endl
              << "        <properties>" << std::endl
              << "            centre: if proportional (see below), centre the shorter dimension in the image" << std::endl
              << "            once: make sure all pixel coordinates of the first block fit into the image" << std::endl
              << "                  use the first block scaling factor for all subsequent blocks" << std::endl
              << "            proportional: use the same scaling factor on x and y" << std::endl
              << "    --background=<colour>; e.g. --background=0, --background=0,-1,-1, etc; default: zeroes" << std::endl
              << "    --from,--begin,--origin=[<x>,<y>]: offset pixel coordinates by a given offset; default: 0,0" << std::endl
              << "    --number-of-blocks,--block-count=[<count>]; if --output-on-missing-blocks, expected number of input blocks" << std::endl
              << "    --output: output options, same as --input for image-from-csv or cv-cat (see --help --verbose)" << std::endl
              << "    --output-on-missing-blocks: output empty images on missing input blocks; input blocks expected ordered" << std::endl
              << "    --output-on-empty-input,--output-on-empty: output empty image on empty input" << std::endl
              << "    --shape=<shape>; default=points" << std::endl
              << "                     <shape>" << std::endl
              << "                         point: each input point represents one pixel" << std::endl
              << "                         lines: connect with line subsequent points with the same id" << std::endl
              << "                                e.g. to draw a trajectory or plot a basic graph" << std::endl
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
        std::cerr << "examples" << std::endl;
        std::cerr << "    basics" << std::endl;
        std::cerr << "        cat pixels.csv | image-from-csv --fields x,y,grey --output=\"rows=1200;cols=1000;type=ub\" | cv-cat --output no-header \"encode=png\" > test.bmp" << std::endl;
        std::cerr << "    autoscale" << std::endl;
        std::cerr << "        csv-random make --type 6f --range 0,1 \\" << std::endl;
        std::cerr << "            | csv-eval --fields i,x,y,r,g,b 'i=round(i*10);y/=3;r=round(r*255);g=round(g*255);b=round(b*255)' \\" << std::endl;
        std::cerr << "            | csv-paste 'line-number;size=10000' -  \\" << std::endl;
        std::cerr << "            | image-from-csv --fields block,id,x,y,r,g,b \\" << std::endl;
        std::cerr << "                             --output 'rows=1000;cols=1000;type=3ub' \\" << std::endl;
        std::cerr << "                             --autoscale='once;proportional;centre'  \\" << std::endl;
        std::cerr << "            | cv-cat 'view;null'" << std::endl;
        std::cerr << "    shapes" << std::endl;
        std::cerr << "        lines" << std::endl;
        std::cerr << "            ( echo 0,0,255,0,0; echo 1,1,255,0,0; echo 1,0.5,255,0,0; echo 0.5,1.5,255,0,0 )\\" << std::endl;
        std::cerr << "                | image-from-csv --fields x,y,r,g,b --shape=lines --autoscale --output 'rows=1000;cols=1000;type=3ub' \\" << std::endl;
        std::cerr << "                | cv-cat 'view=stay;null'" << std::endl;
        std::cerr << "            csv-random make --type 6f --range 0,1 \\" << std::endl;
        std::cerr << "                | csv-eval --fields i,x,y,r,g,b 'i=round(i*10);r=round(r*255);g=round(g*255);b=round(b*255)' \\" << std::endl;
        std::cerr << "                | head -n100  \\" << std::endl;
        std::cerr << "                | image-from-csv --fields id,x,y,r,g,b  \\" << std::endl;
        std::cerr << "                                 --output 'rows=1000;cols=1000;type=3ub' \\" << std::endl;
        std::cerr << "                                 --autoscale='once;proportional;centre'\\" << std::endl;
        std::cerr << "                                 --shape=lines \\" << std::endl;
        std::cerr << "                | cv-cat 'view=stay;null'" << std::endl;
    }
    else
    {
        std::cerr << "input stream csv options..." << std::endl;
        std::cerr << "image serialization output options..." << std::endl;
        std::cerr << "examples..." << std::endl;
        std::cerr << "    for details, run: image-from-csv --help --verbose" << std::endl;
    }
    std::cerr << std::endl;
    exit( 0 );
}

struct input_t
{
    boost::posix_time::ptime t;
    double x{0};
    double y{0};
    std::vector< double > channels;
    comma::uint32 block{0};
    comma::uint32 id{0};
    
    input_t() : x( 0 ), y( 0 ), block( 0 ) {}
};

struct autoscale_t
{
    bool once{false};
    bool proportional{false};
    bool centre{false};
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
        v.apply( "id", r.id );
    }
    
    template < typename K, typename V > static void visit( const K&, const input_t& r, V& v )
    {
        v.apply( "t", r.t );
        v.apply( "x", r.x );
        v.apply( "y", r.y );
        v.apply( "channels", r.channels );
        v.apply( "block", r.block );
        v.apply( "id", r.id );
    }
};

template <> struct traits< autoscale_t >
{
    template < typename K, typename V > static void visit( const K&, autoscale_t& r, V& v )
    {
        v.apply( "once", r.once );
        v.apply( "proportional", r.proportional );
        v.apply( "centre", r.centre );
    }
    
    template < typename K, typename V > static void visit( const K&, const autoscale_t& r, V& v )
    {
        v.apply( "once", r.once );
        v.apply( "proportional", r.proportional );
        v.apply( "centre", r.centre );
    }
};

} } // namespace comma { namespace visiting {

class shape_t // todo: quick and dirty, make polymorphic
{
    public:
        struct types
        { 
            enum values { point, lines };

            static values from_string( const std::string& s )
            {
                if( s == "point" ) { return point; }
                if( s == "lines" ) { return lines; }
                COMMA_THROW_BRIEF( comma::exception, "expected shape, got: '" << s << "'" );
            }
        };

        shape_t( types::values t = types::point ): _type( t ) {}

        static shape_t make( const std::string& s ) { return shape_t( types::from_string( s ) ); }

        void clear() { _previous.clear(); }

        void draw( cv::Mat& m, const input_t& v, const std::pair< double, double >& offset, const std::pair< double, double >& scale ) // quick and dirty; reimplement as templates
        {
            int x = std::floor( ( v.x - offset.first ) * scale.first + 0.5 );
            int y = std::floor( ( v.y - offset.second ) * scale.second + 0.5 );
            switch( _type ) // todo: quick and dirty, make polymorphic, move to traits
            {
                case types::point:
                    snark::cv_mat::set( m, y, x, v.channels );
                    break;
                case types::lines:
                {
                    auto i = _previous.find( v.id );
                    if( i == _previous.end() )
                    { 
                        snark::cv_mat::set( m, y, x, v.channels );
                    }
                    else
                    {
                        int x0 = std::floor( ( i->second.x - offset.first ) * scale.first + 0.5 ); // todo: quick and dirty, save previous
                        int y0 = std::floor( ( i->second.y - offset.second ) * scale.second + 0.5 ); // todo: quick and dirty, save previous
                        cv::Scalar c0, c; for( unsigned int j = 0; j < v.channels.size(); ++j ) { c0[j] = i->second.channels[j]; c[j] = v.channels[j]; }
                        cv::line( m, cv::Point( x0, y0 ), cv::Point( x, y ), ( c0 + c ) / 2, 1, cv::LINE_AA );
                    }
                    _previous[v.id] = v;
                    break;
                }
            }
        }

    private:
        types::values _type{types::point};
        std::unordered_map< std::uint32_t, input_t > _previous;
};

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
        COMMA_ASSERT_BRIEF( !csv.fields.empty(), "please specify --fields" );
        std::vector< std::string > v = comma::split( csv.fields, ',' );
        input_t sample;
        bool is_greyscale = true;
        bool has_alpha = false;
        options.assert_mutually_exclusive( "--offset", "--autoscale" );
        boost::optional< autoscale_t > autoscale = comma::silent_none< autoscale_t >();
        if( options.exists( "--autoscale" ) ) { autoscale = comma::name_value::parser( ';', '=' ).get< autoscale_t >( options.value< std::string >("--autoscale" ) ); }
        auto shape = shape_t::make( options.value< std::string >( "--shape", "point" ) );
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
        std::pair< double, double > scale{1, 1};
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
        std::vector< input_t > inputs;
        while( is.ready() || std::cin.good() )
        {
            const input_t* p = is.read();
            bool block_done = !p || ( last && p->block != last->block );
            if( last ) { t.update( last->t, block_done ); }
            if( !last || block_done )
            {
                COMMA_ASSERT_BRIEF( inputs.size() != 1, "--autoscale: got only 1 point in block " << inputs[0].block << "; not supported; something like --permissive with discard: todo, just ask" );
                if( autoscale && inputs.size() > 1 ) // todo: move to autoscale_t method
                {
                    std::pair< double, double > min{ inputs[0].x, inputs[0].y };
                    std::pair< double, double > max{ inputs[0].x, inputs[0].y };
                    for( const auto& i: inputs )
                    {
                        if( i.x < min.first ) { min.first = i.x; } else if( i.x > max.first ) { max.first = i.x; }
                        if( i.y < min.second ) { min.second = i.y; } else if( i.y > max.second ) { max.second = i.y; }
                    }
                    offset = min;
                    COMMA_ASSERT_BRIEF( max.first != min.first, "--autoscale: all x values are the same (" << min.first << ") in block " << inputs[0].block << "; not supported; something like --permissive with discard: todo, just ask" );
                    COMMA_ASSERT_BRIEF( max.second != min.second, "--autoscale: all y values are the same (" << min.second << ") in block " << inputs[0].block << "; not supported; something like --permissive with discard: todo, just ask" );
                    std::pair< double, double > range{ max.first - min.first, max.second - min.second };
                    std::pair< double, double > size{ pair.second.cols - 1, pair.second.rows - 1 };
                    scale = { size.first / range.first, size.second / range.second }; // todo: check for zeroes
                    if( autoscale->proportional )
                    {
                        if( scale.first < scale.second )
                        {
                            offset.second -= autoscale->centre ? ( size.second / scale.first - range.second ) * 0.5 : 0;
                            scale.second = scale.first;
                        }
                        else
                        {
                            offset.first -= autoscale->centre ? ( size.first / scale.second - range.first ) * 0.5 : 0;
                            scale.first = scale.second;
                        }
                    }
                    comma::saymore() << "offset: " << offset.first << "," << offset.second << " scale: " << scale.first << "," << scale.second << std::endl;
                    for( const auto& i: inputs ) { shape.draw( pair.second, i, offset, scale ); }
                    inputs.clear();
                    if( autoscale->once ) { autoscale.reset(); }
                }
                if( last )
                {
                    pair.first = t.value();
                    t.reset();
                    output.write( std::cout, pair );
                    std::cout.flush();
                }
                shape.clear();
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
            if( autoscale ) { inputs.push_back( *p ); } // todo: watch performance
            else { shape.draw( pair.second, *p, offset, scale ); }
            last = *p;
        }
        if( output_on_empty_input && !output_on_missing_blocks && !last ) { output.write( std::cout, pair ); }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "image-from-csv: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "image-from-csv: unknown exception" << std::endl; }
    return 1;
}
