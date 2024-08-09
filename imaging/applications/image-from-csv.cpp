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
#include <memory>
#include <boost/optional.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <comma/application/command_line_options.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/name_value/parser.h>
#include <comma/csv/options.h>
#include <comma/io/stream.h>
#include "../cv_mat/filters.h"
#include "../cv_mat/serialization.h"
#include "../cv_mat/traits.h"
#include "../cv_mat/type_traits.h"
#include "../cv_mat/utils.h"
#include "../../render/colours/named.h"

static void usage( bool verbose )
{
    std::cerr << R"(
read pixel coordinates and values, output images in the format readable by cv-cat

usage: image-from-csv <options>

options
    --help,-h: show help; --help --verbose: more help
    --autoscale=[<properties>]; make sure all pixel coordinates fit into the image after applying offset
        <properties>
            centre: if proportional (see below), centre the shorter dimension in the image
            grow: todo: on every block, change scale so that all input points fit into the image, never shrink
            once: make sure all pixel coordinates of the first block fit into the image
                  use the first block scaling factor for all subsequent blocks
            proportional: use the same scaling factor on x and y
            shrink: todo: on every block, shrink scale to fit points into the image, never grow
    --background=[<image(s)>]; single image or cv-cat-style image stream, if stream, new csv block will read new image
        <image(s)>
            <image> : e.g. --background=my-image.png
            <stream>: e.g. --background <( cv-cat --file movie.mp4 )
                           --background <( cv-cat images.bin )';rows=600;cols=800;no-header;type=3ub'
    --background-colour,--background-color=<colour>; e.g. --background-colour=0, --background-colour=0,-1,-1, etc; default: zeroes
    --colours,--colors=<colours>; default colours; e.g. --colours=red;0,255,0;blue: colour points with id 0 red, etc
    --from,--begin,--origin=[<x>,<y>]: offset pixel coordinates by a given offset; default: 0,0
    --number-of-blocks,--block-count=[<count>]; if --output-on-missing-blocks, expected number of input blocks
    --output=[<properties>]: output options, same as --input for image-from-csv or cv-cat (see --help --verbose)
                             if --output not present and --background given, properties will be taken from
                             background image
    --output-on-missing-blocks: output empty images on missing input blocks; input blocks expected ordered
    --output-on-empty-input,--output-on-empty: output empty image on empty input
    --scale-factor,--zoom=<factor>; default=1; extra scale factor
    --shape=<shape>; default=point
                     <shape>
                         point: each input point represents one pixel
                         lines: connect with line subsequent points with the same id
                                e.g. to draw a trajectory or plot a basic graph
    --timestamp=<how>: which image timestamp to output
          <how>
              first: timestamp of the first point of a block
              last: timestamp of the last point of a block
              max: maximum timestamp of a block
              mean,average: average timestamp across all points of a block
              middle: average of the first and last timestamp of a block
              min: minimum timestamp of a block
              default: first
    --verbose,-v: more output

fields: t,x,y,r,g,b,block or t,x,y,grey,block
    t: image timestamp, optional
    x,y: pixel coordinates, if double, will get rounded to the nearest integer
    r,g,b: red, green, blue values
    grey: greyscale value
    channels[0],channels[1],channels[2]: blue, green, red values; notice that the order is bgr
                                         if only channels[0] given, it is the same as specifying grey field
    block: image number, optional
)" << std::endl;
    if( verbose )
    {
        std::cerr<< snark::cv_mat::serialization::options::usage() << std::endl;
        std::cerr<< "input stream csv options" << std::endl << comma::csv::options::usage() << std::endl;
        std::cerr << R"(examples
    basics
        cat pixels.csv | image-from-csv --fields x,y,grey --output='rows=1200;cols=1000;type=ub' | cv-cat --output no-header 'encode=png' > test.bmp
    autoscale
        csv-random make --type 6f --range 0,1 \
            | csv-eval --fields i,x,y,r,g,b 'i=round(i*10);y/=3;r=round(r*255);g=round(g*255);b=round(b*255)' \
            | csv-paste 'line-number;size=10000' -  \
            | image-from-csv --fields block,id,x,y,r,g,b \
                             --output 'rows=1000;cols=1000;type=3ub' \
                             --autoscale='once;proportional;centre'  \
            | cv-cat 'view;null'
    autoscale: try to experiment by adding and removing --autoscaling properties
        csv-random make --type 6f --range 0,6.28 \
            | csv-paste 'line-number;size=10000' - \
            | csv-eval --fields block,i,x,y,r,g,b 'i=round(i*10);y=y*sin(x)*sin((block+1)*0.01);r=round(r*255/6.28);g=round(g*255/6.28);b=round(b*255/6.28)' \
            | image-from-csv --fields block,id,x,y,r,g,b \
                             --output 'rows=1000;cols=1000;type=3ub'  \
                             --autoscale='grow;proportional;centre' \
            | cv-cat 'view;null'
    shapes
        lines
            ( echo 0,0,255,0,0; echo 1,1,255,0,0; echo 1,0.5,255,0,0; echo 0.5,1.5,255,0,0 ) \
                | image-from-csv --fields x,y,r,g,b --shape=lines --autoscale --output 'rows=1000;cols=1000;type=3ub' \
                | cv-cat 'view=stay;null'
            csv-random make --type 6f --range 0,1 \
                | csv-eval --fields i,x,y,r,g,b 'i=round(i*10);r=round(r*255);g=round(g*255);b=round(b*255)' \
                | head -n100  \
                | image-from-csv --fields id,x,y,r,g,b  \
                                 --output 'rows=1000;cols=1000;type=3ub' \
                                 --autoscale='once;proportional;centre'\
                                 --shape=lines \
                | cv-cat 'view=stay;null'
    3D projections, binary feeds, autoscaling
        basic
            points-make box --width 50 \
                | points-frame --fields x,y,z --from 0,0,0,1,1,1 \
                | image-from-csv --fields x,y \
                                 --colors yellow \
                                 --autoscale once \
                                 --output 'rows=800;cols=800;type=3ub' \
                | cv-cat view=stay null
        animated
            for angle in $( seq 0 0.01 6.28 ); do \
                points-make box --width 50 --origin -25,-25,-25 \
                    | points-frame --fields x,y,z --from "0,0,0,$angle,$angle,$angle"; \
            done \
                | csv-paste "line-number;"size=$( points-make box --width 50 | wc -l ) - \
                | image-from-csv --fields block,x,y \
                                --colors=yellow \
                                --autoscale once \
                                --zoom 0.5 \
                                --output='rows=800;cols=800;type=3ub' \
                | cv-cat view null
        animated lines
            yes 1,1,1,0,0,1,1,1,1,0,1,0,1,1,1,1,0,0,1,0,0,0,1,0,1,0,0,0,0,1,0,1,0,0,0,1 \
                | csv-shape split -n 3 --repeat \
                | csv-paste 'line-number;size=12' 'line-number;size=2' - \
                | csv-eval --fields block 'roll,pitch,yaw=0.01*block,0.01*block,0.01*block' \
                | points-frame --fields ,,x,y,z,frame/roll,frame/pitch,frame/yaw --emplace \
                | image-from-csv --fields block,id,x,y \
                                 --colors="red;green;blue;cyan;magenta;yellow" \
                                 --autoscale once \
                                 --zoom 0.34 \
                                 --shape lines \
                                 --output='rows=800;cols=800;type=3ub' \
                | cv-cat "draw=status,label:flying tetrahedron|origin:36,-12|color:180,180,180|font-size:0.4|alpha:0.01|spin-up:10|system-time" \
                         'view=,example: flying tetrahedron' \
                         null \
                         --fps 25
    plots
        colours, and sliding window
            csv-random make --type f --range 120,240 \
                | csv-repeat --pace --period 0.02 \
                | csv-paste 'line-number;size=4;index' - \
                | csv-shape sliding-window --size 400 --step 4 --incremental --block \
                | csv-blocks index --fields block \
                | image-from-csv --fields block,id,y,x \
                                --shape lines \
                                --output 'rows=380;cols=400;type=3ub' \
                                --colours="magenta;cyan;yellow;red" \
                | cv-cat 'canvas=420,400,10,10' \
                        'draw=grid,10,10,100,60,400,360,100,100,100,1' \
                        'text=0,10,390,light-grey,scale:0.4' \
                        'text=100,380,390,light-grey,scale:0.4' \
                        'rectangle=110,378,160,394,0,255,255,-1,-1,0,128' \
                        'rectangle=160,378,210,394,255,,255,-1,-1,0,128' \
                        'rectangle=210,378,260,394,255,255,0,-1,-1,0,128' \
                        'rectangle=260,378,310,394,255,0,0,-1,-1,0,128' \
                        'text=A,130,390,light-grey,scale:0.4' \
                        'text=B,180,390,light-grey,scale:0.4' \
                        'text=C,230,390,light-grey,scale:0.4' \
                        'text=D,280,390,light-grey,scale:0.4' \
                        'view=,sliding window example;null'
        polar
            csv-random make --type f --range 1,2 \
                | csv-paste "line-number;"size=$(( 360 * 3 )) \
                            "line-number;size=3;index" \
                            "line-number;index;"size=360 \
                            - \
                | csv-eval --fields ,id,,range 'range=(id+1)*range*50' --flush \
                | math-deg2rad --fields ,,angle \
                | points-to-cartesian --fields ,,bearing,range \
                | image-from-csv --fields block,id,y,x \
                                 --colours="green;yellow;magenta" \
                                 --shape lines \
                                 --output 'rows=800;cols=800;type=3ub' \
                                 --origin=-400,-400 \
                | cv-cat 'draw=grid,0,0,100,100,800,800,80,80,80' \
                         'draw=axis,label:values|step:100|steps:4|size:400|origin:400,400|extents:0,400|color:255,255,255' \
                         'draw=status,label:polar graph|origin:120,-12|color:180,180,180|font-size:0.4|alpha:0.01|no-begin|spin-up:10|system-time' \
                         view=",example: polar graph" \
                         null \
                         --fps 5
            )" << std::endl;
    }
    else
    {
        std::cerr << R"(input stream csv options...
image serialization output options...
examples...
    for details, run: image-from-csv --help --verbose)" << std::endl;
    }
    std::cerr << std::endl;
    exit( 0 );
}

namespace snark { namespace imaging { namespace applications { namespace image_from_csv {

struct input
{
    boost::posix_time::ptime t;
    double x{0};
    double y{0};
    std::vector< double > channels;
    comma::uint32 block{0};
    comma::uint32 id{0};
    
    input() : x( 0 ), y( 0 ), block( 0 ) {}
};

struct autoscale
{
    bool once{false};
    bool proportional{false};
    bool centre{false};
    bool grow{false};
    bool shrink{false};

    void validate() const { COMMA_ASSERT( !grow || !shrink, "autoscale: 'grow' and 'shrink' are mutually exclusive" ); }
};

} } } } // namespace snark { namespace imaging { namespace applications { namespace image_from_csv {

namespace comma { namespace visiting {

template <> struct traits< snark::imaging::applications::image_from_csv::input >
{
    typedef snark::imaging::applications::image_from_csv::input input_t;
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

template <> struct traits< snark::imaging::applications::image_from_csv::autoscale >
{
    typedef snark::imaging::applications::image_from_csv::autoscale autoscale_t;
    template < typename K, typename V > static void visit( const K&, autoscale_t& r, V& v )
    {
        v.apply( "once", r.once );
        v.apply( "proportional", r.proportional );
        v.apply( "centre", r.centre );
        v.apply( "grow", r.grow );
        v.apply( "shrink", r.shrink );
    }
    template < typename K, typename V > static void visit( const K&, const autoscale_t& r, V& v )
    {
        v.apply( "once", r.once );
        v.apply( "proportional", r.proportional );
        v.apply( "centre", r.centre );
        v.apply( "grow", r.grow );
        v.apply( "shrink", r.shrink );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace imaging { namespace applications { namespace image_from_csv {

class shape // todo: quick and dirty, make polymorphic
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

        shape( types::values t = types::point ): _type( t ) {}

        static shape make( const std::string& s ) { return shape( types::from_string( s ) ); }

        void clear() { _previous.clear(); }

        void draw( cv::Mat& m, const input& v, const std::pair< double, double >& offset, const std::pair< double, double >& scale, double factor ) // quick and dirty; reimplement as templates
        {
            std::pair< int, int > extra_offset{ m.cols * ( 1 - factor ) / 2, m.rows * ( 1 - factor ) / 2 };
            int x = std::floor( ( v.x - offset.first ) * scale.first * factor + 0.5 ) + extra_offset.first;
            int y = std::floor( ( v.y - offset.second ) * scale.second * factor + 0.5 ) + extra_offset.second;
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
                        int x0 = std::floor( ( i->second.x - offset.first ) * scale.first * factor + 0.5 ) + extra_offset.first; // todo: quick and dirty, save previous
                        int y0 = std::floor( ( i->second.y - offset.second ) * scale.second * factor + 0.5 ) + extra_offset.second; // todo: quick and dirty, save previous
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
        std::unordered_map< std::uint32_t, input > _previous;
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
            comma::say() << "expected timestamping method, got: '" << s << "'" << std::endl;
            exit( 1 );
        }
        values_ how_;
        boost::optional< boost::posix_time::ptime > t_;
        unsigned int count_;
};

class stream // todo: move to cv_mat
{
    public:
        typedef std::pair< boost::posix_time::ptime, cv::Mat > pair_t;
        stream( const std::string& name );
        stream( const std::string& colour, const snark::cv_mat::serialization::options& options );
        const snark::cv_mat::serialization::options& options() const { return _options; }
        pair_t operator()() const { return _pair; }
        pair_t operator++();
    private:
        bool _eof{false};
        pair_t _pair;
        snark::cv_mat::serialization::options _options;
        snark::cv_mat::serialization _input;
        std::unique_ptr< comma::io::istream > _istream;

};

stream::stream( const std::string& name )
{
    const auto& s = comma::split_head( name, 2, ',' );
    if( s.size() == 1 )
    {
        _pair.second = cv::imread( s[0], cv::IMREAD_UNCHANGED );
        if( _pair.second.data )
        {
            _options.rows = _pair.second.rows;
            _options.cols = _pair.second.cols;
            _options.type = snark::cv_mat::type_as_string( _pair.second.type() );
            return;
        }
    }
    if( s.size() == 2 )
    {
        _options = comma::name_value::parser( ';', '=' ).get< snark::cv_mat::serialization::options >( s[1] );
        _input = snark::cv_mat::serialization( _options );
    }
    _istream.reset( new comma::io::istream( s[0], comma::io::mode::binary, comma::io::mode::blocking ) );
    operator++();
    COMMA_ASSERT_BRIEF( _options.rows > 0 && _options.cols > 0, "failed to read image from '" << _istream->name() << "'" );
    _options.rows = _pair.second.rows;
    _options.cols = _pair.second.cols;
    _options.type = snark::cv_mat::type_as_string( _pair.second.type() );
}

stream::stream( const std::string& colour_string, const snark::cv_mat::serialization::options& options ) // quick and dirty
{
    auto colour = snark::render::colours::named< unsigned char >::from_string( colour_string ); // todo! quick and dirty: resolve on options.get_header().type
    _pair.second = cv::Mat::zeros( options.rows, options.cols, options.get_header().type );
    std::swap( colour[0], colour[2] ); // bgr... sigh...
    std::vector< cv::Mat > channels( _pair.second.channels() );
    for( int i = 0; i < _pair.second.channels(); ++i )
    {
        channels[i] = cv::Mat::zeros( options.rows, options.cols, snark::cv_mat::single_channel_type( _pair.second.type() ) );
        try { channels[i].setTo( float( colour[i] ) ); }
        catch( std::exception& ex ) { COMMA_THROW_BRIEF( comma::exception, "colour: channel " << i << ": invalid value: '" << double( colour[i] ) << "' (" << ex.what() << ")" ); }
        catch( ... ) { COMMA_THROW_BRIEF( comma::exception, "colour:  channel " << i << ": invalid value: '" << double( colour[i] ) << "'" ); }
    }
    cv::merge( channels, _pair.second );
}

stream::pair_t stream::operator++()
{
    if( _istream && !_eof )
    {
        auto p = _input.read< boost::posix_time::ptime >( *( *_istream )() );
        _pair.first = p.first;
        //std::cerr << "==> c: " << ( p.second.data == nullptr ) << " p.second.rows: " << p.second.rows << " p.second.cols: " << p.second.cols << std::endl;
        if( p.second.data ) { _pair.second = p.second; } else { _eof = true; }
    }
    return _pair;
}

} } } } // namespace snark { namespace imaging { namespace applications { namespace image_from_csv {

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options csv( options );
        COMMA_ASSERT_BRIEF( !csv.fields.empty(), "please specify --fields" );
        options.assert_mutually_exclusive( "--offset", "--autoscale" );
        auto autoscale = comma::silent_none< snark::imaging::applications::image_from_csv::autoscale >();
        if( options.exists( "--autoscale" ) ) { autoscale = comma::name_value::parser( ';', '=' ).get< snark::imaging::applications::image_from_csv::autoscale >( options.value< std::string >("--autoscale" ) ); autoscale->validate(); }
        const auto& c = comma::split( options.value< std::string >( "--colours,--colors", "" ), ';', true );
        std::vector< snark::render::colour< unsigned char > > colours( c.size() );
        for( unsigned int i = 0; i < colours.size(); ++i ) { colours[i] = snark::render::colours::named< unsigned char >::from_string( c[i] ); }
        snark::imaging::applications::image_from_csv::input sample;
        bool is_greyscale = colours.empty();
        bool has_alpha = false;
        std::vector< std::string > v = comma::split( csv.fields, ',' );
        for( unsigned int i = 0; i < v.size(); ++i ) // quick and dirty, somewhat silly
        {
            if( v[i] == "grey" ) { v[i] = "channels[0]"; }
            else if( v[i] == "b" ) { v[i] = "channels[0]"; is_greyscale = false; }
            else if( v[i] == "g" || v[i] == "channels[1]" ) { v[i] = "channels[1]"; is_greyscale = false; }
            else if( v[i] == "r" || v[i] == "channels[2]" ) { v[i] = "channels[2]"; is_greyscale = false; }
            else if( v[i] == "a" || v[i] == "channels[3]" ) { v[i] = "channels[3]"; is_greyscale = false; has_alpha = true; }
            else if( v[i] == "channels" ) { comma::say() << "please specify channels fields explicitly, e.g. as 'channels[0],channels[1]', or 'r,g'" << std::endl; return 1; }
        }
        csv.fields = comma::join( v, ',' );
        if( is_greyscale && has_alpha ) { comma::say() << "warning: found alpha channel for a greyscale image; not implemented; ignored" << std::endl; }
        sample.channels.resize( is_greyscale ? 1 : has_alpha ? 4 : 3 );
        std::string offset_string = options.value< std::string >( "--from,--begin,--origin", "0,0" );
        bool output_on_empty_input = options.exists( "--output-on-empty-input,--output-on-empty" );
        bool output_on_missing_blocks = options.exists( "--output-on-missing-blocks" );
        auto number_of_blocks = options.optional< unsigned int >( "--number-of-blocks,--block-count" );
        const auto& w = comma::split_as< double >( offset_string, ',' );
        COMMA_ASSERT_BRIEF( w.size() == 2, "image-from-csv: --from: expected <x>,<y>; got: '" << offset_string << "'" );
        auto shape = snark::imaging::applications::image_from_csv::shape::make( options.value< std::string >( "--shape", "point" ) );
        std::pair< double, double > offset( w[0], w[1] ); // todo: quick and dirty; use better types like cv::Point
        std::pair< double, double > scale{1, 1};
        double scale_factor = options.value( "--scale-factor,--zoom", 1. );
        comma::csv::input_stream< snark::imaging::applications::image_from_csv::input > is( std::cin, csv, sample );
        boost::optional< snark::imaging::applications::image_from_csv::input > last;
        snark::imaging::applications::image_from_csv::timestamping t( options.value< std::string >( "--timestamp", "first" ) );
        std::unique_ptr< snark::imaging::applications::image_from_csv::stream > background;
        options.assert_mutually_exclusive( "--background-colour,--background-color", "--background" );
        snark::cv_mat::serialization::options output_options;
        if( options.exists( "--background" ) )
        {
            background.reset( new snark::imaging::applications::image_from_csv::stream( options.value< std::string >( "--background" ) ) );
            output_options = comma::name_value::parser( ';', '=' ).get< snark::cv_mat::serialization::options >( options.value< std::string >( "--output", "" ) );
            output_options.rows = background->options().rows;
            output_options.cols = background->options().cols;
            output_options.type = background->options().type;
            std::cerr << "==> output_options.rows: " << output_options.rows << " output_options.cols: " << output_options.cols << " output_options.type: " << output_options.type << std::endl;
        }
        else
        {
            output_options = comma::name_value::parser( ';', '=' ).get< snark::cv_mat::serialization::options >( options.value< std::string >( "--output" ) );
            background.reset( new snark::imaging::applications::image_from_csv::stream( options.value< std::string >( "--background-colour,--background-color", "0,0,0" ), output_options ) );
        }
        snark::cv_mat::serialization output( output_options ); // todo: check whether output type matches fields
        std::pair< boost::posix_time::ptime, cv::Mat > pair;
        std::vector< snark::imaging::applications::image_from_csv::input > inputs;        
        bool first_block{true};
        while( is.ready() || std::cin.good() )
        {
            const snark::imaging::applications::image_from_csv::input* p = is.read();
            bool block_done = !p || ( last && p->block != last->block );
            if( last ) { t.update( last->t, block_done ); }
            if( !last || block_done )
            {
                COMMA_ASSERT_BRIEF( inputs.size() != 1, "--autoscale: got only 1 point in block " << inputs[0].block << "; not supported; something like --permissive with discard: todo, just ask" );
                if( autoscale && inputs.size() > 1 ) // todo!!! move to autoscale class method
                {
                    std::pair< double, double > min{ inputs[0].x, inputs[0].y };
                    std::pair< double, double > max{ inputs[0].x, inputs[0].y };
                    for( const auto& i: inputs )
                    {
                        if( i.x < min.first ) { min.first = i.x; } else if( i.x > max.first ) { max.first = i.x; }
                        if( i.y < min.second ) { min.second = i.y; } else if( i.y > max.second ) { max.second = i.y; }
                    }
                    // COMMA_ASSERT_BRIEF( max.first != min.first, "--autoscale: all x values are the same (" << min.first << ") in block " << inputs[0].block << "; not supported; something like --permissive with discard: todo, just ask" );
                    // COMMA_ASSERT_BRIEF( max.second != min.second, "--autoscale: all y values are the same (" << min.second << ") in block " << inputs[0].block << "; not supported; something like --permissive with discard: todo, just ask" );
                    if( max.first != min.first && max.second != min.second )
                    {
                        std::pair< double, double > range{ max.first - min.first, max.second - min.second };
                        std::pair< double, double > size{ pair.second.cols - 1, pair.second.rows - 1 };
                        int min_x, min_y, max_x, max_y;
                        bool grown{first_block}, shrunk{first_block};
                        if( !first_block )
                        {
                            min_x = std::floor( ( min.first - offset.first ) * scale.first + 0.5 );
                            min_y = std::floor( ( min.second - offset.second ) * scale.second + 0.5 );
                            max_x = std::floor( ( max.first - offset.first ) * scale.first + 0.5 );
                            max_y = std::floor( ( max.second - offset.second ) * scale.second + 0.5 );
                            grown =    min_x < 0 || min_x >= pair.second.cols // quick and dirty for now
                                    || min_y < 0 || min_y >= pair.second.rows
                                    || max_x < 0 || max_x >= pair.second.cols
                                    || max_y < 0 || max_y >= pair.second.rows;
                            shrunk =   ( min_x > 0 && min_x < pair.second.cols )
                                    || ( max_x > 0 && max_x < size.first )
                                    || ( min_y > 0 && min_y < pair.second.rows )
                                    || ( max_y > 0 && max_y < size.second );
                            //std::cerr << "==> a: min: " << min_x << "," << min_y << " max: " << max_x << "," << max_y << std::endl;
                        }
                        auto new_offset = min;
                        std::pair< double, double > new_scale = { size.first / range.first, size.second / range.second }; // todo: check for zeroes
                        if( autoscale->proportional )
                        {
                            if( new_scale.first < new_scale.second )
                            {
                                new_offset.second -= autoscale->centre ? ( size.second / new_scale.first - range.second ) * 0.5 : 0;
                                new_scale.second = new_scale.first;
                            }
                            else
                            {
                                new_offset.first -= autoscale->centre ? ( size.first / new_scale.second - range.first ) * 0.5 : 0;
                                new_scale.first = new_scale.second;
                            }
                        }
                        if( first_block || ( grown && !autoscale->shrink ) || ( shrunk && !autoscale->grow ) )
                        {
                            scale = new_scale;
                            offset = new_offset;
                            comma::saymore() << "offset: " << offset.first << "," << offset.second << " scale: " << scale.first << "," << scale.second << " grown: " << grown << " shrunk: " << shrunk << std::endl;
                        }
                        first_block = false;
                    }
                    //for( const auto& i: inputs ) { shape.draw( pair.second, i, offset, scale ); }
                    for( const auto& i: inputs ) { shape.draw( pair.second, i, offset, scale, scale_factor ); }
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
                ( *background )().second.copyTo( pair.second );
                ++( *background );
                if( output_on_missing_blocks )
                {
                    int gap;
                    if( p ) { gap = last ? p->block - last->block - 1 : p->block; } 
                    else if ( number_of_blocks ) { gap = last ? *number_of_blocks - last->block - 1: *number_of_blocks; } 
                    else { gap = 0; }
                    COMMA_ASSERT_BRIEF( gap >= 0, "expected incrementing block numbers, got: " << p->block << " after " << last->block );
                    if( number_of_blocks && p && p->block >= *number_of_blocks ) { comma::say() << "expecting block number less than number-of-blocks (" << *number_of_blocks << "), got: " << p->block << std::endl; exit( 1 ); }
                    for( int i = 0; i < gap; ++i ) { output.write( std::cout, pair ); }
                    std::cout.flush();
                }
            }
            if( !p ) { break; }
            snark::imaging::applications::image_from_csv::input q = *p; // todo! watch performance!
            for( unsigned int i = 0; !colours.empty() && i < colours[0].size() && i < q.channels.size(); ++i ) { q.channels[i] = colours[ q.id % colours.size() ][i]; }
            if( !colours.empty() ) // // todo! watch performance! handle non-unsigned char channel types!
            {
                const auto& c = colours[ q.id % colours.size() ];
                q.channels[0] = c[2]; // bgra; opencv, sigh...
                q.channels[1] = c[1];
                q.channels[2] = c[0]; // bgra; opencv, sigh...
                if( q.channels.size() > 3 ) { q.channels[3] = c[3]; }
            }
            if( autoscale ) { inputs.push_back( q ); } // todo! watch performance
            else { shape.draw( pair.second, q, offset, scale, scale_factor ); }
            last = q;
        }
        if( output_on_empty_input && !output_on_missing_blocks && !last ) { output.write( std::cout, pair ); }
        return 0;
    }
    catch( std::exception& ex ) { comma::say() << "" << ex.what() << std::endl; }
    catch( ... ) { comma::say() << "unknown exception" << std::endl; }
    return 1;
}
