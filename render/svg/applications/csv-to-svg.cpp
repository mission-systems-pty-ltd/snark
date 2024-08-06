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

#include <iostream>
#include <sstream>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include "../../colours/map.h"
#include "../../colours/named.h"
#include "../svg.h"
#include "../traits.h"

std::string example()
{
    std::ostringstream oss;
    oss << "csv-to-svg header --width 800px --height 600px --viewbox \"0 0 800 600\"" << std::endl;
    oss << "csv-to-svg group_begin --style \"stroke:black\"" << std::endl;
    oss << "echo -e \"10,10\\n790,10\\n790,590\\n10,590\" | csv-to-svg polygon --style \"fill:#D0EEEE\"" << std::endl;
    oss << "echo -e \"15,15\\n785,15\\n785,585\\n15,585\" | csv-to-svg polyline --style \"fill:none\" --stroke-width 5" << std::endl;
    oss << "csv-to-svg group_end" << std::endl;
    oss << "echo -e \"100,100\\n200,200\\n400,300\" | csv-to-svg point --point-size 20 --colour blue" << std::endl;
    oss << "echo -e \"100,100,200,200\\n200,200,400,300\" | csv-to-svg line --colour grey --stroke-width 10" << std::endl;
    oss << "echo -e \"50,500,30,0\\n150,500,30,1\\n250,500,30,2\\n350,500,30,3\\n450,500,30,4\\n550,500,30,5\\n650,500,30,6\\n750,500,30,7\" | csv-to-svg circle --fields x,y,r,scalar --colour 0:10,jet --stroke-width 4 --style \"stroke:crimson\"" << std::endl;
    oss << "csv-to-svg footer" << std::endl;
    return oss.str();
}

void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "usage: cat points.csv | csv-to-svg [<what>] [<options>] > points.svg" << std::endl;
    std::cerr << std::endl;
    std::cerr << "what" << std::endl;
    std::cerr << "    header: output svg document header" << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --width=<length>: viewport width of the svg document" << std::endl;
    std::cerr << "            --height=<length>: viewport height of the svg document" << std::endl;
    std::cerr << "            --viewbox=\"min-x min-y width height\": svg viewBox attribute which specifies a rectangle in user space which should be mapped to the bounds of the viewport" << std::endl;
    std::cerr << "                      min-x,min-y may be negative" << std::endl;
    std::cerr << "            --style=<style>" << std::endl;
    std::cerr << "            --css=<path-to-stylesheet>" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    footer: output svg document footer" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    script: link javascript file" << std::endl;
    std::cerr << "        --file=<file>" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    script_begin: begin script definitions" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    script_end: end script definitions" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    style_begin: begin style definitions" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    style_end: end style definitions" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    group_begin: begin group <g>" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    group_end: end group </g>" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    point, circle:" << std::endl;
    std::cerr << "        fields: x,y,r[,scalar] (default for point: x,y)" << std::endl;
    std::cerr << "        --point-size,--weight,--radius,-r=<size> (default: " << snark::render::svg::circle::DEFAULT_RADIUS << ")" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    line:" << std::endl;
    std::cerr << "        fields: x1,y1,x2,y2" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    lines: alternate implementation of polylines using <line>s" << std::endl;
    std::cerr << "        fields: x,y[,scalar]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    polyline" << std::endl;
    std::cerr << "        fields: x,y" << std::endl;
    std::cerr << "        --loop: connect the last point to the first" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --attributes='<attributes>': svg element attributes which will be used as is" << std::endl;
    std::cerr << "    --class=<string>" << std::endl;
    std::cerr << "    --id=<string>" << std::endl;
    std::cerr << "    --example: output example to stdout and exit" << std::endl;
    std::cerr << "    --transform=\"<transform-list>\": svg transformations" << std::endl;
    std::cerr << "    --style=\"<style>\" ;-seperated CSS name:value pairs e.g." << std::endl;
    std::cerr << "          fill:<colour>" << std::endl;
    std::cerr << "          stroke:<colour>" << std::endl;
    std::cerr << "          stroke-width:<width>" << std::endl;
    std::cerr << "    --stroke-width=<width>: specifies stroke-width style attribute" << std::endl;
    std::cerr << "    --colour,--color,-c=<colour>: any svg recognised colour" << std::endl;
    std::cerr << "                                  if scalar field present:" << std::endl;
    std::cerr << "                                      <min>:<max>,<from-colour>:<to-colour> (black, blue, green, cyan, red, magenta, yellow, white)" << std::endl; 
    std::cerr << "                                      <min>:<max>,<colourmap> (jet, hot)" << std::endl; 
    std::cerr << std::endl;
    if( verbose )
    {
        std::cerr << "examples:" << std::endl;
        std::cerr << "    output svg with a few circles and lines" << std::endl;
        std::cerr << example() << std::endl;
    }
    else
    {
        std::cerr << "examples: csv-to-svg --help --verbose for more..." << std::endl;
    }
    std::cerr << std::endl;
}

template< typename T >
struct coloured : public T
{
    double scalar;
};

namespace comma { namespace visiting {

template < typename T > struct traits< coloured< T > >
{
    template< typename K, typename V > static void visit( const K& k, coloured< T >& t, V& v )
    {
        traits< T >::visit( k, t, v );
        v.apply( "scalar", t.scalar );
    }

    template< typename K, typename V > static void visit( const K& k, const coloured< T >& t, V& v )
    {
        traits< T >::visit( k, t, v );
        v.apply( "scalar", t.scalar );
    }
};

} } // namespace comma { namespace visiting {

std::string make_style( const comma::command_line_options& options, const std::string& what, const bool parse_colour )
{
    std::string style = options.value< std::string >( "--style", "" );
    if( options.exists( "--stroke-width" ) )
    {
        if( style.size() ) { style += ";"; }
        style += "stroke-width:" + options.value< std::string >( "--stroke-width" );
    }
    if( parse_colour && options.exists( "--colour,--color,-c" ) )
    {
        if( style.size() ) { style += ";"; }
        std::string attribute = "fill";
        if( what == "line" || what == "lines" || what == "polyline" || what == "polygon" ) { attribute = "stroke"; }
        style += attribute + ":" + options.value< std::string >( "--colour,--color,-c" );
    }
    return style;
}

snark::render::colours::map parse_colour_map( const std::string& colour )
{
    double from = 0;
    double to = 1;
    snark::render::colour< unsigned char > from_colour = snark::render::colours::named< unsigned char >::magenta();
    snark::render::colour< unsigned char > to_colour = snark::render::colours::named< unsigned char >::cyan();
    boost::optional< snark::render::colours::map::values > map;
    std::vector< std::string > v = comma::split( colour, ',' );
    switch( v.size() )
    {
        case 0:
            break;
        case 2:
            {
                std::vector< std::string > w = comma::split( v[1], ':' );
                if( w.empty() || w.size() > 2 ) { COMMA_THROW( comma::exception, "expected <from-colour>:<to-colour> or <colourmap>; got " << v[1] ); }
                switch( w.size() )
                {
                    case 1:
                        if( w[0] == "jet" ) { map = snark::render::colours::map::jet(); }
                        else if( w[0] == "hot" ) { map = snark::render::colours::map::temperature( 96, 96 ); }
                        else { COMMA_THROW( comma::exception, "unknown colourmap: " << w[0] ); }
                        break;
                    case 2:
                        from_colour = snark::render::colours::named< unsigned char >::from_string( w[0] );
                        to_colour = snark::render::colours::named< unsigned char >::from_string( w[1] );
                        break;
                }
            }
        case 1:
            if( v[0].size() )
            {
                std::vector< std::string > w = comma::split( v[0], ':' );
                if( w.size() != 2 ) { COMMA_THROW( comma::exception, "invalid '--colour': expected <min>:<max>; got " << v[0] ); }
                from = boost::lexical_cast< double >( w[0] );
                to = boost::lexical_cast< double >( w[1] );
            }
            break;
        default: COMMA_THROW( comma::exception, "invalid '--colour'; got " << colour );
    }
    if( map ) { return snark::render::colours::map( from, to, *map ); }
    return snark::render::colours::map( from, to, from_colour, to_colour );
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--example" ) ) { std::cout << example() << std::endl; return 0; }
        const std::vector< std::string >& unnamed = options.unnamed( "--verbose,-v,--have-css,--loop" , "-.*" );
        if( unnamed.size() != 1 ) { std::cerr << "csv-to-svg: expected one operation name" << ( unnamed.size() ? "; got " + comma::join( unnamed, ' ' ) : "" ) << std::endl; return 1; }
        std::string what = unnamed[0];
        comma::csv::options csv( options );
        csv.full_xpath = false;
        std::cout.precision( csv.precision );
        snark::render::colours::map colour_map;
        bool parse_colour = true;
        if( csv.has_field( "scalar" ) && ( what == "point" || what == "circle" || what == "line" || what == "lines" || what == "triangle" ) )
        {
            colour_map = parse_colour_map( options.value< std::string >( "--colour,--color,-c", "" ) );
            parse_colour = false;
        }
        if( options.exists( "--class" ) ) { snark::render::svg::element::DEFAULT_ATTRIBUTES += " class=\"" + options.value< std::string >( "--class" ) + "\""; }
        if( options.exists( "--id" ) ) { snark::render::svg::element::DEFAULT_ATTRIBUTES += " id=\"" + options.value< std::string >( "--id" ) + "\""; }
        if( options.exists( "--transform" ) ) { snark::render::svg::element::DEFAULT_ATTRIBUTES += " transform=\"" + options.value< std::string >( "--transform" ) + "\""; }
        if( options.exists( "--attributes" ) ) { snark::render::svg::element::DEFAULT_ATTRIBUTES += " " + options.value< std::string >( "--attributes" ); }
        snark::render::svg::element::DEFAULT_STYLE = make_style( options, what, parse_colour );
        if( what == "header" )
        {
            std::cout << snark::render::svg::header(
                  options.value< std::string >( "--width" )
                , options.value< std::string >( "--height" )
                , options.value< std::string >( "--viewbox" )
                , options.optional< std::string >( "--css" )
                ) << std::endl;
            return 0;
        }
        if( what == "footer" ) { std::cout << snark::render::svg::footer() << std::endl; return 0; }
        if( what == "script" ) { std::cout << snark::render::svg::script( options.value< std::string >( "--file" ) ) << std::endl; return 0; }
        if( what == "script_begin" ) { std::cout << snark::render::svg::script::begin() << std::endl; return 0; }
        if( what == "script_end" ) { std::cout << snark::render::svg::script::end() << std::endl; return 0; }
        if( what == "style_begin" ) { std::cout << snark::render::svg::style::begin() << std::endl; return 0; }
        if( what == "style_end" ) { std::cout << snark::render::svg::style::end() << std::endl; return 0; }
        if( what == "group_begin" ) { std::cout << snark::render::svg::g() << std::endl; return 0; }
        if( what == "group_end" ) { std::cout << snark::render::svg::g::end() << std::endl; return 0; }
        if( what == "point" || what == "circle" )
        { 
            boost::optional< double > r = options.optional< double >( "--point-size,--weight,--radius,-r" );
            if( r ) { snark::render::svg::circle::DEFAULT_RADIUS = *r; }
            if( csv.has_field( "scalar" ) )
            {
                // why?!! if( options.exists( "--binary,-b" ) ) { csv.format( comma::csv::format::value< coloured< snark::render::svg::circle > >( csv.fields, true ) ); }
                comma::csv::input_stream< coloured< snark::render::svg::circle > > istream( std::cin, csv );
                while( istream.ready() || std::cin.good() )
                {
                    const coloured< snark::render::svg::circle >* c = istream.read();
                    if( !c ) { break; }
                    std::cout << snark::render::svg::circle( *c, colour_map( c->scalar ).hex() ) << std::endl;
                }
            }
            else
            {
                if( csv.fields.empty() ) { csv.fields = what == "point" ? "x,y" : comma::join( comma::csv::names< snark::render::svg::circle >(), ',' ); }
                // why?!! if( options.exists( "--binary,-b" ) ) { csv.format( comma::csv::format::value< snark::render::svg::circle >( csv.fields, true ) ); }
                comma::csv::input_stream< snark::render::svg::circle > istream( std::cin, csv );
                while( istream.ready() || std::cin.good() )
                {
                    const snark::render::svg::circle* c = istream.read();
                    if( !c ) { break; }
                    std::cout << *c << std::endl;
                }
            }
            return 0;
        }
        if( what == "line" )
        {
            if( csv.has_field( "scalar" ) )
            {
                // why?!! if( options.exists( "--binary,-b" ) ) { csv.format( options.value< std::string >( "--binary,-b", comma::csv::format::value< coloured< snark::render::svg::line > >( csv.fields, true ) ) ); }
                comma::csv::input_stream< coloured< snark::render::svg::line > > istream( std::cin, csv );
                while( istream.ready() || std::cin.good() )
                {
                    const coloured< snark::render::svg::line >* l = istream.read();
                    if( !l ) { break; }
                    std::cout << snark::render::svg::line( *l, colour_map( l->scalar ).hex() ) << std::endl;
                }
            }
            else
            {
                if( csv.fields.empty() ) { csv.fields = comma::join( comma::csv::names< snark::render::svg::line >(), ',' ); }
                // why?!! if( options.exists( "--binary,-b" ) ) { csv.format( comma::csv::format::value< snark::render::svg::line >( csv.fields, true ) ); }
                comma::csv::input_stream< snark::render::svg::line > istream( std::cin, csv );
                while( istream.ready() || std::cin.good() )
                {
                    const snark::render::svg::line* l = istream.read();
                    if( !l ) { break; }
                    std::cout << *l << std::endl;
                }
            }
            return 0;
        }
        if( what == "lines" )
        {
            if( csv.has_field( "scalar" ) )
            {
                // why?!! if( options.exists( "--binary,-b" ) ) { csv.format( comma::csv::format::value< coloured< snark::render::svg::point > >( csv.fields, true ) ); }
                comma::csv::input_stream< coloured< snark::render::svg::point > > istream( std::cin, csv );
                boost::optional< coloured< snark::render::svg::point > > prev = boost::make_optional< coloured< snark::render::svg::point > >( false, coloured< snark::render::svg::point >() );
                while( istream.ready() || std::cin.good() )
                {
                    const coloured< snark::render::svg::point >* p = istream.read();
                    if( !p ) { break; }
                    if( prev ) { std::cout << snark::render::svg::line( prev->x, prev->y, p->x, p->y, colour_map( prev->scalar ).hex() ) << std::endl; }
                    prev = *p;
                }
            }
            else
            {
                if( csv.fields.empty() ) { csv.fields = comma::join( comma::csv::names< snark::render::svg::point >(), ',' ); }
                // why?!! if( options.exists( "--binary,-b" ) ) { csv.format( comma::csv::format::value< snark::render::svg::point >( csv.fields, true ) ); }
                comma::csv::input_stream< snark::render::svg::point > istream( std::cin, csv );
                boost::optional< snark::render::svg::point > prev = boost::make_optional< snark::render::svg::point >( false, snark::render::svg::point() );
                while( istream.ready() || std::cin.good() )
                {
                    const snark::render::svg::point* p = istream.read();
                    if( !p ) { break; }
                    if( prev ) { std::cout << snark::render::svg::line( prev->x, prev->y, p->x, p->y ) << std::endl; }
                    prev = *p;
                }
            }
            return 0;
        }
        if( what == "polyline" )
        {
            if( csv.fields.empty() ) { csv.fields = comma::join( comma::csv::names< snark::render::svg::point >(), ',' ); }
            // why?!! if( options.exists( "--binary,-b" ) ) { csv.format( comma::csv::format::value< snark::render::svg::point >( csv.fields, true ) ); }
            comma::csv::input_stream< snark::render::svg::point > istream( std::cin, csv );
            snark::render::svg::polyline polyline;
            while( istream.ready() || std::cin.good() )
            {
                const snark::render::svg::point* p = istream.read();
                if( !p ) { break; }
                polyline.points.push_back( *p );
            }
            if( polyline.points.size() )
            {
                if( options.exists( "--loop" ) ) { polyline.points.push_back( polyline.points[0] ); }
                std::cout << polyline << std::endl;
            }
            return 0;
        }
        if( what == "polygon" )
        {
            if( csv.fields.empty() ) { csv.fields = comma::join( comma::csv::names< snark::render::svg::point >(), ',' ); }
            // why?!! if( options.exists( "--binary,-b" ) ) { csv.format( comma::csv::format::value< snark::render::svg::point >( csv.fields, true ) ); }
            comma::csv::input_stream< snark::render::svg::point > istream( std::cin, csv );
            snark::render::svg::polygon polygon;
            boost::optional< snark::render::svg::point > first;
            boost::optional< snark::render::svg::point > prev;
            while( istream.ready() || std::cin.good() )
            {
                const snark::render::svg::point* p = istream.read();
                if( !p ) { break; }
                if( !first ) { first = *p; }
                polygon.points.push_back( *p );
                prev = *p;
            }
            if( polygon.points.size() ) { std::cout << polygon << std::endl; }
            return 0;
        }
        if( what == "text" )
        {
            if( csv.has_field( "scalar" ) )
            {
                comma::csv::input_stream< coloured< snark::render::svg::text > > istream( std::cin, csv );
                while( istream.ready() || std::cin.good() )
                {
                    const coloured< snark::render::svg::text >* t = istream.read();
                    if( !t ) { break; }
                    std::cout << snark::render::svg::text( *t, colour_map( t->scalar ).hex() ) << std::endl;
                }
            }
            else
            {
                if( csv.fields.empty() ) { csv.fields = comma::join( comma::csv::names< snark::render::svg::text >(), ',' ); }
                comma::csv::input_stream< snark::render::svg::text > istream( std::cin, csv );
                while( istream.ready() || std::cin.good() )
                {
                    const snark::render::svg::text* t = istream.read();
                    if( !t ) { break; }
                    std::cout << *t << std::endl;
                }
            }
            return 0;
        }
        if( what == "triangle" )
        {
            bool has_scalar = csv.has_field( "scalar" );
            if( csv.fields.empty() ) { csv.fields = "points"; }
            csv.full_xpath = true;
            coloured< snark::render::svg::polygon > sample;
            sample.points.resize( 3 );
            comma::csv::input_stream< coloured< snark::render::svg::polygon > > istream( std::cin, csv, sample );
            while( std::cin.good() )
            {
                auto p = istream.read();
                if( !p ) { break; }
                if( has_scalar )
                {
                    snark::render::svg::polygon s = *p; // quick and dirty
                    s.style += "fill:" + colour_map( p->scalar ).hex();
                    std::cout << s << std::endl;
                }
                else
                {
                    std::cout << *p << std::endl;
                }
            }
            return 0;
        }
        std::cerr << "csv-to-svg: expected operation, got: \"" << what << "\"" << std::endl;
    }
    catch( std::exception& ex ) { std::cerr << "csv-to-svg: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "csv-to-svg: unknown exception" << std::endl; }
    return 1;
}
