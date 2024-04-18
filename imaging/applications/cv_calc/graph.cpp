// Copyright (c) 2024 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <fstream>
#include <sstream>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <comma/application/signal_flag.h>
#include <comma/base/exception.h>
#include <comma/csv/names.h>
#include <comma/csv/stream.h>
#include <comma/io/select.h>
#include <comma/name_value/parser.h>
#include <comma/name_value/serialize.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include "../../cv_mat/traits.h"
#include "graph.h"

namespace snark { namespace cv_calc { namespace graph {

std::string options()
{
    std::ostringstream oss;
    oss << "        options" << std::endl;
    oss << "            --color,--colour=[<r>,<g>,<b>]; use fixed colour, e.g. --colour=255,0,0" << std::endl;
    oss << "            --fps=[<n>]; if --view, default: 25, given frame rate, otherwise redraw on each input on change" << std::endl;
    oss << "            --fade=[<seconds>]; if node state does not change, node colour into oblivion until next block arrives" << std::endl;
    //oss << "            --list; list svg graph entities" << std::endl;
    oss << "            --input-fields; print csv input fields to stdin and exit (usual csv/binary options supported)" << std::endl;
    oss << "            --list; list svg graph entities" << std::endl;
    oss << "            --pass; if --view, still output image to stdout" << std::endl;
    oss << "            --svg=<image>; background svg image created using graphviz dot and transparent fill" << std::endl;
    oss << "            --update-on-each-input,-u; update view on each input, clear on block change" << std::endl;
    oss << "            --view; view instead of outputting images to stdout, use --pass to override the latter" << std::endl;
    oss << "                    pressing 'p' on the view window will save the current image as <timestamp>.png" << std::endl;
    oss << "            --view-title,--title=[<title>]; if --view, title in titlebar of view window, default: svg filename" << std::endl;
    oss << "            --window-geometry=<x>,<y>[,<width>,<height>]; todo" << std::endl;
    oss << "        examples" << std::endl;
    oss << "            make a sample svg" << std::endl;
    oss << "                see: https://gitlab.com/orthographic/comma/-/wikis/name_value/visualizing-key-value-data-as-a-graph" << std::endl;
    oss << "                cat sample.json | name-value-convert --to dot | dot -Tsvg > sample.svg" << std::endl;
    oss << "            basics" << std::endl;
    oss << "                csv-random make --type=2ui --range 0,25 \\" << std::endl;
    oss << "                    | csv-paste 'line-number;size=5' - \\" << std::endl;
    oss << "                    | csv-repeat --pace --period 0.05 \\" << std::endl;
    oss << "                    | cv-calc graph --svg sample.svg --view \\" << std::endl;
    oss << "            status bar, colour fading" << std::endl;
    oss << "                csv-random make --type=2ui --range 0,25 \\" << std::endl;
    oss << "                    | csv-paste 'line-number;size=5' - \\" << std::endl;
    oss << "                    | csv-repeat --pace --period 1 \\" << std::endl;
    oss << "                    | cv-calc graph --svg sample.svg --view --status --fade=1 --update-on-each-input \\" << std::endl;
    // csv-paste line-number 'line-number;size=5;index;begin=1' value=1 line-number 'line-number;size=5;index;begin=10;step=-1' value=2 | csv-repeat --pace --period 0.1 | csv-shape split -n 3 | cv-calc graph --svg sm.svg --view --null
    // csv-paste line-number 'line-number;size=5;index;begin=1' value=1 line-number 'line-number;size=5;index;begin=10;step=-1' value=2 | csv-repeat --pace --period 0.1 | csv-shape split -n 3 | cv-calc graph --svg sm.svg | cv-cat 'invert;view;null'
    return oss.str();
}

struct input
{
    boost::posix_time::ptime t;
    std::uint32_t block{0};
    std::uint32_t id{0};
    std::uint32_t state{0};
};

struct svg_t // todo? move to snark/render?
{
    struct graph_t
    {
        struct attr_t
        {
            std::string width;
            std::string height;
            std::string viewbox;
        };
        struct g_t
        {
            struct shapes { enum values { ellipse, rectangle, circle }; }; // quick and dirty for now, use dispatch?
            struct attr_t
            {
                std::string idname;
                std::string classname;
                unsigned int id() const { return boost::lexical_cast< unsigned int >( idname.substr( classname.size() ) ); }
            };
            struct ellipse_t
            {
                struct attr_t
                {
                    float cx{0};
                    float cy{0};
                    float rx{0};
                    float ry{0};
                };
                attr_t attr;
            };
            struct circle_t
            {
                struct attr_t
                {
                    float cx{0};
                    float cy{0};
                    float r{0};
                };
                attr_t attr;
            };
            attr_t attr;
            std::string title;
            ellipse_t ellipse;
            circle_t circle;
            // todo: rectangle
        };
        attr_t attr;
        std::vector< g_t > g;
    };
};

} } } // namespace snark { namespace cv_calc { namespace graph {

namespace comma { namespace visiting {

template <> struct traits< snark::cv_calc::graph::input >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::cv_calc::graph::input& p, Visitor& v )
    { 
        v.apply( "t", p.t );
        v.apply( "block", p.block );
        v.apply( "id", p.id );
        v.apply( "state", p.state );
    }
    template < typename Key, class Visitor > static void visit( const Key&, const snark::cv_calc::graph::input& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "block", p.block );
        v.apply( "id", p.id );
        v.apply( "state", p.state );
    }
};

template <> struct traits< snark::cv_calc::graph::svg_t::graph_t::g_t::ellipse_t::attr_t >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::cv_calc::graph::svg_t::graph_t::g_t::ellipse_t::attr_t& p, Visitor& v )
    { 
        v.apply( "cx", p.cx );
        v.apply( "cy", p.cy );
        v.apply( "rx", p.rx );
        v.apply( "ry", p.ry );
    }
    template < typename Key, class Visitor > static void visit( const Key&, const snark::cv_calc::graph::svg_t::graph_t::g_t::ellipse_t::attr_t& p, Visitor& v )
    { 
        v.apply( "cx", p.cx );
        v.apply( "cy", p.cy );
        v.apply( "rx", p.rx );
        v.apply( "ry", p.ry );
    }
};

template <> struct traits< snark::cv_calc::graph::svg_t::graph_t::g_t::circle_t::attr_t >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::cv_calc::graph::svg_t::graph_t::g_t::circle_t::attr_t& p, Visitor& v )
    { 
        v.apply( "cx", p.cx );
        v.apply( "cy", p.cy );
        v.apply( "r", p.r );
    }
    template < typename Key, class Visitor > static void visit( const Key&, const snark::cv_calc::graph::svg_t::graph_t::g_t::circle_t::attr_t& p, Visitor& v )
    { 
        v.apply( "cx", p.cx );
        v.apply( "cy", p.cy );
        v.apply( "r", p.r );
    }
};

template <> struct traits< snark::cv_calc::graph::svg_t::graph_t::g_t::ellipse_t >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::cv_calc::graph::svg_t::graph_t::g_t::ellipse_t& p, Visitor& v )
    { 
        v.apply( "<xmlattr>", p.attr );
    }
    template < typename Key, class Visitor > static void visit( const Key&, const snark::cv_calc::graph::svg_t::graph_t::g_t::ellipse_t& p, Visitor& v )
    { 
        v.apply( "<xmlattr>", p.attr );
    }
};

template <> struct traits< snark::cv_calc::graph::svg_t::graph_t::g_t::circle_t >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::cv_calc::graph::svg_t::graph_t::g_t::circle_t& p, Visitor& v )
    { 
        v.apply( "<xmlattr>", p.attr );
    }
    template < typename Key, class Visitor > static void visit( const Key&, const snark::cv_calc::graph::svg_t::graph_t::g_t::circle_t& p, Visitor& v )
    { 
        v.apply( "<xmlattr>", p.attr );
    }
};

template <> struct traits< snark::cv_calc::graph::svg_t::graph_t::g_t::attr_t >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::cv_calc::graph::svg_t::graph_t::g_t::attr_t& p, Visitor& v )
    { 
        v.apply( "id", p.idname );
        v.apply( "class", p.classname );
    }
    template < typename Key, class Visitor > static void visit( const Key&, const snark::cv_calc::graph::svg_t::graph_t::g_t::attr_t& p, Visitor& v )
    { 
        v.apply( "id", p.idname );
        v.apply( "class", p.classname );
    }
};

template <> struct traits< snark::cv_calc::graph::svg_t::graph_t::g_t >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::cv_calc::graph::svg_t::graph_t::g_t& p, Visitor& v )
    { 
        v.apply( "<xmlattr>", p.attr );
        v.apply( "title", p.title );
        v.apply( "ellipse", p.ellipse );
        v.apply( "circle", p.circle );
    }
    template < typename Key, class Visitor > static void visit( const Key&, const snark::cv_calc::graph::svg_t::graph_t::g_t& p, Visitor& v )
    { 
        v.apply( "<xmlattr>", p.attr );
        v.apply( "title", p.title );
        v.apply( "ellipse", p.ellipse );
        v.apply( "circle", p.circle );
    }
};

template <> struct traits< snark::cv_calc::graph::svg_t::graph_t::attr_t >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::cv_calc::graph::svg_t::graph_t::attr_t& p, Visitor& v )
    {
        v.apply( "width", p.width );
        v.apply( "height", p.height );
        v.apply( "viewBox", p.viewbox );
    }
    template < typename Key, class Visitor > static void visit( const Key&, const snark::cv_calc::graph::svg_t::graph_t::attr_t& p, Visitor& v )
    { 
        v.apply( "width", p.width );
        v.apply( "height", p.height );
        v.apply( "viewBox", p.viewbox );
    }
};

template <> struct traits< snark::cv_calc::graph::svg_t::graph_t >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::cv_calc::graph::svg_t::graph_t& p, Visitor& v )
    {
        v.apply( "<xmlattr>", p.attr );
        v.apply( "g", p.g );
    }
    template < typename Key, class Visitor > static void visit( const Key&, const snark::cv_calc::graph::svg_t::graph_t& p, Visitor& v )
    {
        v.apply( "<xmlattr>", p.attr );
        v.apply( "g", p.g );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace cv_calc { namespace graph {

// todo: better colour map
// todo: --colours: colour map from command line options
static std::unordered_map< unsigned int, cv::Scalar > colours = { { 0, cv::Scalar( 220, 220, 220 ) }
                                                                , { 1, cv::Scalar( 160, 160, 220 ) }
                                                                , { 2, cv::Scalar( 160, 220, 160 ) }
                                                                , { 3, cv::Scalar( 220, 160, 160 ) }
                                                                , { 4, cv::Scalar( 220, 160, 220 ) }
                                                                , { 5, cv::Scalar( 160, 220, 220 ) }
                                                                , { 6, cv::Scalar( 220, 220, 160 ) } };

int run( const comma::command_line_options& options )
{
    if( options.exists( "--input-fields" ) ) { std::cout << comma::join( comma::csv::names< input >(), ',' ) << std::endl; return 0; }
    std::string filename = options.value< std::string >( "--svg" );
    std::string title = options.value( "--view-title,--title", filename );
    auto graph = comma::read_xml< svg_t::graph_t >( filename, "svg" );
    if( options.exists( "--list" ) )
    {
        for( const auto& g: graph.g ) // todo: use traits -> csv
        {
            if( g.attr.classname != "node" && g.attr.classname != "edge" ) { continue; }
            std::cout << g.attr.id() << "," << g.attr.classname << "," << g.title;
            if( g.ellipse.attr.rx != 0 ) { std::cout << ",ellipse," << g.ellipse.attr.cx << "," << g.ellipse.attr.cy << "," << g.ellipse.attr.rx << "," << g.ellipse.attr.ry; }
            if( g.circle.attr.r != 0 ) { std::cout << ",circle," << g.circle.attr.cx << "," << g.circle.attr.cy << "," << g.circle.attr.r; }
            std::cout << std::endl;
        }
        return 0;
    }
    std::string transform;
    {
        boost::property_tree::ptree t;
        std::ifstream ifs( filename );
        boost::property_tree::read_xml( ifs, t );
        auto s = t.get_child( "svg" );
        for( auto i = s.begin(); i != s.end() && transform.empty(); ++i ) // todo! do it via traits; it sucks so much! i cannot get their data model
        {
            for( auto j = i->second.begin(); j != i->second.end() && transform.empty(); ++j )
            {
                for( auto k = j->second.begin(); k != j->second.end(); ++k )
                {
                    if( k->first == "transform" ) { transform = k->second.get_value< std::string >(); break; }
                }
            }
        }
    }
    cv::Point translate( 0, 0 );
    if( !transform.empty() ) // quick and dirty, svg decided why make it easy?
    {
        for( unsigned int i = 0; i < transform.size(); ++i )
        {
            if( transform[i] == '(' && i > 9 && transform.substr( i - 9, 9 ) == "translate" )
            {
                auto q = transform.substr( i + 1 );
                auto p = q.find_first_of( ')' );
                COMMA_ASSERT_BRIEF( p != std::string::npos, "invalid transform string: '" << transform << "'" );
                const auto& r = comma::split_as< float >( q.substr( 0, p ), ' ' );
                translate = cv::Point( r[0], r[1] );
                break;
            }
        }
    }
    const auto& geometry = comma::split_as< float >( graph.attr.viewbox, ' ' );
    std::unordered_map< unsigned int, svg_t::graph_t::g_t > nodes;
    std::unordered_map< unsigned int, svg_t::graph_t::g_t > edges;
    for( const auto& g: graph.g ) { if( g.attr.classname == "node" ) { nodes[g.attr.id()] = g; } }
    for( const auto& g: graph.g ) { if( g.attr.classname == "edge" ) { edges[g.attr.id()] = g; } }
    std::string output_options_string = options.value< std::string >( "--output", "" );
    snark::cv_mat::serialization::options output_options = comma::name_value::parser( ';', '=' ).get< snark::cv_mat::serialization::options >( output_options_string );
    snark::cv_mat::serialization output_serialization( output_options );
    comma::csv::options csv( options, "block,id,state" );
    bool has_block = csv.fields.empty() || csv.has_field( "block" );
    bool has_time = csv.fields.empty() || csv.has_field( "t" );
    bool view = options.exists( "--view" );
    bool pass = options.exists( "--pass" ) || !view;
    bool status = options.exists( "--status" );
    float fps = options.value( "--fps", view ? 25 : 0 );
    auto fade = options.optional< double >( "--fade" );
    COMMA_ASSERT_BRIEF( fps < 1000, "--fps greater than 1000 not supported; got: " << fps );
    bool update_on_each_input = options.exists( "--update-on-each-input,-u" );
    std::optional< cv::Scalar > colour;
    std::string colour_string = options.value< std::string >( "--color,--colour", "" );
    if( !colour_string.empty() )
    {
        const auto& v = comma::split_as< float >( colour_string, ',' );
        COMMA_ASSERT_BRIEF( v.size() == 3, "expected --colour=<r>,<g>,<b> base 256; got '" << colour_string << "'" );
        colour = cv::Scalar( 255 - v[2], 255 - v[1], 255 - v[0] );
    }
    cv::Mat svg;
    cv::VideoCapture capture;
    capture.open( filename );
    capture >> svg;
    std::unordered_map< std::uint32_t, input > inputs;
    //std::unordered_map< std::uint32_t, std::pair<  > > 
    comma::csv::input_stream< input > istream( std::cin, csv );
    boost::posix_time::ptime now;
    std::optional< boost::posix_time::ptime > deadline;
    cv::Mat result;
    result = cv::Scalar( 255, 255, 255 ) - svg;
    std::recursive_mutex mutex;
    std::uint64_t count{0};
    comma::signal_flag is_shutdown;
    bool done{false};
    auto imshow = [&]()
    {
        std::pair< boost::posix_time::ptime, cv::Mat > m;
        while( !( is_shutdown || done ) )
        {
            { std::scoped_lock lock( mutex ); m.first = now; result.copyTo( m.second ); }
            if( pass ) { output_serialization.write_to_stdout( m, true ); }
            cv::imshow( &title[0], m.second );
            char c = cv::waitKey( 1000 / fps );
            if( c == 27 || c == 119 ) { done = true; } // ctrl-w: 119
            else if( c == ' ' || c == 'p' )
            {
                std::string f = boost::posix_time::to_iso_string( m.first ) + ".png";
                std::cerr << "cv-calc: graph: screenshot saved to " << f << std::endl;
                cv::imwrite( f, m.second );
            }
        }
    };
    auto draw = [&]( boost::posix_time::ptime t )
    {
        int rows = svg.rows + ( status ? svg.cols > 400 ? 40 : 80 : 0 );
        static cv::Mat canvas( rows, svg.cols, CV_8UC3 );
        canvas = cv::Scalar( 255, 255, 255 ); // quick and dirty, watch performance
        static cv::Mat overlay( rows, svg.cols, CV_8UC3 ); // todo! make once and reuse
        overlay = cv::Scalar( 255, 255, 255 ); // quick and dirty, watch performance
        svg.copyTo( overlay( cv::Rect( 0, 0, svg.cols, svg.rows ) ) );
        for( auto i: inputs )
        {
            auto n = nodes.find( i.first );
            if( n == nodes.end() ) { continue; }
            auto how = cv::FILLED; // parametrize: cv::LINE_AA
            cv::Scalar c = colour ? *colour : colours[i.second.state % colours.size()];
            if( fade )
            { 
                float r = float( ( now - i.second.t ).total_milliseconds() ) / 1000 / *fade;
                r = r < 0 ? 0 : r > 1 ? 1 : r;
                c = cv::Scalar( 255, 255, 255 ) * r + c * ( 1. - r );
            }
            // todo: rectangle
            if( n->second.ellipse.attr.rx > 0 )
            {
                auto e = n->second.ellipse.attr; // todo: other shapes
                cv::Point centre( ( e.cx + translate.x ) / geometry[2] * svg.cols, (  e.cy + translate.y ) / geometry[3] * svg.rows ); // todo: precalculate
                cv::Size size( e.rx / geometry[2] * svg.cols, e.ry / geometry[3] * svg.rows ); // todo: precalculate
                cv::ellipse( canvas, centre, size, 0, -180, 180, c, -1, how );
            }
            else if( n->second.circle.attr.r > 0 )
            {
                auto e = n->second.circle.attr; // todo: other shapes
                cv::Point centre( ( e.cx + translate.x ) / geometry[2] * svg.cols, (  e.cy + translate.y ) / geometry[3] * svg.rows ); // todo: precalculate
                cv::Size size( e.r / geometry[2] * svg.cols, e.r / geometry[3] * svg.rows ); // todo: precalculate
                cv::ellipse( canvas, centre, size, 0, -180, 180, c, -1, how );
            }
        }
        if( status )
        {
            static std::uint64_t last_count = count;
            static boost::posix_time::ptime last;
            //static boost::posix_time::ptime start = t;
            //rate += ( inputs.size() - average_input_size ) * 0.5;
            cv::Point p0{10, canvas.rows - 11}, p1{170, canvas.rows - 11}, p2{280, canvas.rows - 11};
            if( svg.cols <= 400 ) { p0 = {10, canvas.rows - 51}, p1 = {10, canvas.rows - 31}, p2 = {10, canvas.rows - 11}; }
            cv::Scalar text_color( 100, 100, 100 );
            cv::putText( canvas, boost::posix_time::to_iso_string( t ).substr( 0, 18 ), p0, cv::FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1, cv::LINE_AA );
            cv::putText( canvas, "inputs: " + boost::lexical_cast< std::string >( count ), p1, cv::FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1, cv::LINE_AA );
            cv::putText( canvas, "updated: " + ( last.is_not_a_date_time() ? "never" : boost::lexical_cast< std::string >( ( t - last ).seconds() ) + "." + boost::lexical_cast< std::string >( ( t - last ).total_milliseconds() % 1000 ) ), p2, cv::FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1, cv::LINE_AA );
            if( !last.is_not_a_date_time() ) { cv::putText( canvas, "sec ago", p2 + cv::Point( 120, 0 ), cv::FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1, cv::LINE_AA ); }
            if( last_count < count ) { last = t; }
            last_count = count;
        }
        {
            std::scoped_lock lock( mutex );
            cv::min( canvas, overlay, result );
            result = cv::Scalar( 255, 255, 255 ) - result;
        }
    };
    auto update = [&]( const input* p, bool clear )->bool
    {
        if( !p ) { return false; }
        if( clear ) { inputs.clear(); }
        auto& i = inputs[p->id];
        i = *p;
        i.t = now;
        return true;
    };
    std::optional< std::thread > imshow_thread;
    if( view ) { imshow_thread.emplace( imshow ); }
    comma::io::select select;
    select.read().add( 0 );
    while( ( istream.ready() || std::cin.good() ) && !is_shutdown && !done )
    {
        if( !istream.ready() ) { select.wait( boost::posix_time::milliseconds( 100 ) ); }
        if( is_shutdown || done ) { break; }        
        bool ready = istream.ready() || select.read().ready( 0 );
        bool block_changed{false};
        const input* p{nullptr};
        if( ready )
        {
            p = istream.read();
            ++count;
            { std::scoped_lock lock( mutex ); now = has_time ? p ? p->t : now : boost::posix_time::microsec_clock::universal_time(); } // todo: quick and dirty, better synchronization
            block_changed = ( !p && has_block ) || ( p && !has_block ) || ( p && !inputs.empty() && p->block != inputs.begin()->second.block );
        }
        else
        {
            if( has_time ) { continue; } // not much we can do
            { std::scoped_lock lock( mutex ); now = boost::posix_time::microsec_clock::universal_time(); } // todo: quick and dirty, better synchronization
        }
        bool deadline_expired = deadline && now >= *deadline;
        if( update_on_each_input && ready ) { if( !update( p, block_changed ) ) { break; } }
        if( update_on_each_input || block_changed || deadline_expired )
        {
            draw( now );
            if( !view ) { output_serialization.write_to_stdout( std::make_pair( now, result ), true ); }
            if( fps > 0 ) { deadline = now + boost::posix_time::microseconds( long( 1000000. / fps ) ); }
        }
        if( ready ) { if( !update( p, block_changed ) ) { break; } }
    }
    done = true;
    if( view ) { imshow_thread->join(); }
    return 0;
}

} } } // namespace snark { namespace cv_calc { namespace graph {
