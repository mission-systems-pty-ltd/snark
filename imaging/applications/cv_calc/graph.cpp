// Copyright (c) 2024 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <array>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <comma/base/exception.h>
#include <comma/csv/names.h>
#include <comma/csv/stream.h>
#include <comma/name_value/parser.h>
#include <comma/name_value/ptree.h>
#include <comma/name_value/serialize.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include <comma/xpath/xpath.h>
#include "../../cv_mat/traits.h"
#include "graph.h"

namespace snark { namespace cv_calc { namespace graph {

std::string options()
{
    std::ostringstream oss;
    oss << "        --input-fields; print csv input fields to stdin and exit" << std::endl;
    oss << "        --fps=<n>; given frame rate, otherwise redraw on each input on change" << std::endl;
    oss << "        --list; list svg graph entities" << std::endl;
    oss << "        --svg=<image>; background svg image created using graphviz dot and transparent fill" << std::endl;
    oss << "        todo" << std::endl;
    return oss.str();
}

struct input
{
    std::uint32_t block{0};
    std::uint32_t id{0};
    std::uint32_t state{0};
};

struct svg_t // todo? move to snark/render?
{
    struct g_t
    {
        struct attr_t
        {
            std::string transform;
        };
        attr_t attr;
    };
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
            attr_t attr;
            std::string title;
            ellipse_t ellipse;
            // todo: circle
            // todo: rectangle
        };
        svg_t::g_t _g;
        attr_t attr;
        std::vector< g_t > g;
    };

    g_t::attr_t attr;
    graph_t g;
};

} } } // namespace snark { namespace cv_calc { namespace graph {

namespace comma { namespace visiting {

template <> struct traits< snark::cv_calc::graph::input >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::cv_calc::graph::input& p, Visitor& v )
    { 
        v.apply( "block", p.block );
        v.apply( "id", p.id );
        v.apply( "state", p.state );
    }
    template < typename Key, class Visitor > static void visit( const Key&, const snark::cv_calc::graph::input& p, Visitor& v )
    { 
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
    }
    template < typename Key, class Visitor > static void visit( const Key&, const snark::cv_calc::graph::svg_t::graph_t::g_t& p, Visitor& v )
    { 
        v.apply( "<xmlattr>", p.attr );
        v.apply( "title", p.title );
        v.apply( "ellipse", p.ellipse );
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

template <> struct traits< snark::cv_calc::graph::svg_t::g_t::attr_t >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::cv_calc::graph::svg_t::g_t::attr_t& p, Visitor& v )
    {
        v.apply( "transform", p.transform );
    }
    template < typename Key, class Visitor > static void visit( const Key&, const snark::cv_calc::graph::svg_t::g_t::attr_t& p, Visitor& v )
    {
        v.apply( "transform", p.transform );
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

template <> struct traits< snark::cv_calc::graph::svg_t >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::cv_calc::graph::svg_t& p, Visitor& v )
    {
        v.apply( "<xmlattr>", p.attr );
        v.apply( "g", p.g );
    }
    template < typename Key, class Visitor > static void visit( const Key&, const snark::cv_calc::graph::svg_t& p, Visitor& v )
    {
        v.apply( "<xmlattr>", p.attr );
        v.apply( "g", p.g );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace cv_calc { namespace graph {

// todo: better colour map
// todo: colour map from command line options
static std::unordered_map< unsigned int, cv::Scalar > colours = { { 0, cv::Scalar( 255, 255, 255 ) }
                                                                , { 1, cv::Scalar( 190, 190, 255 ) }
                                                                , { 2, cv::Scalar( 190, 255, 190 ) }
                                                                , { 3, cv::Scalar( 255, 190, 190 ) }
                                                                , { 4, cv::Scalar( 255, 190, 255 ) }
                                                                , { 5, cv::Scalar( 190, 255, 255 ) }
                                                                , { 6, cv::Scalar( 255, 255, 190 ) } };

int run( const comma::command_line_options& options )
{
    if( options.exists( "--input-fields" ) ) { std::cout << comma::join( comma::csv::names< input >(), ',' ) << std::endl; return 0; }
    std::string filename = options.value< std::string >( "--svg" );
    //auto svg_tree = comma::read_xml< svg_t >( filename );
    //const auto& graph = svg_tree.g;
    auto graph = comma::read_xml< svg_t::graph_t >( filename, "svg" ); // todo! get viewbox!
    if( options.exists( "--list" ) )
    {
        for( const auto& g: graph.g ) // todo: use traits -> csv
        {
            if( g.attr.classname != "node" && g.attr.classname != "edge" ) { continue; }
            std::cout << g.attr.id() << "," << g.attr.classname << "," << g.title << ",";
            if( g.ellipse.attr.rx != 0 ) { std::cout << "ellipse," << g.ellipse.attr.cx << "," << g.ellipse.attr.cy << "," << g.ellipse.attr.rx << "," << g.ellipse.attr.ry; }
            std::cout << std::endl;
        }
        return 0;
    }
    const auto& geometry = comma::split_as< float >( graph.attr.viewbox, ' ' );
    //std::cerr << "==> graph: transform: " << graph._g.attr.transform << std::endl;
    //std::cerr << "==> graph.attr.viewbox: " << graph.attr.viewbox << " geometry.size(): " << geometry.size() << std::endl;
    std::unordered_map< unsigned int, svg_t::graph_t::g_t > nodes;
    std::unordered_map< unsigned int, svg_t::graph_t::g_t > edges;
    for( const auto& g: graph.g ) { if( g.attr.classname == "node" ) { nodes[g.attr.id()] = g; } }
    for( const auto& g: graph.g ) { if( g.attr.classname == "edge" ) { edges[g.attr.id()] = g; } }
    std::string output_options_string = options.value< std::string >( "--output", "" );
    snark::cv_mat::serialization::options output_options = comma::name_value::parser( ';', '=' ).get< snark::cv_mat::serialization::options >( output_options_string );
    snark::cv_mat::serialization output_serialization( output_options );
    comma::csv::options csv( options );
    bool has_block = csv.fields.empty() || csv.has_field( "block" );
    unsigned int fps = options.value( "--fps", 0 );
    cv::Mat svg;
    cv::VideoCapture capture;
    capture.open( filename );
    capture >> svg;
    std::unordered_map< std::uint32_t, input > previous;
    std::unordered_map< std::uint32_t, input > inputs;
    comma::csv::input_stream< input > istream( std::cin, csv );
    boost::posix_time::ptime deadline;
    while( std::cin.good() || istream.ready() )
    {
        auto p = istream.read();
        bool do_output = ( !p && has_block ) || ( p && !has_block ) || ( p && !inputs.empty() && p->block != inputs.begin()->second.block );
        if( do_output )
        {
            bool changed = true; // todo: skip if no change and --on-change
            auto now = boost::posix_time::microsec_clock::universal_time();
            if( ( deadline.is_not_a_date_time() || now >= deadline ) && changed )
            {
                cv::Mat canvas; //cv::Mat canvas = cv::Mat::zeros( svg.rows, svg.cols, CV_8UC3 );
                svg.copyTo( canvas );
                for( auto n: nodes )
                {
                    auto i = inputs.find( n.first );
                    auto e = n.second.ellipse.attr;
                    cv::Point centre( e.cx / geometry[2] * svg.cols, ( 1 + e.cy / geometry[3] ) * svg.rows ); // todo: precalculate
                    centre += cv::Point( 4, -4 ); // todo!!! read transform from svg!!! i hate xml
                    cv::Size size( e.rx / geometry[2] * svg.cols, e.ry / geometry[3] * svg.rows ); // todo: precalculate
                    auto how = cv::FILLED; // parametrize: cv::LINE_AA
                    if( i == inputs.end() ) {} // { cv::ellipse( canvas, centre, size, 0, -180, 180, cv::Scalar( 0, 0, 0 ), 1, cv::LINE_AA ); }
                    else { cv::ellipse( canvas, centre, size, 0, -180, 180, colours[i->second.state % colours.size()], -1, how ); }
                }
                cv::Mat result;
                cv::min( canvas, svg, result );
                // csv-random make --type=2ui --range 0,25 | csv-paste 'line-number;size=5' - | csv-repeat --pace --period 0.05 | cv-calc graph --svg sample.svg | cv-cat 'view;null'
                output_serialization.write_to_stdout( std::make_pair( now, result ), true );
                if( fps > 0 ) { deadline = now + boost::posix_time::microseconds( long( 1000000. / fps ) ); }
            }
            previous = std::move( inputs );
            inputs.clear();
        }
        if( !p ) { break; }
        //std::cerr << "==> a: p->id: " << p->id << std::endl;
        inputs[p->id] = *p;
    }
    return 0;
}

} } } // namespace snark { namespace cv_calc { namespace graph {


// static void traverse( boost::property_tree::ptree::const_iterator i, comma::xpath& path, comma::xpath& display_path )
// {
//     if( i->second.begin() == i->second.end() )
//     {
//         std::cout << ( display_path / i->first ).to_string() << ": " << i->second.get_value< std::string >() << std::endl;
//     }
//     else
//     {
//         path /= i->first;
//         display_path /= i->first;
//         boost::optional< std::string > v = i->second.get_value_optional< std::string >();
//         if( v ) // quick and dirty
//         {
//             const std::string& stripped = comma::strip( *v );
//             if( !stripped.empty() ) { std::cout << display_path.to_string() << ": " << i->second.get_value< std::string >() << std::endl; }
//         }
//         comma::uint32 index=0;
//         //std::cout << "==> a" << std::endl;
//         for( boost::property_tree::ptree::const_iterator j = i->second.begin(); j != i->second.end(); ++j )
//         {
//             if( j->first == "g" ) { display_path.elements.back().index = index++; }
//             traverse( j, path, display_path );
//         }
//         //std::cout << "==> b" << std::endl;
//         if( !( i->first.empty() ) ) { path = path.head(); display_path = display_path.head(); }
//     }
// }

// boost::property_tree::ptree t;
// {
//     std::ifstream ifs( filename );
//     COMMA_ASSERT_BRIEF( ifs.is_open(), "failed to open svg file '" << filename << "'" );
//     boost::property_tree::read_xml( ifs, t );
// }

// comma::xpath path, display_path;
// auto s  = t.get_child_optional( "svg" );
// COMMA_ASSERT_BRIEF( s, "section 'svg' not found in '" << filename << "'" );
// for( boost::property_tree::ptree::const_iterator j = s->begin(); j != s->end(); ++j ) { traverse( j, path, display_path ); } // quick and dirty