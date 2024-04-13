// Copyright (c) 2024 Vsevolod Vlaskine

/// @author vsevolod vlaskine

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
    oss << "            --input-fields; print csv input fields to stdin and exit (usual csv/binary options supported)" << std::endl;
    oss << "            --fps=<n>; given frame rate, otherwise redraw on each input on change" << std::endl;
    oss << "            --list; list svg graph entities" << std::endl;
    oss << "            --svg=<image>; background svg image created using graphviz dot and transparent fill" << std::endl;
    oss << "        example" << std::endl;
    oss << "            see: https://gitlab.com/orthographic/comma/-/wikis/name_value/visualizing-key-value-data-as-a-graph" << std::endl;
    oss << "            cat sample.json | name-value-convert --to dot | dot -Tsvg > sample.svg" << std::endl;
    oss << "            csv-random make --type=2ui --range 0,25 \\" << std::endl;
    oss << "                | csv-paste 'line-number;size=5' - \\" << std::endl;
    oss << "                | csv-repeat --pace --period 0.05 \\" << std::endl;
    oss << "                | cv-calc graph --svg sample.svg  \\" << std::endl;
    oss << "                | cv-cat 'view;null'" << std::endl;
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
    auto graph = comma::read_xml< svg_t::graph_t >( filename, "svg" );
    if( options.exists( "--list" ) )
    {
        for( const auto& g: graph.g ) // todo: use traits -> csv
        {
            if( g.attr.classname != "node" && g.attr.classname != "edge" ) { continue; }
            std::cout << g.attr.id() << "," << g.attr.classname << "," << g.title;
            if( g.ellipse.attr.rx != 0 ) { std::cout << ",ellipse," << g.ellipse.attr.cx << "," << g.ellipse.attr.cy << "," << g.ellipse.attr.rx << "," << g.ellipse.attr.ry; }
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
                for( auto i: inputs )
                {
                    auto n = nodes.find( i.first );
                    if( n == nodes.end() ) { continue; }
                    auto e = n->second.ellipse.attr; // todo: other shapes
                    cv::Point centre( ( e.cx + translate.x ) / geometry[2] * svg.cols, (  e.cy + translate.y ) / geometry[3] * svg.rows ); // todo: precalculate
                    cv::Size size( e.rx / geometry[2] * svg.cols, e.ry / geometry[3] * svg.rows ); // todo: precalculate
                    auto how = cv::FILLED; // parametrize: cv::LINE_AA
                    cv::ellipse( canvas, centre, size, 0, -180, 180, colours[i.second.state % colours.size()], -1, how );
                }
                cv::Mat result;
                cv::min( canvas, svg, result );
                output_serialization.write_to_stdout( std::make_pair( now, result ), true );
                if( fps > 0 ) { deadline = now + boost::posix_time::microseconds( long( 1000000. / fps ) ); }
            }
            previous = std::move( inputs );
            inputs.clear();
        }
        if( !p ) { break; }
        inputs[p->id] = *p;
    }
    return 0;
}

} } } // namespace snark { namespace cv_calc { namespace graph {
