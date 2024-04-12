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
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
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

} } // namespace comma { namespace visiting {

namespace snark { namespace cv_calc { namespace graph {

// static void traverse( boost::property_tree::ptree::const_iterator i
//                     , bool is_begin
//                     , xpath& path
//                     , xpath& display_path
//                     , property_tree::path_mode mode
//                     , char equal_sign
//                     , char delimiter
//                     , const std::string& root
//                     , bool const unquote_numbers )
// {
//     if( i->second.begin() == i->second.end() )
//     {
//         if( os ) { ptree_output_value_( *os, i->second.get_value< std::string >(), is_begin, display_path / i->first, equal_sign, delimiter, root, unquote_numbers ); }
//         else { pv.push_back( std::make_pair( display_path / i->first, i->second.get_value< std::string >() ) ); }
//     }
//     else
//     {
//         path /= i->first;
//         display_path /= i->first;
//         boost::optional< std::string > v = i->second.get_value_optional< std::string >();
//         if( v ) // quick and dirty
//         {
//             const std::string& stripped = comma::strip( *v );
//             if( !stripped.empty() )
//             { 
//                 if( os ) { ptree_output_value_( *os, stripped, is_begin, display_path, equal_sign, delimiter, root, unquote_numbers ); }
//                 else { pv.push_back( std::make_pair( display_path, stripped ) ); }
//             }
//         }
//         comma::uint32 index=0;
//         for( boost::property_tree::ptree::const_iterator j = i->second.begin(); j != i->second.end(); ++j )
//         {
//             // Test if it is json array data, if so all keys are empty. If so display indices in path if requested
//             if( mode == property_tree::without_brackets && j->first.empty()  ) { display_path /= boost::lexical_cast< std::string >( index++ ); }
//             else if( mode == property_tree::with_brackets && j->first.empty() ) { display_path.elements.back().index = index++; }
//             ptree_to_path_value_string_impl( os, pv, j, is_begin, path, display_path, mode, equal_sign, delimiter, root, unquote_numbers );
//             if( mode == property_tree::without_brackets && j->first.empty() ) { display_path = display_path.head(); }
//             is_begin = false;
//         }
//         if( !(i->first.empty()) ) { path = path.head(); display_path = display_path.head(); } // for json arrays, the keys are empty
//     }
// }

int run( const comma::command_line_options& options )
{
    if( options.exists( "--input-fields" ) ) { std::cout << comma::join( comma::csv::names< input >(), ',' ) << std::endl; return 0; }
    std::string filename = options.value< std::string >( "--svg" );
    boost::property_tree::ptree t;
    {
        std::ifstream ifs( filename );
        COMMA_ASSERT_BRIEF( ifs.is_open(), "failed to open svg file '" << filename << "'" );
        boost::property_tree::read_xml( ifs, t );
    }
    if( options.exists( "--list" ) )
    {
        // todo
        return 0;
    }
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
    cv::Mat canvas = cv::Mat::zeros( svg.rows, svg.cols, CV_8UC3 );
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
                output_serialization.write_to_stdout( std::make_pair( now, svg ), true );
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
