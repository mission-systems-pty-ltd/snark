// Copyright (c) 2024 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <sstream>
#include <unordered_map>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/core/version.hpp>
// #if defined( CV_VERSION_EPOCH ) && CV_VERSION_EPOCH == 2
// #include <opencv2/calib3d/calib3d.hpp>
// #else
// #include <opencv2/calib3d.hpp>
// #endif
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/visiting/traits.h>
#include "graph.h"

namespace snark { namespace cv_calc { namespace graph {

std::string options()
{
    std::ostringstream oss;
    oss << "        --input-fields; print csv input fields to stdin and exit" << std::endl;
    oss << "        --fps=<n>; given frame rate, otherwise redraw on each input on change" << std::endl;
    oss << "        --svg=<image>; background svg image" << std::endl;
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

int run( const comma::command_line_options& options, const snark::cv_mat::serialization& input_options, const snark::cv_mat::serialization& output_options )
{
    snark::cv_mat::serialization input_serialization( input_options ); ( void )( input_serialization ); // in case we later decide to have svg input stream or alike
    snark::cv_mat::serialization output_serialization( output_options );
    comma::csv::options csv( options );
    comma::csv::input_stream< input > istream( std::cin, csv );
    bool has_block = csv.has_field( "block" );
    std::unordered_map< std::uint32_t, input > previous;
    std::unordered_map< std::uint32_t, input > inputs;
    while( std::cin.good() || istream.ready() )
    {
        auto p = istream.read();
        if( !p || !has_block || ( !inputs.empty() && p->block != inputs.begin()->second.block ) )
        {
            // todo: fps
            // todo: changed?
            previous = std::move( inputs );
            inputs.clear();
        }
        if( !p ) { break; }
        inputs[p->id] = *p;
    }
    return 0;
}

} } } // namespace snark { namespace cv_calc { namespace graph {
