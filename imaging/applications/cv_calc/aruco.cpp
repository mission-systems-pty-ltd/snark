// Copyright (c) 2026 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/base/exception.h>
#include <comma/csv/names.h>
#include <comma/csv/stream.h>
#include "../../../imaging/cv_mat/traits.h"
#include "aruco.h"

namespace snark { namespace cv_calc { namespace aruco { namespace detection {

std::string options()
{
    std::ostringstream oss;
    oss << "        --output-fields; output csv fields to stdout and exit" << std::endl;
    oss << "        --output-format; output csv format to stdout and exit" << std::endl;
    // todo? output rejected?
    return oss.str();
}

struct output
{
    boost::posix_time::ptime t;
    unsigned int block{0};
    unsigned int id{0};
    unsigned int marker{0};
    std::array< cv::Point2f, 4 > corners;
};

} } } } // namespace snark { namespace cv_calc { namespace aruco { namespace detection {

namespace comma { namespace visiting {

template <> struct traits< snark::cv_calc::aruco::detection::output >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::cv_calc::aruco::detection::output& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "block", p.block );
        v.apply( "id", p.id );
        v.apply( "marker", p.marker );
        v.apply( "corners", p.corners );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, snark::cv_calc::aruco::detection::output& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "block", p.block );
        v.apply( "id", p.id );
        v.apply( "marker", p.marker );
        v.apply( "corners", p.corners );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace cv_calc { namespace aruco { namespace detection {

int run( const comma::command_line_options& options, const snark::cv_mat::serialization::options& input_options )
{
    if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< output >(), ',' ) << std::endl; return 0; };
    if( options.exists( "--output-format" ) ) { std::cout << comma::csv::format( comma::csv::format::value< output >() ).collapsed_string() << std::endl; return 0; }
    snark::cv_mat::serialization input( input_options );
    COMMA_THROW( comma::exception, "todo" );
    while( std::cin.good() && !std::cin.eof() )
    {
        std::pair< boost::posix_time::ptime, cv::Mat > p = input.read< boost::posix_time::ptime >( std::cin );
        if( p.second.empty() ) { return 0; }

        
    }
    return 0;
}

} } } } // namespace snark { namespace cv_calc { namespace aruco { namespace detection {
