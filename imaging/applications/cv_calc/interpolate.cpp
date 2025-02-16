// Copyright (c) 2025 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <tbb/parallel_for.h>
#include "../../../imaging/cv_mat/filters/life.h"
#include "../../../imaging/cv_mat/traits.h"
#include "../../../imaging/cv_mat/serialization.h"
#include "interpolate.h"

namespace snark { namespace cv_calc { namespace interpolate {

std::string options()
{
    std::ostringstream oss;
    oss << R"(        options
            --radius=<pixels>; default=0; look-up radius for a nearest pixel
            --steps=<n>; how many intermediate frames to output)";
    return oss.str();
}

//template < typename T, unsigned int D >
static void _interpolate( cv::Mat& m, const cv::Mat& p, const cv::Mat& t, unsigned int row, unsigned int col, unsigned int steps, unsigned int radius )
{

}

//template < typename T, unsigned int D >
static void _morph( cv::Mat& m, const cv::Mat& p, const cv::Mat& t, unsigned int row, unsigned int col, unsigned int steps, unsigned int radius )
{
    COMMA_ASSERT_BRIEF( radius == 0, "todo" );
}

int run( const comma::command_line_options& options )
{
    auto [ input_serialization, output_serialization ] = cv_mat::serialization::make( options, true );
    unsigned int radius = options.value( "--radius", 0u );
    unsigned int steps = options.value< unsigned int >( "--steps" );
    COMMA_ASSERT_BRIEF( radius == 0, "radius > 0: todo" );
    typedef std::pair< boost::posix_time::ptime, cv::Mat > pair_t;
    pair_t previous, next, last;
    auto morph = radius == 0 ? _interpolate : _morph;
    while( std::cin.good() )
    {
        pair_t last = input_serialization.read< boost::posix_time::ptime >( std::cin );
        if( last.second.empty() ) { return 0; }
        if( !previous.second.empty() )
        {
            COMMA_ASSERT_BRIEF( last.second.type() == previous.second.type(), "expected images of the same type " << previous.second.type() << "; got: " << last.second.type() );
            if( next.second.empty() ) { next.second = cv::Mat( last.second.rows, last.second.cols, last.second.type() ); }
            bool has_timestamp = !last.first.is_not_a_date_time() && !previous.first.is_not_a_date_time();
            boost::posix_time::time_duration dt;
            if( has_timestamp )
            {
                dt = ( last.first - previous.first ) / ( steps + 1 );
                next.first = previous.first + dt;
            }
            for( unsigned int i{0}, remaining{steps}; i < steps; ++i, --remaining, next.first = has_timestamp ? next.first + dt : boost::posix_time::ptime() )
            {
                tbb::parallel_for( tbb::blocked_range< std::size_t >( 0, last.second.rows )
                                 , [&]( const tbb::blocked_range< std::size_t >& r )
                    {
                        for( unsigned int row = r.begin(); row < r.end(); ++row )
                        {
                            for( unsigned int col = 0; int( col ) < last.second.cols; ++col ) { morph( next.second, previous.second, last.second, row, col, remaining, radius ); }
                        }
                    } );
                output_serialization.write_to_stdout( next, true );
                previous = next;
            }
        }
        output_serialization.write_to_stdout( last, true );
        previous = last;
    }
    return 0;
}

} } } // namespace snark { namespace cv_calc { namespace interpolate {
