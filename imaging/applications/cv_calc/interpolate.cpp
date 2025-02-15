// Copyright (c) 2025 Vsevolod Vlaskine

/// @author vsevolod vlaskine

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

int run( const comma::command_line_options& options )
{
    auto [ input_serialization, output_serialization ] = cv_mat::serialization::make( options, true );
    unsigned int radius = options.value( "--radius", 0u );
    unsigned int steps = options.value< unsigned int >( "--steps" );
    typedef std::pair< boost::posix_time::ptime, cv::Mat > pair_t;
    pair_t previous, next;
    while( std::cin.good() )
    {
        pair_t p = input_serialization.read< boost::posix_time::ptime >( std::cin );
        if( p.second.empty() ) { return 0; }
        if( !previous.second.empty() )
        {
            COMMA_ASSERT_BRIEF( p.second.type() == previous.second.type(), "expected images of the same type " << previous.second.type() << "; got: " << p.second.type() );
            if( next.second.empty() ) { next.second = cv::Mat( p.second.rows, p.second.cols, p.second.type() ); }
            boost::posix_time::time_duration dt;
            if( !p.first.is_not_a_date_time() && !previous.first.is_not_a_date_time() ) { dt = ( p.first - previous.first ) / ( steps + 1 ); }
            next.first = previous.first + dt;
            for( unsigned int i = 1; i < steps; ++i, next.first += dt )
            {
                ( void )radius;

                // todo: interpolate pixels
                
                output_serialization.write_to_stdout( next, true );
                previous = next;
            }
        }
        previous = p;
        output_serialization.write_to_stdout( p, true );
    }
    return 0;
}

} } } // namespace snark { namespace cv_calc { namespace interpolate {
