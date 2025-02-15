// Copyright (c) 2025 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include "../../../imaging/cv_mat/filters/life.h"
#include "../../../imaging/cv_mat/traits.h"
#include "../../../imaging/cv_mat/serialization.h"
#include "melt.h"

namespace snark { namespace cv_calc { namespace melt {

std::string options()
{
    std::ostringstream oss;
    oss << R"(        options
            --radius=<pixels>; look-up radius for a nearest pixel
            --steps=<n>; how many intermediate frames to output)";
    return oss.str();
}

int run( const comma::command_line_options& options )
{
    auto [ input_serialization, output_serialization ] = cv_mat::serialization::make( options, true );
    unsigned int radius = options.value< unsigned int >( "--radius" );
    unsigned int steps = options.value< unsigned int >( "--steps" );
    typedef std::pair< boost::posix_time::ptime, cv::Mat > pair_t;
    std::vector< pair_t > outputs( steps + 1 );
    while( std::cin.good() )
    {
        pair_t p = input_serialization.read< boost::posix_time::ptime >( std::cin );
        if( p.second.empty() ) { return 0; }
        if( outputs[1].second.empty() )
        {
            for( unsigned int i = 1; i < outputs.size(); ++i )
            {
                // todo: allocate
            }
        }
        else
        {
            // todo: check size and type
            for( unsigned int i = 1; i < outputs.size(); ++i )
            {
                ( void )radius;
                // todo: interpolate pixels
                // todo: interpolate timestamp
                output_serialization.write_to_stdout( outputs[i], true );
            }
        }
        outputs[0] = p;
        output_serialization.write_to_stdout( p, true );
    }
    return 0;
}

} } } // namespace snark { namespace cv_calc { namespace melt {
