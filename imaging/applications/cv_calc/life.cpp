// Copyright (c) 2024 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <comma/base/exception.h>
#include "../../../imaging/cv_mat/filters/life.h"
#include "../../../imaging/cv_mat/serialization.h"
#include "../../../imaging/cv_mat/traits.h"
#include "life.h"

namespace snark { namespace cv_calc { namespace life {

std::string options()
{
    std::ostringstream oss;
    oss << R"(        options
            --exit-on-stability: exit, if no change
            --procreation-treshold,--procreation=[<threshold>]: todo: document; default: 3.0
            --stability-treshold,--stability,--extinction-threshold,--extinction=[<threshold>]: todo: document; default: 4.0
            --step=[<step>]: todo: document; default: 1.0)";
    return oss.str();
}

int run( const comma::command_line_options& options )
{
    auto [ input_serialization, output_serialization ] = cv_mat::serialization::make( options );
    double procreation_threshold = options.value( "--procreation-threshold,--procreation", 3.0 );
    double stability_threshold = options.value( "--stability-threshold,--stability,--extinction-threshold,--extinction", 4.0 );
    double step = options.value( "--step", 1.0 );
    bool exit_on_stability = options.exists( "--exit-on-stability" );
    snark::cv_mat::filters::life< boost::posix_time::ptime > life( procreation_threshold, stability_threshold, step, exit_on_stability );
    auto iterations = options.optional< unsigned int >( "--iterations,-i" );
    std::pair< boost::posix_time::ptime, cv::Mat > p = input_serialization.read< boost::posix_time::ptime >( std::cin );
    if( p.second.empty() ) { return 0; }
    for( unsigned int i = 0; ( !iterations || i < *iterations ) && std::cout.good(); ++i )
    {
        output_serialization.write_to_stdout( life( p ) ); // todo: decouple step from output
        std::cout.flush();
    }
    COMMA_ASSERT_BRIEF( input_serialization.last_error().empty(), input_serialization.last_error() );
    COMMA_ASSERT_BRIEF( output_serialization.last_error().empty(), output_serialization.last_error() );
    return 0;
}

} } } // namespace snark { namespace cv_calc { namespace life {
