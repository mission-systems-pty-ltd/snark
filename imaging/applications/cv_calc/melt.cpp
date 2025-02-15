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
    oss << "        options" << std::endl;
    oss << "            todo" << std::endl;
    return oss.str();
}

int run( const comma::command_line_options& options )
{
    auto [ input_serialization, output_serialization ] = cv_mat::serialization::make( options );
    while( std::cin.good() )
    {
        
    }
    return 0;
}

} } } // namespace snark { namespace cv_calc { namespace melt {
