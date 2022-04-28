// Copyright (c) 2019 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <stdio.h>
#include <iostream>
#include <memory>
#include <sstream>
#include <boost/static_assert.hpp>
#include <opencv2/core/core.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/base/exception.h>
#include <comma/csv/impl/epoch.h>
#include "enumerate.h"

namespace snark { namespace cv_calc { namespace enumerate {

std::string options()
{
    std::ostringstream oss;
    oss << "        --prepend; prepend the count as 32-bit int before the timestamp" << std::endl;
    oss << "        --start-from,--begin=<from>; default=0; initial value" << std::endl;
    oss << std::endl;
    return oss.str();
}

int run( const comma::command_line_options& options, const snark::cv_mat::serialization::options& input_options, const snark::cv_mat::serialization::options& output_options )
{
    comma::int32 count = options.value( "--start-from,--begin", 0 );
    bool prepend = options.exists( "--prepend" );
    snark::cv_mat::serialization input( input_options );
    snark::cv_mat::serialization output( output_options );
    while( std::cin.good() && !std::cin.eof() )
    {
        std::pair< boost::posix_time::ptime, cv::Mat > p = input.read< boost::posix_time::ptime >( std::cin );
        if( p.second.empty() ) { return 0; }
        if( prepend ) { if( ::write( 1, reinterpret_cast< char* >( &count ), sizeof( comma::int32 ) ) < int( sizeof( comma::int32 ) ) ) { break; }; } // quick and dirty
        else { p.first = boost::posix_time::ptime( comma::csv::impl::epoch ) + boost::posix_time::microseconds( count ); }
        output.write_to_stdout( p );
        ++count;
    }
    return 0;
}

} } } // namespace snark { namespace cv_calc { namespace enumerate {
