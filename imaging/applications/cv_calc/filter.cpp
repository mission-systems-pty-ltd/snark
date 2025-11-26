// Copyright (c) 2019 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <memory>
#include <sstream>
#include <boost/ptr_container/ptr_vector.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>
#if defined( CV_VERSION_EPOCH ) && CV_VERSION_EPOCH == 2
#include <opencv2/calib3d/calib3d.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif
#include <tbb/parallel_for.h>
#include <comma/base/exception.h>
#include <comma/csv/binary.h>
#include <comma/csv/options.h>
#include "../../../imaging/cv_mat/filters.h"
#include "../../../imaging/cv_mat/traits.h"
#include "grep.h"
#include "stride.h"

namespace snark { namespace cv_calc { namespace filter {
    
std::string options()
{
    return R"(
        --filters=[<filters>]; todo)";
}

int run( const comma::command_line_options& options, const snark::cv_mat::serialization& input_options, const snark::cv_mat::serialization& output_options )
{
    snark::cv_mat::serialization input_serialization( input_options );
    snark::cv_mat::serialization output_serialization( output_options );
    typedef snark::cv_mat::serialization::header::buffer_t first_t; // typedef boost::posix_time::ptime first_t;
    typedef std::pair< first_t, cv::Mat > pair_t;
    typedef snark::cv_mat::filter_with_header filter_t; // typedef snark::cv_mat::filter filter_t;
    typedef snark::cv_mat::filters_with_header filters_t; // typedef snark::cv_mat::filters filters_t;
    const comma::csv::binary< snark::cv_mat::serialization::header >* binary = input_serialization.header_binary();
    comma::csv::options csv( options );
    auto get_timestamp_from_header = [&]( const snark::cv_mat::serialization::header::buffer_t& h )->boost::posix_time::ptime
    {
        if( h.empty() || !binary ) { return boost::posix_time::not_a_date_time; }
        snark::cv_mat::serialization::header d;
        return binary->get( d, &h[0] ).timestamp;
    };
    const std::vector< filter_t >& filters = filters_t::make( options.value< std::string >( "--filters", "" ), get_timestamp_from_header );
    while( std::cin.good() && !std::cin.eof() )
    {
        pair_t p = input_serialization.read< first_t >( std::cin );
        if( p.second.empty() ) { return 0; }
        pair_t filtered;
        if( !filters.empty() )
        {
            p.second.copyTo( filtered.second );
            for( auto& filter: filters ) { filtered = filter( filtered ); }
        }
        output_serialization.write_to_stdout( filtered, csv.flush );
    }
    if( !input_serialization.last_error().empty() ) { comma::say() << input_serialization.last_error() << std::endl; }
    if( !output_serialization.last_error().empty() ) { comma::say() << output_serialization.last_error() << std::endl; }
    return 0;
}

} } } // namespace snark { namespace cv_calc { namespace filter {
