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

namespace snark { namespace cv_calc { namespace stride {
    
std::string options()
{
    return R"(
        --filter,--filters=[<filters>]; see grep operation; added to stride for performance
        --fit-last; fit last stride exactly to the image size, i.e. last stride may be irregular
        --input=[<options>]; input options; run cv-cat --help --verbose for details
        --non-zero=[<what>]; see grep operation; added to stride for performance
        --output=[<options>]; output options; run cv-cat --help --verbose for details
        --padding=[<padding>]; padding, 'same' or 'valid' (see e.g. tensorflow for the meaning); default: valid
        --shape,--kernel,--size=<x>,<y>; image size
        --strides=[<x>,<y>]; stride size; default: 1,1)";
}

int run( const comma::command_line_options& options, const snark::cv_mat::serialization& input_options, const snark::cv_mat::serialization& output_options )
{
    snark::cv_mat::serialization input_serialization( input_options );
    snark::cv_mat::serialization output_serialization( output_options );
    bool fit_last = options.exists( "--fit-last" );
    const std::vector< std::string >& strides_vector = comma::split( options.value< std::string >( "--strides", "1,1" ), ',' );
    if( strides_vector.size() != 2 ) { std::cerr << "cv-calc: stride: expected strides as <x>,<y>, got: \"" << options.value< std::string >( "--strides" ) << std::endl; return 1; }
    std::pair< unsigned int, unsigned int > strides( boost::lexical_cast< unsigned int >( strides_vector[0] ), boost::lexical_cast< unsigned int >( strides_vector[1] ) );
    const std::vector< std::string >& shape_vector = comma::split( options.value< std::string >( "--shape,--size,--kernel" ), ',' );
    if( shape_vector.size() != 2 ) { std::cerr << "cv-calc: stride: expected shape as <x>,<y>, got: \"" << options.value< std::string >( "--shape,--size,--kernel" ) << std::endl; return 1; }
    std::pair< unsigned int, unsigned int > shape( boost::lexical_cast< unsigned int >( shape_vector[0] ), boost::lexical_cast< unsigned int >( shape_vector[1] ) );
    unsigned int shape_size = shape.first * shape.second;
    struct padding_types { enum values { same, valid }; };
    std::string padding_string = options.value< std::string >( "--padding", "valid" );
    padding_types::values padding = padding_types::same;
    if( padding_string == "same" || padding_string == "SAME" ) { padding = padding_types::same; std::cerr << "cv-calc: stride: padding 'same' not implemented; please use --padding=valid" << std::endl; return 1; }
    else if( padding_string == "valid" || padding_string == "VALID" ) { padding = padding_types::valid; }
    else { std::cerr << "cv-calc: stride: expected padding type, got: \"" << padding_string << "\"" << std::endl; return 1; }
    grep::non_zero non_zero( options.value< std::string >( "--non-zero", "" ) );
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
    const std::vector< filter_t >& filters = filters_t::make( options.value< std::string >( "--filter,--filters", "" ), get_timestamp_from_header );
    if( !non_zero && !filters.empty() ) { std::cerr << "cv-calc: stride: warning: --filters specified, but --non-zero is not; --filters will have no effect" << std::endl; }
    while( std::cin.good() && !std::cin.eof() )
    {
        pair_t p = input_serialization.read< first_t >( std::cin );
        if( p.second.empty() ) { return 0; }
        pair_t filtered;
        if( !filters.empty() )
        {
            p.second.copyTo( filtered.second );
            for( auto& filter: filters ) { filtered = filter( filtered ); }
            if( filtered.second.rows != p.second.rows || filtered.second.cols != p.second.cols ) { std::cerr << "cv-calc: stride: expected original and filtered images of the same size, got " << p.second.rows << "," << p.second.cols << " vs " << filtered.second.rows << "," << filtered.second.cols << std::endl; return 1; }
        }
        switch( padding )
        {
            case padding_types::same: // todo
                break;
            case padding_types::valid:
            {
                if( p.second.cols < int( shape.first ) || p.second.rows < int( shape.second ) ) { std::cerr << "cv-calc: stride: expected image greater than rows: " << shape.second << " cols: " << shape.first << "; got rows: " << p.second.rows << " cols: " << p.second.cols << std::endl; return 1; }
                pair_t q;
                q.first = p.first;
                bool is_last_row = false;
                for( unsigned int j = 0; !is_last_row; j += strides.second )
                {
                    if( fit_last && int( j ) < p.second.rows && j > p.second.rows - shape.second ) { j = p.second.rows - shape.second; is_last_row = true; }
                    if( j >= ( p.second.rows + 1 - shape.second ) ) { break; }
                    bool is_last_col = false;
                    for( unsigned int i = 0; !is_last_col; i += strides.first )
                    {
                        if( fit_last && int( i ) < p.second.cols && i > p.second.cols - shape.first ) { i = p.second.cols - shape.first; is_last_col = true; }
                        if( i >= ( p.second.cols + 1 - shape.first ) ) { break; }
                        if( !filtered.second.empty() )
                        {
                            filtered.second( cv::Rect( i, j, shape.first, shape.second ) ).copyTo( q.second );
                            non_zero.size( shape_size );
                            if( !non_zero.keep( q.second ) ) { continue; }
                        }
                        p.second( cv::Rect( i, j, shape.first, shape.second ) ).copyTo( q.second );
                        output_serialization.write_to_stdout( q, csv.flush );
                    }
                }
                break;
            }
        }
    }
    if( !input_serialization.last_error().empty() ) { comma::say() << input_serialization.last_error() << std::endl; }
    if( !output_serialization.last_error().empty() ) { comma::say() << output_serialization.last_error() << std::endl; }
    return 0;
}

} } } // namespace snark { namespace cv_calc { namespace stride {
