// Copyright (c) 2025 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <numeric>
#include <tbb/parallel_for.h>
#include <comma/base/exception.h>
#include <comma/string/split.h>
#include <comma/csv/binary.h>
#include <comma/csv/options.h>
#include "../../../imaging/cv_mat/filters.h"
#include "../../../imaging/cv_mat/traits.h"
#include "grep.h"

namespace snark { namespace cv_calc { namespace grep {

non_zero::non_zero( const std::string& s )
{
    if( s.empty() ) { return; }
    const std::vector< std::string >& v = comma::split( s, ',' );
    if( v[0] == "ratio" )
    {
        if( v.size() < 2 ) { COMMA_THROW( comma::exception, "expected --non-zero=ratio,<min>,<max>; got --non-zero=ratio" ); }
        if( !v[1].empty() ) { ratio_.first = boost::lexical_cast< double >( v[1] ); }
        if( v.size() > 2 && !v[2].empty() ) { ratio_.second = boost::lexical_cast< double >( v[2] ); }
        return;
    }
    if( v[0] == "size" )
    {
        if( v.size() < 2 ) { COMMA_THROW( comma::exception, "expected --non-zero=size,<min>,<max>; got --non-zero=size" ); }
        if( !v[1].empty() ) { size_.first = boost::lexical_cast< unsigned int >( v[1] ); }
        if( v.size() > 2 && !v[2].empty() ) { size_.second = boost::lexical_cast< unsigned int >( v[2] ); }
        return;
    }
    COMMA_THROW( comma::exception, "--non-zero: expected 'ratio' or 'size', got: '" << v[0] << "'" );
}

non_zero::operator bool() const { return static_cast< bool >( ratio_.first ) || static_cast< bool >( ratio_.second ) || static_cast< bool >( size_.first ) || static_cast< bool >( size_.second ); }

void non_zero::size( unsigned int image_size )
{
    if( ratio_.first ) { size_.first = image_size * *ratio_.first; }
    if( ratio_.second ) { size_.second = image_size * *ratio_.second; }
}

bool non_zero::keep( unsigned int count ) const { return ( !size_.first || *size_.first <= count ) && ( !size_.second || count < *size_.second ); }

bool non_zero::keep( const cv::Mat& m ) const { return keep( count( m ) ); }

unsigned int non_zero::count( const cv::Mat& m ) const
{
    static std::vector< char > zero_pixel( m.elemSize(), 0 );
    std::vector< unsigned int > counts( m.rows, 0 );
    tbb::parallel_for( tbb::blocked_range< std::size_t >( 0, m.rows )
                     , [&]( const tbb::blocked_range< std::size_t >& r )
                       {
                           for( unsigned int i = r.begin(); i < r.end(); ++i )
                           {
                               for( const unsigned char* ptr = m.ptr( i ); ptr < m.ptr( i + 1 ); ptr += m.elemSize() )
                               {
                                   if( ::memcmp( ptr, &zero_pixel[0], zero_pixel.size() ) != 0 ) { ++counts[i]; }
                               }
                           }
                       } );
    return std::accumulate( counts.begin(), counts.end(), 0 );
}

bool non_zero::keep_counting_( unsigned int count ) const
{
    if( size_.second ) { return *size_.second < count; }
    return size_.first && ( count < *size_.first );
}

std::string options()
{
    return R"(
        --filter,--filters=[<filters>]; apply --non-zero logic to the image with filters applied, not to image itself
                                        run cv-cat --help --verbose for filters available
        --non-zero=[<what>]; output only images that have non-zero pixels
            <what>
                ratio,[<min>][,<max>]: output only images with number of non-zero pixels within the limits of given ratios, e.g:
                                           --non-zero=ratio,0.2,0.8: output images that have from 20 to 80% of non-zero pixels
                                           --non-zero=ratio,,0.8: output images that have up to 80% of non-zero pixels
                                           --non-zero=ratio,0.8: output images that have at least 80% of non-zero pixels
                size,[<min>][,<max>]: output only images with number of non-zero pixels within given limits
                                      lower limit inclusive, upper limit exclusive; e.g
                                          --non-zero=size,10,1000: output images that have from 10 to 999 non-zero pixels
                                          --non-zero=size,10: output images that have at least 10 non-zero pixels
                                          --non-zero=size,,1000: output images that have not more than 999 non-zero pixels
                                          --non-zero=size,,1: output images with all pixels zero; makes sense only when used with --filters)";
}

int run( const comma::command_line_options& options, const snark::cv_mat::serialization& input_options, const snark::cv_mat::serialization& output_options )
{
    // Need to be created inside, some operation (roi) has other default fields. If not using --binary also requires --fields
    snark::cv_mat::serialization input_serialization( input_options );
    snark::cv_mat::serialization output_serialization( output_options );
    snark::cv_calc::grep::non_zero non_zero( options.value< std::string >( "--non-zero", "" ) );
    const std::vector< snark::cv_mat::filter >& filters = snark::cv_mat::impl::filters<>::make( options.value< std::string >( "--filter,--filters", "" ) );
    if( !non_zero && !filters.empty() ) { comma::say() << "warning: --filters specified, but --non-zero is not; --filters will have no effect" << std::endl; }
    while( std::cin.good() && !std::cin.eof() )
    {
        std::pair< boost::posix_time::ptime, cv::Mat > p = input_serialization.read< boost::posix_time::ptime >( std::cin );
        if( p.second.empty() ) { return 0; }
        std::pair< boost::posix_time::ptime, cv::Mat > filtered;
        if( filters.empty() ) { filtered = p; } else { p.second.copyTo( filtered.second ); }
        for( auto& filter: filters ) { filtered = filter( filtered ); }
        non_zero.size( filtered.second.rows * filtered.second.cols );
        if( non_zero.keep( filtered.second ) ) { output_serialization.write_to_stdout( p ); }
        std::cout.flush();
    }
    if( !input_serialization.last_error().empty() ) { comma::say() << input_serialization.last_error() << std::endl; }
    if( !output_serialization.last_error().empty() ) { comma::say() << output_serialization.last_error() << std::endl; }
    return 0;
}

} } } // namespace grep {namespace snark { namespace cv_calc { namespace grep {