// Copyright (c) 2020 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <fstream>
#include <sstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/lexical_cast.hpp>
#include <comma/base/exception.h>
#include <comma/string/split.h>
#include "../serialization.h"
#include "convolution.h"

namespace snark { namespace cv_mat { namespace filters {
    
template < typename H >
convolution< H >::convolution( cv::Mat kernel, int ddepth, const cv::Point& anchor, double delta, int border ): kernel_( kernel ), ddepth_( ddepth ), anchor_( anchor ), delta_( delta ), border_( border ) {}

template < typename H >
typename convolution< H >::value_type convolution< H >::operator()( typename convolution< H >::value_type m )
{
    value_type n( m.first, cv::Mat( m.second.rows, m.second.cols, m.second.type() ) );
    cv::filter2D( m.second, n.second, ddepth_ , kernel_, anchor_, delta_, cv::BORDER_DEFAULT );
    return n;
}

template < typename H >
convolution< H > convolution< H >::make( const std::vector< std::string >& options )
{
    const auto& v = comma::split( options[1], ',' );
    if( v.size() < 3 ) { COMMA_THROW( comma::exception, "convolution: please specify <kernel>,<rows>,<cols>" ); }
    serialization::options o;
    o.rows = boost::lexical_cast< int >( v[1] );
    o.cols = boost::lexical_cast< int >( v[2] );
    o.type = "f";
    o.no_header = true;
    serialization s( o );
    std::ifstream ifs( v[0] );
    if( !ifs.is_open() ) { COMMA_THROW( comma::exception, "convolution: failed to open kernel file '" << v[0] << "'" ); }
    cv::Mat kernel = serialization( o ).read< boost::posix_time::ptime >( ifs ).second;
    int depth = -1;
    double delta = 0;
    cv::Point anchor( -1, -1 );
    int border = cv::BORDER_DEFAULT;
    for( unsigned int i = 3; i < v.size(); ++i )
    {
        if( v[i].substr( 0, 6 ) == "delta:" ) { delta = boost::lexical_cast< double >( v[i].substr( 6 ) ); }
        else if( v[i].substr( 0, 7 ) == "anchor:" ) { COMMA_THROW( comma::exception, "convolution: anchor: todo" ); }
        else if( v[i].substr( 0, 6 ) == "depth:" ) { COMMA_THROW( comma::exception, "convolution: depth: todo" ); }
        else if( v[i].substr( 0, 7 ) == "border:" ) { COMMA_THROW( comma::exception, "convolution: border: todo" ); }
        else { COMMA_THROW( comma::exception, "convolution: expected option, got: '" << v[i] << "'" ); }
    }
    return convolution< H >( kernel, depth, anchor, delta, border );
}

template < typename H >
std::string convolution< H >::usage( unsigned int indent )
{
    std::ostringstream oss;
    std::string i( indent, ' ' );
    oss << i << "convolution=<kernel>,<rows>,<cols>[,<options>]: convolution on a given kernel, see cv::filter2d for details\n";
    oss << i << "    <kernel>: filename of binary file containing <rows>x<cols> 4-byte floats\n";
    oss << i << "    <rows>: number of rows in kernel\n";
    oss << i << "    <cols>: number of columns in kernel\n";
    oss << i << "    <options> (see cv::filter2d for details)\n";
    oss << i << "        <delta>:<value>: offset to add to convolved values, e.g. delta:0.3\n";
    oss << i << "        <depth>:<value>: todo\n";
    oss << i << "        <anchor>:<value>: todo\n";
    return oss.str();
}

template class snark::cv_mat::filters::convolution< boost::posix_time::ptime >;
template class snark::cv_mat::filters::convolution< std::vector< char > >;

} } } // namespace snark { namespace cv_mat { namespace filters {
