// Copyright (c) 2011 The University of Sydney
// All rights reserved.

#include <iostream>
#include <string>
#include <vector>
#include <boost/date_time/posix_time/ptime.hpp>
#include "../utils.h"
#include "arithmetic.h"

namespace snark { namespace cv_mat { namespace filters {
    
template < typename H >
typename arithmetic< H >::operation arithmetic< H >::str_to_operation(const std::string& s)
{
    if( s == "add" ) { return operation::add; }
    else if( s == "absdiff" ) { return operation::absdiff; }
    else if( s == "divide" ) { return operation::divide; }
    else if( s == "maximum" ) { return operation::maximum; }
    else if( s == "minimum" ) { return operation::minimum; }
    else if( s == "multiply" ) { return operation::multiply; }
    else if( s == "overlay" ) { return operation::overlay; }
    else if( s == "subtract" ) { return operation::subtract; }
    else if( s == "underlay" ) { return operation::underlay; }
    else { COMMA_THROW(comma::exception, "expected arithmetic operation; got: \"" << s << "\"" ); }
}

template < typename H >
typename std::string arithmetic< H >::operation_to_str( arithmetic< H >::operation op )
{
    switch(op)
    {
        case operation::add:      return "add";
        case operation::absdiff:  return "absdiff";
        case operation::divide:   return "divide";
        case operation::maximum:  return "maximum";
        case operation::minimum:  return "minimum";
        case operation::multiply: return "multiply";
        case operation::overlay:  return "overlay";
        case operation::subtract: return "subtract";
        case operation::underlay:  return "underlay";
    }
    COMMA_THROW( comma::exception, "arithmetic: expected operation code; got: '" << int( op ) << "'" );
}

template < typename H > arithmetic< H >::arithmetic( operation op ) : operation_( op ) {}

template < typename H >
typename arithmetic< H >::value_type arithmetic< H >::operator()( value_type m, boost::function< value_type( value_type ) >& operand ) // have to pass mask by value, since filter functors may change on call
{
    const cv::Mat& rhs = operand( m ).second;
    if( rhs.empty() ) { return arithmetic< H >::value_type(); }
    if ( rhs.channels() != m.second.channels() ) { COMMA_THROW( comma::exception,  operation_to_str(operation_) << ": channel mis-match, input image channels: " << m.second.channels() << ", the operand channels: " << rhs.channels() ); }
    return apply_(m, rhs);
}

template < typename H >
typename arithmetic< H >::value_type arithmetic< H >::apply_(const value_type& m, const cv::Mat& operand )
{
    value_type n( m.first, cv::Mat(m.second.rows, m.second.cols, m.second.type()) );
    switch( operation_ )
    {
        case operation::add:      cv::add( m.second, operand, n.second, cv::noArray(), m.second.type() );    break;
        case operation::absdiff:  cv::absdiff( m.second, operand, n.second ); break;
        case operation::divide:   cv::divide( m.second, operand, n.second, 1.0, m.second.type() ); break;
        case operation::maximum:  cv::max( m.second, operand, n.second ); break;
        case operation::minimum:  cv::min( m.second, operand, n.second ); break;
        case operation::multiply: cv::multiply( m.second, operand, n.second, 1.0, m.second.type() ); break;
        case operation::overlay:  COMMA_THROW( comma::exception, "overlay: todo" ); break;
        case operation::subtract: cv::subtract( m.second, operand, n.second, cv::noArray(), m.second.type() ); break;
        case operation::underlay: COMMA_THROW( comma::exception, "underlay: todo" ); ; break;
    }
    return n;
}
    
} } }  // namespace snark { namespace cv_mat { namespace impl {

template class snark::cv_mat::filters::arithmetic< boost::posix_time::ptime >;
template class snark::cv_mat::filters::arithmetic< std::vector< char > >;
