// Copyright (c) 2011 The University of Sydney
// All rights reserved.

#pragma once

#include <string>
#include <boost/function.hpp>
#include <comma/base/exception.h>
#include <opencv2/core/core.hpp>

namespace snark{ namespace cv_mat { namespace filters {

template < typename H >
class arithmetic
{
public:
    typedef std::pair< H, cv::Mat > value_type;

    enum class operation { add, absdiff, divide, maximum, minimum, multiply, overlay, subtract, underlay };

    static operation str_to_operation( const std::string& s ); 

    static std::string operation_to_str( operation op );
    
    arithmetic( operation op );

    value_type operator()( value_type n, boost::function< value_type( value_type ) >& operand );
    
private:
    operation operation_;
    value_type apply_( const value_type& m, const cv::Mat& operand );
};


} } }  // namespace snark { namespace cv_mat { namespace impl {
