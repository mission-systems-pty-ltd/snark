// Copyright (c) 2011 The University of Sydney

#pragma once

#include <comma/base/exception.h>
#include "depth_traits.h"

namespace snark { namespace cv_mat {

double min_value( int depth )
{
    switch( depth )
    {
        case CV_8U: return depth_traits< CV_8U >::min_value(); 
        case CV_8S: return depth_traits< CV_8S >::min_value();
        case CV_16U: return depth_traits< CV_16U >::min_value(); 
        case CV_16S: return depth_traits< CV_16S >::min_value(); 
        case CV_32S: return depth_traits< CV_32S >::min_value(); 
        case CV_32F: return depth_traits< CV_32F >::min_value(); 
        case CV_64F: return depth_traits< CV_64F >::min_value(); 
        default: break;
    }
    COMMA_THROW( comma::exception, "expected depth; got: " << depth );
}

double max_value( int depth )
{
    switch( depth )
    {
        case CV_8U: return depth_traits< CV_8U >::max_value(); 
        case CV_8S: return depth_traits< CV_8S >::max_value();
        case CV_16U: return depth_traits< CV_16U >::max_value(); 
        case CV_16S: return depth_traits< CV_16S >::max_value(); 
        case CV_32S: return depth_traits< CV_32S >::max_value(); 
        case CV_32F: return depth_traits< CV_32F >::max_value(); 
        case CV_64F: return depth_traits< CV_64F >::max_value(); 
        default: break;
    }
    COMMA_THROW( comma::exception, "expected depth; got: " << depth );
}

} } // namespace snark { namespace cv_mat {
