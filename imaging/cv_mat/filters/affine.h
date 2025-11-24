// Copyright (c) 2025 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <comma/base/types.h>
#include <opencv2/core/core.hpp>

namespace snark { namespace cv_mat { namespace filters { namespace affine {
    
template < typename H >
class rotate
{
    public:
        typedef std::pair< H, cv::Mat > value_type;

        rotate( const cv::Mat& rotation, int interpolation = cv::INTER_LINEAR );

        rotate( double angle, const std::pair< double, double >& centre = std::make_pair( 0.5, 0.5 ), int interpolation = cv::INTER_LINEAR );

        value_type operator()( value_type );
        
        static rotate make( const std::string& options );
        
        static std::string usage( unsigned int indent = 0 );
        
    private:
        cv::Mat _rotation;
        double _angle{0};
        std::pair< double, double > _centre{0.5, 0.5};
        int _interpolation{cv::INTER_CUBIC};
};

} } } } // namespace snark { namespace cv_mat { namespace filters { namespace affine {
