// Copyright (c) 2025 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <comma/base/types.h>
#include <opencv2/core/core.hpp>

namespace snark { namespace cv_mat { namespace filters {
    
template < typename H >
class resize
{
    public:
        typedef std::pair< H, cv::Mat > value_type;

        enum class by { shape, orientation, longest, shortest, width, height };

        resize( unsigned int width, unsigned int height, double width_factor, double height_factor, int interpolation = cv::INTER_LINEAR, resize::by how = by::shape );

        value_type operator()( value_type );
        
        static resize make( const std::string& options );
        
        static std::string usage( unsigned int indent = 0 );
        
    private:
        unsigned int _width{0};
        unsigned int _height{0};
        double _width_factor{1};
        double _height_factor{1};
        int _interpolation;
        resize::by _how{by::shape};
};

} } } // namespace snark { namespace cv_mat { namespace filters {
