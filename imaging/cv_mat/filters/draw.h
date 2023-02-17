// Copyright (c) 2023 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <string>
#include <boost/function.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace snark { namespace cv_mat { namespace filters {

template < typename H >
struct draw
{
    typedef boost::function< std::pair< H, cv::Mat >( std::pair< H, cv::Mat > ) > functor_t;

    static std::pair< functor_t, bool > make( const std::string& options, char delimiter = ',' );
    
    static std::string usage( unsigned int indent = 0 );

    class colorbar
    {
        public:
            static std::pair< functor_t, bool > make( const std::string& options, char delimiter = ',' );

            std::pair< H, cv::Mat > operator()( std::pair< H, cv::Mat > m );
            
            static std::string usage( unsigned int indent = 0 );

        private:
            cv::Mat _colorbar;
            cv::Rect _rectangle;
    };
};

} } }  // namespace snark { namespace cv_mat { namespace filters {
