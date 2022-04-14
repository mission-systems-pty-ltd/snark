// Copyright (c) 2019 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <string>
#include <vector>
#include <boost/function.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "map.h"

namespace snark { namespace cv_mat { namespace filters {

template < typename H >
class gamma
{
    typedef boost::function< std::pair< H, cv::Mat >( std::pair< H, cv::Mat > ) > functor_t;

    public:
        gamma( double value );
        std::pair< H, cv::Mat > operator()( std::pair< H, cv::Mat > m );
        static std::pair< functor_t, bool > make( const std::string& options );
        static std::string usage( unsigned int indent = 0 );

    private:
        filters::map< H > _map_8;
        filters::map< H > _map_16;
};

} } } /// namespace snark { namespace cv_mat { namespace filters {
