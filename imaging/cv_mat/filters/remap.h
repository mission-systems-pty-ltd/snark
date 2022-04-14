// Copyright (c) 2019 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <string>

#include <boost/optional.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace snark { namespace cv_mat { namespace filters {

template < typename H >
class remap
{
    public:
        typedef std::pair< H, cv::Mat > value_type;

        remap( const std::string& map, unsigned int width, unsigned int height, int interpolation );

        value_type operator()( value_type ) const;

    private:
        int interpolation_;
        cv::Mat x_;
        cv::Mat y_;
};

template < typename H >
class undistort
{
    public:
        typedef std::pair< H, cv::Mat > value_type;

        undistort( const std::string& filename, bool do_remap = false, int interpolation = cv::INTER_LINEAR );

        value_type operator()( value_type m );

    private:
        cv::Mat camera_matrix_;
        cv::Vec< double, 5 > distortion_coefficients_;
        int interpolation_;
        std::string filename_;
        boost::optional< remap< H > > remap_;
};

} } }  // namespace snark { namespace cv_mat { namespace impl {
