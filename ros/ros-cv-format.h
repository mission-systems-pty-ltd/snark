// Copyright (c) 2026 Mission Systems Pty Ltd

#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <unordered_map>

namespace snark { namespace ros {

static unsigned ros_to_cv_format( std::string const& ros_encoding )
{
    static const std::unordered_map< std::string, unsigned > map = {
        { "rgb8", CV_8UC3 },
        { "rgba8", CV_8UC4 },
        { "rgb16", CV_16UC3 },
        { "rgba16", CV_16UC4 },
        { "bgr8", CV_8UC3 },
        { "bgra8", CV_8UC4 },
        { "bgr16", CV_16UC3 },
        { "bgra16", CV_16UC4 },
        { "mono8", CV_8UC1 },
        { "mono16", CV_16UC1 },
        { "8UC1", CV_8UC1 },
        { "8UC2", CV_8UC2 },
        { "8UC3", CV_8UC3 },
        { "8UC4", CV_8UC4 },
        { "8SC1", CV_8SC1 },
        { "8SC2", CV_8SC2 },
        { "8SC3", CV_8SC3 },
        { "8SC4", CV_8SC4 },
        { "16UC1", CV_16UC1 },
        { "16UC2", CV_16UC2 },
        { "16UC3", CV_16UC3 },
        { "16UC4", CV_16UC4 },
        { "16SC1", CV_16SC1 },
        { "16SC2", CV_16SC2 },
        { "16SC3", CV_16SC3 },
        { "16SC4", CV_16SC4 },
        { "32SC1", CV_32SC1 },
        { "32SC2", CV_32SC2 },
        { "32SC3", CV_32SC3 },
        { "32SC4", CV_32SC4 },
        { "32FC1", CV_32FC1 },
        { "32FC2", CV_32FC2 },
        { "32FC3", CV_32FC3 },
        { "32FC4", CV_32FC4 },
        { "64FC1", CV_64FC1 },
        { "64FC2", CV_64FC2 },
        { "64FC3", CV_64FC3 },
        { "64FC4", CV_64FC4 },
        { "bayer_rggb8", CV_8UC4 },
        { "bayer_bggr8", CV_8UC4 },
        { "bayer_gbrg8", CV_8UC4 },
        { "bayer_grbg8", CV_8UC4 },
        { "bayer_rggb16", CV_16UC4 },
        { "bayer_bggr16", CV_16UC4 },
        { "bayer_gbrg16", CV_16UC4 },
        { "bayer_grbg16", CV_16UC4 },
    };
    return map.at( ros_encoding );
}

static std::string cv_to_ros_format( unsigned const cv_encoding )
{
    static const std::unordered_map< unsigned, std::string > map = {
        { CV_8UC3, "rgb8" },
        { CV_8UC4, "rgba8" },
        { CV_16UC3, "rgb16" },
        { CV_16UC4, "rgba16" },
        { CV_8UC3, "bgr8" },
        { CV_8UC4, "bgra8" },
        { CV_16UC3, "bgr16" },
        { CV_16UC4, "bgra16" },
        { CV_8UC1, "mono8" },
        { CV_16UC1, "mono16" },
        { CV_8UC1, "8UC1" },
        { CV_8UC2, "8UC2" },
        { CV_8UC3, "8UC3" },
        { CV_8UC4, "8UC4" },
        { CV_8SC1, "8SC1" },
        { CV_8SC2, "8SC2" },
        { CV_8SC3, "8SC3" },
        { CV_8SC4, "8SC4" },
        { CV_16UC1, "16UC1" },
        { CV_16UC2, "16UC2" },
        { CV_16UC3, "16UC3" },
        { CV_16UC4, "16UC4" },
        { CV_16SC1, "16SC1" },
        { CV_16SC2, "16SC2" },
        { CV_16SC3, "16SC3" },
        { CV_16SC4, "16SC4" },
        { CV_32SC1, "32SC1" },
        { CV_32SC2, "32SC2" },
        { CV_32SC3, "32SC3" },
        { CV_32SC4, "32SC4" },
        { CV_32FC1, "32FC1" },
        { CV_32FC2, "32FC2" },
        { CV_32FC3, "32FC3" },
        { CV_32FC4, "32FC4" },
        { CV_64FC1, "64FC1" },
        { CV_64FC2, "64FC2" },
        { CV_64FC3, "64FC3" },
        { CV_64FC4, "64FC4" },
        { CV_8UC4, "bayer_rggb8" },
        { CV_8UC4, "bayer_bggr8" },
        { CV_8UC4, "bayer_gbrg8" },
        { CV_8UC4, "bayer_grbg8" },
        { CV_16UC4, "bayer_rggb16" },
        { CV_16UC4, "bayer_bggr16" },
        { CV_16UC4, "bayer_gbrg16" },
        { CV_16UC4, "bayer_grbg16" },
    };
    return map.at( cv_encoding );
}

} } // namespace snark { namespace ros {
