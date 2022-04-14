// Copyright (c) 2019 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <boost/function.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <comma/base/types.h>

namespace snark { namespace cv_mat { namespace filters {

template < typename H >
class map
{
    public:
        typedef typename std::pair< H, cv::Mat > value_type;
        typedef std::unordered_map< comma::int32, double > map_t;
        
        map( const std::string& map_filter_options, bool permissive );
        map( const map_t& map, bool permissive );
        value_type operator()( value_type m ); // todo: support multiple channels

    private:
        map_t map_;
        bool permissive_;

        template < typename T, int Size > static T get_channel_( const cv::Vec< T, Size >& pixel, int channel ) { return pixel[channel]; }
        template < typename T > static T get_channel_( const T& pixel, int channel ) { return pixel; }
        template < typename T, int Size > static void set_channel_( cv::Vec< T, Size >& pixel, int channel, T value ) { return pixel[channel] = value; }
        template < typename T > static void set_channel_( T& pixel, int channel, T value ) { pixel = value; }
        template < typename input_value_type > void apply_map_( const cv::Mat& input, cv::Mat& output );
};

} } } // namespace snark { namespace cv_mat { namespace filters {
