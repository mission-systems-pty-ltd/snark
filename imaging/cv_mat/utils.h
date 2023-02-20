// Copyright (c) 2011 The University of Sydney

#pragma once

#include <string>
#include <boost/optional.hpp>
#include <boost/unordered_map.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace snark { namespace cv_mat {

boost::unordered_map< std::string, int > fill_types();

cv::Scalar scalar_from_strings( const std::string* begin, unsigned int size );

unsigned int cvt_color_type_from_string( const std::string& t );

std::string type_as_string( int t );

std::string make_filename( const boost::posix_time::ptime& t, const std::string& extension, boost::optional< unsigned int > index = boost::none );

void check_image_type( const cv::Mat& m, const std::string& type );

std::vector< int > imwrite_params( const std::string& type, const int quality );

cv::ColormapTypes colormap_from_string( const std::string& name );

cv::Scalar color_from_string( const std::string& name );

void apply_color_range( const cv::Mat& in, cv::Mat& out, const std::pair< cv::Scalar, cv::Scalar >& color_range );

template < typename T >
void set_channel( unsigned char* channel, T value, int depth );

template < typename T >
T get_channel( unsigned char* channel, int depth );

} }  // namespace snark { namespace cv_mat {
