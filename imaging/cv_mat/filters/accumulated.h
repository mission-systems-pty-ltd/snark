// Copyright (c) 2011 The University of Sydney

#pragma once

#include <deque>
#include <opencv2/core/core.hpp>
#include <comma/base/types.h>

namespace snark{ namespace cv_mat { namespace accumulated {
    
template < typename H >
class average
{
    public:
        typedef std::pair< H, cv::Mat > value_type;
        
        value_type operator()( const value_type& n );
    private:
        comma::uint64 count_;   // How many input images so far
        cv::Mat result_;        // This is a float depth image
};

template < typename H >
class ema
{
    public:
        typedef std::pair< H, cv::Mat > value_type;
        
        // https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
        ema( float alpha, comma::uint32 spin_up_size = 1 );
        value_type operator()( const value_type& n );
    private:
        comma::uint64 count_;   // How many input images so far
        cv::Mat result_;        // This is a float depth image
        float alpha_;
        comma::uint32 spin_up_;
};

template < typename H >
class moving_average
{
    public:
        typedef std::pair< H, cv::Mat > value_type;
        
        moving_average( comma::uint32 size );
        
        value_type operator()( const value_type& n );
        
    private:
        comma::uint64 count_;
        cv::Mat result_;
        comma::uint32 size_;  // sliding window size
        std::deque< cv::Mat > window_;
};

template < typename H >
class min
{
    public:
        typedef std::pair< H, cv::Mat > value_type;
        value_type operator()( const value_type& n );
    private:
        value_type value_;
};

template < typename H >
class max
{
    public:
        typedef std::pair< H, cv::Mat > value_type;
        value_type operator()( const value_type& n );
    private:
        value_type value_;
};

} } }  // namespace snark { namespace cv_mat { namespace accumulated {
