// Copyright (c) 2023 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <string>
#include <boost/function.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <comma/base/types.h>

namespace snark { namespace cv_mat { namespace filters {

template < typename H >
class frame_rate
{
    public:
        typedef boost::function< boost::posix_time::ptime( const H& ) > timestamp_functor_t;
        typedef boost::function< std::pair< H, cv::Mat >( std::pair< H, cv::Mat > ) > functor_t;
        static std::pair< functor_t, bool > make( const std::string& options, timestamp_functor_t get_timestamp, char delimiter = ',' );
        std::pair< H, cv::Mat > operator()( std::pair< H, cv::Mat > m );
        static std::string usage( unsigned int indent = 0 );
    private:
        timestamp_functor_t _get_timestamp;
        bool _use_timestamp;
        double _average_interval{0};
        double _alpha{0.5}; // arbitrary default
        unsigned int _spin_up_size{1};
        boost::posix_time::ptime _previous;
        comma::uint64 _count{0};
};

} } }  // namespace snark { namespace cv_mat { namespace filters {
