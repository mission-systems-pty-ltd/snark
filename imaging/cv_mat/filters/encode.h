// Copyright (c) 2023 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <string>
#include <boost/function.hpp>
#include <boost/optional.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace snark { namespace cv_mat { namespace filters {

template < typename H >
class encode
{
    public:
        typedef boost::function< boost::posix_time::ptime( const H& ) > timestamp_functor_t;
        typedef boost::function< std::pair< H, cv::Mat >( std::pair< H, cv::Mat > ) > functor_t;
        static std::pair< functor_t, bool > make( const std::string& options
                                                , const std::string& next_filter
                                                , timestamp_functor_t get_timestamp
                                                , char delimiter = ',' );
        std::pair< H, cv::Mat > operator()( std::pair< H, cv::Mat > m );
        static std::string usage( unsigned int indent = 0 );
    private:
        timestamp_functor_t _get_timestamp;
        std::string _type;
        std::string _format;
        boost::optional< int > _quality;
        bool _embed_timestamp{false};
};

} } }  // namespace snark { namespace cv_mat { namespace filters {
