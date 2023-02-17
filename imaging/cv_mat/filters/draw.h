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

    class bar
    {
        public:
            std::pair< H, cv::Mat > operator()( std::pair< H, cv::Mat > m );

        protected:
            cv::Mat _bar;
            cv::Rect _rectangle;
    };

    struct colorbar: public bar
    {
        static std::pair< functor_t, bool > make( const std::string& options, char delimiter = ',' );
        static std::string usage( unsigned int indent = 0 );
    };

    // struct scale: public bar
    // {
    //     static std::pair< functor_t, bool > make( const std::string& options, char delimiter = ',' );
    //     static std::string usage( unsigned int indent = 0 );
    // };
};

} } }  // namespace snark { namespace cv_mat { namespace filters {
