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

    class grid
    {
        public:
            static std::pair< functor_t, bool > make( const std::string& options, char delimiter = ',' );
            static std::string usage( unsigned int indent = 0 );
            std::pair< H, cv::Mat > operator()( std::pair< H, cv::Mat > m );
        private:
            cv::Point _origin{0, 0};
            cv::Point _step{0, 0};
            cv::Point _end{0, 0};
            cv::Size _size{0, 0};
            cv::Scalar _color{0, 0, 0};
            bool _ends_included{false};
    };

    class axis
    {
        public:
            struct properties
            {
                std::string title;
                std::pair< float, float > extents{0, 0};
                float step{1};
                std::pair< cv::Point, cv::Point > geometry{{0, 0}, {0, 0}};
                unsigned int size{0};
                cv::Scalar color;
                bool vertical{false};
            };
            static std::pair< functor_t, bool > make( const std::string& options, char delimiter = ',' );
            static std::string usage( unsigned int indent = 0 );
            std::pair< H, cv::Mat > operator()( std::pair< H, cv::Mat > m );
        private:
            properties _properties;
            unsigned int _step{0};
            cv::Point _text_position;
    };
};

} } }  // namespace snark { namespace cv_mat { namespace filters {
