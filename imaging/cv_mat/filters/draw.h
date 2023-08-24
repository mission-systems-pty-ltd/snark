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
    typedef boost::function< boost::posix_time::ptime( const H& ) > timestamp_functor_t;
    static std::pair< functor_t, bool > make( const std::string& options, timestamp_functor_t get_timestamp, char delimiter = ',' );
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
            std::vector< std::string > _labels;
    };

    class status
    {
        public:
            struct properties
            {
                std::string label;
                cv::Point origin{20, 20};
                cv::Scalar color{0, 0, 0};
                cv::Scalar bg_color{220, 220, 220, 255};
                float font_size{0.5};
                float alpha{0.5};
                float spin_up{1};
                bool system_time{false};
            };
            static std::pair< functor_t, bool > make( const std::string& options, timestamp_functor_t get_timestamp, char delimiter = ',' );
            static std::string usage( unsigned int indent = 0 );
            std::pair< H, cv::Mat > operator()( std::pair< H, cv::Mat > m );
        private:
            properties _properties;
            timestamp_functor_t _timestamp;
            double _average_interval{0};
            boost::posix_time::ptime _previous;
            comma::uint64 _count{0};
            cv::Size _text_size;
    };

    class time
    {
        public:
            static std::pair< functor_t, bool > make( const std::string& options, timestamp_functor_t get_timestamp, char delimiter = ',' );
            static std::string usage( unsigned int indent = 0 );
            std::pair< H, cv::Mat > operator()( std::pair< H, cv::Mat > m );
        private:
            cv::Point _origin{0, 0};
            cv::Scalar _color{0, 0, 0};
            float _font_size{0.5};
            timestamp_functor_t _timestamp;
    };
};

} } }  // namespace snark { namespace cv_mat { namespace filters {
