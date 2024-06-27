// Copyright (c) 2023 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <string>
#include <boost/function.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace snark { namespace cv_mat { namespace filters {

namespace drawing {

struct shape
{
    cv::Scalar color;
    int thickness;
    int line_type;
    int shift;
    shape() : thickness( 1 ), line_type( 8 ), shift( 0 ) {}
    shape( const cv::Scalar& color, int thickness = 1, int line_type = 8, int shift = 0 ) : color( color ), thickness( thickness ), line_type( line_type ), shift( shift ) {}
};

struct circle : public shape
{
    cv::Point center;
    int radius;
    circle() {}
    circle( const cv::Point& center, int radius, const cv::Scalar& color, int thickness = 1, int line_type = 8, int shift = 0 ) : shape( color, thickness, line_type, shift ), center( center ), radius( radius ) {}
    void draw( cv::Mat m ) const;
};

struct line : public shape
{
    cv::Point begin;
    cv::Point end;
    line() {};
    line( const cv::Point& begin, const cv::Point& end, const cv::Scalar& color, int thickness = 1, int line_type = 8, int shift = 0 );
    void draw( cv::Mat m ) const;
};

struct rectangle : public shape
{
    cv::Point upper_left;
    cv::Point lower_right;
    rectangle() {};
    rectangle( const cv::Point& upper_left, const cv::Point& lower_right, const cv::Scalar& color, int thickness = 1, int line_type = 8, int shift = 0 );
    void draw( cv::Mat m ) const;
};

struct cross : public shape
{
    cv::Point centre;
    cross(): centre( 0, 0 ) {};
    cross( const cv::Point& centre, const cv::Scalar& color, int thickness = 1, int line_type = 8, int shift = 0 ) : shape( color, thickness, line_type, shift ), centre( centre ) {}
    void draw( cv::Mat m ) const;
};

} // namespace drawing {

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
                std::string label;
                std::pair< float, float > extents{0, 0};
                float step{1};
                unsigned int steps{0};
                std::pair< cv::Point, cv::Point > geometry{{0, 0}, {0, 0}};
                unsigned int size{0};
                cv::Scalar color;
                bool vertical{false};
                bool no_begin{false};
                bool no_end{false};
            };
            static std::pair< functor_t, bool > make( const std::string& options, char delimiter = ',' );
            static std::string usage( unsigned int indent = 0 );
            std::pair< H, cv::Mat > operator()( std::pair< H, cv::Mat > m );
        private:
            properties _properties;
            unsigned int _step{0};
            cv::Point _label_position;
            cv::Mat _label;
            cv::Rect _label_rectangle;
            cv::Mat _labels_image;
            cv::Rect _labels_rectangle;
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

    static typename std::pair< H, cv::Mat > circle( std::pair< H, cv::Mat > m, const drawing::circle& circle );

    static typename std::pair< H, cv::Mat > cross( std::pair< H, cv::Mat > m, const drawing::cross& cross );

    static typename std::pair< H, cv::Mat > line( std::pair< H, cv::Mat > m, const drawing::line& line );

    static typename std::pair< H, cv::Mat > rectangle( std::pair< H, cv::Mat > m, const drawing::rectangle& rectangle );
};

} } }  // namespace snark { namespace cv_mat { namespace filters {
