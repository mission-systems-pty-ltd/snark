// Copyright (c) 2023 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <string>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/function.hpp>
#include <boost/optional.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace snark { namespace cv_mat { namespace filters {

template < typename H >
class view
{
    public:
        typedef boost::function< std::pair< H, cv::Mat >( std::pair< H, cv::Mat > ) > functor_t;

        typedef boost::function< boost::posix_time::ptime( const H& ) > timestamp_functor_t;

        view( const timestamp_functor_t& get_timestamp
            , const std::string& title
            , double delay
            , const std::string& suffix
            , const boost::optional< std::pair< int, int > >& window_position = boost::none
            , const boost::optional< std::pair< int, int > >& window_size = boost::none
            , int flags = 0
            , bool capture_on_exit = false
            , const std::string& capture_on_exit_filename = "" );
            
        ~view();
        
        static std::pair< functor_t, bool > make( const std::string& options
                                                , const timestamp_functor_t& get_timestamp );

        std::pair< H, cv::Mat > operator()( std::pair< H, cv::Mat > m );
        
        static std::string usage( unsigned int indent = 0 );        
    private:
        timestamp_functor_t _get_timestamp;
        std::string _name;
        std::string _title;
        int _delay;
        std::string _suffix;
        boost::optional< std::pair< int, int > > _window_position;
        boost::optional< std::pair< int, int > > _window_size;
        bool _window_created{false};
        int _flags{0};
        bool _capture_on_exit;
        std::string _capture_on_exit_filename;
        std::pair< boost::posix_time::ptime, cv::Mat > _last;
};

} } }  // namespace snark { namespace cv_mat { namespace impl {
