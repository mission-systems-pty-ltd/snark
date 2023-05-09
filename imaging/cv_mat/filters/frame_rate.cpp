// Copyright (c) 2023 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <iostream>
#include <sstream>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/lexical_cast.hpp>
#include <exiv2/exiv2.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <comma/base/exception.h>
#include <comma/string/string.h>
#include "../utils.h"
#include "frame_rate.h"

namespace snark { namespace cv_mat { namespace filters {

template < typename H >
std::pair< typename frame_rate< H >::functor_t, bool > frame_rate< H >::make( const std::string& options
                                                                            , typename frame_rate< H >::timestamp_functor_t get_timestamp
                                                                            , char delimiter )
{
    const auto& v = comma::split( options, delimiter, true );
    frame_rate< H > e;
    e._get_timestamp = get_timestamp;
    e._alpha = v.size() > 0 && !v[0].empty() ? boost::lexical_cast< double >( v[0] ) : 0.5;
    e._spin_up_size = v.size() > 1 && !v[1].empty() ? boost::lexical_cast< unsigned int >( v[1] ) : 1;
    e._use_timestamp = v.size() > 2 && v[2] == "use-timestamp";
    COMMA_ASSERT( e._alpha > 0 && e._alpha <= 1, "expected ema alpha between 0 and 1; got: " << e._alpha );
    return std::make_pair( boost::bind< std::pair< H, cv::Mat > >( e, _1 ), false );
    // todo? return std::make_pair( boost::bind< std::pair< H, cv::Mat > >( e, _1 ), true );
}

template < typename H >
std::string frame_rate< H >::usage( unsigned int indent )
{
    std::ostringstream oss;
    std::string i( indent, ' ' );
    oss << i << "frame_rate[=<alpha>],[<spin-up-size>],[use-timestamp]; print estimated frame on the image\n";
    oss << i << "    <alpha>: ema (exponential moving average) alpha between 0 and 1; default=0.5\n";
    oss << i << "    <spin-up-size>: ema spin-up; default=1\n";
    oss << i << "    <use-timestamp>: use image timestamp instead of system time\n";
    return oss.str();
}

template < typename H >
std::pair< H, cv::Mat > frame_rate< H >::operator()( std::pair< H, cv::Mat > m )
{
    auto t = _use_timestamp ? _get_timestamp( m.first ) : boost::posix_time::microsec_clock::universal_time();
    COMMA_ASSERT( !t.is_not_a_date_time(), "frame-rate: asked to use timestamp from image, but input image has no timestamp" );
    if( !_previous.is_not_a_date_time() )
    {
        auto d = t - _previous;
        double interval = d.total_seconds() + double( d.total_microseconds() ) / 1000000;
        _average_interval += ( interval - _average_interval ) * ( _count <= _spin_up_size ? 1. / _count : _alpha );
    }
    _previous = t;
    ++_count;
    std::ostringstream oss;
    oss.precision( 4 );
    oss << "fps: " << ( 1. / _average_interval ) << "Hz";
    //std::cerr << "==> average_interval: " << _average_interval << " " << oss.str() << std::endl;
    #if defined( CV_VERSION_EPOCH ) && CV_VERSION_EPOCH == 2 // pain
        cv::rectangle( m.second, cv::Point( 5, 5 ), cv::Point( 155, 25 ), cv::Scalar( 0, 0, 0 ), 1, CV_AA );
        cv::putText( m.second, oss.str().c_str(), cv::Point( 10, 20 ), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0xffff, 0xffff, 0xffff ), 1, CV_AA );
    #else
        cv::rectangle( m.second, cv::Point( 5, 5 ), cv::Point( 155, 25 ), cv::Scalar( 0, 0, 0 ), cv::FILLED, cv::LINE_AA );
        cv::putText( m.second, oss.str().c_str(), cv::Point( 10, 20 ), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0xffff, 0xffff, 0xffff ), 1, cv::LINE_AA );
    #endif
    return m;
}

template struct frame_rate< boost::posix_time::ptime >;
template struct frame_rate< std::vector< char > >;

} } }  // namespace snark { namespace cv_mat { namespace filters {