// Copyright (c) 2025 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <sstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/lexical_cast.hpp>
#include <comma/base/exception.h>
#include <comma/string/split.h>
#include "../serialization.h"
#include "../utils.h"
#include "rotate.h"

namespace snark { namespace cv_mat { namespace filters {
    
template < typename H >
rotate< H >::rotate( const cv::Mat& rotation, int interpolation ): _rotation( rotation ), _interpolation( interpolation ) {}

template < typename H >
rotate< H >::rotate( double angle, const std::pair< double, double >& centre, int interpolation ): _angle( angle * 180 / M_PI ), _centre( centre ), _interpolation( interpolation ) {}

template < typename H >
typename rotate< H >::value_type rotate< H >::operator()( typename rotate< H >::value_type m )
{
    typename rotate< H >::value_type n{ m.first, cv::Mat() };
    cv::Point2f c( m.second.cols * _centre.first, m.second.rows * _centre.second );
    std::cerr << "==> a: _centre: " << _centre.first << "," << _centre.second << " c: " << c.x << "," << c.y << " _rotation.empty(): " << _rotation.empty() << std::endl;
    cv::warpAffine( m.second, n.second, _rotation.empty() ? cv::getRotationMatrix2D( cv::Point2f( m.second.cols * _centre.first, m.second.rows * _centre.second ), _angle, 1 ) : _rotation, cv::Size( m.second.cols, m.second.rows ) );
    return n;
}

template < typename H >
rotate< H > rotate< H >::make( const std::string& options )
{
    const std::vector< std::string >& r = comma::split( options, ',' );
    int interpolation = cv::INTER_LINEAR;
    cv::Mat rotation;
    double angle{0};
    std::pair< double, double > centre{0.5, 0.5};
    bool relative{false};
    unsigned int size = 0;
    for( const auto& e: r )
    {
        if( e[0] < 'a' || e[0] > 'z' ) { ++size; }
        else if( e == "relative" ) { relative = true; }
        else { interpolation = cv_mat::interpolation( e ); }
    }
    switch( size )
    {
        case 1:
            angle = boost::lexical_cast< double >( r[0] );
            relative = true;
            break;
        case 3:
            angle = boost::lexical_cast< double >( r[0] );
            centre = std::make_pair( boost::lexical_cast< double >( r[1] ), boost::lexical_cast< double >( r[2] ) );
            break;
        default:
            COMMA_THROW( comma::exception, "expected rotate=<options> with 1 or 3 numeric parameters; got: '" << options << "' with " << size << " numeric parameters" );
    }
    return relative ? rotate( angle, centre, interpolation ) : rotate( cv::getRotationMatrix2D( cv::Point2f( centre.first, centre.second ), angle * 180 / M_PI, 1 ), interpolation );
}

template < typename H >
std::string rotate< H >::usage( unsigned int indent )
{
    std::ostringstream oss;
    std::string i( indent, ' ' );
    oss << i << "    rotate=<options>: rotate image\n";
    oss << i << "        <options>\n";
    oss << i << "            <angle>,<interpolation>\n";
    oss << i << "            <angle>,<centre/x>,<centre/y>,<interpolation>[,relative]\n";
    oss << i << "            where\n";
    oss << i << "                <angle>              : rotation angle in radians\n";
    oss << i << "                <centre/x>,<centre/y>: rotation centre\n";
    oss << i << "                relative             : rotation centre as ratio of image size\n";
    oss << i << "                                       otherwise rotation centre in pixels\n";
    oss << i << "        <interpolation>";
    oss << i << "            nearest ("  << cv::INTER_NEAREST  << ")\n";
    oss << i << "            linear ("   << cv::INTER_LINEAR   << ") - default\n";
    oss << i << "            cubic ("    << cv::INTER_CUBIC    << ")\n";
    oss << i << "            area ("     << cv::INTER_AREA     << ")\n";
    oss << i << "            lanczos4 (" << cv::INTER_LANCZOS4 << ")\n";
    return oss.str();
}

template class snark::cv_mat::filters::rotate< boost::posix_time::ptime >;
template class snark::cv_mat::filters::rotate< std::vector< char > >;

} } } // namespace snark { namespace cv_mat { namespace filters {
