// Copyright (c) 2020 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <sstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/lexical_cast.hpp>
#include <comma/base/exception.h>
#include <comma/string/split.h>
#include "../serialization.h"
#include "../utils.h"
#include "resize.h"

namespace snark { namespace cv_mat { namespace filters {
    
template < typename H >
resize< H >::resize( unsigned int width, unsigned int height, double width_factor, double height_factor, int interpolation ): _width( width ), _height( height ), _width_factor( width_factor ), _height_factor( height_factor ), _interpolation( interpolation ) {}

template < typename H >
typename resize< H >::value_type resize< H >::operator()( typename resize< H >::value_type m )
{
    typename resize< H >::value_type n;
    n.first = m.first;
    cv::resize( m.second, n.second, cv::Size( _width ? _width : m.second.cols * _width_factor, _height ? _height : m.second.rows * _height_factor ), 0, 0, _interpolation );
    return n;
}

template < typename H >
resize< H > resize< H >::make( const std::string& options )
{
    unsigned int width = 0;
    unsigned int height = 0;
    double width_factor = 0;
    double height_factor = 0;
    const std::vector< std::string >& r = comma::split( options, ',' );
    int interpolation = cv::INTER_LINEAR;
    unsigned int size = r.size();
    if( r.size() > 1 && r.back()[0] >= 'a' && r.back()[0] <= 'z' )
    {
        interpolation = cv_mat::interpolation( r.back() );
        --size;
    }
    switch( size )
    {
        case 1:
            width_factor = height_factor = boost::lexical_cast< double >( r[0] );
            break;
        case 2:
        case 3:
            try { width = boost::lexical_cast< unsigned int >( r[0] ); }
            catch ( ... ) { width_factor = boost::lexical_cast< double >( r[0] ); }
            try { height = boost::lexical_cast< unsigned int >( r[1] ); }
            catch ( ... ) { height_factor = boost::lexical_cast< double >( r[1] ); }
            break;
        default:
            COMMA_THROW( comma::exception, "expected resize=<width>,<height>,... got: \"" << options << "\"" );
    }
    return resize< H >( width, height, width_factor, height_factor, interpolation);
}

template < typename H >
std::string resize< H >::usage( unsigned int indent )
{
    std::ostringstream oss;
    std::string i( indent, ' ' );
    oss << i << "    resize=<factor>[,<interpolation>]; resize=<width>,<height>[,<interpolation>]\n";
    oss << i << "        <interpolation>";
    oss << i << "            nearest (" << cv::INTER_NEAREST << ")\n";
    oss << i << "            linear (" << cv::INTER_LINEAR << ") - default\n";
    oss << i << "            cubic (" << cv::INTER_CUBIC << ")\n";
    oss << i << "            area (" << cv::INTER_AREA << ")\n";
    oss << i << "            lanczos4 (" << cv::INTER_LANCZOS4 << ")\n";
    oss << i << "        examples\n";
    oss << i << "            resize=0.5,1024 : 50% of width; heigth 1024 pixels\n";
    oss << i << "            resize=512,1024,cubic : resize to 512x1024 pixels, cubic interpolation\n";
    oss << i << "            resize=0.2,0.4 : resize to 20% of width and 40% of height\n";
    oss << i << "            resize=0.5 : resize proportionally to 50%\n";
    oss << i << "            resize=0.5,1024 : 50% of width; heigth 1024 pixels\n";
    oss << i << "        note: if no decimal dot '.', size is in pixels; if decimal dot present, size as a fraction\n";
    oss << i << "              i.e. 5 means 5 pixels; 5.0 means 5 times\n";
    return oss.str();
}

template class snark::cv_mat::filters::resize< boost::posix_time::ptime >;
template class snark::cv_mat::filters::resize< std::vector< char > >;

} } } // namespace snark { namespace cv_mat { namespace filters {
