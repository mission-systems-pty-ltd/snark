// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <fstream>
#include <queue>
#include <sstream>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/unordered_map.hpp>
#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <comma/csv/ascii.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/csv/options.h>
#include <comma/string/string.h>
#include <comma/name_value/parser.h>
#include <Eigen/Core>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include "../../timing/timestamped.h"
#include "../../timing/traits.h"
#include "filters.h"
#include "../cv_mat/serialization.h"
#include "../cv_mat/traits.h"
#include "../vegetation/filters.h"

namespace snark { namespace imaging { namespace vegetation {

template < typename T > static void set_ndvi_pixel_( cv::Mat& red, const cv::Mat& nir )
{
    T* r = &red.at< T >( 0, 0 );
    const T* n = &nir.at< T >( 0, 0 );
    const T* end = r + red.cols * red.rows;
    for( ; r != end; ++r, ++n ) // slightly faster perhaps?
    {
        T s = *r + *n;
        *r = s == 0 ? 0: ( ( *n - *r ) / s );
    }
//     for( int i = 0; i < red.cols; ++i ) // watch performance
//     {
//         for( int j = 0; j < red.rows; ++j )
//         {
//             T& r = red.at< T >( i, j );
//             const T& n = nir.at< T >( i, j );
//             T s = r + n;
//             r = s == 0 ? 0: ( ( n - r ) / s );
//         }
//     }
}
    
template < typename H >
static typename impl::filters< H >::value_type ndvi_impl_( typename impl::filters< H >::value_type m ) // too quick, too dirty?
{
    typedef typename  impl::filters< H >::value_type value_type;
    if( m.second.channels() != 4 ) { std::cerr << "cv vegetation filters: expected 4 channels, got " << m.second.channels() << std::endl; return value_type(); }
    switch( m.second.type() )
    {
        case CV_32FC4: break; //type = CV_32FC3; break;
        case CV_8SC4: //type = CV_8SC3; break;
        case CV_8UC4: //type = CV_8UC3; break;
        default: std::cerr << "cv vegetation filters: expected type CV_32FC4; got: " << m.second.type() << std::endl; return value_type();
    }
    cv::Mat split( m.second.rows * m.second.channels(), m.second.cols, cv_mat::single_channel_type( m.second.type() ) ); // todo: check number of channels!
    std::vector< cv::Mat > channels;
    channels.reserve( m.second.channels() );    
    for( unsigned int i = 0; i < static_cast< unsigned int >( m.second.channels() ); ++i )
    {
        channels.push_back( cv::Mat( split, cv::Rect( 0, i * m.second.rows, m.second.cols, m.second.rows ) ) );
    }
    cv::split( m.second, channels ); // watch performance, do we really need to split?
    switch( channels[0].type() )
    {
        case CV_32FC1: set_ndvi_pixel_< float >( channels[2], channels[3] ); break;
        case CV_8SC1: //set_ndvi_pixel_< char >( channels[2], channels[3] ); break;
        case CV_8UC1: //set_ndvi_pixel_< char >( channels[2], channels[3] ); break;
        default: break; // never here
    }
    return value_type( m.first, channels[2] );
}

template < typename H >
static typename impl::filters< H >::value_type exponential_combination_impl_( const typename impl::filters< H >::value_type m, const std::vector< double >& powers)
{
    typedef typename  impl::filters< H >::value_type value_type;
    if( m.second.channels() != static_cast< int >( powers.size() ) ) { COMMA_THROW( comma::exception, "exponential-combination: the number of powers does not match the number of channels; channels = " << m.second.channels() << ", powers = " << powers.size() ); }
    int single_type=cv_mat::single_channel_type(m.second.type());
    if( single_type != CV_32FC1 && single_type != CV_64FC1) { COMMA_THROW( comma::exception, "expected image type CV_32FC1 or CV_64FC1; got: " << cv_mat::type_as_string( m.second.type() ) << "single type: " <<cv_mat::type_as_string(single_type) ); }
    unsigned int chs=m.second.channels();
    cv::Mat result( m.second.rows,m.second.cols, single_type== CV_32FC1? CV_32FC1:CV_64FC1 );
    //split
    std::vector<cv::Mat> planes;
    planes.reserve(chs);
    for(unsigned int i=0;i<chs;i++) { planes.push_back(cv::Mat(1,1,single_type)); }
    cv::split(m.second,planes);
    //calc exponent
    for(unsigned int i=0;i<chs;i++) { cv::pow(planes[i],powers[i],planes[i]); }
    //combine
    result.setTo(cv::Scalar(1,1,1,1));
    for(unsigned int i=0;i<chs;i++) { cv::multiply(result,planes[i],result); }
    return value_type(m.first, result);
}

template < typename H >
boost::function< typename cv_mat::impl::filters< H >::value_type( typename cv_mat::impl::filters< H >::value_type ) > impl::filters< H >::make_functor( const std::vector< std::string >& e )
{
    if( e[0] == "ndvi" ) { return boost::bind< value_type >( ndvi_impl_< H >, _1 ); }
    if( e[0]=="exponential-combination" )
    {
        const std::vector< std::string >& s = comma::split( e[1], ',' );
        if( s[0].empty() ) { COMMA_THROW( comma::exception, "exponential-combination: expected powers got: \"" << e[1] << "\"" ); }
        std::vector< double > power( s.size() );
        for( unsigned int j = 0; j < s.size(); ++j ) { power[j] = boost::lexical_cast< double >( s[j] ); }
        return boost::bind< value_type >( exponential_combination_impl_< H >, _1, power );
    }
    return NULL;
}

template < typename H >
boost::optional< typename impl::filters< H >::filter > impl::filters< H >::make( const std::string& what )
{
    boost::function< value_type( value_type ) > functor = make_functor( what );
    if( functor ) { return filter( functor ); }
    return boost::none;
}

static std::string usage_impl_()
{
    std::ostringstream oss;
    oss << "cv::Mat vegetation-specific filters:\n";
    oss << "    ndvi:\n";
    oss << "        calculate normalized difference vegetation index with formula (R<nir> - R<red>) / (R<nir> + R<red>);\n";
    oss << "        expect input to be 4-channel, 32-bit float\n";
    oss << "    exponential-combination=<e1>,<e2>,<e3>,...:\n";
    oss << "        output single channel float/double: multiplication of each channel powered to the exponent;\n";
    oss << "        expect input to be 32-bit float or 64-bit double\n";
    return oss.str();
}

template < typename H >
const std::string& impl::filters< H >::usage()
{
    static const std::string s = usage_impl_();
    return s;
}

template class impl::filters< boost::posix_time::ptime >;
template class impl::filters< cv_mat::header_type >;

} } } // namespace snark { namespace imaging { namespace vegetation {
