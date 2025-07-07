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

#include <iostream>
#include <memory>
#include <sstream>
#include <vector>
#include <boost/date_time/posix_time/ptime.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <comma/base/exception.h>
#include <comma/string/split.h>
#include "colors.h"
#include "../utils.h" // quick and dirty

namespace snark { namespace cv_mat { namespace filters { namespace colors {

#if OPENCV_HAS_BALANCE_WHITE && SNARK_OPENCV_CONTRIB
    
template < typename H > balance_white< H >::balance_white(): wb_( cv::xphoto::createSimpleWB() ) {}

template < typename H > std::pair< H, cv::Mat > balance_white< H >::operator()( std::pair< H, cv::Mat > m )
{
    std::pair< H, cv::Mat > n;
    n.first = m.first;
    wb_->balanceWhite( m.second, n.second );
    return n;
}

#endif // OPENCV_HAS_BALANCE_WHITE && SNARK_OPENCV_CONTRIB

template < typename H > std::string balance_white< H >::usage( unsigned int indent_size )
{
    std::ostringstream oss;
    std::string indent( indent_size, ' ' );
    oss << indent << "balance-white: apply white balance to image" << std::endl;
    #ifdef BALANCE_WHITE_ERROR_MSG
    oss << indent << "               " << BALANCE_WHITE_ERROR_MSG << std::endl;
    #endif
    return oss.str();
}

static std::uint32_t pixel_to_uint( const cv::Vec3b& pixel )
{
    std::uint32_t i = 0;
    std::memcpy( &i, &pixel[0], 3 );
    return i;
}

template < typename H > map< H > map< H >::map::make( const std::string& params )
{
    const auto& v = comma::split( params, ',' );
    return map( colormap_from_string( v[0] ), v.size() > 1 && v[1] == "revert" );
}

template < typename H > map< H >::map( int type, bool revert )
    : _type( type )
{
    if( revert )
    {
        cv::Mat m( 1, 256, CV_8UC1 );
        for( unsigned int i = 0; i < 256; ++i ) { m.at< char >( 0, i ) = i; }
        cv::Mat n;
        cv::applyColorMap( m, n, _type );
        for( unsigned int i = 0; i < 256; ++i ) { _reverted[ pixel_to_uint( n.at< cv::Vec3b >( 0, i ) ) ] = i; }
    }
}

template < typename H > std::pair< H, cv::Mat > map< H >::operator()( std::pair< H, cv::Mat > m )
{
    std::pair< H, cv::Mat > n;
    n.first = m.first;
    if( _reverted.empty() ) { cv::applyColorMap( m.second, n.second, _type ); return n; }
    COMMA_ASSERT( m.second.type() == CV_8UC3, "expected rgb 8-byte input image of type CV_8UC3 (" << CV_8UC3 << "); got: " << m.second.type() );
    n.second = cv::Mat( m.second.rows, m.second.cols, CV_8UC1 );
    for( int row = 0; row < m.second.rows; ++row ) // todo? use tbb::parallel_for? use an opencv native function if such exists?
    {
        for( int col = 0; col < m.second.rows; ++col )
        {
            const auto& p = m.second.template at< cv::Vec3b >( row, col );
            auto i = _reverted.find( pixel_to_uint( p ) );
            COMMA_ASSERT( i != _reverted.end(), "pixel value at row: " << row << " col: " << col << " " << p[0] << "," << p[1] << "," << p[2] << " not found in the colour map" );
            n.second.template at< char >( row, col ) = i->second;
        }
    }
    return n;
}

template < typename H > std::string map< H >::usage( unsigned int indent_size )
{
    std::ostringstream oss;
    std::string indent( indent_size, ' ' );
    oss << indent << "color-map=<type>[,revert]: take image, apply colour map; see cv::applyColorMap for detail" << std::endl;
    oss << indent << "    <type>: autumn, bone, jet, winter, rainbow, ocean, summer, spring, cool, hsv, pink, hot" << std::endl;
    oss << indent << "        or numeric colormap code (names for colormaps in newer opencv versions: todo)" << std::endl;
    oss << indent << "        colormap type numeric values in opencv:" << std::endl;
    oss << indent << "            autumn: 0, bone: 1, jet: 2, winter: 3, rainbow: 4, ocean: 5, summer: 6, spring: 7, cool: 8" << std::endl;
    oss << indent << "            hsv: 9, pink: 10, hot: 11, parula: 12, magma: 13, inferno: 14, plasma: 15, viridis: 16" << std::endl;
    oss << indent << "            cividis: 17, twilight: 18, twilight_shifted: 19, turbo: 20, deepgreen: 21" << std::endl;
    oss << indent << "    revert: if present, expect rgb 8-bit image as input coloured as <type>" << std::endl;
    oss << indent << "            output grey-scale 8-bit image with values corresponding to colors" << std::endl;
    oss << indent << "            currently, expects exact color-mapped values, i.e. will exit with error" << std::endl;
    oss << indent << "            on input images with lossy compression (e.g. jpg) " << std::endl;
    return oss.str();
}

template class balance_white< boost::posix_time::ptime >;
template class balance_white< std::vector< char > >;
template class map< boost::posix_time::ptime >;
template class map< std::vector< char > >;

} } } }  // namespace snark { namespace cv_mat { namespace filters { namespace colors {
