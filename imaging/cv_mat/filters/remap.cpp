// Copyright (c) 2019 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <fstream>
#include <opencv2/core/version.hpp> // pain
#if defined( CV_VERSION_EPOCH ) && CV_VERSION_EPOCH == 2
#include <opencv2/calib3d/calib3d.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif
#include <boost/date_time/posix_time/ptime.hpp>
#include <comma/base/exception.h>
#include <comma/string/string.h>
#include <comma/name_value/serialize.h>
#include "../../camera/pinhole.h"
#include "../../camera/traits.h"
#include "remap.h"

namespace snark { namespace cv_mat { namespace filters {

template < typename H >
remap< H >::remap( const std::string& filename, unsigned int width, unsigned int height, int interpolation )
    : interpolation_( interpolation )
    , x_( int( height ), int( width ), CV_32FC1 )
    , y_( int( height ), int( width ), CV_32FC1 )
{
    std::ifstream ifs( filename );
    if( !ifs.is_open() ) { COMMA_THROW( comma::exception, "failed to open '" << filename << "'" ); }
    unsigned int size = width * height * 4;
    ifs.read( reinterpret_cast< char* >( x_.ptr() ), size );
    if( ifs.gcount() != int( size ) ) { COMMA_THROW( comma::exception, "on reading x map: expected " << size << " bytes, got: " << ifs.gcount() ); }
    ifs.read( reinterpret_cast< char* >( y_.ptr() ), size );
    if( ifs.gcount() != int( size ) ) { COMMA_THROW( comma::exception, "on reading y map: expected " << size << " bytes, got: " << ifs.gcount() ); }
    ifs.peek(); // quick and dirty
    if( !ifs.eof() ) { COMMA_THROW( comma::exception, "expected " << ( size * 2 ) << " bytes in \"" << filename << "\", got more bytes; invalid map size" ); }
}

template < typename H >
typename remap< H >::value_type remap< H >::operator()( value_type m ) const
{
    std::pair< H, cv::Mat > n;
    n.first = m.first;
    cv::remap( m.second, n.second, x_, y_, interpolation_ );
    return n;
}

template < typename H >
std::string remap< H >::usage( unsigned int indent )
{
    std::string i( indent, ' ' );
    std::ostringstream oss;
    oss << i << "remap=<map-filename>,<width>,<height>[,<interpolation>]: remap, input image dimensions\n";
    oss << i << "    expected to match map dimentions; see cv::remap() for details\n";
    oss << i << "    <interpolation>: nearest, linear, cubic, area, lanczos4; default: linear\n";
    return oss.str();
}

template < typename H >
undistort< H >::undistort( const std::string& filename, bool do_remap, int interpolation )
    : distortion_coefficients_( 0, 0, 0, 0, 0 )
    , interpolation_( interpolation )
{
    if( do_remap ) { filename_ = filename; return; }
    try
    {
        const std::vector< std::string >& v = comma::split( filename, ':' );
        snark::camera::pinhole::config_t config;
        switch( v.size() )
        {
            case 1: comma::read( config, filename ); break;
            case 2: comma::read( config, v[0], v[1] ); break;
            default: COMMA_THROW( comma::exception, "undistort: expected <filename>[:<path>]; got: \"" << filename << "\"" );
        }
        if( config.distortion )
        {
            if( config.distortion->map_filename.empty() )
            {
                distortion_coefficients_ = config.distortion->as< cv::Vec< double, 5 > >();
                camera_matrix_ = config.camera_matrix();
            }
            else
            {
                filename_ = config.distortion->map_filename;
            }
        }
    }
    catch( ... )
    {
        filename_ = filename;
    }
}

template < typename H >
typename undistort< H >::value_type undistort< H >::operator()( typename undistort< H >::value_type m )
{
    if( filename_.empty() )
    {
        value_type n;
        n.first = m.first;
        if( camera_matrix_.empty() ) { n.second = m.second.clone(); }
        else { cv::undistort( m.second, n.second, camera_matrix_, distortion_coefficients_ ); }
        return n;
    }
    if( !remap_ ) { remap_.reset( remap< H >( filename_, m.second.cols, m.second.rows, interpolation_ ) ); }
    return remap_->operator()( m );
}

template class snark::cv_mat::filters::remap< boost::posix_time::ptime >;
template class snark::cv_mat::filters::remap< std::vector< char > >;
template class snark::cv_mat::filters::undistort< boost::posix_time::ptime >;
template class snark::cv_mat::filters::undistort< std::vector< char > >;

} } }  // namespace snark { namespace cv_mat { namespace impl {
