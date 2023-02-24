// Copyright (c) 2023 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <iostream>
#include <sstream>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/lexical_cast.hpp>
#include <exiv2/exiv2.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <comma/base/exception.h>
#include <comma/csv/impl/epoch.h>
#include <comma/string/string.h>
#include "../utils.h"
#include "encode.h"

namespace snark { namespace cv_mat { namespace filters {

// todo? super-quick and dirty; move to util?
template < typename H > struct empty;
template <> struct empty< boost::posix_time::ptime > { static bool is_empty( const boost::posix_time::ptime& t ) { return t.is_not_a_date_time(); } };
template <> struct empty< std::vector< char > > { static bool is_empty( const std::vector< char >& v ) { return v.empty(); } };
template < typename H, typename T >
static bool is_empty( std::pair< H, cv::Mat > m, const T& get_timestamp ) { return ( m.second.empty() && ( empty< H >::is_empty(m.first) || get_timestamp(m.first) == boost::posix_time::not_a_date_time ) ); }

template < typename H >
std::pair< typename encode< H >::functor_t, bool > encode< H >::make( const std::string& options
                                                                    , const std::string& next_filter
                                                                    , typename encode< H >::timestamp_functor_t get_timestamp
                                                                    , char delimiter )
{
    if( !next_filter.empty() && next_filter != "head" ) { COMMA_THROW( comma::exception, "encode: only 'head' filter is allowed after encode; got next filter: '" << next_filter << "'" ); }
    const auto& v = comma::split( options, delimiter, true );
    if( v.empty() ) { COMMA_THROW( comma::exception, "encode: please specify image type, e.g. jpg, ppm, etc" ); }
    encode< H > e;
    e._get_timestamp = get_timestamp;
    e._type = v[0];
    e._format = "." + e._type;
    if( v.size() > 1 && !v[1].empty() ) { e._quality = boost::lexical_cast< int >( v[1] ); }
    for( unsigned int i = 2; i < v.size(); ++i ) { if( v[i] == "timestamp" ) { e._embed_timestamp = true; } }
    return std::make_pair( boost::bind< std::pair< H, cv::Mat > >( e, _1 ), false );
}

template < typename H >
std::string encode< H >::usage( unsigned int indent )
{
    std::ostringstream oss;
    std::string i( indent, ' ' );
    oss << i << "encode=<type>[,<quality>],[timestamp]; encode image\n";
    oss << i << "    ATTENTION: make sure to use --output=no-header\n";
    oss << i << "    <type>: image type jpg, png, ppm, tiff (any type cv::imwrite supports)\n";
    oss << i << "    <quality>: jpg quality in percent, e.g. 90; default: 100\n";
    oss << i << "    timestamp; set exif timestamp tags in metadata to image timestamp as:\n";
    oss << i << "               Exif.Image.DateTime: YYYY:MM:DD HH:MM:SS\n";
    oss << i << "               Exif.Photo.SubSecTime: <fractions>\n";
    oss << i << "               e.g. for 20230203T123456.001234 tags will be:\n";
    oss << i << "                   DateTime: 2023:02:03 12:34:56\n";
    oss << i << "                   SubSecTime: 001234\n";
    oss << i << "    example: timestamp image with system time UTC\n";
    oss << i << "        cv-cat --file image.jpg 'encode=jpg,,timestamp' --output=no-header > timestamped.jpg\n";
    oss << i << "        exif timestamped.jpg\n";
    return oss.str();
}

template < typename H >
std::pair< H, cv::Mat > encode< H >::operator()( std::pair< H, cv::Mat > m )
{
    if( is_empty< H >( m, _get_timestamp ) ) { return m; }
    check_image_type( m.second, _type );
    std::vector< unsigned char > buffer;
    std::vector< int > params;
    if( _quality ) { params = snark::cv_mat::imwrite_params( _type, *_quality ); }
    cv::imencode( _format, m.second, buffer, params );
    std::pair< H, cv::Mat > p;
    p.first = m.first;
    if( !_embed_timestamp )
    {
        p.second = cv::Mat( buffer.size(), 1, CV_8UC1 );
        ::memcpy( p.second.data, &buffer[0] , buffer.size() );
        return p;
    }
    // according to the specification (which is hard to find and sucks once found),
    // the subsecond timing value is to be interpreted as decimal digits in the
    // seconds value of the time
    Exiv2::Image::AutoPtr exiv2_image = Exiv2::ImageFactory::open( ( const Exiv2::byte* )( &buffer[0] ), buffer.size() );
    exiv2_image->readMetadata();
    Exiv2::ExifData& exif_data = exiv2_image->exifData();
    auto t = _get_timestamp( p.first );
    std::ostringstream oss;
    oss.imbue( std::locale( std::cout.getloc(), new boost::posix_time::time_facet( "%Y:%m:%d %H:%M:%S" ) ) ); // "YYYY:MM:DD HH:MM:SS"
    oss << t;
    const boost::posix_time::ptime base( comma::csv::impl::epoch );
    const boost::posix_time::time_duration d = t - base;
    std::string microseconds = boost::lexical_cast< std::string >( d.total_microseconds() - d.total_seconds() * 1000000 );
    microseconds = std::string( 6 - microseconds.length(), '0' ) + microseconds;
    exif_data["Exif.Image.DateTime"] = oss.str();
    exif_data["Exif.Photo.SubSecTime"] = microseconds;
    exiv2_image->setExifData( exif_data );
    exiv2_image->writeMetadata();
    unsigned int exiv_size = exiv2_image->io().size();
    exiv2_image->io().seek( 0, Exiv2::BasicIo::beg );
    Exiv2::DataBuf exiv_buffer = exiv2_image->io().read( exiv_size );
    // exiv2_image->readMetadata();
    // Exiv2::ExifData& eee = exiv2_image->exifData();
    // for( const auto& i: eee ) { std::cerr << "==> " << i.key() << ": " << i.toString() << std::endl; }
    // std::cerr << "===========================" << std::endl;
    // Exiv2::ExifTags::taglist( std::cerr );
    // std::cerr << "===========================" << std::endl;
    p.second = cv::Mat( exiv_size, 1, CV_8UC1 );
    ::memcpy( p.second.data, ( const char* )( exiv_buffer.pData_ ), exiv_size );
    return p;
}

template struct encode< boost::posix_time::ptime >;
template struct encode< std::vector< char > >;

} } }  // namespace snark { namespace cv_mat { namespace filters {