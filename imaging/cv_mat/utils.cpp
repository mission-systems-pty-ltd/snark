// Copyright (c) 2011 The University of Sydney

#include <memory>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include "../../timing/time.h"
#include "utils.h"

namespace snark{ namespace cv_mat {

boost::unordered_map< std::string, int > fill_types()
{
    boost::unordered_map< std::string, int > types;
    types[ "CV_8UC1" ] = types[ "ub" ] = CV_8UC1;
    types[ "CV_8UC2" ] = types[ "2ub" ] = CV_8UC2;
    types[ "CV_8UC3" ] = types[ "3ub" ] = CV_8UC3;
    types[ "CV_8UC4" ] = types[ "4ub" ] = CV_8UC4;
    types[ "CV_8SC1" ] = types[ "b" ] = CV_8SC1;
    types[ "CV_8SC2" ] = types[ "2b" ] = CV_8SC2;
    types[ "CV_8SC3" ] = types[ "3b" ] = CV_8SC3;
    types[ "CV_8SC4" ] = types[ "4b" ] = CV_8SC4;
    types[ "CV_16UC1" ] = types[ "uw" ] = CV_16UC1;
    types[ "CV_16UC2" ] = types[ "2uw" ] = CV_16UC2;
    types[ "CV_16UC3" ] = types[ "3uw" ] = CV_16UC3;
    types[ "CV_16UC4" ] = types[ "4uw" ] = CV_16UC4;
    types[ "CV_16SC1" ] = types[ "w" ] = CV_16SC1;
    types[ "CV_16SC2" ] = types[ "2w" ] = CV_16SC2;
    types[ "CV_16SC3" ] = types[ "3w" ] = CV_16SC3;
    types[ "CV_16SC4" ] = types[ "4w" ] = CV_16SC4;
    types[ "CV_32SC1" ] = types[ "i" ] = CV_32SC1;
    types[ "CV_32SC2" ] = types[ "2i" ] = CV_32SC2;
    types[ "CV_32SC3" ] = types[ "3i" ] = CV_32SC3;
    types[ "CV_32SC4" ] = types[ "4i" ] = CV_32SC4;
    types[ "CV_32FC1" ] = types[ "f" ] = CV_32FC1;
    types[ "CV_32FC2" ] = types[ "2f" ] = CV_32FC2;
    types[ "CV_32FC3" ] = types[ "3f" ] = CV_32FC3;
    types[ "CV_32FC4" ] = types[ "4f" ] = CV_32FC4;
    types[ "CV_64FC1" ] = types[ "d" ] = CV_64FC1;
    types[ "CV_64FC2" ] = types[ "2d" ] = CV_64FC2;
    types[ "CV_64FC3" ] = types[ "3d" ] = CV_64FC3;
    types[ "CV_64FC4" ] = types[ "4d" ] = CV_64FC4;
    return types;
}

boost::unordered_map< int, std::string > fill_types_as_string_()
{
    boost::unordered_map< int, std::string > types;
    types[ CV_8UC1 ] = "CV_8UC1";
    types[ CV_8UC2 ] = "CV_8UC2";
    types[ CV_8UC3 ] = "CV_8UC3";
    types[ CV_8UC4 ] = "CV_8UC4";
    types[ CV_8SC1 ] = "CV_8SC1";
    types[ CV_8SC2 ] = "CV_8SC2";
    types[ CV_8SC3 ] = "CV_8SC3";
    types[ CV_8SC4 ] = "CV_8SC4";
    types[ CV_16UC1 ] = "CV_16UC1";
    types[ CV_16UC2 ] = "CV_16UC2";
    types[ CV_16UC3 ] = "CV_16UC3";
    types[ CV_16UC4 ] = "CV_16UC4";
    types[ CV_16SC1 ] = "CV_16SC1";
    types[ CV_16SC2 ] = "CV_16SC2";
    types[ CV_16SC3 ] = "CV_16SC3";
    types[ CV_16SC4 ] = "CV_16SC4";
    types[ CV_32SC1 ] = "CV_32SC1";
    types[ CV_32SC2 ] = "CV_32SC2";
    types[ CV_32SC3 ] = "CV_32SC3";
    types[ CV_32SC4 ] = "CV_32SC4";
    types[ CV_32FC1 ] = "CV_32FC1";
    types[ CV_32FC2 ] = "CV_32FC2";
    types[ CV_32FC3 ] = "CV_32FC3";
    types[ CV_32FC4 ] = "CV_32FC4";
    types[ CV_64FC1 ] = "CV_64FC1";
    types[ CV_64FC2 ] = "CV_64FC2";
    types[ CV_64FC3 ] = "CV_64FC3";
    types[ CV_64FC4 ] = "CV_64FC4";
    return types;
}

static std::unordered_map< int, cv::Scalar > _white =
{
    { CV_8UC1, cv::Scalar( 255 ) },
    { CV_8UC2, cv::Scalar( 255, 255 ) },
    { CV_8UC3, cv::Scalar( 255, 255, 255 ) },
    { CV_8UC4, cv::Scalar( 255, 255, 255, 255 ) },
    { CV_8SC1, cv::Scalar( 128 ) },
    { CV_8SC2, cv::Scalar( 128, 128 ) },
    { CV_8SC3, cv::Scalar( 128, 128, 128 ) },
    { CV_8SC4, cv::Scalar( 128, 128, 128, 128 ) },
    { CV_16UC1, cv::Scalar( 0xffff ) },
    { CV_16UC2, cv::Scalar( 0xffff, 0xffff ) },
    { CV_16UC3, cv::Scalar( 0xffff, 0xffff, 0xffff ) },
    { CV_16UC4, cv::Scalar( 0xffff, 0xffff, 0xffff, 0xffff ) },
    { CV_16SC1, cv::Scalar( 0x8fff ) },
    { CV_16SC2, cv::Scalar( 0x8fff, 0x8fff ) },
    { CV_16SC3, cv::Scalar( 0x8fff, 0x8fff, 0x8fff ) },
    { CV_16SC4, cv::Scalar( 0x8fff, 0x8fff, 0x8fff, 0x8fff ) },
    { CV_32SC1, cv::Scalar( 0x8fffffff ) },
    { CV_32SC2, cv::Scalar( 0x8fffffff, 0x8fffffff ) },
    { CV_32SC3, cv::Scalar( 0x8fffffff, 0x8fffffff, 0x8fffffff ) },
    { CV_32SC4, cv::Scalar( 0x8fffffff, 0x8fffffff, 0x8fffffff, 0x8fffffff ) },
    { CV_32FC1, cv::Scalar( 1. ) },
    { CV_32FC2, cv::Scalar( 1., 1. ) },
    { CV_32FC3, cv::Scalar( 1., 1., 1. ) },
    { CV_32FC4, cv::Scalar( 1., 1., 1., 1. ) },
    { CV_64FC1, cv::Scalar( 1. ) },
    { CV_64FC2, cv::Scalar( 1., 1. ) },
    { CV_64FC3, cv::Scalar( 1., 1., 1. ) },
    { CV_64FC4, cv::Scalar( 1., 1., 1., 1. ) }
};

cv::Scalar white( int t )
{
    auto i = _white.find( t );
    COMMA_THROW_IF( i == _white.end(), "white colour is not defined for type: " << t );
    return i->second;
}

boost::unordered_map< std::string, unsigned int > fill_cvt_color_types_() // todo: split on comma, otherwise quadratic complexity
{
    boost::unordered_map<std::string, unsigned int> types;
    //note RGB is exactly the same as BGR
    types[ "CV_BGR2GRAY" ] = types[ "BGR,GRAY" ] = types[ "cv::COLOR_RGB2GRAY" ] = types[ "RGB,GRAY" ] = types[ "rgb,gray" ] = CV_BGR2GRAY;
    types[ "CV_GRAY2BGR" ] = types[ "GRAY,BGR" ] = types[ "CV_GRAY2RGB" ] = types[ "GRAY,RGB" ] = types[ "gray,rgb" ] = CV_GRAY2BGR;
    types[ "CV_BGR2XYZ" ] = types[ "BGR,XYZ" ] = types[ "CV_RGB2XYZ" ] = types[ "RGB,XYZ" ] = types[ "rgb,xyz" ] = CV_BGR2XYZ;
    types[ "CV_XYZ2BGR" ] = types[ "XYZ,BGR" ] = types[ "CV_XYZ2RGB" ] = types[ "XYZ,RGB" ] = types[ "xyz,rgb" ] = CV_XYZ2BGR;
    types[ "CV_RGB2HSV" ] = types[ "RGB,HSV" ] = types[ "rgb,hsv" ] = CV_RGB2HSV;
    types[ "CV_BGR2HSV" ] = types[ "BGR,HSV" ] = types[ "bgr,hsv" ] = CV_BGR2HSV;
    types[ "CV_HSV2RGB" ] = types[ "HSV,RGB" ] = types[ "hsv,rgb" ] = CV_HSV2RGB;
    types[ "CV_HSV2BGR" ] = types[ "HSV,BGR" ] = types[ "hsv,bgr" ] = CV_HSV2BGR;
    types[ "CV_BGR2Lab" ] = types[ "BGR,Lab" ] = types[ "CV_RGB2Lab" ] = types[ "RGB,Lab" ] = types[ "rgb,lab" ] = CV_BGR2Lab;
    types[ "CV_Lab2BGR" ] = types[ "Lab,BGR" ] = types[ "CV_Lab2RGB" ] = types[ "Lab,RGB" ] = types[ "lab,rgb" ] = CV_Lab2BGR;
    types[ "CV_BayerBG2BGR" ] = types[ "BayerBG,BGR" ] = types[ "CV_BayerBG2RGB" ] = types[ "BayerBG,RGB" ] = types[ "bayerbg,rgb" ] = CV_BayerBG2BGR;
    types[ "CV_BayerGB2BGR" ] = types[ "BayerGB,BGR" ] = types[ "CV_BayerGB2RGB" ] = types[ "BayerGB,RGB" ] = types[ "bayergb,rgb" ] = CV_BayerGB2BGR;
    types[ "CV_BayerRG2BGR" ] = types[ "BayerRG,BGR" ] = types[ "CV_BayerRG2RGB" ] = types[ "BayerRG,RGB" ] = types[ "bayerrg,rgb" ] = CV_BayerRG2BGR;
    types[ "CV_BayerGR2BGR" ] = types[ "BayerGR,BGR" ] = types[ "CV_BayerGR2RGB" ] = types[ "BayerGR,RGB" ] = types[ "bayergr,rgb" ] = CV_BayerGR2BGR;
    types[ "CV_BayerBG2GRAY" ] = types[ "BayerBG,GRAY" ] = types[ "bayerbg,gray" ] = CV_BayerBG2GRAY;
    types[ "CV_BayerGB2GRAY" ] = types[ "BayerGB,GRAY" ] = types[ "bayergb,gray" ] = CV_BayerGB2GRAY;
    types[ "CV_BayerRG2GRAY" ] = types[ "BayerRG,GRAY" ] = types[ "bayerrg,gray" ] = CV_BayerRG2GRAY;
    types[ "CV_BayerGR2GRAY" ] = types[ "BayerGR,GRAY" ] = types[ "bayergr,gray" ] = CV_BayerGR2GRAY;
    types[ "CV_BGR2RGB" ] = types[ "BGR,RGB" ] = types[ "bgr,rgb" ] = CV_BGR2RGB;
    types[ "CV_RGB2BGR" ] = types[ "RGB,BGR" ] = types[ "rgb,bgr" ] = CV_RGB2BGR;
    types[ "CV_BGR2BGRA" ] = types[ "BGR,BGRA" ] = types[ "bgr,bgra" ] = CV_BGR2BGRA;
    types[ "CV_RGB2RGBA" ] = types[ "RGB,RGBA" ] = types[ "rgb,rgba" ] = CV_RGB2RGBA;
    types[ "CV_BGRA2BGR" ] = types[ "BGRA,BGR" ] = types[ "bgra,bgr" ] = CV_BGRA2BGR;
    types[ "CV_RGBA2RGB" ] = types[ "RGBA,RGB" ] = types[ "rgba,rgb" ] = CV_RGBA2RGB;
    types[ "CV_BGR2RGBA" ] = types[ "BGR,RGBA" ] = types[ "bgr,rgba" ] = CV_BGR2RGBA;
    types[ "CV_RGB2BGRA" ] = types[ "RGB,BGRA" ] = types[ "rgb,bgra" ] = CV_RGB2BGRA;
    types[ "CV_RGBA2BGR" ] = types[ "RGBA,BGR" ] = types[ "rgba,bgr" ] = CV_RGBA2BGR;
    types[ "CV_BGRA2RGB" ] = types[ "BGRA,RGB" ] = types[ "bgra,rgb" ] = CV_BGRA2RGB;
    types[ "CV_BGRA2RGBA" ] = types[ "BGRA,RGBA" ] = types[ "bgra,rgba" ] = CV_BGRA2RGBA;
    types[ "CV_RGBA2BGRA" ] = types[ "RGBA,BGRA" ] = types[ "rgba,bgra" ] = CV_RGBA2BGRA;
    return types;
}

cv::Scalar scalar_from_strings( const std::string* begin, unsigned int size )
{
    switch( size )
    {
        case 1: return cv::Scalar( boost::lexical_cast< float >( begin[0] ) );
        case 2: return cv::Scalar( boost::lexical_cast< float >( begin[0] ), boost::lexical_cast< float >( begin[1] ) );
        case 3: return cv::Scalar( boost::lexical_cast< float >( begin[0] ), boost::lexical_cast< float >( begin[1] ), boost::lexical_cast< float >( begin[2] ) );
        case 4: return cv::Scalar( boost::lexical_cast< float >( begin[0] ), boost::lexical_cast< float >( begin[1] ), boost::lexical_cast< float >( begin[2] ), boost::lexical_cast< float >( begin[3] ) );
        default: break;
    }
    COMMA_THROW( comma::exception, "expected a scalar of the size up to 4, got: " << size << " elements" );
}

unsigned int cvt_color_type_from_string( const std::string& t )
{
    static boost::unordered_map< std::string, unsigned int > cvt_color_types_ = fill_cvt_color_types_();
    boost::unordered_map< std::string, unsigned int >::const_iterator it = cvt_color_types_.find( t );
    if( it != cvt_color_types_.end() ) { return it->second; }
    std::string s = t;
    boost::to_upper( s );
    it = cvt_color_types_.find( s ); // quick and dirty, until split on comma trivially implemented fill_cvt_color_types_
    if( it != cvt_color_types_.end() ) { return it->second; }
    try { return boost::lexical_cast< int >( t ); } catch( ... ) {}
    COMMA_THROW( comma::exception, "unknown conversion enum '" << t << "' for convert-color" );
}

std::string type_as_string( int t ) // to avoid compilation warning
{
    static const boost::unordered_map< int, std::string > types_as_string = fill_types_as_string_();
    boost::unordered_map< int, std::string >::const_iterator it = types_as_string.find( t );
    return it == types_as_string.end() ? boost::lexical_cast< std::string >( t ) : it->second;
}

std::string make_filename( const boost::posix_time::ptime& t, const std::string& extension, boost::optional< unsigned int > index )
{
    std::ostringstream ss;
    ss << snark::timing::to_iso_string_always_with_fractions( t );
    if( index ) { ss << '.' << *index; }
    ss << '.' << extension;
    return ss.str();
}

std::vector< int > imwrite_params( const std::string& type, const int quality )
{
    std::vector< int > params;
    if ( type == "jpg" ) { params.push_back( cv::IMWRITE_JPEG_QUALITY ); }
    else { COMMA_THROW( comma::exception, "quality only supported for jpg images, not for \"" << type << "\" yet" ); }
    params.push_back( quality );
    return params;
}
    
void check_image_type( const cv::Mat& m, const std::string& type )
{
    int channels = m.channels();
    int size = m.elemSize() / channels;
    int cv_type = m.type();
    if( !( channels == 1 || channels == 3 ) ) { COMMA_THROW( comma::exception, "expected image with 1 or 3 channel, got " << channels << " channels" ); }
    if( !( size == 1 || size == 2 ) ) { COMMA_THROW( comma::exception, "expected 8- or 16-bit image, got " << size*8 << "-bit image" ); }
    if( size == 2 && !( cv_type == CV_16UC1 || cv_type == CV_16UC3 ) ) {  COMMA_THROW( comma::exception, "expected 16-bit image with unsigned elements, got image of type " << type_as_string( cv_type ) ); }
    if( size == 2 && !( type == "tiff" || type == "tif" || type == "png" || type == "jp2" ) ) { COMMA_THROW( comma::exception, "cannot convert 16-bit image to type " << type << "; use tif or png instead" ); }
}

template < typename S, typename T >
static void assign_( unsigned char* channel, T value ) { *( reinterpret_cast< S* >( channel ) ) = static_cast< S >( value ); }

template < typename T >
static T get_( unsigned char* channel ) { return *( reinterpret_cast< T* >( channel ) ); }

template < typename T >
void set_channel( unsigned char* channel, T value, int depth )
{
    switch( depth )
    {
        case CV_8U: assign_< unsigned char >( channel, value ); break;
        case CV_8S: assign_< char >( channel, value ); break;
        case CV_16U: assign_< comma::uint16 >( channel, value ); break;
        case CV_16S: assign_< comma::int16 >( channel, value ); break;
        case CV_32S: assign_< comma::int32 >( channel, value ); break;
        case CV_32F: assign_< float >( channel, value ); break;
        case CV_64F: assign_< double >( channel, value ); break;
        default: COMMA_THROW( comma::exception, "never here" );
    }
}

template < typename T >
T get_channel( unsigned char* channel, int depth )
{
    switch( depth )
    {
        case CV_8U: return get_< unsigned char >( channel );
        case CV_8S: return get_< char >( channel );
        case CV_16U: return get_< comma::uint16 >( channel );
        case CV_16S: return get_< comma::int16 >( channel );
        case CV_32S: return get_< comma::int32 >( channel );
        case CV_32F: return get_< float >( channel );
        case CV_64F: return get_< double >( channel );
        default: COMMA_THROW( comma::exception, "never here" );
    }
}

template void set_channel< char >( unsigned char*, char, int );
template void set_channel< unsigned char >( unsigned char*, unsigned char, int );
template void set_channel< comma::int16 >( unsigned char*, comma::int16, int );
template void set_channel< comma::uint16 >( unsigned char*, comma::uint16, int );
template void set_channel< comma::int32 >( unsigned char*, comma::int32, int );
template void set_channel< comma::uint32 >( unsigned char*, comma::uint32, int );
template void set_channel< comma::int64 >( unsigned char*, comma::int64, int );
template void set_channel< comma::uint64 >( unsigned char*, comma::uint64, int );
template void set_channel< float >( unsigned char*, float, int );
template void set_channel< double >( unsigned char*, double, int );

template char get_channel< char >( unsigned char*, int );
template unsigned char get_channel< unsigned char >( unsigned char*, int );
template comma::int16 get_channel< comma::int16 >( unsigned char*, int );
template comma::uint16 get_channel< comma::uint16 >( unsigned char*, int );
template comma::int32 get_channel< comma::int32 >( unsigned char*, int );
template comma::uint32 get_channel< comma::uint32 >( unsigned char*, int );
template comma::int64 get_channel< comma::int64 >( unsigned char*, int );
template comma::uint64 get_channel< comma::uint64 >( unsigned char*, int );
template float get_channel< float >( unsigned char*, int );
template double get_channel< double >( unsigned char*, int );

cv::ColormapTypes colormap_from_string( const std::string& name )
{
    static std::map< std::string, cv::ColormapTypes > types = { { "autumn", cv::COLORMAP_AUTUMN }
                                                              , { "bone", cv::COLORMAP_BONE }
                                                              , { "jet", cv::COLORMAP_JET }
                                                              , { "winter", cv::COLORMAP_WINTER }
                                                              , { "rainbow", cv::COLORMAP_RAINBOW }
                                                              , { "ocean", cv::COLORMAP_OCEAN }
                                                              , { "summer", cv::COLORMAP_SUMMER }
                                                              , { "spring", cv::COLORMAP_SPRING }
                                                              , { "cool", cv::COLORMAP_COOL }
                                                              , { "hsv", cv::COLORMAP_HSV }
                                                              , { "pink", cv::COLORMAP_PINK }
                                                              , { "hot", cv::COLORMAP_HOT } };
    auto it = types.find( name );
    if( it != types.end() ) { return it->second; }
    try { return static_cast< cv::ColormapTypes >( boost::lexical_cast< int >( name ) ); } catch( ... ) {}
    COMMA_THROW( comma::exception, "expected colour-map type; got: \"" << name << "\"; symbolic name may not be implemented, yet; see --help --verbose for numeric color-map types" );
}

cv::Scalar color_from_string( const std::string& name )
{
    static std::map< std::string, cv::Scalar > colors = { { "red", cv::Scalar( 0, 0, 0xffff ) }
                                                        , { "green", cv::Scalar( 0, 0xffff, 0 ) }
                                                        , { "blue", cv::Scalar( 0xffff, 0, 0 ) }
                                                        , { "cyan", cv::Scalar( 0xffff, 0xffff, 0 ) }
                                                        , { "magenta", cv::Scalar( 0xffff, 0, 0xffff ) }
                                                        , { "white", cv::Scalar( 0xffff, 0xffff, 0xffff ) }
                                                        , { "yellow", cv::Scalar( 0, 0xffff, 0xffff ) }
                                                        , { "black", cv::Scalar( 0, 0, 0 ) }
                                                        , { "light-grey", cv::Scalar( 200, 200, 200 ) }
                                                        , { "grey", cv::Scalar( 128, 128, 128 ) }
                                                        , { "perceptual-green", cv::Scalar( 0x9f43, 0xe725, 0 ) }
                                                        , { "", cv::Scalar( 0, 0xffff, 0xffff ) } };
    auto it = colors.find( name );
    if( it != colors.end() ) { return it->second; }
    COMMA_THROW( comma::exception, "expected colour, e.g. 'red', got '" << name << "'" );
}

void apply_color_range( const cv::Mat& in, cv::Mat& out, const std::pair< cv::Scalar, cv::Scalar >& color_range )
{
    if( in.type() != CV_8UC1 ) { COMMA_THROW( comma::exception, "expected greyscale 8-bit image (CV_8UC1); got image type: " << type_as_string( in.type() ) ); }
    out = cv::Mat( in.rows, in.cols, CV_8UC3 );
    const unsigned char* begin = &( in.at< unsigned char >( 0 ) );
    const unsigned char* end = begin + in.rows * in.cols;
    unsigned char* p = &( out.at< unsigned char >( 0 ) );
    for( const unsigned char* in_p = begin; in_p < end; ++in_p )
    {
        float c = float( *in_p ) / 255;
        auto s = color_range.first * ( 1. - c ) + color_range.second * c;
        for( unsigned int i = 0; i < 3; ++i ) { *p++ = s[i] / 255; }
    }
}

} }  // namespace snark { namespace cv_mat {
