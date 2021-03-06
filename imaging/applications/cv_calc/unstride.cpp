// Copyright (c) 2019 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <memory>
#include <sstream>
#include <boost/ptr_container/ptr_vector.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>
#if defined( CV_VERSION_EPOCH ) && CV_VERSION_EPOCH == 2
#include <opencv2/calib3d/calib3d.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif
#include <tbb/parallel_for.h>
#include <comma/base/exception.h>
#include "unstride.h"

namespace snark { namespace cv_calc { namespace unstride {
    
std::string options()
{
    std::ostringstream oss;
    oss << "        --fit-last; last stride is fit exactly to the image size, i.e. last stride may be irregular" << std::endl;
    oss << "        --how-to-handle-overlaps,--how=<how>; default: last; how to handle overlaps" << std::endl;
    oss << "            last: simply put the next tile on top on the previous one" << std::endl;
    oss << "            linear: linearly interpolate across stride overlaps (todo)" << std::endl;
    oss << "            min: take min pixel value: todo" << std::endl;
    oss << "            max: take max pixel value" << std::endl;
    oss << "            other policies: todo" << std::endl;
    oss << "        --input=[<options>]; input options; run cv-cat --help --verbose for details" << std::endl;
    oss << "        --input-interleaving-size,--interleaving=<number>; default=1; <number> of input strides interleave" << std::endl;
    oss << "        --output=[<options>]; output options; run cv-cat --help --verbose for details" << std::endl;
    oss << "        --output-number-of-strides,--number-of-strides: output number of strides as <x>,<y> to stdout and exit" << std::endl;
    //oss << "        --padding=[<padding>]; padding, 'same' or 'valid' (see e.g. tensorflow for the meaning); default: valid" << std::endl;
    oss << "        --shape,--kernel,--size=<x>,<y>; image size" << std::endl;
    oss << "        --strides=[<x>,<y>]; stride size; default: 1,1" << std::endl;
    oss << "        --unstrided-size,--unstrided=<width>,<height>; original (unstrided) image size" << std::endl;
    return oss.str();
}

namespace overlap {

struct base
{
    virtual ~base() {}
    
    virtual void append( cv::Mat image, cv::Mat tile, unsigned int x, unsigned int y ) = 0;
};
    
struct last: public overlap::base
{
    void append( cv::Mat image, cv::Mat tile, unsigned int x, unsigned int y ) { tile.copyTo( cv::Mat( image, cv::Rect( x, y, tile.cols, tile.rows ) ) ); }
};

struct min: public overlap::base
{
    void append( cv::Mat image, cv::Mat tile, unsigned int x, unsigned int y ) { COMMA_THROW( comma::exception, "todo" ); }
};

struct max: public overlap::base
{
    void append( cv::Mat image, cv::Mat tile, unsigned int x, unsigned int y )
    {
        cv::Mat m( image, cv::Rect( x, y, tile.cols, tile.rows ) );
        cv::max( m, tile, m );
    }
};
    
class linear: public overlap::base
{
    public:
        linear(): x_( 0 ), y_( 0 ), y_prev_( 0 ) {}
        
        void append( cv::Mat image, cv::Mat tile, unsigned int x, unsigned int y )
        {
            if( image.type() != CV_32F ) { std::cerr << "cv-calc: unstride: --how=linear: currently works only on images of type f (CV_32F, " << CV_32F << "); got: " << image.type() << std::endl; exit( 1 ); }
            if( x == 0 ) { y_ = y_prev_; }
            cv::Mat s = y == 0 ? tile : vertical_( image, tile, x, y );
            cv::Mat t = x == 0 ? s : horizontal_( image, s, x, y );
            t.copyTo( cv::Mat( image, cv::Rect( x, y, tile.cols, tile.rows ) ) );
            x_ = x;
            y_prev_ = y;
        }
        
    private:
        unsigned int x_;
        unsigned int y_;
        unsigned int y_prev_;
        cv::Mat horizontal_( cv::Mat image, cv::Mat tile, unsigned int x, unsigned int y ) // todo: more reuse between horizontal_() and vertical_()
        {
            cv::Mat t;
            tile.copyTo( t );
            unsigned int overlap = x_ + tile.cols - x;
            tbb::parallel_for( tbb::blocked_range< std::size_t >( 0, tile.rows ), [&]( const tbb::blocked_range< std::size_t >& r )
            {
                for( unsigned int i = r.begin(); i < r.end(); ++i )
                {
                    for( unsigned int j = 0; j < overlap; ++j )
                    {
                        double ratio = double( j ) / overlap;
                        t.at< float >( i, j ) = t.at< float >( i, j ) * ratio + image.at< float >( y + i, x + j ) * ( 1 - ratio );
                    }
                }
            } );
            return t;
        }
        cv::Mat vertical_( cv::Mat image, cv::Mat tile, unsigned int x, unsigned int y )
        {
            cv::Mat t;
            tile.copyTo( t );
            unsigned int overlap = y_ + tile.rows - y;
            tbb::parallel_for( tbb::blocked_range< std::size_t >( 0, overlap ), [&]( const tbb::blocked_range< std::size_t >& r )
            {
                for( unsigned int i = r.begin(); i < r.end(); ++i )
                {
                    double ratio = double( i ) / overlap;
                    for( unsigned int j = 0; int( j ) < tile.cols; ++j )
                    {
                        t.at< float >( i, j ) = t.at< float >( i, j ) * ratio + image.at< float >( y + i, x + j ) * ( 1 - ratio );
                    }
                }
            } );
            return t;
        }
};

class most_central: public overlap::base
{
    public:
        most_central()
        {
            // todo: get resulting image size, striding size, step, and fit last
            // todo: make cv::mat: each pixel: coordinates of tile centre to which this pixel belongs
            COMMA_THROW( comma::exception, "todo" );
        }
        
        void append( cv::Mat image, cv::Mat tile, unsigned int x, unsigned int y )
        {
            // todo: get tile centre
            // todo: in parallel for, for each pixel
            //       ? check if pixel is set; if it is, continue
            //       - find corresponding tile centre coordinates
            //       - if given tile centre is same as target tile centre, update pixel
        }
        
    private:
        
};

base* make( const std::string& how )
{
    if( how == "most-central" ) { return new overlap::most_central; }
    if( how == "last" ) { return new overlap::last; }
    if( how == "linear" ) { return new overlap::linear; }
    if( how == "min" ) { return new overlap::min; }
    if( how == "max" ) { return new overlap::max; }
    std::cerr << "cv-calc: unstride: expected policy to handle overlaps, got: '" << how << "' (more policies: todo)" << std::endl;
    exit( 1 );
}

} // namespace overlap {

class unstrided
{
    public:
        typedef std::pair< boost::posix_time::ptime, cv::Mat > pair_t;
        
        unstrided( const cv::Point2i& size
                 , const cv::Point2i& shape
                 , const cv::Point2i& strides
                 , const cv::Point2i& strides_count
                 , bool fit_last
                 , overlap::base* overlap )
           : size_( size )
           , shape_( shape )
           , strides_( strides )
           , strides_count_( strides_count )
           , fit_last_( fit_last )
           , overlap_( overlap )
           , ix_( 0 )
           , iy_( 0 )
        {
        }
        
        void append( const pair_t& p )
        {
            if( output_.second.empty() ) { output_.first = p.first; output_.second = cv::Mat( size_.y, size_.x, p.second.type() ); }
            if( output_.second.type() != p.second.type() ) { std::cerr << "cv-calc: unstride: expected input image type " << output_.second.type() << "; got: " << p.second.type() << std::endl; exit( 1 ); }
            if( p.second.rows != shape_.y ) { std::cerr << "cv-calc: unstride: expected input image with " << shape_.y << "; got: " << p.second.rows << std::endl; exit( 1 ); }
            if( p.second.cols != shape_.x ) { std::cerr << "cv-calc: unstride: expected input image with " << shape_.x << "; got: " << p.second.cols << std::endl; exit( 1 ); }
            if( ix_ == 0 && iy_ == 0 ) { output_.first = p.first; }
            unsigned int x = ix_ * strides_.x;
            unsigned int y = iy_ * strides_.y;
            if( fit_last_ )
            {
                if( ix_ + 1 == strides_count_.x ) { x = size_.x - shape_.x; }
                if( iy_ + 1 == strides_count_.y ) { y = size_.y - shape_.y; }
            }
            overlap_->append( output_.second, p.second, x, y );
            ++ix_;
            if( ix_ >= strides_count_.x )
            {
                ix_ = 0;
                ++iy_;
                if( iy_ >= strides_count_.y ) { iy_ = 0; }
            }
        }
        
        bool ready() const { return ix_ == 0 && iy_ == 0 && !output_.second.empty(); }
        
        const pair_t& operator()() const { return output_; }
        
        void reset() { output_.second.setTo( 0 ); }
        
    private:
        cv::Point2i size_;
        cv::Point2i shape_;
        cv::Point2i strides_;
        cv::Point2i strides_count_;
        bool fit_last_;
        std::unique_ptr< overlap::base > overlap_;
        int ix_;
        int iy_;
        pair_t output_;
};

int run( const comma::command_line_options& options, const snark::cv_mat::serialization& input_options, const snark::cv_mat::serialization& output_options )
{
    snark::cv_mat::serialization input_serialization( input_options );
    snark::cv_mat::serialization output_serialization( output_options );
    const std::vector< std::string >& unstrided_size_vector = comma::split( options.value< std::string >( "--unstrided-size,--unstrided" ), ',' );
    if( unstrided_size_vector.size() != 2 ) { std::cerr << "cv-calc: unstride-positions: expected --unstrided-size as <width>,<height>, got: \"" << options.value< std::string >( "--unstrided-size,--unstrided" ) << std::endl; return 1; }
    cv::Point2i unstrided_size( boost::lexical_cast< unsigned int >( unstrided_size_vector[0] ), boost::lexical_cast< unsigned int >( unstrided_size_vector[1] ) );
    const std::vector< std::string >& strides_vector = comma::split( options.value< std::string >( "--strides", "1,1" ), ',' );
    if( strides_vector.size() != 2 ) { std::cerr << "cv-calc: unstride-positions: expected strides as <x>,<y>, got: \"" << options.value< std::string >( "--strides" ) << std::endl; return 1; }
    cv::Point2i strides( boost::lexical_cast< unsigned int >( strides_vector[0] ), boost::lexical_cast< unsigned int >( strides_vector[1] ) );
    const std::vector< std::string >& shape_vector = comma::split( options.value< std::string >( "--shape,--size,--kernel" ), ',' );
    if( shape_vector.size() != 2 ) { std::cerr << "cv-calc: unstride-positions: expected shape as <x>,<y>, got: \"" << options.value< std::string >( "--shape,--size,--kernel" ) << std::endl; return 1; }
    cv::Point2i shape( boost::lexical_cast< unsigned int >( shape_vector[0] ), boost::lexical_cast< unsigned int >( shape_vector[1] ) );
    cv::Point2i strides_count;
    strides_count.x = ( unstrided_size.x - shape.x ) / strides.x + 1;
    strides_count.y = ( unstrided_size.y - shape.y ) / strides.y + 1;
    bool fit_last = options.exists( "--fit-last" );
    if( fit_last )
    {
        if( int( strides_count.x - 1 ) * strides.x + shape.x < unstrided_size.x ) { ++strides_count.x; }
        if( int( strides_count.y - 1 ) * strides.y + shape.y < unstrided_size.y ) { ++strides_count.y; }
    }
    if( options.exists( "--output-number-of-strides,--number-of-strides" ) ) { std::cout << strides_count.x << "," << strides_count.y << std::endl; exit( 0 ); }
    typedef std::pair< boost::posix_time::ptime, cv::Mat > pair_t;
    unsigned int interleaving_size = options.value( "--input-interleaving-size,--interleaving", 1 );
    std::vector< unstrided > outputs;
    auto how = options.value< std::string >( "--how-to-handle-overlaps,--how", "last" );
    for( unsigned int i = 0; i < interleaving_size; ++i ) { outputs.push_back( unstrided( unstrided_size, shape, strides, strides_count, fit_last, overlap::make( how ) ) ); }
    while( std::cin.good() )
    {
        for( unsigned int i = 0; i < interleaving_size && std::cin.good(); ++i )
        {
            pair_t p = input_serialization.read< boost::posix_time::ptime >( std::cin );
            if( p.second.empty() ) { return 0; }
            outputs[i].append( p );
            if( !outputs[i].ready() ) { continue; }
            output_serialization.write_to_stdout( outputs[i](), true );
            outputs[i].reset();
        }
    }
    return 0;
}

} } } // namespace snark { namespace cv_calc { namespace unstride {
