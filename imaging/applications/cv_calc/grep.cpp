// Copyright (c) 2025 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <numeric>
#include <tbb/parallel_for.h>
#include <comma/base/exception.h>
#include <comma/string/split.h>
#include "grep.h"

namespace snark { namespace cv_calc { namespace grep {

non_zero::non_zero( const std::string& s )
{
    if( s.empty() ) { return; }
    const std::vector< std::string >& v = comma::split( s, ',' );
    if( v[0] == "ratio" )
    {
        if( v.size() < 2 ) { COMMA_THROW( comma::exception, "expected --non-zero=ratio,<min>,<max>; got --non-zero=ratio" ); }
        if( !v[1].empty() ) { ratio_.first = boost::lexical_cast< double >( v[1] ); }
        if( v.size() > 2 && !v[2].empty() ) { ratio_.second = boost::lexical_cast< double >( v[2] ); }
        return;
    }
    if( v[0] == "size" )
    {
        if( v.size() < 2 ) { COMMA_THROW( comma::exception, "expected --non-zero=size,<min>,<max>; got --non-zero=size" ); }
        if( !v[1].empty() ) { size_.first = boost::lexical_cast< unsigned int >( v[1] ); }
        if( v.size() > 2 && !v[2].empty() ) { size_.second = boost::lexical_cast< unsigned int >( v[2] ); }
        return;
    }
    COMMA_THROW( comma::exception, "--non-zero: expected 'ratio' or 'size', got: '" << v[0] << "'" );
}

non_zero::operator bool() const { return static_cast< bool >( ratio_.first ) || static_cast< bool >( ratio_.second ) || static_cast< bool >( size_.first ) || static_cast< bool >( size_.second ); }

void non_zero::size( unsigned int image_size )
{
    if( ratio_.first ) { size_.first = image_size * *ratio_.first; }
    if( ratio_.second ) { size_.second = image_size * *ratio_.second; }
}

bool non_zero::keep( unsigned int count ) const { return ( !size_.first || *size_.first <= count ) && ( !size_.second || count < *size_.second ); }

bool non_zero::keep( const cv::Mat& m ) const { return keep( count( m ) ); }

unsigned int non_zero::count( const cv::Mat& m ) const
{
    static std::vector< char > zero_pixel( m.elemSize(), 0 );
    std::vector< unsigned int > counts( m.rows, 0 );
    tbb::parallel_for( tbb::blocked_range< std::size_t >( 0, m.rows )
                     , [&]( const tbb::blocked_range< std::size_t >& r )
                       {
                           for( unsigned int i = r.begin(); i < r.end(); ++i )
                           {
                               for( const unsigned char* ptr = m.ptr( i ); ptr < m.ptr( i + 1 ); ptr += m.elemSize() )
                               {
                                   if( ::memcmp( ptr, &zero_pixel[0], zero_pixel.size() ) != 0 ) { ++counts[i]; }
                               }
                           }
                       } );
    return std::accumulate( counts.begin(), counts.end(), 0 );
}

bool non_zero::keep_counting_( unsigned int count ) const
{
    if( size_.second ) { return *size_.second < count; }
    return size_.first && ( count < *size_.first );
}

} } } // namespace grep {namespace snark { namespace cv_calc { namespace grep {