// Copyright (c) 2019 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <memory>
#include <sstream>
#include <vector>
#include <boost/bind/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <tbb/parallel_for.h>
#include <comma/base/exception.h>
#include "hard_edge.h"
#include "../utils.h"

namespace snark { namespace cv_mat { namespace filters {

template < typename H >
std::pair< H, cv::Mat > hard_edge< H >::handle( std::pair< H, cv::Mat > m, float background )
{
    std::pair< H, cv::Mat > n;
    n.first = m.first;
    m.second.copyTo( n.second );
    std::vector< unsigned char > background_pixel( m.second.elemSize() );
    for( unsigned int i = 0; i < m.second.elemSize(); i += m.second.elemSize1() ) { set_channel( &background_pixel[i], background, m.second.depth() ); }
    auto is_background = [&]( int i, int j ) -> bool { return std::memcmp( m.second.ptr( i, j ), &background_pixel[0], m.second.elemSize() ) == 0; };
    auto same = [&]( int i0, int j0, int i1, int j1 ) -> bool
    {
        if( i1 < 0 || i1 >= m.second.rows || j1 < 0 || j1 >= m.second.cols ) { return true; }
        return std::memcmp( m.second.ptr( i0, j0 ), m.second.ptr( i1, j1 ), m.second.elemSize() ) == 0;
    };
    auto reset = [&]( int i, int j ) { std::memcpy( n.second.ptr( i, j ), &background_pixel[0], m.second.elemSize() ); };
    tbb::parallel_for( tbb::blocked_range< std::size_t >( 0, m.second.rows, m.second.rows / 8 ), [&]( const tbb::blocked_range< std::size_t >& r )
    {
        for( unsigned int i = r.begin(); i < r.end(); ++i )
        {
            for( int j = 0; j < m.second.cols; ++ j )
            {
                if( is_background( i, j ) ) { continue; }
                if(    same( i, j, i - 1, j - 1 )
                    && same( i, j, i - 1, j     )
                    && same( i, j, i - 1, j + 1 )
                    && same( i, j, i    , j - 1 )
                    && same( i, j, i    , j + 1 )
                    && same( i, j, i + 1, j - 1 )
                    && same( i, j, i + 1, j     )
                    && same( i, j, i + 1, j + 1 ) )
                {
                    reset( i, j );
                }
            }
        }
    } );
    return n;
}

template < typename H >
std::pair< typename hard_edge< H >::functor_t, bool > hard_edge< H >::make( const std::string& options )
{
    return std::make_pair( boost::bind( &hard_edge< H >::handle, boost::placeholders::_1, options.empty() ? 0. : boost::lexical_cast< float >( options ) ), true );
}

template < typename H >
std::string hard_edge< H >::usage( unsigned int indent )
{
    std::ostringstream oss;
    std::string i( indent, ' ' );
    oss << i << "hard-edge[=<background>]: find hard edge (currently only single-channel images supported)\n";
    oss << i << "                          set all non-edge pixels to <background>\n";
    return oss.str();
}

template struct hard_edge< boost::posix_time::ptime >;
template struct hard_edge< std::vector< char > >;

} } }  // namespace snark { namespace cv_mat { namespace impl {
