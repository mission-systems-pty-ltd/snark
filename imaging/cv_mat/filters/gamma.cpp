// Copyright (c) 2019 Vsevolod Vlaskine

// @author vsevolod vlaskine

#include <cmath>
#include <iostream>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/lexical_cast.hpp>
#include <tbb/parallel_for.h>
#include <opencv2/highgui/highgui.hpp>
#include "gamma.h"

namespace snark { namespace cv_mat { namespace filters {

std::unordered_map< comma::int32, double > _make_map( double gamma, unsigned int size ) // todo? make static? probably not...
{
    std::unordered_map< comma::int32, double > m;
    double exp = 1. / gamma;
    //for( unsigned int i = 0; i < size; ++i ) { m[i] = std::pow( double( i ) / ( size - 1 ), exp ) * ( size - 1 ); } // todo? quick and dirty; make map optionally taking vector?
    for( unsigned int i = 0; i < size; ++i ) { m[i] = std::pow( double( i ) / ( size - 1 ), exp ); }
    return m;
}

template < typename H > std::string gamma< H >::usage( unsigned int indent ) { return "gamma=<value>; convert image gamma; output will be floating point image; todo: optionally convert to original type\n"; }
template < typename H > gamma< H >::gamma( double value ): _map_8( _make_map( value, 256 ), true ), _map_16( _make_map( value, 256 * 256 ), true ) {}
template < typename H > std::pair< typename gamma< H >::functor_t, bool > gamma< H >::make( const std::string& options ) { return std::make_pair( gamma( boost::lexical_cast< double >( options ) ), true ); }

template < typename H > std::pair< H, cv::Mat > gamma< H >::operator()( std::pair< H, cv::Mat > m )
{
    switch( m.second.type() ) // quick and dirty; opencv really got their design wrong: type is known in runtime whereas handling types is a compile-time thing
    {
        case CV_8UC1:
        case CV_8UC2:
        case CV_8UC3:
        case CV_8UC4:
            return _map_8( m );
        case CV_16UC1:
        case CV_16UC2:
        case CV_16UC3:
        case CV_16UC4:
            return _map_16( m );
        default:
            std::cerr << "map filter: expected 1- or 2-byte unsigned integer cv type, got " << m.second.type() << std::endl;
            return std::pair< H, cv::Mat >(); // todo: throw exception, don't trash stderr
    }
}

template class gamma< boost::posix_time::ptime >;
template class gamma< std::vector< char > >;

} } }  // namespace snark { namespace cv_mat { namespace filters {
