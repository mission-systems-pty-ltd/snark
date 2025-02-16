// Copyright (c) 2019 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <array>
#include <cinttypes>
#include <functional>
#include <vector>
#include <opencv2/core/core.hpp>
#include <comma/base/exception.h>
#include <comma/base/types.h>

// #include <iostream>

namespace snark { namespace cv_mat {

template < typename V >
void set( cv::Mat m, int row, int col, const V& value );

template < template < typename, unsigned int > typename F, typename R = void, class ...Args >
auto choose_method( int type );

template < int Type, typename V >
void assign( cv::Mat m, int row, int col, const V& value );

template < int Type > struct type_traits;

template <> struct type_traits< CV_8UC1 > { typedef unsigned char type; enum { channels = 1 }; };
template <> struct type_traits< CV_8UC2 > { typedef unsigned char type; enum { channels = 2 }; };
template <> struct type_traits< CV_8UC3 > { typedef unsigned char type; enum { channels = 3 }; };
template <> struct type_traits< CV_8UC4 > { typedef unsigned char type; enum { channels = 4 }; };

template <> struct type_traits< CV_8SC1 > { typedef char type; enum { channels = 1 }; };
template <> struct type_traits< CV_8SC2 > { typedef char type; enum { channels = 2 }; };
template <> struct type_traits< CV_8SC3 > { typedef char type; enum { channels = 3 }; };
template <> struct type_traits< CV_8SC4 > { typedef char type; enum { channels = 4 }; };

template <> struct type_traits< CV_16UC1 > { typedef comma::uint16 type; enum { channels = 1 }; };
template <> struct type_traits< CV_16UC2 > { typedef comma::uint16 type; enum { channels = 2 }; };
template <> struct type_traits< CV_16UC3 > { typedef comma::uint16 type; enum { channels = 3 }; };
template <> struct type_traits< CV_16UC4 > { typedef comma::uint16 type; enum { channels = 4 }; };

template <> struct type_traits< CV_16SC1 > { typedef comma::int16 type; enum { channels = 1 }; };
template <> struct type_traits< CV_16SC2 > { typedef comma::int16 type; enum { channels = 2 }; };
template <> struct type_traits< CV_16SC3 > { typedef comma::int16 type; enum { channels = 3 }; };
template <> struct type_traits< CV_16SC4 > { typedef comma::int16 type; enum { channels = 4 }; };

template <> struct type_traits< CV_32SC1 > { typedef comma::int32 type; enum { channels = 1 }; };
template <> struct type_traits< CV_32SC2 > { typedef comma::int32 type; enum { channels = 2 }; };
template <> struct type_traits< CV_32SC3 > { typedef comma::int32 type; enum { channels = 3 }; };
template <> struct type_traits< CV_32SC4 > { typedef comma::int32 type; enum { channels = 4 }; };

template <> struct type_traits< CV_32FC1 > { typedef float type; enum { channels = 1 }; };
template <> struct type_traits< CV_32FC2 > { typedef float type; enum { channels = 2 }; };
template <> struct type_traits< CV_32FC3 > { typedef float type; enum { channels = 3 }; };
template <> struct type_traits< CV_32FC4 > { typedef float type; enum { channels = 4 }; };

template <> struct type_traits< CV_64FC1 > { typedef double type; enum { channels = 1 }; };
template <> struct type_traits< CV_64FC2 > { typedef double type; enum { channels = 2 }; };
template <> struct type_traits< CV_64FC3 > { typedef double type; enum { channels = 3 }; };
template <> struct type_traits< CV_64FC4 > { typedef double type; enum { channels = 4 }; };

template < int Type > struct vector { typedef cv::Vec< typename type_traits< Type >::type, type_traits< Type >::channels > type; };

namespace impl {
    
template < int Type, unsigned int Channels > struct operations
{
    template < typename V > static void set( cv::Mat m, int row, int col, const V& value )
    {
        auto& pixel = m.at< typename cv_mat::vector< Type >::type >( row, col );
        //std::cerr << "--> a: row: " << row << " col: " << col << " values: ";
        //for( unsigned int i = 0; i < cv_mat::type_traits< Type >::channels; ++i ) { std::cerr << " " << value[i]; }
        //std::cerr << std::endl;
        for( unsigned int i = 0; i < cv_mat::type_traits< Type >::channels; ++i ) { pixel[i] = value[i]; }
    }
};

template < int Type > struct operations< Type, 1 > // quick and dirty
{
    template < typename V, unsigned int Size > static void set( cv::Mat m, int row, int col, const std::array< V, Size >& values ) { m.at< typename cv_mat::type_traits< Type >::type >( row, col ) = values[0]; }
    
    template < typename V > static void set( cv::Mat m, int row, int col, const std::vector< V >& values ) { m.at< typename cv_mat::type_traits< Type >::type >( row, col ) = values[0]; }
    
    template < typename V > static void set( cv::Mat m, int row, int col, const V& value ) { m.at< typename cv_mat::type_traits< Type >::type >( row, col ) = value; }
};

template < int Type, typename V > inline void set( cv::Mat m, int row, int col, const V& value ) { impl::operations< Type, cv_mat::type_traits< Type >::channels >::set( m, row, col, value ); }

} // namespace impl

template < typename V > inline void set( cv::Mat m, int row, int col, const V& value )
{
    if( col < 0 || col >= m.cols ) { return; }
    if( row < 0 || row >= m.rows ) { return; }
    switch( m.type())
    {
        case CV_8UC1: impl::set< CV_8UC1 >( m, row, col, value ); return;
        case CV_8UC2: impl::set< CV_8UC2 >( m, row, col, value ); return;
        case CV_8UC3: impl::set< CV_8UC3 >( m, row, col, value ); return;
        case CV_8UC4: impl::set< CV_8UC4 >( m, row, col, value ); return;
        
        case CV_8SC1: impl::set< CV_8SC1 >( m, row, col, value ); return;
        case CV_8SC2: impl::set< CV_8SC2 >( m, row, col, value ); return;
        case CV_8SC3: impl::set< CV_8SC3 >( m, row, col, value ); return;
        case CV_8SC4: impl::set< CV_8SC4 >( m, row, col, value ); return;
        
        case CV_16UC1: impl::set< CV_16UC1 >( m, row, col, value ); return;
        case CV_16UC2: impl::set< CV_16UC2 >( m, row, col, value ); return;
        case CV_16UC3: impl::set< CV_16UC3 >( m, row, col, value ); return;
        case CV_16UC4: impl::set< CV_16UC4 >( m, row, col, value ); return;
        
        case CV_16SC1: impl::set< CV_16SC1 >( m, row, col, value ); return;
        case CV_16SC2: impl::set< CV_16SC2 >( m, row, col, value ); return;
        case CV_16SC3: impl::set< CV_16SC3 >( m, row, col, value ); return;
        case CV_16SC4: impl::set< CV_16SC4 >( m, row, col, value ); return;
        
        case CV_32SC1: impl::set< CV_32SC1 >( m, row, col, value ); return;
        case CV_32SC2: impl::set< CV_32SC2 >( m, row, col, value ); return;
        case CV_32SC3: impl::set< CV_32SC3 >( m, row, col, value ); return;
        case CV_32SC4: impl::set< CV_32SC4 >( m, row, col, value ); return;
        
        case CV_32FC1: impl::set< CV_32FC1 >( m, row, col, value ); return;
        case CV_32FC2: impl::set< CV_32FC2 >( m, row, col, value ); return;
        case CV_32FC3: impl::set< CV_32FC3 >( m, row, col, value ); return;
        case CV_32FC4: impl::set< CV_32FC4 >( m, row, col, value ); return;
        
        case CV_64FC1: impl::set< CV_64FC1 >( m, row, col, value ); return;
        case CV_64FC2: impl::set< CV_64FC2 >( m, row, col, value ); return;
        case CV_64FC3: impl::set< CV_64FC3 >( m, row, col, value ); return;
        case CV_64FC4: impl::set< CV_64FC4 >( m, row, col, value ); return;
        
        default: COMMA_THROW( comma::exception, "expected image type; got: " << m.type() << ", which is not supported" );
    }
}

template < template < typename, unsigned int > typename F, typename R, class ...Args >
inline auto choose_method( int type )
{
    typedef std::function< R( Args... ) > function_t;
    switch( type )
    {
        case CV_8UC1: return function_t( F< unsigned char, 1 >() );
        case CV_8UC2: return function_t( F< unsigned char, 2 >() );
        case CV_8UC3: return function_t( F< unsigned char, 3 >() );
        case CV_8UC4: return function_t( F< unsigned char, 4 >() );

        case CV_8SC1: return function_t( F< char, 1 >() );
        case CV_8SC2: return function_t( F< char, 2 >() );
        case CV_8SC3: return function_t( F< char, 3 >() );
        case CV_8SC4: return function_t( F< char, 4 >() );
        
        case CV_16UC1: return function_t( F< std::uint16_t, 1 >() );
        case CV_16UC2: return function_t( F< std::uint16_t, 2 >() );
        case CV_16UC3: return function_t( F< std::uint16_t, 3 >() );
        case CV_16UC4: return function_t( F< std::uint16_t, 4 >() );

        case CV_16SC1: return function_t( F< std::int16_t, 1 >() );
        case CV_16SC2: return function_t( F< std::int16_t, 2 >() );
        case CV_16SC3: return function_t( F< std::int16_t, 3 >() );
        case CV_16SC4: return function_t( F< std::int16_t, 4 >() );

        case CV_32SC1: return function_t( F< std::int32_t, 1 >() );
        case CV_32SC2: return function_t( F< std::int32_t, 2 >() );
        case CV_32SC3: return function_t( F< std::int32_t, 3 >() );
        case CV_32SC4: return function_t( F< std::int32_t, 4 >() );

        case CV_32FC1: return function_t( F< float, 1 >() );
        case CV_32FC2: return function_t( F< float, 2 >() );
        case CV_32FC3: return function_t( F< float, 3 >() );
        case CV_32FC4: return function_t( F< float, 4 >() );

        case CV_64FC1: return function_t( F< double, 1 >() );
        case CV_64FC2: return function_t( F< double, 2 >() );
        case CV_64FC3: return function_t( F< double, 3 >() );
        case CV_64FC4: return function_t( F< double, 4 >() );
        
        default: COMMA_THROW( comma::exception, "expected image type; got: " << type << ", which is not supported" );
    }
}

} } // namespace snark { namespace cv_mat {
