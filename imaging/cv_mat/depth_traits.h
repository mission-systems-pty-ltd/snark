// Copyright (c) 2011 The University of Sydney

#pragma once

#include <opencv2/core/core.hpp>
#include <comma/base/types.h>

namespace snark { namespace cv_mat {

template < typename T> struct base_depth_traits
{
    typedef T value_t;
};

template < int Depth > struct depth_traits;

template <> struct depth_traits< CV_8U  > : public base_depth_traits< unsigned char >
{
    static double max_value() { return UCHAR_MAX; }
    static double min_value() { return 0; }
    static const int depth = CV_8U;
};

template <> struct depth_traits< CV_8S  > : public base_depth_traits< char >
{
    static double max_value() { return  SCHAR_MAX; }
    static double min_value() { return  SCHAR_MIN; }
    static const int depth = CV_8S;
};

template <> struct depth_traits< CV_16U > : public base_depth_traits< comma::uint16 >
{
    static double max_value() { return  USHRT_MAX; }
    static double min_value() { return  0; }
    static const int depth = CV_16U;
};

template <> struct depth_traits< CV_16S > : public base_depth_traits< comma::int16 >
{
    static double max_value() { return  SHRT_MAX; }
    static double min_value() { return  SHRT_MIN; }
    static const int depth = CV_16S;
};

template <> struct depth_traits< CV_32S > : public base_depth_traits< comma::int32 >
{
    static double max_value() { return  INT_MAX; }
    static double min_value() { return  INT_MIN; }
};

template <> struct depth_traits< CV_32F > : public base_depth_traits< float >
{
    static double max_value() { return  1; }
    static double min_value() { return  0; }
};

template <> struct depth_traits< CV_64F > : public base_depth_traits< double >
{
    static double max_value() { return  1; }
    static double min_value() { return  0; }
};

double min_value( int depth );

double max_value( int depth );

template< typename traits > struct traits_to_depth;
template<> struct traits_to_depth< depth_traits< CV_8U >::value_t > { enum { value = CV_8U }; };
template<> struct traits_to_depth< depth_traits< CV_8S >::value_t > { enum { value = CV_8S }; };
template<> struct traits_to_depth< depth_traits< CV_16U >::value_t > { enum { value = CV_16U }; };
template<> struct traits_to_depth< depth_traits< CV_16S >::value_t > { enum { value = CV_16S }; };
template<> struct traits_to_depth< depth_traits< CV_32S >::value_t > { enum { value = CV_32S }; };
template<> struct traits_to_depth< depth_traits< CV_32F >::value_t > { enum { value = CV_32F }; };
template<> struct traits_to_depth< depth_traits< CV_64F >::value_t > { enum { value = CV_64F }; };

} } // namespace snark { namespace cv_mat {
