// Copyright (c) 2011 The University of Sydney

#pragma once

#include <iomanip>
#include <sstream>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <comma/math/compare.h>
#include <comma/visiting/traits.h>

namespace snark { namespace render {

/// generic colour traits
template < typename T >
struct colour_traits
{
    typedef T numeric_type;
    static numeric_type min() { return 0; }
    static numeric_type max() { return 1; }
    // returns 8-bit value [0,255]
    static unsigned int value( numeric_type t ) { return t / static_cast< double >( max() ) * 255; }
};

/// colour traits for char
template <>
struct colour_traits< unsigned char >
{
    typedef unsigned int numeric_type;
    static numeric_type min() { return 0; }
    static numeric_type max() { return 255; }
    // returns 8-bit value [0,255]
    static unsigned int value( numeric_type t ) { return t; }
};

/// RGB + transparency colour type
template < typename T >
class colour : public boost::array< T, 4 >
{
    public:
        /// default constructor
        colour();

//         /// copy constructor
//         colour( const colour& rhs );

        /// constructor
        colour( T r, T g, T b, T a = colour_traits< T >::max() );

//         /// conversion constructor
//         template < typename S >
//         colour( const colour< S >& rhs );
//
//         /// assignment
//         const colour< T >& operator=( const colour& rhs );
//
//         /// conversion assignment
//         template < typename S >
//         const colour< T >& operator=( const colour< S >& rhs );

        /// convert
        template < typename S >
        S as() const;

        /// return red
        T red() const;

        /// return green
        T green() const;

        /// return blue
        T blue() const;

        /// return trasparency
        T alpha() const;

        /// @todo: return hue (HSV/HSB scheme)
        T hue() const;

        /// @todo: return saturation (HSV/HSB scheme)
        T saturation() const;

        /// @todo: return brightness (HSV/HSB scheme)
        T brightness() const;

        /// set red
        colour< T >& red( T t );

        /// set green
        colour< T >& green( T t );

        /// set blue
        colour< T >& blue( T t );

        /// set transparency
        colour< T >& alpha( T t );

        /// base class
        typedef boost::array< T, 4 > Base;

        /// operators
        /// @todo implement proper colour ariphmetics
        //bool operator==( const colour& rhs ) const { return boost::array< T, 4 >::operator==( rhs ); }
        //bool operator!=( const colour& rhs ) const { return !operator==( rhs ); }
        colour operator-() const { return colour( colour_traits< T >::max() - red(), colour_traits< T >::max() - green(), colour_traits< T >::max() - blue() ); }
        const colour& operator+=( const colour& rhs ) { red( red() + rhs.red() ); green( green() + rhs.green() ); blue( blue() + rhs.blue() ); alpha( ( alpha() + rhs.alpha() ) / 2 ); return *this; }
        const colour& operator*=( double d );
        const colour& operator-=( const colour& rhs ) { return operator+=( -rhs ); }
        const colour& operator/=( double d ) { return operator*=( 1 / d ); }
        colour operator+( const colour& rhs ) const { colour c( *this ); c += rhs; return c; }
        colour operator-( const colour& rhs ) const { colour c( *this ); c -= rhs; return c; }
        colour operator*( double d ) const { colour c( *this ); c *= d; return c; }
        colour operator/( double d ) const { colour c( *this ); c /= d; return c; }

        /// colour from string like 0x123456 or 0x12345678
        static colour< T > fromString( const std::string& rgba );
        std::string hex() const;
};

namespace impl {

template < typename T >
inline void validate( T t )
{
    if( comma::math::less( t, colour_traits< T >::min() ) || comma::math::less( colour_traits< T >::max(), t ) )
    {
        COMMA_THROW( comma::exception, "expected value in [" << colour_traits< T >::min() << ", " << colour_traits< T >::max() << "], got " << t );
    }
}

template < typename T > T fromString( const std::string& s ) { return boost::lexical_cast< T >( s ); }

template <>
inline unsigned char fromString< unsigned char >( const std::string& s ) // todo: check value
{
    unsigned char high = static_cast< unsigned char >( s.at( 0 ) );
    unsigned char low = static_cast< unsigned char >( s.at( 1 ) );
    return 16 * ( '0' <= high && high <= '9' ? high - '0' : 'a' <= high && high <= 'f' ? high - 'a' - 10 : high - 'A' - 10 )
                + '0' <= low && low <= '9' ? low - '0' : 'a' <= low && low <= 'f' ? low - 'a' - 10 : low - 'A' - 10;
}

} // namespace impl {

template <>
inline colour< unsigned char > colour< unsigned char >::fromString( const std::string& rgba )
{
    if( ( rgba.length() != 8 && rgba.length() != 10 ) || rgba.at( 0 ) != '0' || ( rgba.at( 1 ) != 'x' && rgba.at( 1 ) != 'X' ) ) { COMMA_THROW( comma::exception, "expected hex colour, got \"" << rgba << "\"" ); }
    colour< unsigned char > c( impl::fromString< unsigned char >( rgba.substr( 2 ) )
                            , impl::fromString< unsigned char >( rgba.substr( 4 ) )
                            , impl::fromString< unsigned char >( rgba.substr( 6 ) ) );
    if( rgba.length() == 10 ) { c.alpha( impl::fromString< unsigned char >( rgba.substr( 8 ) ) ); }
    return c;
}

template< typename T >
std::string colour< T >::hex() const
{
    std::stringstream ss;
    ss  << '#'
        << std::hex << std::setfill('0') << std::uppercase
        << std::setw(2) << colour_traits< T >::value( red() )
        << std::setw(2) << colour_traits< T >::value( green() )
        << std::setw(2) << colour_traits< T >::value( blue() );
    return ss.str();
}

template < typename T >
inline colour< T >::colour() {}

// template < typename T >
// inline colour< T >::colour( const colour& rhs ) { operator=( rhs ); }

template < typename T >
inline colour< T >::colour( T r, T g, T b, T a )
{
    red( r );
    green( g );
    blue( b );
    alpha( a );
}

// template < typename T >
// template < typename S >
// inline colour< T >::colour( const colour< S >& rhs ) { operator=( rhs ); }
//
// template < typename T >
// inline const colour< T >& colour< T >::operator=( const colour& rhs )
// {
//     this->comma::Point< T, 4 >::operator=( rhs );
//     return *this;
// }
//
// template < typename T >
// template < typename S >
// inline const colour< T >& colour< T >::operator=( const colour< S >& rhs )
// {
//     for( unsigned int i = 0; i < Base::Dimension; ++i )
//     {
//         this->operator[]( i ) = static_cast< T >( ( ( colour_traits< T >::max() - colour_traits< T >::min() ) * rhs.colour< S >::Base::operator[]( i ) ) / ( colour_traits< S >::max() - colour_traits< S >::min() ) );
//     }
//     return *this;
// }

template < typename T >
template < typename S >
inline S colour< T >::as() const
{
    typedef typename S::Type Type;
    static const Type fs = colour_traits< Type >::max() - colour_traits< Type >::min();
    static const Type ft = colour_traits< T >::max() - colour_traits< T >::min();
    return S( static_cast< Type >( ( float( this->red() ) / ft ) * fs )
            , static_cast< Type >( ( float( this->green() ) / ft ) * fs )
            , static_cast< Type >( ( float( this->blue() ) / ft ) * fs )
            , static_cast< Type >( ( float( this->alpha() ) / ft ) * fs ) );
}

template < typename T >
inline T colour< T >::red() const { return this->operator[]( 0 ); } // a quick fix

template < typename T >
inline T colour< T >::green() const { return this->operator[]( 1 ); }

template < typename T >
inline T colour< T >::blue() const { return this->operator[]( 2 ); }

template < typename T >
inline T colour< T >::alpha() const { return this->operator[]( 3 ); }

template < typename T >
inline colour< T >& colour< T >::red( T t )
{
    impl::validate( t );
    this->operator[]( 0 ) = t;
    return *this;
}

template < typename T >
inline colour< T >& colour< T >::green( T t )
{
    impl::validate( t );
    this->operator[]( 1 ) = t;
    return *this;
}

template < typename T >
inline colour< T >& colour< T >::blue( T t )
{
    impl::validate( t );
    this->operator[]( 2 ) = t;
    return *this;
}

template < typename T >
inline colour< T >& colour< T >::alpha( T t )
{
    impl::validate( t );
    this->operator[]( 3 ) = t;
    return *this;
}

template < typename T >
const colour< T >& colour< T >::operator*=( double d ) // todo: quick and dirty now
{
    float f = static_cast< float> ( d );
    red( red() * f );
    green( green() * f );
    blue( blue() * f );
    alpha( alpha() * f );
    return *this;
}

template < typename T >
T colour< T >::hue() const
{
    // todo
}

template < typename T >
T colour< T >::saturation() const
{
    // todo
}

template < typename T >
T colour< T >::brightness() const
{
    // todo
}

} } // namespace snark { namespace render {
