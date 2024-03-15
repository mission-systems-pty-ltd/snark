// Copyright (c) 2024 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <array>
#include <cmath>
#include <cstring>

namespace snark {

template < typename T, unsigned int Dim, unsigned int Degree >
struct polynomial
{
    typedef T value_type;
    static constexpr unsigned int dim = Dim;
    static constexpr unsigned int degree = Degree;
    static constexpr unsigned int number_of_coefficients = 0; // todo: quick and dirty; generalise, it's easy

    std::array< T, number_of_coefficients > coef;

    polynomial() { std::memset( reinterpret_cast< char* >( &coef[0] ), 0, coef.size() * sizeof( T ) ); }
    T operator()( const std::array< T, dim >& rhs ) const;  // todo: quick and dirty; generalise, it's easy
};

template < typename T, unsigned int Degree >
struct polynomial< T, 2, Degree >
{
    static constexpr unsigned int dim = 2;
    static constexpr unsigned int degree = Degree;
    static constexpr unsigned int number_of_coefficients = ( degree + 1 ) * ( degree + 2 ) / 2;

    std::array< T, number_of_coefficients > coef;

    polynomial() { std::memset( reinterpret_cast< char* >( &coef[0] ), 0, coef.size() * sizeof( T ) ); }
    T operator()( const std::array< T, dim >& rhs ) const { return operator()( rhs[0], rhs[1] ); }
    T operator()( T x, T y ) const; // todo: generalise with variadic templates
};


template < typename T, unsigned int Degree >
inline T polynomial< T, 2, Degree >::operator()( T x, T y ) const // todo: generalise, it's easy
{
    static constexpr auto power = []( T t, unsigned int d )->T { T r{t}; for( unsigned int i{1}; i < d; r *= t, ++i ); return r; }; // todo: quick and dirty for now; unroll
    T r{0};
    for( unsigned int i{0}, k{0}; i < degree + 1; ++i ) // todo? optimise power? or let the compiler to do optimisation?
    {
        for( unsigned int j = 0; j < degree + 1 - i; r += power( x, i ) * power( y, j ) * coef[k++], ++j ); 
    }
    return r;
}

} // namespace snark {
