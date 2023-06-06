// Copyright (c) 2011 The University of Sydney

/// @author vsevolod vlaskine

#pragma once

#include <boost/array.hpp>
#include <boost/functional/hash.hpp>
#include <boost/unordered_map.hpp>
#include <Eigen/Core>
#include <comma/base/types.h>
#include <comma/containers/multidimensional/map.h>

namespace comma { namespace impl {

template < typename T, std::size_t Size > struct array_traits< Eigen::Matrix< T, Size, 1 > >
{
    enum { size = Size };

    typedef Eigen::Matrix< T, Size, 1 > vector_t;

    static vector_t subtract( const vector_t& lhs, const vector_t& rhs ) { return lhs - rhs; }

    static vector_t divide( const vector_t& lhs, const vector_t& rhs ) { return vector_t( lhs.array() / rhs.array() ); }

    static vector_t zero() { return vector_t::Zero(); }
};

} } // namespace comma { namespace impl {

namespace snark {

// /// quick and dirty hash for array-like containers (its support is awkward in boost)
// /// @todo if we have a second use case, move to ark::containers... i guess...
// template < typename Array, std::size_t Size >
// struct array_hash : public std::unary_function< Array, std::size_t >
// {
//     std::size_t operator()( Array const& array ) const
//     {
//         std::size_t seed = 0;
//         for( std::size_t i = 0; i < Size; ++i ) { boost::hash_combine( seed, array[i] ); }
//         return seed;
//         // return boost::hash_range( &array[0], &array[Size] ); // not so easy...
//     }
// };

namespace impl {

template < typename T > struct vector_traits;

// template < typename T, std::size_t Size > struct vector_traits< Eigen::Matrix< T, Size, 1 > >
// {
//     enum { size = Size };

//     typedef Eigen::Matrix< T, Size, 1 > vector_t;

//     static vector_t subtract( const vector_t& lhs, const vector_t& rhs ) { return lhs - rhs; }

//     static vector_t divide( const vector_t& lhs, const vector_t& rhs ) { return vector_t( lhs.array() / rhs.array() ); }

//     static vector_t zero() { return vector_t::Zero(); }
// };

template <> struct vector_traits< Eigen::Vector2d > // todo: trivially generalize to Eigen::Matrix< T, Size, 1 >
{
    enum { size = 2 };

    typedef Eigen::Vector2d vector_t;

    static vector_t subtract( const vector_t& lhs, const vector_t& rhs ) { return lhs - rhs; }

    static vector_t divide( const vector_t& lhs, const vector_t& rhs ) { return vector_t( lhs.array() / rhs.array() ); }

    static vector_t zero() { return vector_t::Zero(); }
};

template <> struct vector_traits< Eigen::Vector3d >
{
    enum { size = 3 };

    typedef Eigen::Vector3d vector_t;

    static vector_t subtract( const vector_t& lhs, const vector_t& rhs ) { return lhs - rhs; }

    static vector_t divide( const vector_t& lhs, const vector_t& rhs ) { return vector_t( lhs.array() / rhs.array() ); }

    static vector_t zero() { return vector_t::Zero(); }
};

} // namespace impl {

template < typename V, unsigned int D, typename F = double, typename P = Eigen::Matrix< F, D, 1 > >
class voxel_map : public comma::containers::multidimensional::map< F, V, D, P, impl::vector_traits< P > >
{
    public:
        /// base class type
        typedef comma::containers::multidimensional::map< F, V, D, P, impl::vector_traits< P > > base_type;

        /// number of dimensions
        enum { dimensions = D };
        
        /// voxel type
        typedef V voxel_type;
        
        /// point type
        typedef P point_type;
        
        /// index type
        typedef typename base_type::index_type index_type;
        
        /// iterator type (otherwise it does not build on windows...)
        typedef typename base_type::iterator iterator;
        
        /// const iterator type (otherwise it does not build on windows...)
        typedef typename base_type::const_iterator const_iterator;

        /// constructor
        voxel_map( const point_type& origin, const point_type& resolution ): base_type( origin, resolution ) {}
        
        /// constructor for default origin of all zeroes
        voxel_map( const point_type& resolution ): base_type( resolution ) {}
};

} // namespace snark {
