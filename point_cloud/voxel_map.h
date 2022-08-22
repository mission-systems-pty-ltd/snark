// Copyright (c) 2011 The University of Sydney

/// @author vsevolod vlaskine

#pragma once

#include <boost/array.hpp>
#include <boost/functional/hash.hpp>
#include <boost/unordered_map.hpp>
#include <Eigen/Core>
#include <comma/base/types.h>

namespace snark {

/// quick and dirty hash for array-like containers (its support is awkward in boost)
/// @todo if we have a second use case, move to ark::containers... i guess...
template < typename Array, std::size_t Size >
struct array_hash : public std::unary_function< Array, std::size_t >
{
    std::size_t operator()( Array const& array ) const
    {
        std::size_t seed = 0;
        for( std::size_t i = 0; i < Size; ++i ) { boost::hash_combine( seed, array[i] ); }
        return seed;
        // return boost::hash_range( &array[0], &array[Size] ); // not so easy...
    }
};

/// unordered voxel map
///
/// it may be a much better choice than voxel grid, whenever
/// operations on a voxel do not depend on its neighbours,
/// and also when the grid extents are not known beforehand
template < typename V, unsigned int D, typename F = double, typename P = Eigen::Matrix< F, D, 1 > >
class voxel_map : public boost::unordered_map< boost::array< comma::int32, D >, V, snark::array_hash< boost::array< comma::int32, D >, D > >
{
    public:
        /// number of dimensions
        enum { dimensions = D };
        
        /// voxel type
        typedef V voxel_type;
        
        /// point type
        typedef P point_type;
        
        /// index type
        typedef boost::array< comma::int32, D > index_type;
        
        /// base class type
        typedef boost::unordered_map< index_type, voxel_type, snark::array_hash< index_type, D > > base_type;
        
        /// iterator type (otherwise it does not build on windows...)
        typedef typename base_type::iterator iterator;
        
        /// const iterator type (otherwise it does not build on windows...)
        typedef typename base_type::const_iterator const_iterator;

        /// constructor
        voxel_map( const point_type& origin, const point_type& resolution );
        
        /// constructor for default origin of all zeroes
        voxel_map( const point_type& resolution );
        
        /// add voxel at the given point, if it does not exist
        iterator touch_at( const point_type& point );
        
        /// add voxel at the given point, if it does not exist
        std::pair< iterator, bool > insert( const point_type& point, const voxel_type& voxel );
        
        /// return index of the point, always rounds it down (does floor for given resolution)
        index_type index_of( const point_type& point ) const;
        
        /// same as index_of( point ), but static
        static index_type index_of( const point_type& point, const point_type& origin, const point_type& resolution );
        
        /// same as index_of( point ), but static
        static index_type index_of( const point_type& point, const point_type& resolution );
        
        /// find voxel by point
        iterator find( const point_type& point );
        
        /// find voxel by point
        const_iterator find( const point_type& point ) const;
        
        /// find voxel by index
        iterator find( const index_type& index );
        
        /// find voxel by index
        const_iterator find( const index_type& index ) const;
        
        /// return origin
        const point_type& origin() const;
        
        /// return resolution
        const point_type& resolution() const;

    private:
        point_type origin_;
        point_type resolution_;
};

template < typename V, unsigned int D, typename F, typename P >
inline voxel_map< V, D, F, P >::voxel_map( const typename voxel_map< V, D, F, P >::point_type& origin, const typename voxel_map< V, D, F, P >::point_type& resolution )
    : origin_( origin )
    , resolution_( resolution )
{
}

template < typename V, unsigned int D, typename F, typename P >
inline voxel_map< V, D, F, P >::voxel_map( const typename voxel_map< V, D, F, P >::point_type& resolution )
    : origin_( point_type::Zero() ) // todo: use traits, if decoupling from eigen required
    , resolution_( resolution )
{
}

template < typename V, unsigned int D, typename F, typename P >
inline typename voxel_map< V, D, F, P >::iterator voxel_map< V, D, F, P >::touch_at( const typename voxel_map< V, D, F, P >::point_type& point )
{
    index_type index = index_of( point );
    iterator it = this->base_type::find( index );
    if( it != this->end() ) { return it; }
    return this->base_type::insert( std::make_pair( index, voxel_type() ) ).first;
}

template < typename V, unsigned int D, typename F, typename P >
inline std::pair< typename voxel_map< V, D, F, P >::iterator, bool > voxel_map< V, D, F, P >::insert( const typename voxel_map< V, D, F, P >::point_type& point, const typename voxel_map< V, D, F, P >::voxel_type& voxel )
{
    return this->base_type::insert( std::make_pair( index_of( point ), voxel ) );
}

namespace impl {

static int negative_flooring_ = static_cast< int >( -1.5 ) == -1 ? -1 : static_cast< int >( -1.5 ) == -2 ? 0 : 0;
static int positive_flooring_ = static_cast< int >( 1.5 ) == 1 ? 0 : static_cast< int >( 1.5 ) == 2 ? -1 : -1;

} // namespace impl {

template < typename V, unsigned int D, typename F, typename P >
inline typename voxel_map< V, D, F, P >::index_type voxel_map< V, D, F, P >::index_of( const typename voxel_map< V, D, F, P >::point_type& point, const typename voxel_map< V, D, F, P >::point_type& origin, const typename voxel_map< V, D, F, P >::point_type& resolution )
{
    point_type diff = ( point - origin ).array() / resolution.array();
    index_type index;
    for( unsigned int i = 0; i < dimensions; ++i )
    {
        int d = diff[i];
        index[i] = d;
        if( diff[i] == d ) { continue; }
        index[i] += diff[i] < 0 ? impl::negative_flooring_ : ( d == 0 ? 0 : impl::positive_flooring_ );
    }
    return index;
}

template < typename V, unsigned int D, typename F, typename P >
inline typename voxel_map< V, D, F, P >::index_type voxel_map< V, D, F, P >::index_of( const typename voxel_map< V, D, F, P >::point_type& point, const typename voxel_map< V, D, F, P >::point_type& resolution )
{
    return index_of( point, point_type::Zero(), resolution );
}

template < typename V, unsigned int D, typename F, typename P >
inline typename voxel_map< V, D, F, P >::index_type voxel_map< V, D, F, P >::index_of( const typename voxel_map< V, D, F, P >::point_type& point ) const
{
    return index_of( point, origin_, resolution_ );
}

template < typename V, unsigned int D, typename F, typename P >
inline typename voxel_map< V, D, F, P >::iterator voxel_map< V, D, F, P >::find( const typename voxel_map< V, D, F, P >::point_type& point )
{
    index_type i = index_of( point );
    return this->base_type::find( i );
}

template < typename V, unsigned int D, typename F, typename P >
inline typename voxel_map< V, D, F, P >::const_iterator voxel_map< V, D, F, P >::find( const typename voxel_map< V, D, F, P >::point_type& point ) const
{
    index_type i = index_of( point );
    return this->base_type::find( i );
}

template < typename V, unsigned int D, typename F, typename P >
inline typename voxel_map< V, D, F, P >::iterator voxel_map< V, D, F, P >::find( const typename voxel_map< V, D, F, P >::index_type& index )
{
    return this->base_type::find( index ); // otherwise strange things happen... debug, when we have time
}

template < typename V, unsigned int D, typename F, typename P >
inline typename voxel_map< V, D, F, P >::const_iterator voxel_map< V, D, F, P >::find( const typename voxel_map< V, D, F, P >::index_type& index ) const
{
    return this->base_type::find( index ); // otherwise strange things happen... debug, when we have time
}

template < typename V, unsigned int D, typename F, typename P >
inline const typename voxel_map< V, D, F, P >::point_type& voxel_map< V, D, F, P >::origin() const { return origin_; }

template < typename V, unsigned int D, typename F, typename P >
inline const typename voxel_map< V, D, F, P >::point_type& voxel_map< V, D, F, P >::resolution() const { return resolution_; }

} // namespace snark {
