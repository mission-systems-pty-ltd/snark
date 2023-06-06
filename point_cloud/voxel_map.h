// Copyright (c) 2011 The University of Sydney

/// @author vsevolod vlaskine

#pragma once

#include <boost/array.hpp>
#include <boost/functional/hash.hpp>
#include <boost/unordered_map.hpp>
#include <Eigen/Core>
#include <comma/base/types.h>
#include <comma/containers/multidimensional/map.h>

namespace snark {

template < typename V, unsigned int D, typename F = double, typename P = Eigen::Matrix< F, D, 1 > >//class voxel_map : public comma::containers::multidimensional::map< F, V, D, P, impl::vector_traits< P > >
class voxel_map : public comma::containers::multidimensional::map< F, V, D, P >
{
    public:
        /// base class type
        typedef comma::containers::multidimensional::map< F, V, D, P > base_type;

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
