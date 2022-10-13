// Copyright (c) 2017 The University of Sydney

#pragma once

#include <Eigen/Core>
#include "../../../math/interval.h"
#include "reader.h"
#include "types.h"
#include "../../block_buffer.h"

namespace snark { namespace graphics { namespace view {

class shape_reader_base : public Reader
{
public:
    void add_vertex( const vertex_t& v, unsigned int block );
    void add_label( const label_t& l, unsigned int block );
    void extent_hull( const snark::math::closed_interval< float, 3 >& x );
    void extent_hull( const Eigen::Vector3f& p );
protected:
    shape_reader_base( const reader_parameters& params, colored* c, const std::string& label, std::size_t shape_size );
    block_buffer< vertex_t > buffer_;
    block_buffer< label_t > labels_;
};

inline shape_reader_base::shape_reader_base( const reader_parameters& params, colored* c, const std::string& label, std::size_t shape_size )
    : Reader( params, c, label )
    , buffer_( size * shape_size )
    , labels_( size )
{
}

inline void shape_reader_base::add_vertex( const vertex_t& v, unsigned int block ) { buffer_.add( v, block ); }

inline void shape_reader_base::add_label( const label_t& l, unsigned int block ) { labels_.add( l, block ); }

inline void shape_reader_base::extent_hull( const snark::math::closed_interval< float, 3 >& x ) { m_extents = m_extents ? m_extents->hull( x ) : x; }

inline void shape_reader_base::extent_hull( const Eigen::Vector3f& p ) { m_extents = m_extents ? m_extents->hull( p ) : snark::math::closed_interval< float, 3 >( p ); }

} } } // namespace snark { namespace graphics { namespace view {
