// Copyright (c) 2017 The University of Sydney

#include "shape_reader_base.h"

namespace snark { namespace graphics { namespace view {

shape_reader_base::shape_reader_base( const reader_parameters& params
                                    , colored* c
                                    , const std::string& label
                                    , std::size_t shape_size
                                    , std::size_t labels_per_instance )
    : Reader( params, c, label )
    , _buffer( size * shape_size )
    , _labels( size * labels_per_instance )
    , _axis_labels( comma::split( params.labels, ':' ) )
{
    _axis_labels.resize( 3 );
}

void shape_reader_base::add_vertex( const vertex_t& v, unsigned int block ) { _buffer.add( v, block ); }

void shape_reader_base::add_label( const label_t& l, unsigned int block ) { _labels.add( l, block ); }

void shape_reader_base::extent_hull( const snark::math::closed_interval< float, 3 >& x ) { m_extents = m_extents ? m_extents->hull( x ) : x; }

void shape_reader_base::extent_hull( const Eigen::Vector3f& p ) { m_extents = m_extents ? m_extents->hull( p ) : snark::math::closed_interval< float, 3 >( p ); }

} } } // namespace snark { namespace graphics { namespace view {
