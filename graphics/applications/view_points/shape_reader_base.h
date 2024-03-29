// Copyright (c) 2017 The University of Sydney

#pragma once

#include <string>
#include <vector>
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
    const block_buffer< vertex_t >& vertices() const { return _buffer; }
    const block_buffer< label_t >& labels() const { return _labels; }
    const std::vector< std::string >& axis_labels() const { return _axis_labels; }

protected:
    shape_reader_base( const reader_parameters& params
                     , colored* c
                     , const std::string& label
                     , std::size_t shape_size
                     , std::size_t labels_per_instance = 1 );
    block_buffer< vertex_t > _buffer;
    block_buffer< label_t > _labels;
    std::vector< std::string > _axis_labels;
    bool _labels_no_more_updates{false}; // uber-quick and dirty
};

} } } // namespace snark { namespace graphics { namespace view {
