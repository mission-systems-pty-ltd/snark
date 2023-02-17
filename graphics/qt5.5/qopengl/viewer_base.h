// Copyright (c) 2016 The University of Sydney

/// @author Navid Pirmarzdashti

#pragma once

#include "shapes.h"
#include "label_shader.h"
#include "texture_shader.h"
#include "mesh_shader.h"

namespace snark { namespace graphics { namespace qopengl {

/// this class is simplified interface of widget, passed to reader; each reader adds its required shaders to the widget (using this interface)
struct viewer_base
{
    virtual ~viewer_base() {}
    virtual void add_shape(const std::shared_ptr<shape>& shape)=0;
    virtual void add_label_shader(const std::shared_ptr<label_shader>& label_shader)=0;
    virtual void add_texture_shader(const std::shared_ptr<texture_shader>& texture_shader)=0;
    virtual void add_mesh_shader(const std::shared_ptr<mesh_shader>& mesh_shader)=0;
};

} } } // namespace snark { namespace graphics { namespace qopengl {
