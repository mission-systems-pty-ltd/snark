// Copyright (c) 2017 The University of Sydney

/// @author Navid Pirmarzdashti

#include <comma/base/exception.h>
#include "model.h"

namespace snark { namespace graphics { namespace view { namespace qopengl {

void model::load( const std::string& file_name ) { model_importer.import(file_name); }

void model::add_shaders( snark::graphics::qopengl::viewer_base* viewer_base )
{
    mesh_shader.reset(new snark::graphics::qopengl::mesh_shader);
    viewer_base->add_mesh_shader(mesh_shader);
}
void model::update_view() { if( mesh_shader->meshes.empty() ) { model_importer.make_meshes(*mesh_shader); } }
     
} } } } // namespace snark { namespace graphics { namespace view { namespace qopengl {
    
