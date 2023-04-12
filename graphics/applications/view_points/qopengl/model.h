// Copyright (c) 2017 The University of Sydney

/// @author Navid Pirmarzdashti

#pragma once

#include <memory>
#include "../../../qt5.5/qopengl/viewer_base.h"
#include "../../../qt5.5/qopengl/model.h"

namespace snark { namespace graphics { namespace view { namespace qopengl {

class model
{
    public:
        void load( const std::string& file_name );
        void add_shaders( snark::graphics::qopengl::viewer_base* viewer_base );
        void update_view();
        std::shared_ptr< snark::graphics::qopengl::mesh_shader > mesh_shader; // todo? make it unique_ptr? make it private?

    private:
        snark::graphics::qopengl::model _model;
};
    
} } } } // namespace snark { namespace graphics { namespace view { namespace qopengl {
    
