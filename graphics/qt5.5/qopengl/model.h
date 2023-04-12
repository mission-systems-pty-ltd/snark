// Copyright (c) 2017 The University of Sydney

/// @author Navid Pirmarzdashti

#pragma once

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "mesh_shader.h"

namespace snark { namespace graphics { namespace qopengl {

/// import model files using ASSIMP library
/// creates and adds meshes to a mesh_shader
/// the mesh_shader then does the rendering, so all paint related code should be in mesh_shader and mesh classes
/// options: 
///     hold an array of shared ptr to meshes in case they need updating -> this is better for future animation
///     clear shader mesh array and add new ones every time we load model
class model
{
public:
    model();

    /// load model from file
    void load( const std::string& file_name );
    
    /// creates meshes from model and adds them to shader
    void make_meshes( mesh_shader& shader );
    
    void node_make_meshes( aiNode* node,mesh_shader& shader );
    
    void debug();
    
//     std::vector<int> vertex;
    
protected:
    Assimp::Importer _importer;
    const aiScene* _scene;
};
    
} } } // namespace snark { namespace graphics { namespace qopengl {
