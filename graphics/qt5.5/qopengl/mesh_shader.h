// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2017 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/// @author Navid Pirmarzdashti

#pragma once

#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLFramebufferObject>
#include <vector>
#include <memory>
#include <QPainter>
#include <Eigen/Core>
#include <assimp/scene.h>

/*
 * TODO
 *  rename model* to mesh*, except for model importer
 *  add subclass model_mesh for rendering assimp model
 *      if possible, use pointer to vertex,normal etc directly and avoid copying or storing duplicate
 *          in that case importer is lifetime part of model
 *          rename model importer to model which also implements rendering?
*/

namespace snark { namespace graphics { namespace qopengl {

typedef aiVector3D mesh_vertex_t;

/// model vertex has a 3d point and its normalized texture position
struct mesh_data
{
    Eigen::Vector3f* vertices;
    Eigen::Vector3f* normals;
    unsigned size;
    unsigned int* faces;
    unsigned faces_size;
    mesh_data();
    //Eigen::Vector2f texture_coords
//     model_vertex(float x,float y,float z,float ox,float oy);
    // material index?
};
/*
 * x mesh_data -> mesh
 * x mesh -> model
 * x mesh_shader -> model_shader
 * x each model has one transform and multiple mesh
 * x each mesh has one material? -> material should go with vertex data
 *          that wouldn't work, 
 *      alt1 -> bind and run program for each transform
 *      alt2 -> one transform per mesh; user updates all meshes (may have their own node structure)
 *      *** alt3 -> apply transform to shader only, good trick for now! one shader per part if necessary as we need one reader to get pos,ori anyway
 */
/// a mesh paints vertices, normals and one material
/// each model consists of one or more meshes
class mesh : protected QOpenGLFunctions
{
public:
    mesh();
    virtual ~mesh();
    
    /// set to false to hide it
    bool visible;
    
    /// update data
//     void update(const mesh_data& data);
    void update( const mesh_vertex_t* data, unsigned size );
    
//     material m;
    
    void init();
    void paint();
    void destroy();
    
    QOpenGLVertexArrayObject vao;
    QOpenGLBuffer vbo;
    
    unsigned size;
    unsigned faces_size;
    
protected:
    friend class mesh_shader;
    bool initd;
//     std::unique_ptr<QOpenGLFramebufferObject> fbo;
};

/// mesh shader only keeps list of meshes to issue init and paint commands
/// each model will add several meshes to the mesh shader
class mesh_shader : protected QOpenGLFunctions
{
    friend class widget;
public:
    mesh_shader();
    virtual ~mesh_shader();
    void clear();   //delete labels
    
public:
    std::vector<std::shared_ptr<mesh>> meshes;
    bool visible;
    
    void update_transform(const Eigen::Vector3d& position,const Eigen::Vector3d& orientation);

protected:
    //GL context should be set by caller (i.e. gl_widget)
    virtual void init();    //create texture buffer
    virtual void paint(const QMatrix4x4& transform_matrix, const QSize& size);
    virtual void destroy();   //destroy buffer
protected:
    
    QOpenGLShaderProgram program;
    int view_transform_location;
//     int sampler_location;
    int model_transform_location;
    QMatrix4x4 model_transform;
};

} } } // namespace snark { namespace graphics { namespace qopengl {
    
