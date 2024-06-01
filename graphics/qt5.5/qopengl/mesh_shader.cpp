// Copyright (c) 2017 The University of Sydney

/// @author Navid Pirmarzdashti

#include <iostream>
#include <QOpenGLPaintDevice>
#include "mesh_shader.h"
#include "../../../math/rotation_matrix.h"

namespace snark { namespace graphics { namespace qopengl {

/// draw model
// static const char *mesh_shader_source = R"(
//     #version 130
//     in vec3 vertex;
//     in vec2 offset;
//     out vec2 textCoord;
//     uniform mat4 view_transform;
//     uniform mat4 model_transform;
//     void main() {
//        gl_Position = view_transform*model_transform*vec4(vertex,1) ;
//        textCoord = offset;
//     }
// )";
// 
// 
// static const char *mesh_fragment_source = R"(
//     #version 130
//     in vec2 textCoord;
//     uniform sampler2D sampler;
//     out highp vec4 frag_color;
//     void main() {
//        frag_color = texture2D(sampler,textCoord);
//     }
// )";

static const char *vertex_shader_source = R"(
    #version 130
    in vec4 vertex;
    uniform mat4 view_transform;
    uniform mat4 model_transform;
    void main() {
       gl_Position = view_transform* model_transform*vertex;
    }
)";

static const char *fragment_shader_source = R"(
    #version 130
    out highp vec4 frag_color;
    void main() {
       frag_color = vec4(1,1,1,1);
    }
)";

mesh_shader::mesh_shader() { model_transform.setToIdentity(); }

mesh_shader::~mesh_shader() { clear(); }

void mesh_shader::clear()
{
    destroy();
    meshes.clear();
}

void mesh_shader::init()
{
    //std::cerr<<"mesh_shader::init "<<std::endl;
    initializeOpenGLFunctions();
    program.addShaderFromSourceCode( QOpenGLShader::Vertex, vertex_shader_source );
    program.addShaderFromSourceCode( QOpenGLShader::Fragment, fragment_shader_source);
    program.bindAttributeLocation("vertex",0);
//     program.bindAttributeLocation("offset",1);
    program.link();
    program.bind();
    view_transform_location=program.uniformLocation("view_transform");
    model_transform_location=program.uniformLocation("model_transform");
//     sampler_location=program.uniformLocation("sampler");
    program.release();    
    for( auto& j : meshes ) { j->init(); }
}

void mesh_shader::paint(const QMatrix4x4& transform_matrix, const QSize& size)
{
    if(visible)
    {
        program.bind();
        program.setUniformValue(view_transform_location,transform_matrix);
        program.setUniformValue(model_transform_location,model_transform);
//         program.setUniformValue(sampler_location,0);
        glEnable(GL_DEPTH_TEST);
        //?disable back-face culling
        //?disable depth test
        for( auto& j : meshes ) { j->paint(); }
        program.release();
        glDisable(GL_DEPTH_TEST);
    }
}

void mesh_shader::destroy() { for( auto& i : meshes ) { i->destroy(); } }

void mesh_shader::update_transform( const Eigen::Vector3d& position, const Eigen::Vector3d& orientation )
{
    //std::cerr<<"mesh_shader::update_transform"<<std::endl;
    model_transform.setToIdentity();
    const Eigen::Quaterniond& q=snark::rotation_matrix(orientation).quaternion();
    model_transform.rotate(QQuaternion(q.w(),QVector3D(q.x(),q.y(),q.z())));
    model_transform.translate(QVector3D(position.x(),position.y(),position.z()));
}

mesh::mesh() : visible( true ), size( 0 ), _initialised( false ) {}

mesh::~mesh() {}

void mesh::init()
{
    if( _initialised ) { return; }
//     std::cerr<<"mesh::init"<<std::endl;
    initializeOpenGLFunctions();
    vao.create();
    QOpenGLVertexArrayObject::Binder binder( &vao );
    vbo.create();
    vbo.setUsagePattern( QOpenGLBuffer::StreamDraw );
    vbo.bind();
    glEnableVertexAttribArray( 0 );
//     glEnableVertexAttribArray( 1 );
    glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( mesh_vertex_t ), 0);
//     glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, sizeof( mesh_vertex ), reinterpret_cast<void *>( offsetof( mesh_vertex, offset )));
    vbo.release();
//     std::cerr<<"/mesh::init"<<std::endl;
    _initialised = true;
}
// void mesh::update(const mesh_data& data)
// void mesh::update(const vertex_t* data,unsigned sz)

void mesh::update( const mesh_vertex_t* data,unsigned sz )
{
    init();
//     std::cerr<<"mesh::update"<<std::endl;
    QOpenGLVertexArrayObject::Binder binder( &vao );
    vbo.bind();
    size = sz;
    vbo.allocate( size * sizeof( mesh_vertex_t ) );
    vbo.write( 0, data, size * sizeof( mesh_vertex_t ) );
    vbo.release();
//     std::cerr<<"/mesh::update"<<std::endl;
}

void mesh::paint()
{
//     std::cerr<<"mesh::paint"<<std::endl;
    QOpenGLVertexArrayObject::Binder binder(&vao);
    glDrawArrays( GL_POINTS, 0, size );
//     std::cerr<<"/mesh::paint"<<std::endl;
    /*if(visible)// && fbo)
    {
        glActiveTexture(GL_TEXTURE0);
//         glBindTexture(GL_TEXTURE_2D, fbo->mesh());
        
        QOpenGLVertexArrayObject::Binder binder(&vao);
        glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
        glDrawArrays(GL_TRIANGLES,0,size);
//         static int debug=0;
//         if(debug++<10)
//             std::cerr<<"mesh::paint"<<std::endl;
    }*/
}

// void mesh::draw()
// {
//     if(fbo)
//     {
//         fbo->bind();
//         QOpenGLPaintDevice paint_dev(width, height);
//         QPainter painter(&paint_dev);
//         draw(painter);
//         painter.end();
//         fbo->release();
// //         std::cerr<<"mesh::draw"<<std::endl;
//     }
// }

void mesh::destroy()
{
//     std::cerr<<"mesh::destroy"<<std::endl;
//     fbo.release();
//     std::cerr<<"mesh::destroy"<<std::endl;
}

} } } // namespace snark { namespace graphics { namespace qopengl {
