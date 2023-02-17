// Copyright (c) 2017 The University of Sydney

/// @author Navid Pirmarzdashti

#pragma once

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLFramebufferObject>
#include <QPainter>
#include <Eigen/Core>
#include <memory>

namespace snark { namespace graphics { namespace qopengl {

/// has a 3d point and its normalized position on texture plus texture size
struct label_vertex
{
    /// x,y,z of the corner point 
    Eigen::Vector3f position;
    /// normalized offset of other corners 0/1
    Eigen::Vector2f offset;
    /// size of the image texture in pixels
    Eigen::Vector2f texture_size;

    label_vertex(float x,float y,float z,float ox,float oy,float width, float height);
};
    
/// a billboard 2d texture, drawn at the apparent position of a 3d point
/// this class has the texture buffer for each label
class label : protected QOpenGLFunctions
{
    friend class label_shader;
public:
    label();
    virtual ~label();
protected:
    /// see labels::text for an example of implementation
    virtual void update()=0;
    /// update position vertex
    void update(float x,float y,float z);
    /// resize texture buffer
    void resize(int width,int height);
protected:
    virtual void draw(QPainter& painter)=0;
    virtual void init();
    virtual void paint();
    void draw();
    virtual void destroy();
    
    QOpenGLVertexArrayObject vao;
    QOpenGLBuffer vbo;
    std::vector<label_vertex> quad;
    std::unique_ptr<QOpenGLFramebufferObject> fbo;
    int width;
    int height;
};

/// manages shader program and has a vector of labels
/// e.g. each reader adds one shader to the widget and many labels to that shader
class label_shader : protected QOpenGLFunctions
{
public:
    label_shader();
    virtual ~label_shader() {}
    void clear();   //delete labels
    void update();  //init and update all added labels
    
    std::vector< std::shared_ptr< label > > labels; // todo? does it need to be shared_ptr at all?
    
    /// set to false to hide labels
    /// changing won't take effect until next redraw (call update to force it)
    bool visible;
    
    /// set to resize labels with according to their depth (need to be set for orthographic projection)
    /// changing won't take effect until next redraw (call update to force it)
    bool scaled;

protected:
    friend class widget;
    //GL context should be set and voa bound for these functions by caller (i.e. gl_widget)
    virtual void init();    //create texture buffer
    virtual void paint(const QMatrix4x4& projection_matrix, const QSize& size);  //invoke glDraw*
    virtual void destroy();   //destroy buffer
    
    QOpenGLShaderProgram program;
    int projection_matrix_location;
    int sampler_location;
    int screen_size_location;
    int scaled_location;
};
    
} } } // namespace snark { namespace graphics { namespace qopengl {
    
