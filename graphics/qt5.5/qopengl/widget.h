// Copyright (c) 2016 The University of Sydney

/// @author Navid Pirmarzdashti

#pragma once

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QMatrix4x4>
#include "../../qt3d/camera_options.h"
#include "shapes.h"
#include "label_shader.h"
#include "texture_shader.h"
#include <vector>
#include <memory>
#include "camera.h"
#include "viewer_base.h"

QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram)

namespace snark { namespace graphics { namespace qopengl {

class widget : public QOpenGLWidget, protected QOpenGLFunctions, public viewer_base
{
    Q_OBJECT

public:
    widget( const color_t& background_color, const qt3d::camera_options& camera_options, QWidget *parent = 0 );
    ~widget();

    QSize minimumSizeHint() const Q_DECL_OVERRIDE;
    QSize sizeHint() const Q_DECL_OVERRIDE;

    std::vector< std::shared_ptr< shape > > shapes;
    std::vector< std::shared_ptr< label_shader > > label_shaders;
    std::vector< std::shared_ptr< texture_shader > > texture_shaders;
    std::vector< std::shared_ptr< mesh_shader > > mesh_shaders;

    void begin_update();
    void end_update();

    //viewer_base
    virtual void add_shape(const std::shared_ptr<shape>& shape);
    virtual void add_label_shader(const std::shared_ptr<label_shader>& label_shader);
    virtual void add_texture_shader(const std::shared_ptr<texture_shader>& texture_shader);
    virtual void add_mesh_shader(const std::shared_ptr<mesh_shader>& mesh_shader);

public slots:
    void cleanup();

protected:
    // this will be called in initializeGL
    virtual void init() { }
    /// this will be called on mouse double click with right button if a 3d points is found/clicked
    virtual void double_right_click(const boost::optional<QVector3D>& point) { }

protected:
    void initializeGL() Q_DECL_OVERRIDE;
    void paintGL() Q_DECL_OVERRIDE;
    void resizeGL( int width, int height ) Q_DECL_OVERRIDE;
    void mousePressEvent( QMouseEvent *event ) Q_DECL_OVERRIDE;
    void mouseMoveEvent( QMouseEvent *event ) Q_DECL_OVERRIDE;
    void mouseDoubleClickEvent( QMouseEvent *event ) Q_DECL_OVERRIDE;
    void wheelEvent( QWheelEvent *event ) Q_DECL_OVERRIDE;
    void set_far_plane( float f ); // void set_near_plane(float near_plane);

protected:
    void update_projection();

    boost::optional< QVector3D > viewport_to_3d( const QPoint& point_2d );
    boost::optional< QVector3D > pixel_at_point( const QPoint& viewport_point, int search_width );
    boost::optional< QVector3D > pixel_nearest_centre( const std::vector< float >& depth, int search_width );

    QPoint last_pos_;
    QOpenGLShaderProgram *program_;
    int projection_matrix_location_;
    int mv_matrix_location_;
    camera_transform _camera;

public:
    float near_plane;
    float far_plane;
    double scene_radius;
    color_t background_color;
};

} } } // namespace snark { namespace graphics { namespace qopengl {
