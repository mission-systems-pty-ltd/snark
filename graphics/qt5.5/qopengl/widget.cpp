// Copyright (c) 2016 The University of Sydney

#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QPainter>
#include <math.h>
#include "widget.h"
#include "types.h"
#include <iostream>

namespace snark { namespace graphics { namespace qopengl {

// If you want a good explanation of the projection, model and view matrices
// used in the shader code (model and view combined in the mv_matrix) see
// http://www.opengl-tutorial.org/beginners-tutorials/tutorial-3-matrices/

// TODO: might be good (necessary) to make these shaders work with an older GLSL version. Perhaps 1.20.
static const char *vertex_shader_source = R"(
    #version 130
    in vec4 vertex;
    in vec4 color;
    in float point_size;
    out vec4 vert_color;
    uniform mat4 projection_matrix;
    uniform mat4 mv_matrix;
    void main() {
       vert_color = color;
       gl_Position = projection_matrix * mv_matrix * vertex;
       gl_PointSize=point_size;
    }
)";

static const char *fragment_shader_source = R"(
    #version 130
    in highp vec4 vert_color;
    out highp vec4 frag_color;
    void main() {
       frag_color = clamp( vert_color, 0.0, 1.0 );
    }
)";

widget::widget( const color_t& background_color, const qt3d::camera_options& camera_options, QWidget *parent )
    : QOpenGLWidget( parent )
    , program_( 0 )
    , _camera( camera_options.orthographic
             , camera_options.field_of_view
             , QVector3D( 0, 0, camera_options.z_is_up ? 1 : -1 ) )
             //, camera_options.center )
    , scene_radius( 10 )
    , background_color( background_color )
{
}

widget::~widget() { cleanup(); }

QSize widget::minimumSizeHint() const { return QSize( 50, 50 ); }

QSize widget::sizeHint() const { return QSize( 200, 200 ); }

void widget::cleanup()
{
    makeCurrent();
    //TODO refactor all shader types to have unifrom interface (pure virtual base); is order of paint important?
    for( auto& i : shapes ) { i->destroy(); }
    for( auto& i : label_shaders ) { i->destroy(); }
    for( auto& i : texture_shaders ) { i->destroy(); }
    for( auto& i : mesh_shaders ) { i->destroy(); }
    if( program_ ) { delete program_; program_ = nullptr; }
    doneCurrent();
}

void widget::begin_update() { makeCurrent(); }

void widget::end_update() { doneCurrent(); }

void widget::add_shape( const std::shared_ptr<shape>& shape ) { shapes.push_back(shape); }

void widget::add_label_shader( const std::shared_ptr<label_shader>& label_shader )
{
    label_shader->scaled = _camera.orthographic;
    label_shaders.push_back( label_shader );
}

void widget::add_texture_shader( const std::shared_ptr<texture_shader>& texture_shader ) { texture_shaders.push_back( texture_shader ); }

void widget::add_mesh_shader( const std::shared_ptr<mesh_shader>& mesh_shader ) { mesh_shaders.push_back( mesh_shader ); }

void widget::initializeGL()
{
    // In this example the widget's corresponding top-level window can change
    // several times during the widget's lifetime. Whenever this happens, the
    // QOpenGLWidget's associated context is destroyed and a new one is created.
    // Therefore we have to be prepared to clean up the resources on the
    // aboutToBeDestroyed() signal, instead of the destructor. The emission of
    // the signal will be followed by an invocation of initializeGL() where we
    // can recreate all resources.
    connect( context(), &QOpenGLContext::aboutToBeDestroyed, this, &widget::cleanup );

    initializeOpenGLFunctions();
    program_ = new QOpenGLShaderProgram();
    program_->addShaderFromSourceCode( QOpenGLShader::Vertex, vertex_shader_source );
    program_->addShaderFromSourceCode( QOpenGLShader::Fragment, fragment_shader_source );
    program_->bindAttributeLocation( "vertex", 0 );
    program_->bindAttributeLocation( "color", 1 );
    program_->bindAttributeLocation("point_size",2);
    program_->link();

    program_->bind();
    projection_matrix_location_ = program_->uniformLocation( "projection_matrix" );
    mv_matrix_location_ = program_->uniformLocation( "mv_matrix" );

    for( auto& i : shapes ) { i->init(); }
    for( auto& i : label_shaders ) { i->init(); }
    for( auto& i : texture_shaders ) { i->init(); }
    for( auto& i : mesh_shaders ) { i->init(); }
//     program_->release();

    init();
}

void widget::paintGL()
{
    QPainter painter( this );
    painter.beginNativePainting();

    glClearColor(background_color.red(),background_color.green(),background_color.blue(),background_color.alpha());
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    // this is needed for screen to 3d map e.g. mouse click
    glEnable( GL_DEPTH_TEST );
    glDepthFunc(GL_ALWAYS);
    glEnable( GL_BLEND );
    glBlendEquation( GL_FUNC_ADD );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
//     glEnable( GL_CULL_FACE );

//     QOpenGLVertexArrayObject::Binder binder(&(shapes[0]->vao));
    program_->bind();
    program_->setUniformValue( projection_matrix_location_, _camera.projection );
    program_->setUniformValue( mv_matrix_location_, _camera.camera * _camera.world );
    for( auto& i : shapes ) { if( i->visible ) { i->paint(); } }
//     glDisable( GL_DEPTH_TEST );

    program_->release();
    const auto& t = _camera.transform();
    for( auto& i : label_shaders ) { i->paint( t, size() ); }
    for( auto& i : texture_shaders ) { i->paint( t, size() ); }
    for( auto& i : mesh_shaders ) { i->paint( t, size() ); }
    painter.endNativePainting();
    painter.setPen( Qt::gray );
    painter.setFont( QFont( "Arial", 10 ));
    painter.drawText( rect()
                    , Qt::AlignRight | Qt::AlignBottom
                    , QString( "centre of rotation: %1 %2 %3" ).arg( _camera.center.x() )
                                                               .arg( _camera.center.y() )
                                                               .arg( _camera.center.z() ));
}

void widget::set_far_plane( float f )
{
    _camera.far_plane = f;
    _camera.update_projection();
    update();
}

void widget::resizeGL( int w, int h ) { _camera.update_projection( size() ); }

void widget::mousePressEvent( QMouseEvent *event ) { last_pos_ = event->pos(); }

void widget::mouseMoveEvent( QMouseEvent *event )
{
    float dx = event->x() - last_pos_.x();
    float dy = event->y() - last_pos_.y();
    if( event->buttons() & Qt::LeftButton )
    {
        _camera.pivot( dx, dy );
        update();
    }
    else if( event->buttons() & Qt::RightButton )
    {
        float factor = 1 / 500;
        double distance = _camera.distance();
        factor = ( distance > 5 ? ( 1.5 * distance ) : ( 0.5 * scene_radius ) ) / width(); // HACK for the case where the camera center has been moved by zooming in
        _camera.pan( factor * dx, -factor * dy );
        update();
    }
    last_pos_ = event->pos();
}

void widget::mouseDoubleClickEvent( QMouseEvent *event )
{
    if( event->button() == Qt::RightButton )
    {
        boost::optional< QVector3D > point = viewport_to_3d( event->pos() );
        double_right_click( point );
    }
    else if( event->button() == Qt::LeftButton )
    {
        boost::optional< QVector3D > point = viewport_to_3d( event->pos() );
        if( point ) { _camera.set_center( *point, false ); }
    }
}

void widget::wheelEvent( QWheelEvent *event )
{
    qreal distance = _camera.distance();
    const qreal coef = ( event->modifiers() & Qt::ShiftModifier ) ? 0.2 * scene_radius : qMax( distance, 0.2 * scene_radius );
    qreal zoomIncrement = 0.001 * event->delta() * coef;
    if( !qFuzzyIsNull( zoomIncrement ) ) { _camera.zoom( zoomIncrement ); }
    update();
}

static const unsigned int pixel_search_width = 15;

// Take a 2d point on the viewport and determine the corresponding 3d point in space.
// The 2d point has to correspond to a pixel (or be close to one).
boost::optional< QVector3D > widget::viewport_to_3d( const QPoint& viewport_point )
{
    boost::optional< QVector3D > pixel = pixel_at_point( viewport_point, pixel_search_width );
    if( pixel ) { return QVector3D( pixel->unproject( _camera.camera * _camera.world, _camera.projection , QRect( 0, 0, width(), height() ) ) ); }
    return boost::none;
}

// Take a viewport location and return the nearest active pixel within a square search area.
// Also converting from Qt coordinates (0,0 at top left) to OpenGL (0,0 at bottom left)
boost::optional< QVector3D > widget::pixel_at_point( const QPoint& viewport_point, int search_width )
{
    std::vector< float > depth( search_width * search_width );
    // Convert the coordinates to openGL, offset to corner of search,
    // and clamp to limits to ensure we don't search outside the valid range.
    int x_min = std::min( std::max( viewport_point.x() - search_width / 2, 0 ), width() - search_width );
    int y_min = std::min( std::max( height() - 1 - viewport_point.y() - search_width / 2, 0 ), height() - search_width );
    makeCurrent();
    glReadPixels( x_min, y_min, search_width, search_width, GL_DEPTH_COMPONENT, GL_FLOAT, depth.data() );
    doneCurrent();
    boost::optional< QVector3D > best_offset = pixel_nearest_centre( depth, search_width );
    if( best_offset ) { return *best_offset + QVector3D( x_min, y_min, 0 ); }
    return boost::none;
}

// Given a vector of pixel depths, representing a square search area,
// find the entry nearest the centre with a value less than 1.
// A value of 1 represents an empty pixel.
boost::optional< QVector3D > widget::pixel_nearest_centre( const std::vector< float >& depth, int search_width )
{
    // distance represents the distance of the pixel from the centre of the search area
    int best_distance = std::numeric_limits<int>::max();
    boost::optional< QVector3D > best_offset;
    for( int j = 0; j < search_width; j++ )
    {
        for( int i = 0; i < search_width; i++ )
        {
            float pixel_depth = depth[ j * search_width + i ];
            if( pixel_depth < 1.0f )
            {
                int distance = abs( i - search_width / 2 ) + abs( j - search_width / 2 );
                if( distance < best_distance )
                {
                    best_offset = QVector3D( i, j, pixel_depth );
                    best_distance = distance;
                }
            }
        }
    }
    return best_offset;
}

} } } // namespace snark { namespace graphics { namespace qopengl {
