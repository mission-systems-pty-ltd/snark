// Copyright (c) 2011 The University of Sydney

/// @author Vsevolod Vlaskine, Cedric Wohlleber

#include <signal.h>
#ifndef Q_MOC_RUN
#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/thread/thread_time.hpp>
#include <comma/csv/stream.h>
#include <comma/name_value/ptree.h>
#include <comma/visiting/apply.h>
#endif
#include "viewer.h"
#include <QTimer>
#include "texture.h"
#include "../../../../math/rotation_matrix.h"

namespace snark { namespace graphics { namespace view {

void Viewer::load_camera_config(const std::string& file_name)
{
    boost::property_tree::ptree camera_config;
    boost::property_tree::read_json( file_name, camera_config );
    comma::from_ptree from_ptree( camera_config, true );
    comma::visiting::apply( from_ptree ).to( *camera() );
}
void Viewer::write_camera_config(std::ostream& os)
{
    boost::property_tree::ptree p;
    comma::to_ptree to_ptree( p );
    comma::visiting::apply( to_ptree ).to( *camera() );
    boost::property_tree::write_json( os, p );
}

Viewer::Viewer( const QColor4ub& background_color
              , const qt3d::camera_options& camera_options
              , bool exit_on_end_of_input
              , boost::optional< comma::csv::options > camera_csv, boost::optional< Eigen::Vector3d > cameraposition
              , boost::optional< Eigen::Vector3d > cameraorientation
              , const std::string& camera_config_file_name
              , boost::optional< Eigen::Vector3d > scene_center
              , boost::optional< double > scene_radius
              , bool output_camera_config )
    : qt3d::view( background_color
                , camera_options
                , scene_center ? boost::optional< QVector3D >( QVector3D( scene_center->x()
                                                                        , scene_center->y()
                                                                        , scene_center->z() ) )
                               : boost::optional< QVector3D >()
                , scene_radius )
    , m_lookAt( false )
    , m_cameraposition( cameraposition )
    , m_cameraorientation( cameraorientation )
    , m_stdout_allowed( true )
    , m_exit_on_end_of_input( exit_on_end_of_input )
    , output_camera_config(output_camera_config)
{
    QTimer* timer = new QTimer( this );
    timer->start( 40 );
    connect( timer, SIGNAL( timeout() ), this, SLOT( read() ) );
    if( camera_csv ) { m_cameraReader.reset( new CameraReader( *camera_csv ) ); }
    if( !camera_config_file_name.empty() )
    {
        load_camera_config(camera_config_file_name);
        //boost::property_tree::ptree p;
        //comma::to_ptree to_ptree( p );
        //comma::visiting::apply( to_ptree ).to( *camera() );
        //std::cerr << "view-points: camera set to:" << std::endl;
        //boost::property_tree::write_json( std::cerr, p );
    }
    m_cameraFixed = m_cameraposition || m_cameraReader || !camera_config_file_name.empty();
}

void Viewer::shutdown()
{
    m_shutdown = true;
    #ifdef WIN32
    ::raise( SIGTERM ); // according to windows docs, SIGINT does not work on windows
    #else // #ifdef WIN32
    ::raise( SIGINT ); // quick and dirty... or maybe not so dirty: to interrupt blocking read() in reader threads, if closed from qt window
    #endif // #ifdef WIN32
    if( m_cameraReader ) { m_cameraReader->shutdown(); }
    for( unsigned int i = 0; i < readers.size(); ++i ) { readers[i]->shutdown(); }
}
void Viewer::add(std::unique_ptr<snark::graphics::view::Reader>&& reader) { readers.push_back(std::move(reader)); }
void Viewer::initializeGL( QGLPainter *painter )
{
    (void) painter;
    ::glEnable( GL_BLEND );
    //::glEnable( GL_LIGHTING );
    //::glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
#if QT_VERSION < 0x050000
    qglClearColor( m_background_color.toColor() ); // TODO QGLView does not derive from QGLWidget anymore in Qt5, use straight OpenGL ?
#endif
    if( m_cameraReader ) { m_cameraReader->start(); }
    for( unsigned int i = 0; i < readers.size(); ++i ) { readers[i]->start(); }
}

void Viewer::read()
{
    for( unsigned int i = 0; !m_offset && i < readers.size(); ++i )
    {
        if( readers[i]->empty() ) { continue; }
        Eigen::Vector3d p = readers[i]->some_point();
        m_offset = std::fabs( p.x() ) > 1000 || std::fabs( p.y() ) > 1000 || std::fabs( p.z() ) > 1000 ? p : Eigen::Vector3d( 0, 0, 0 );
        std::cerr << "view-points: reader no. " << i << " scene offset (" << m_offset->transpose() << "); scene radius: " << scene_radius() << std::endl;
    }
    if( !m_offset ) { return; }
    bool need_update = false;
    for( unsigned int i = 0; i < readers.size(); ++i )
    {
        if( readers[i]->update( *m_offset ) > 0 ) { need_update = true; };
    }
    m_shutdown = true;
    bool ready_to_look = true;
    for( unsigned int i = 0; m_shutdown && i < readers.size(); ++i )
    {
        m_shutdown = m_shutdown && readers[i]->isShutdown();
        ready_to_look = ready_to_look && ( readers[i]->isShutdown() || ( readers.size() > 1 && readers[i]->is_stdin() ) );
    }
    if( !m_cameraReader && m_cameraposition )
    {
        set_camera_position( *m_cameraposition, *m_cameraorientation );
        m_cameraposition.reset();
        m_cameraorientation.reset();
    }
    else if( m_cameraReader )
    {
        Eigen::Vector3d position = m_cameraReader->position();
        Eigen::Vector3d orientation = m_cameraReader->orientation();
        if( !m_cameraposition || !m_cameraposition->isApprox( position ) || !m_cameraorientation->isApprox( orientation ) )
        {
            m_cameraposition = position;
            m_cameraorientation = orientation;
            set_camera_position( position, orientation );
        }
    }
    else if( readers[0]->m_extents && readers[0]->m_num_points > 0 && ( m_shutdown || ready_to_look || readers[0]->m_num_points >= readers[0]->size / 10 ) )
    {
        QVector3D min( readers[0]->m_extents->min().x(), readers[0]->m_extents->min().y(), readers[0]->m_extents->min().z() );
        QVector3D max( readers[0]->m_extents->max().x(), readers[0]->m_extents->max().y(), readers[0]->m_extents->max().z() );
        updateView( min, max );
        if( !m_cameraFixed && !m_lookAt )
        {
            m_lookAt = true;
            lookAtCenter();
        }
    }
    if( need_update ) { update(); }
    if( m_shutdown && m_exit_on_end_of_input ) { shutdown(); }
}

void Viewer::paintGL( QGLPainter *painter )
{
    for( unsigned int i = 0; i < readers.size(); ++i )
    {
        if( !readers[i]->show() ) { continue; }
        if( readers[i]->point_size > 1 ) { ::glEnable( GL_POINT_SMOOTH ); }
        ::glPointSize( readers[i]->point_size );
        readers[i]->render( *this, painter );
        if( readers[i]->point_size > 1 ) { ::glDisable( GL_POINT_SMOOTH ); }
    }
    draw_coordinates( painter );
    if( output_camera_config && m_stdout_allowed ) { write_camera_config(std::cout); }
}

void Viewer::set_camera_position ( const Eigen::Vector3d& position, const Eigen::Vector3d& orientation )
{
    Eigen::Vector3d p = position - *m_offset;
    camera()->setUpVector( QVector3D( 0, 0, m_z_up ? 1 : -1 ) );
    camera()->setEye( QVector3D( p.x(), p.y(), p.z() ) );
    double cos_yaw = std::cos( orientation.z() );
    double sin_yaw = std::sin( orientation.z() );
    double sin_pitch = std::sin( orientation.y() );
    double cos_pitch = std::cos( orientation.y() );
    Eigen::Vector3d direction( cos_pitch * cos_yaw, cos_pitch * sin_yaw, sin_pitch ); // todo: quick and dirty, forget about roll for now
    Eigen::Vector3d scene_center = p + direction * 50; // quick and dirty
    //Eigen::Vector3d where = p + direction;
    //camera()->setCenter( QVector3D( where.x(), where.y(), where.z() ) );
    //m_sceneCenter = QVector3D( scene_center.x(), scene_center.y(), scene_center.z() );
    m_sceneCenter = QVector3D( scene_center.x(), scene_center.y(), scene_center.z() );
    camera()->setCenter( QVector3D( m_sceneCenter ) );
}

void Viewer::mouse_double_right_click_event(  QMouseEvent *e )
{
    if( !m_stdout_allowed )
    {
        std::cerr << "point under mouse output is disabled when \"pass\" option is in use" << std::endl;
        return;
    }
    boost::optional< QVector3D > point = getPoint( e->pos() );
    if( !point ) { std::cerr << "warning: no point found near the double right click" << std::endl; return; }
    Eigen::Vector3d p( point->x(), point->y(), point->z() );
    if( !m_offset ) { std::cerr << "warning: offset is not defined yet, wait until it is found first" << std::endl; return; }
    p += *m_offset;
    std::cout << std::setprecision(16) << p.x() << "," << p.y() << "," << p.z() << std::endl;
}

void Viewer::draw_label( QGLPainter *painter, const Eigen::Vector3d& position, const QColor4ub& color, const std::string& label )
{
    if( label.empty() ) { return; }
    painter->modelViewMatrix().push();
    Eigen::Vector3d d = position;
    painter->modelViewMatrix().translate( QVector3D( d.x(), d.y(), d.z() ) ); // painter->modelViewMatrix().translate( position );
    QMatrix4x4 world = painter->modelViewMatrix().top();
    Eigen::Matrix3d R;
    R << world( 0, 0 ) , world( 0, 1 ), world( 0, 2 ),
         world( 1, 0 ) , world( 1, 1 ), world( 1, 2 ),
         world( 2, 0 ) , world( 2, 1 ), world( 2, 2 );
    R.transposeInPlace();
    snark::rotation_matrix rotation( R );
    Eigen::Quaterniond q = rotation.quaternion();
    painter->modelViewMatrix().rotate( QQuaternion( q.w(), q.x(), q.y(), q.z() ) );
    //painter->modelViewMatrix().translate( m_offset );
    double scale = 1.0 / double( height() );
    scale *= camera()->projectionType() == QGLCamera::Orthographic
           ? ( 0.25 * camera()->viewSize().width() )
           : ( 0.2 * Eigen::Vector3d( world( 0, 3 ) , world( 1, 3 ), world( 2, 3 ) ).norm() );
    painter->modelViewMatrix().scale( scale ); // TODO make size configurable ?
    drawText( painter, QString::fromUtf8( &label[0] ), color );
    painter->modelViewMatrix().pop();
}

void Viewer::drawText( QGLPainter *painter, const QString& string, const QColor4ub& color )
{
    Texture texture( string, color );
    texture.draw( painter );
}


} } } // namespace snark { namespace graphics { namespace view {
