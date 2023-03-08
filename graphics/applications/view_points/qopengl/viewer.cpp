// Copyright (c) 2017 The University of Sydney
// Copyright (c) 2020 Vsevolod Vlaskine

#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <QChar>
#include <QGuiApplication>
#include <QImageWriter>
#include <QTimer>
#include <QVector3D>
#include "../../../qt5.5/qopengl/traits.h"
#ifndef Q_MOC_RUN
#include <boost/property_tree/json_parser.hpp>
#include <comma/csv/format.h>
#include <comma/name_value/parser.h>
#include <comma/name_value/ptree.h>
#include <comma/visiting/apply.h>
#include <comma/visiting/traits.h>
#endif
#include "../../../../math/rotation_matrix.h"
#include "viewer.h"

namespace comma { namespace visiting {

template <> struct traits< snark::graphics::view::qopengl::viewer::grab::options_t >
{
    template < typename Key, class Visitor > static void visit( Key, snark::graphics::view::qopengl::viewer::grab::options_t& p, Visitor& v )
    {
        v.apply( "filename", p.filename );
        v.apply( "fps", p.fps );
        v.apply( "cols", p.cols );
        v.apply( "rows", p.rows );
    }

    template < typename Key, class Visitor > static void visit( Key, const snark::graphics::view::qopengl::viewer::grab::options_t& p, Visitor& v )
    {
        v.apply( "filename", p.filename );
        v.apply( "fps", p.fps );
        v.apply( "cols", p.cols );
        v.apply( "rows", p.rows );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace graphics { namespace view { namespace qopengl {

std::ostream& operator<<( std::ostream& os, const QVector3D& v ) { return os << v.x() << "," << v.y() << "," << v.z(); }

viewer::viewer( controller_base* handler
              , const color_t& background_color
              , const qt3d::camera_options& camera_options
              , const QVector3D& arg_scene_center
              , double arg_scene_radius
              , const snark::graphics::view::click_mode& click_mode
              , const std::string& grab_options
              , QMainWindow* parent )
    : snark::graphics::qopengl::widget( background_color, camera_options, parent )
    , scene_center( arg_scene_center )
    , handler( handler )
    , scene_radius_fixed( false )
    , scene_center_fixed( false )
    , stdout_allowed( true )
    , click_mode( click_mode )
    , _grab( comma::name_value::parser( "filename", ';', '=', false ).get( grab_options, viewer::grab::options_t() ) )
{
    if( _grab.options().cols > 0 )
    {
        resize( _grab.options().cols, _grab.options().rows );
        resizeGL( _grab.options().cols, _grab.options().rows );
    }
    scene_radius = arg_scene_radius;
    QTimer* timer = new QTimer( this );
    connect( timer, SIGNAL( timeout() ), this, SLOT( on_timeout() ) );
    timer->start( 40 );
}

void viewer::reset_handler( controller_base* h ) { handler = h; }

void viewer::init() { if( handler != nullptr ) { handler->init(); } }

void viewer::on_timeout() { if( handler != nullptr ) { handler->tick(); } }

void viewer::paintGL()
{
    widget::paintGL();
    _grab.once( this );
    if( output_camera_config && stdout_allowed ) { write_camera_config( std::cout, true, false ); }
    if( output_camera_position && stdout_allowed ) { write_camera_position_( std::cout, true ); }
    QPainter painter( this );
    painter.setPen( Qt::gray );
    painter.setFont( QFont( "Arial", 10 ) );
    painter.drawText( rect(), Qt::AlignLeft | Qt::AlignBottom, QString::fromStdString( click_mode.double_right_click.to_info_string() ) );
}

void viewer::double_right_click(const boost::optional< QVector3D >& point)
{
    if( !stdout_allowed ) { std::cerr << "view-points: point under mouse output is disabled when --pass option is in use" << std::endl; return; }
    if( !point ) { std::cerr << "view-points: warning: no point found near the double right click" << std::endl; return; }
    Eigen::Vector3d p( point->x(), point->y(), point->z() );
    if( !m_offset ) { std::cerr << "view-points: warning: offset is not defined yet, wait until it is found first" << std::endl; return; }
    p += *m_offset;
    std::cout << std::setprecision( 16 ) << p.x() << "," << p.y() << "," << p.z() << click_mode.double_right_click.to_output_string() << std::endl;
}

void viewer::keyPressEvent( QKeyEvent *event )
{
    click_mode.double_right_click.on_key_press( event );
    switch( event->key() )
    {
        case Qt::Key_P:
            if( event->modifiers() == Qt::NoModifier )
            {
                auto s = boost::posix_time::to_iso_string( boost::posix_time::microsec_clock::universal_time() );
                s += s.find( "." ) == std::string::npos ? ".000000.png" : ".png";
                QImageWriter( &s[0], "png" ).write( grabFramebuffer() );
                std::cerr << "view-points: screenshot saved in " << s << std::endl;
            }
            else if( event->modifiers() == Qt::ControlModifier )
            {
                _grab.toggle();
            }
            break;
        case Qt::Key_R:
            if( !_camera_bookmarks.empty() )
            {
                if( event->modifiers() == Qt::ControlModifier ) // todo: undocumented feature; finalise and document
                {
                    ++_camera_bookmarks_offset;
                    if( _camera_bookmarks_offset > _camera_bookmarks.size() ) { _camera_bookmarks_offset = 1; }
                }
                else if( event->modifiers() == ( Qt::ControlModifier | Qt::ShiftModifier ) ) // todo: undocumented feature; finalise and document
                {
                    --_camera_bookmarks_offset;
                    if( _camera_bookmarks_offset == 0 ) { _camera_bookmarks_offset = _camera_bookmarks.size(); }
                }
                std::cerr << "view-points: camera position restored to camera position " << ( _camera_bookmarks.size() - _camera_bookmarks_offset + 1 ) << " of " << _camera_bookmarks.size() << " positions(s)" << std::endl;
                camera = _camera_bookmarks[ _camera_bookmarks.size() - _camera_bookmarks_offset ];
            }
            break;
        case Qt::Key_V:
            if( event->modifiers() == Qt::NoModifier )
            {
                _camera_bookmarks.push_back( camera );
                std::cerr << "view-points: saved camera position; currently: " << _camera_bookmarks.size() << " saved camera position(s)" << std::endl;
                std::cerr << "             press 'r' to restore view to this camera position" << std::endl;
                std::cerr << "             press 'ctrl+r' to restore previos view" << std::endl;
                std::cerr << "             press 'ctrl+r' to restore next view" << std::endl;
            }
            else if( event->modifiers() == Qt::ControlModifier )
            {
                if( !_camera_bookmarks.empty() )
                {
                    _camera_bookmarks.pop_front();
                    std::cerr << "view-points: popped first saved camera position; currently: " << _camera_bookmarks.size() << " saved camera position(s)" << std::endl;
                    if( _camera_bookmarks_offset > _camera_bookmarks.size() + 1 ) { _camera_bookmarks_offset = 1; }
                }
            }
            break;
        default:
            break;
    }
    update();
}

void viewer::toggle_block_mode( bool flag ) { click_mode.double_right_click.toggle( snark::graphics::view::click_mode::double_right_click_t::modes::block, flag ); }

void viewer::toggle_label_mode( bool flag ) { click_mode.double_right_click.toggle( snark::graphics::view::click_mode::double_right_click_t::modes::label, flag ); }

void viewer::update_view(const QVector3D& min, const QVector3D& max)
{
    float r = 0.5 * ( max - min ).length();
    if( !scene_radius_fixed ) { scene_radius = r; }
    if( !scene_center_fixed ) { scene_center = 0.5 * ( min + max ); camera.set_center( scene_center ); }
    const auto& d = camera.get_position() - camera.center;
//     std::cerr<<"viewer::update_view "<<min<<" "<<max<<"; "<<scene_radius<<"; "<<scene_center<<std::endl;
//     update the position of the far plane so that the full scene is displayed
    //std::cerr << "--> scene_radius: " << scene_radius << " radius: " << radius << " far_plane: " << 4.6 * radius << std::endl;
    set_far_plane( 4.6 * ( Eigen::Vector3d( d.x(), d.y(), d.z() ).norm() + r ) ); // vodoo: 4.6
}

void viewer::look_at_center()
{
    //std::cerr<<"look_at_center "<<scene_center<<"; "<<scene_radius<<std::endl;
    camera.set_center(scene_center);
    camera.set_orientation(3*M_PI/4, -M_PI/4, -M_PI/4);
    camera.set_position(QVector3D(0,0,-2.6*scene_radius));
}

// void viewer::set_camera_position(const Eigen::Vector3d& position, const Eigen::Vector3d& orientation)
// {
//     Eigen::Vector3d p = position - *m_offset;
//     double cos_yaw = std::cos( orientation.z() );
//     double sin_yaw = std::sin( orientation.z() );
//     double sin_pitch = std::sin( orientation.y() );
//     double cos_pitch = std::cos( orientation.y() );
//     Eigen::Vector3d direction( cos_pitch * cos_yaw, cos_pitch * sin_yaw, sin_pitch ); // todo: quick and dirty, forget about roll for now
//     Eigen::Vector3d c = p + direction * 50; // quick and dirty
//     scene_center=QVector3D(c.x(),c.y(),c.z());
//     //to be tested
//     camera.set_center(scene_center);    //set center
// //     camera.set_orientation(orientation.x(),M_PI-orientation.y(),-M_PI/2-orientation.z());
//     camera.set_orientation(orientation.x(),orientation.y(),orientation.z());
//     camera.set_position(QVector3D(0,0,-p.norm()));    //camera is in 0,0,-z in world coordinate
// }

void viewer::set_camera_position( const Eigen::Vector3d& position, const Eigen::Vector3d& orientation ) // todo?! move this method to camera.h/cpp?! orientation obliterates position?
{
    //std::cerr << "==> viewer::set_camera_position: " << std::setprecision(16) << position.transpose() << "; " << orientation.transpose() << std::endl;
    const Eigen::Vector3d& p = position - *m_offset;
    const QVector3D& c = QVector3D( p.x(), p.y(), p.z() );
    camera.set_center( c ); // camera.set_center( c, true );
    camera.set_position( QVector3D( 0, 0, 0 ) ); // camera.set_position( c, true );
    camera.set_orientation( orientation.x(), orientation.y(), orientation.z(), true );
}

void viewer::load_camera_config(const std::string& file_name)
{
    boost::property_tree::ptree camera_config;
    boost::property_tree::read_json( file_name, camera_config );
    comma::from_ptree from_ptree( camera_config, true );
    comma::visiting::apply( from_ptree ).to( camera );
    _camera_bookmarks.push_back( camera ); // todo! quick and dirty; better usage semantics?
}

void viewer::write_camera_config( std::ostream& os, bool on_change, bool pretty )
{
    if( on_change && previous_camera_ && camera == *previous_camera_ ) { return; }
    previous_camera_ = camera;
    boost::property_tree::ptree p;
    comma::to_ptree to_ptree( p );
    comma::visiting::apply( to_ptree ).to( camera );
    boost::property_tree::write_json( os, p, pretty );
}

void viewer::write_camera_position_( std::ostream& os, bool on_change )
{
    if( on_change && previous_camera_ && camera.camera == previous_camera_->camera && camera.world == previous_camera_->world ) { return; }
    previous_camera_ = camera;
    const auto& position = camera.get_position( true );
    const auto& orientation = camera.get_orientation( true );
    os << std::setprecision( 16 ) << position.x() + m_offset->x() << ',' << position.y() + m_offset->y() << ',' << position.z() + m_offset->z() << ',' << orientation.x() << ',' << orientation.y() << ',' << orientation.z() << std::endl;
}

void viewer::grab::_reopen()
{
    _close();
    if( _options.filename.empty() )
    {
        _current_filename = boost::posix_time::to_iso_string( boost::posix_time::microsec_clock::universal_time() );
        _current_filename += _current_filename.find( "." ) == std::string::npos ? ".000000.bin" : ".bin";
    }
    else
    {
        _current_filename = _options.filename;
    }
    _ostream.reset( new comma::io::ostream( _current_filename ) );
    std::cerr << "view-points: started recording screen to " << _current_filename << " at " << _options.fps << "fps" << std::endl;
}

void viewer::grab::_close()
{
    _last = boost::posix_time::ptime();
    if( _ostream )
    {
        std::cerr << "view-points: stopped recording screen to " << _current_filename << std::endl;
        _ostream.reset();
    }
}

void viewer::grab::toggle() { if( _ostream ) { _close(); } else { _reopen(); } }

// todo
// ? encode=png
// ? cv filters
// - output to stdout
// - configurable image size
// ? no-header
void viewer::grab::once( QOpenGLWidget* w )
{
    if( !_ostream ) { return; }
    auto t = boost::posix_time::microsec_clock::universal_time();
    if( _last.is_not_a_date_time() && t - _last < _period ) { return; }
    _last = t;
    const auto& b = w->grabFramebuffer();
    unsigned int width( b.width() ), height( b.height() ), cv_type( 24 ); // uber-quick and dirty
    std::array< char, 8 > a;
    comma::csv::format::traits< boost::posix_time::ptime >::to_bin( t, &a[0] ); // todo? uber-quick and dirty; to avoid dependency on snark::imaging
    ( *_ostream )()->write( &a[0], a.size() ); // todo: put all in a buffer
    ( *_ostream )()->write( ( const char* )( &height ), sizeof( unsigned int ) );
    ( *_ostream )()->write( ( const char* )( &width ), sizeof( unsigned int ) );
    ( *_ostream )()->write( ( const char* )( &cv_type ), sizeof( unsigned int ) );
    ( *_ostream )()->write( ( const char* )( b.bits() ), width * height * 4 );
}

} } } } // namespace snark { namespace graphics { namespace view { namespace qopengl {
