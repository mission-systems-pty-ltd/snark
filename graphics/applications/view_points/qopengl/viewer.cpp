// Copyright (c) 2017 The University of Sydney
// Copyright (c) 2020 Vsevolod Vlaskine

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
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
        v.apply( "on-change", p.on_change );
        p.on_change = p.on_change || p.fps == 0;
    }

    template < typename Key, class Visitor > static void visit( Key, const snark::graphics::view::qopengl::viewer::grab::options_t& p, Visitor& v )
    {
        v.apply( "filename", p.filename );
        v.apply( "fps", p.fps );
        v.apply( "cols", p.cols );
        v.apply( "rows", p.rows );
        v.apply( "on-change", p.on_change );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace graphics { namespace view { namespace qopengl {

std::ostream& operator<<( std::ostream& os, const QVector3D& v ) { return os << v.x() << "," << v.y() << "," << v.z(); }

std::ostream& operator<<( std::ostream& os, const QVector4D& v ) { return os << v.x() << "," << v.y() << "," << v.z() << "," << v.w(); }

std::ostream& operator<<( std::ostream& os, const QMatrix4x4& v )
{ 
    os << v.row(0) << std::endl;
    os << v.row(1) << std::endl;
    os << v.row(2) << std::endl;
    os << v.row(3) << std::endl;
    return os;
}

static void _print_keys_help()
{
    std::cerr << "             'p'            : save screenshot to timestamped file" << std::endl;
    std::cerr << "             'ctrl+p'       : start/stop continuous screen grab (see --help for --grab)" << std::endl;
    std::cerr << "             'r'            : restore view to this camera configuration" << std::endl;
    std::cerr << "             'ctrl+r'       : restore next view" << std::endl;
    std::cerr << "             'shift+ctrl+r' : restore previous view" << std::endl;
    std::cerr << "             'v'            : store camera config" << std::endl;
    std::cerr << "             'alt-v'        : discard the oldest camera config" << std::endl;
    std::cerr << "             'shift-alt-v'  : discard the current camera config" << std::endl;
    std::cerr << "             'ctrl-v'       : output camera config to stdout" << std::endl;
}

viewer::viewer( controller_base* handler
              , const color_t& background_color
              , const viewer::camera::options& camera_options
              , const QVector3D& scene_center
              , double arg_scene_radius
              , const snark::graphics::view::click_mode& click_mode
              , const std::string& grab_options
              , QMainWindow* parent )
    : snark::graphics::qopengl::widget( background_color, camera_options, parent )
    , handler( handler )
    , scene_radius_fixed( false )
    , scene_center_fixed( false )
    , stdout_allowed( true )
    , click_mode( click_mode )
    , _scene_center( scene_center )
    , _grab( comma::name_value::parser( "filename", ';', '=', false ).get( grab_options, viewer::grab::options_t() ) )
    , _camera_options( camera_options )
{
    QTimer* timer = new QTimer( this );
    connect( timer, SIGNAL( timeout() ), this, SLOT( _on_timeout() ) );
    connect( &_camera_transition_timer, SIGNAL( timeout() ), this, SLOT( _on_camera_transition() ) );
    connect( &_grab._timer, SIGNAL( timeout() ), this, SLOT( _on_grab() ) );
    timer->start( 40 );
}

void viewer::reset_handler( controller_base* h ) { handler = h; }

void viewer::init() { if( handler != nullptr ) { handler->init(); } }

void viewer::_on_timeout() { if( handler != nullptr ) { handler->tick(); } }

void viewer::_on_camera_transition()
{
    ++_camera_transition_index;
    if( _camera_transition_index < _camera_transitions.size() )
    { 
        _camera = _camera_transitions[_camera_transition_index];
        update();
        return;
    }
    _camera_transitions.clear();
    _camera_transition_index = 0;
    _camera_transition_timer.stop();
}

void viewer::paintGL()
{
    widget::paintGL();
    _grab.once( this, true );
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

template < typename T > static void _write_json( const T& t, std::ostream& os, bool pretty )
{
    boost::property_tree::ptree p;
    comma::to_ptree to_ptree( p );
    comma::visiting::apply( to_ptree ).to( t );
    boost::property_tree::write_json( os, p, pretty );
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
            _camera_transition_timer.stop();
            _camera_transitions.clear();
            _camera_transition_index = 0;
            if( !_camera_bookmarks.empty() )
            {
                if( event->modifiers() == Qt::ControlModifier )
                {
                    ++_camera_bookmarks_index;
                    if( _camera_bookmarks_index >= _camera_bookmarks.size() ) { _camera_bookmarks_index = 0; }
                }
                else if( event->modifiers() == ( Qt::ControlModifier | Qt::ShiftModifier ) )
                {
                    _camera_bookmarks_index = ( _camera_bookmarks_index == 0 ? _camera_bookmarks.size() : _camera_bookmarks_index ) - 1;
                }
                std::cerr << "view-points: restoring camera position to camera configuration " << ( _camera_bookmarks_index + 1 ) << " of " << _camera_bookmarks.size() << " positions(s)..." << std::endl;
                _print_keys_help();
                // if( _camera_options.transitions.enabled )
                // {
                //     unsigned int size = _camera_options.transitions.size; // for brevity
                //     _camera_transitions.resize( size, _camera ); // quick and dirty for now
                //     _camera_transitions.back() = _camera_bookmarks[_camera_bookmarks_index];
                //     QVector3D p = _camera.get_position();
                //     QVector3D dp = ( _camera_transitions.back().get_position() - p ) / ( size - 1 );
                //     _camera_transitions[0] = _camera;
                //     QVector3D axis;
                //     float a;
                //     QQuaternion::fromRotationMatrix( ( _camera.world.inverted() * _camera_transitions.back().world ).toGenericMatrix< 3, 3 >() ).getAxisAndAngle( &axis, &a );
                //     auto dq = QQuaternion::fromAxisAndAngle( axis, a / ( size - 1 ) );
                //     for( unsigned int i = 1; i < size - 1; ++i ) // quick and dirty; implement using set_camera_position instead
                //     {
                //         p += dp;
                //         _camera_transitions[i] = _camera_transitions[i-1];
                //         _camera_transitions[i].set_center( _camera_transitions.back().center );
                //         _camera_transitions[i].set_position( p );
                //         _camera_transitions[i].world.translate( _camera_transitions.back().center );
                //         _camera_transitions[i].world.rotate( dq );
                //         _camera_transitions[i].world.translate( -_camera_transitions.back().center );
                //         _camera_transitions[i].update_projection();
                //     }
                //     _camera_transition_timer.start( _camera_options.transitions.duration * 1000 / size ); // _camera_transition_timer.start( 250 / size );
                // }
                if( _camera_options.transitions.enabled )
                {
                    unsigned int size = _camera_options.transitions.size == 0
                                      ? static_cast< unsigned int >( _camera_options.transitions.duration * 25 )
                                      : _camera_options.transitions.size;
                    _camera_transitions.resize( size, _camera ); // quick and dirty for now
                    _camera_transitions.back() = _camera_bookmarks[_camera_bookmarks_index];
                    QVector3D p = _camera.get_position();
                    QVector3D dp = ( _camera_transitions.back().get_position() - p ) / ( size - 1 );
                    QVector3D c = _camera_bookmarks[0].center;
                    _camera_transitions[0].center = c;
                    _camera_transitions.back().center = c;
                    QVector3D axis;
                    float a;
                    QQuaternion::fromRotationMatrix( ( _camera.world.inverted() * _camera_transitions.back().world ).toGenericMatrix< 3, 3 >() ).getAxisAndAngle( &axis, &a );
                    float da = a / ( size - 1 );
                    for( unsigned int i = 1; i < size - 1; ++i ) // todo! quick and dirty; still jumps if saved camera positions have different scene centers
                    {
                        p += dp;
                        _camera_transitions[i] = _camera_transitions[i-1];
                        _camera_transitions[i].world.translate( c );
                        _camera_transitions[i].world.rotate( da, axis );
                        _camera_transitions[i].world.translate( -c );
                        //_camera_transitions[i].set_center( _camera_transitions.back().center );
                        _camera_transitions[i].set_position( p );
                        _camera_transitions[i].update_projection();
                    }
                    _camera_transition_timer.start( _camera_options.transitions.duration * 1000 / size ); // _camera_transition_timer.start( 250 / size );
                }
                else
                {
                    _camera = _camera_bookmarks[_camera_bookmarks_index]; // todo: enable, make configurable
                }
            }
            break;
        case Qt::Key_V:
            if( event->modifiers() == Qt::NoModifier )
            {
                _camera_bookmarks.push_back( _camera );
                //_camera_bookmarks.back().center = _camera_bookmarks[0].center; // todo! a terrible hack to make transitions smoot
                std::cerr << "view-points: stored camera configuration; currently: " << _camera_bookmarks.size() << " saved camera configuration(s)" << std::endl;
                _print_keys_help();
            }
            else if( event->modifiers() == Qt::ControlModifier )
            {
                if( !stdout_allowed ) { std::cerr << "view-points: on ctrl+v: ignored since stdout is used by another stream; outputting to stderr instead:" << std::endl; }
                _write_json( _camera, stdout_allowed ? std::cout : std::cerr, false );
            }
            else if( event->modifiers() == Qt::AltModifier )
            {
                if( !_camera_bookmarks.empty() )
                {
                    _camera_bookmarks.pop_front();
                    if( _camera_bookmarks_index >= _camera_bookmarks.size() ) { _camera_bookmarks_index = _camera_bookmarks.empty() ? 0 : _camera_bookmarks.size() - 1; }
                    std::cerr << "view-points: removed first camera configuration; remaining: " << _camera_bookmarks.size() << " saved camera configuration(s)" << std::endl;
                    _print_keys_help();
                }
            }
            else if( event->modifiers() == ( Qt::AltModifier | Qt::ShiftModifier ) )
            {
                if( !_camera_bookmarks.empty() )
                {
                    unsigned int i{0};
                    for( auto it = _camera_bookmarks.begin(); it != _camera_bookmarks.end(); ++it, ++i ) { if( i == _camera_bookmarks_index ) { _camera_bookmarks.erase( it ); break; } }
                    if( _camera_bookmarks_index >= _camera_bookmarks.size() ) { _camera_bookmarks_index = _camera_bookmarks.size() - 1; }
                    std::cerr << "view-points: removed current camera configuration; remaining: " << _camera_bookmarks.size() << " saved camera configuration(s)" << std::endl;
                    _print_keys_help();
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
    if( !scene_center_fixed ) { _scene_center = 0.5 * ( min + max ); _camera.set_center( _scene_center ); }
    const auto& d = _camera.get_position() - _camera.center;
    //std::cerr<<"viewer::update_view "<<min<<" "<<max<<"; "<<scene_radius<<"; "<<scene_center<<std::endl;
    //update the position of the far plane so that the full scene is displayed
    //std::cerr << "--> scene_radius: " << scene_radius << " radius: " << radius << " far_plane: " << 4.6 * radius << std::endl;
    set_far_plane( 4.6 * ( Eigen::Vector3d( d.x(), d.y(), d.z() ).norm() + r ) ); // vodoo: 4.6
}

void viewer::look_at_center()
{
    //std::cerr << "==> viewer::look_at_center: center: " << _scene_center << "; scene radius: " << scene_radius << std::endl;
    //std::cerr << "==> a: world:" << std::endl << _camera.world << std::endl;
    _camera.set_center( _scene_center );
    _camera.set_orientation( 3 * M_PI / 4, -M_PI / 4, -M_PI / 4 );
    _camera.set_position( QVector3D( 0, 0, -2.6 * scene_radius ) ); // _camera.set_position( _scene_center + QVector3D( 0, 0, -2.6 * scene_radius ) );
    //std::cerr << "==> b: world:" << std::endl << _camera.world << std::endl;
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

void viewer::set_camera_position( const Eigen::Vector3d& position, const Eigen::Vector3d& rotation ) // todo?! move this method to camera.h/cpp?! orientation obliterates position?
{
    //std::cerr << "==> viewer::set_camera_position: " << std::setprecision(16) << position.transpose() << "; " << orientation.transpose() << std::endl;
    const Eigen::Vector3d& p = position - *m_offset;
    _camera.set_position( QVector3D( p.x(), p.y(), p.z() ), QVector3D( rotation.x(), rotation.y(), rotation.z() ), true );
}

// void viewer::set_camera_position( const Eigen::Vector3d& position, const Eigen::Vector3d& orientation ) // todo?! move this method to camera.h/cpp?! orientation obliterates position?
// {
//     //std::cerr << "==> viewer::set_camera_position: " << std::setprecision(16) << position.transpose() << "; " << orientation.transpose() << std::endl;
//     const Eigen::Vector3d& p = position - *m_offset;
//     const QVector3D& c = QVector3D( p.x(), p.y(), p.z() );
//     _camera.set_center( c ); // camera.set_center( c, true );
//     _camera.set_position( QVector3D( 0, 0, 0 ) ); // camera.set_position( c, true );
//     _camera.set_orientation( orientation.x(), orientation.y(), orientation.z(), true );
// }

void viewer::load_camera_config( const std::string& filename )
{
    auto camera_bookmarks = _camera_bookmarks;
    auto camera = _camera;
    _camera_bookmarks.clear();
    try
    {
        boost::property_tree::ptree camera_config;
        boost::property_tree::read_json( filename, camera_config );
        comma::from_ptree from_ptree( camera_config, true );
        comma::visiting::apply( from_ptree ).to( _camera );
        _camera_bookmarks.push_back( _camera ); // todo! quick and dirty; better usage semantics?
        _camera.update_projection( size() );
        std::cerr << "view-points: loaded camera config from " << filename << std::endl;
    }
    catch( ... )
    {
        try
        {
            std::ifstream ifs( filename );
            if( !ifs.is_open() ) { COMMA_THROW( comma::exception, "failed to open file: '" << filename << "'" ); }
            while( true )
            {
                std::string line;
                std::getline( ifs, line );
                if( line.empty() ) { break; }
                const auto& s = comma::strip( line );
                if( s.empty() || s[0] == '#' ) { continue; }
                std::istringstream iss( s );
                boost::property_tree::ptree camera_config;
                boost::property_tree::read_json( iss, camera_config );
                comma::from_ptree from_ptree( camera_config, true );
                graphics::qopengl::camera_transform camera = _camera; // auto camera = _camera;
                comma::visiting::apply( from_ptree ).to( camera );
                camera.update_projection( size() );
                _camera_bookmarks.push_back( camera ); // todo! quick and dirty; better usage semantics?
            }
            if( _camera_bookmarks.empty() ) { COMMA_THROW( comma::exception, "no camera configs found in '" << filename << "'" ); }
            std::cerr << "view-points: loaded " << _camera_bookmarks.size() << " camera config(s) from " << filename << std::endl;
        }
        catch( ... )
        {
            _camera_bookmarks = camera_bookmarks;
            _camera = camera;
            throw;
        }
    }
    _camera_bookmarks_index = 0;
    _camera = _camera_bookmarks.front();
    //_write_json( _camera, std::cerr, false );
    _print_keys_help();
}

void viewer::write_camera_config( std::ostream& os, bool on_change, bool pretty )
{
    if( on_change && previous_camera_ && _camera == *previous_camera_ ) { return; }
    previous_camera_ = _camera;
    _write_json( _camera, os, pretty );
}

void viewer::write_camera_position_( std::ostream& os, bool on_change )
{
    if( on_change && previous_camera_ && _camera.camera == previous_camera_->camera && _camera.world == previous_camera_->world ) { return; }
    previous_camera_ = _camera;
    const auto& position = _camera.get_position( true ); // todo: fix frame
    const auto& orientation = _camera.get_orientation( true );  // todo: fix frame
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
    if( !_options.on_change && _options.fps > 0 ) { _timer.start( 1000 / _options.fps ); }
    std::cerr << "view-points: started recording screen to " << ( _current_filename == "-" ? std::string( "stdout" ) : _current_filename ) << ( _options.on_change ? " on change" : "" );
    if( _options.fps > 0 ) { std::cerr << " at " << _options.fps << "fps"; }
    std::cerr << std::endl;
}

void viewer::grab::_close()
{
    _last = boost::posix_time::ptime();
    if( !_options.on_change ) { _timer.stop(); }
    if( _ostream )
    {
        std::cerr << "view-points: stopped recording screen to " << ( _current_filename == "-" ? std::string( "stdout" ) : _current_filename ) << std::endl;
        _ostream.reset();
    }
}

void viewer::grab::toggle() { if( _ostream ) { _close(); } else { _reopen(); } }

void viewer::_on_grab() { _grab.once( this, false ); } // todo? just bind it in grab?

// todo
// ? encode=png
// ? cv filters
// - output to stdout
// - configurable image size
// ? no-header
void viewer::grab::once( QOpenGLWidget* w, bool on_change )
{
    if( !_ostream ) { return; }
    auto t = boost::posix_time::microsec_clock::universal_time();
    if( on_change )
    {
        if( !_last.is_not_a_date_time() && t - _last < _period ) { return; }
        _last = t;
    }
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
