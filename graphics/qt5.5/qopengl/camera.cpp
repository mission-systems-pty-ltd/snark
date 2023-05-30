// Copyright (c) 2017 The University of Sydney

/// @author Navid Pirmarzdashti

#include <cmath>
#include <iomanip>
#include <iostream>
#include <tuple>
#include "../../../math/rotation_matrix.h"
#include "camera.h"

namespace snark { namespace graphics { namespace qopengl {

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

camera_transform::camera_transform( bool orthographic
                                  , double field_of_view
                                  , const QVector3D& up
                                  , const QVector3D& c )
                                  //, float z )
    : center( c )
{
    _projection.up = up;
    _projection.orthographic = orthographic;
    //_projection.near_plane = near_plane;
    //_projection.far_plane = far_plane;
    _projection.field_of_view = field_of_view;
    // The camera always points along the z-axis. Pan moves the camera in x,y
    // coordinates and zoom moves in and out on the z-axis.
    // It starts at -1 because in OpenGL-land the transform is actually applied
    // to the world and the camera is stationary at 0,0,0.
    camera.setToIdentity();
    camera.translate( 0, 0, -1 );
    world.setToIdentity();
    world.translate( -center );
}

camera_transform camera_transform::make( const camera_transform::config& config, bool from_ned )
{
    camera_transform camera( config.projection.orthographic, config.projection.field_of_view, config.projection.up, config.center );
    camera._projection = config.projection; // quick and dirty
    camera.set_world( config.world.translation, config.world.rotation, from_ned );
    camera.set_position( config.camera.translation, from_ned );
    // camera.set_camera_rotation( config.camera.rotation, from_ned ); // todo!
    camera.update_projection();
    return camera;
}

camera_transform::config camera_transform::to_config( bool to_ned ) const
{
    camera_transform::config c;
    c.projection = _projection;
    c.center = center;
    std::tie( c.world.translation, c.world.rotation ) = get_world( to_ned );
    c.camera.translation = get_position( to_ned );
    return c;
}

void camera_transform::pan( float dx, float dy ) { camera.translate( dx, dy, 0 ); }

void camera_transform::zoom( float dz )
{
    camera.translate( 0, 0, dz );
    if( _projection.orthographic ) { update_projection(); }
}

void camera_transform::pivot( float dx,float dy )
{
    world.translate( center );
    QMatrix4x4 inverted_world = world.inverted();
    QVector4D x_axis = inverted_world * QVector4D( 1, 0, 0, 0 );
    QVector4D y_axis = inverted_world * QVector4D( 0, 1, 0, 0 );
    world.rotate( dy, x_axis.toVector3D() );
    world.rotate( dx, y_axis.toVector3D() );
    world.translate( -center );
}

QMatrix4x4 camera_transform::transform() const { return projection * camera * world; }

//static QVector3D _from_ned( const QVector3D& v ) { return -QVector3D( v.y(), -v.z(), -v.x() ); } // quick and dirty: north-east-down -> east-up-south camera -> west-down-north world
// static QVector3D _from_ned( const QVector3D& v ) { return -QVector3D( v.y(), -v.z(), -v.x() ); } // quick and dirty: north-east-down -> east-up-south camera -> west-down-north world
// static QVector3D _to_ned( const QVector3D& v ) { return -QVector3D( -v.z(), v.x(), -v.y() ); } // quick and dirty: north-east-down <- east-up-south camera <- west-down-north world

static QVector3D _from_ned( const QVector3D& v ) { return QVector3D( -v.y(), v.z(), v.x() ); }
static QVector3D _to_ned( const QVector3D& v ) { return QVector3D( v.z(), -v.x(), v.y() ); }

void camera_transform::set( const QVector3D& center
                          , const QVector3D& position
                          , const QVector3D& world_position
                          , const QVector3D& world_orientation
                          , bool from_ned )
{
    set_center( center, from_ned ); // todo? should it also take from_ned somehow?
    set_position( position, from_ned );
    set_world( world_position, world_orientation, from_ned );
}

void camera_transform::set_world( const QVector3D& position, const QVector3D& orientation, bool from_ned ) // todo! quick and dirty; better usage semantics!
{
    world.setToIdentity();
    world.translate( from_ned ? _from_ned( position ) : position );
    set_orientation( orientation, from_ned );
    if( _projection.orthographic ) { update_projection(); } // todo? do we need it here?
}

std::pair< QVector3D, QVector3D > camera_transform::get_world( bool to_ned ) const
{
    std::pair< QVector3D, QVector3D > p;
    p.first = to_ned ? _to_ned( world.column(3).toVector3DAffine() ) : world.column( 3 ).toVector3DAffine();
    p.second = get_orientation( to_ned );
    return p;
}

void camera_transform::set_center( const QVector3D& v, bool from_ned )
{
    //std::cerr << "==> camera_transform::set_center(): before: " << center << " after: " << v << " translate: " << translate << std::endl;
    //world.translate( center ); // todo! should we translate at all? if so, then change the centre logic everywhere (e.g. in set_orientation)
    center = from_ned ? _from_ned( v ) : v;
    //world.translate( -center ); // todo! should we translate at all? if so, then change the centre logic everywhere (e.g. in set_orientation)
}

bool camera_transform::operator==( const camera_transform& rhs ) const // todo? quick and dirty; use approximated comparison?
{
    return world == rhs.world
        && camera == rhs.camera
        && projection == rhs.projection
        && center == rhs.center // should we?
        && _projection.up == rhs._projection.up
        && _projection.near_plane == rhs._projection.near_plane
        && _projection.far_plane == rhs._projection.far_plane
        && _projection.field_of_view == rhs._projection.field_of_view
        && _view_size == rhs._view_size;
}

static QQuaternion _quaternion( float r, float p, float y ) { return QQuaternion::fromEulerAngles( QVector3D( p, y, r ) * 180 / M_PI ).normalized(); } // quick and dirty; Qt wants angles in degrees
//static QQuaternion _quaternion( const QVector3D& rpy ) { return _quaternion( rpy.x(), rpy.y(), rpy.z() ); }

void camera_transform::set_orientation( float roll,float pitch,float yaw, bool from_ned )
{
    static const QQuaternion ned = QQuaternion::fromEulerAngles( QVector3D( 90, 90, 0 ) ); // quick and dirty; see https://doc.qt.io/qt-5/qquaternion.html#fromEulerAngles: QQuaternion::fromEulerAngles(pitch, yaw, roll); roll around z; pitch around x; yaw around y
    //COMMA_THROW_IF( from_ned, "from_ned: todo!" );
    QMatrix4x4 w;
    w.setToIdentity();
    w.translate( center );
    //w.rotate( from_ned ? _quaternion( -pitch, roll, yaw ) * ned : _quaternion( roll, pitch, yaw ) ); // todo! hyper-quick and dirty; just work out correct "ned" rotation, will you?
    w.rotate( from_ned ? _quaternion( roll, pitch, yaw ) * ned : _quaternion( roll, pitch, yaw ) );
    w.translate( -center );
    for( unsigned int y = 0; y < 3; ++y ) // quick and dirty for now; other things just suck
    {
        for( unsigned int x = 0; x < 3; ++x ) { world( x, y ) = w( x, y ); }
    }

    //world = w;
    // auto q = from_ned ? _quaternion( -pitch, roll, yaw ) * ned : _quaternion( roll, pitch, yaw ); // todo! hyper-quick and dirty; just work out correct "ned" rotation, will you?
    // QVector3D axis; float angle;
    // q.getAxisAndAngle( &axis, &angle );
    // std::cerr << "==> angle: " << angle << std::endl;
    // w.rotate( angle, axis );

    // world.translate( center );
    // QMatrix4x4 inverted_world = world.inverted();
    // QVector4D x_axis = inverted_world * QVector4D( 1, 0, 0, 0 );
    // QVector4D y_axis = inverted_world * QVector4D( 0, 1, 0, 0 );
    // QVector4D z_axis = inverted_world * QVector4D( 0, 0, 1, 0 );
    // world.rotate( yaw * 180 / M_PI, y_axis.toVector3D() );
    // world.rotate( pitch * 180 / M_PI, x_axis.toVector3D() );
    // world.rotate( roll * 180 / M_PI, z_axis.toVector3D() );
    // world.translate( -center );
    
    //world.setToIdentity(); // auto translation = world.column( 3 );
    //world.translate( center );
    //world.rotate( from_ned ? _quaternion( -pitch, roll, yaw ) * ned : _quaternion( roll, pitch, yaw ) ); // todo! hyper-quick and dirty; just work out correct "ned" rotation, will you?

    // if( from_ned )
    // {
    //     world.rotate( _quaternion( roll, pitch, yaw ) ); // world.rotate( _quaternion( -pitch, roll, yaw ) ); // sic... sick...
    //     static const QQuaternion e = QQuaternion::fromEulerAngles( QVector3D( 0, 0, 90 ) ); // quick and dirty; see https://doc.qt.io/qt-5/qquaternion.html#fromEulerAngles: QQuaternion::fromEulerAngles(pitch, yaw, roll); roll around z; pitch around x; yaw around y
    //     world.rotate( e );
    //     static const QQuaternion ned = QQuaternion::fromEulerAngles( QVector3D( 90, 90, 0 ) ); // quick and dirty; see https://doc.qt.io/qt-5/qquaternion.html#fromEulerAngles: QQuaternion::fromEulerAngles(pitch, yaw, roll); roll around z; pitch around x; yaw around y
    //     world.rotate( ned );
    // }
    // else
    // {
    //     world.rotate( _quaternion( roll, pitch, yaw ) ); // todo! hyper-quick and dirty; just work out correct "ned" rotation, will you?
    // }

    // static const Eigen::Matrix3d ned = snark::rotation_matrix( Eigen::Vector3d( M_PI / 2, 0, -M_PI / 2 ) ).rotation(); // super-quick and dirty
    // snark::rotation_matrix m( Eigen::Vector3d( roll, pitch, yaw ) );
    // Eigen::Matrix3d p = ned * m.rotation();
    // auto rpy = snark::rotation_matrix( p ).roll_pitch_yaw();
    // //world.rotate( from_ned ? _quaternion( roll, pitch, yaw ) : _quaternion( roll, pitch, yaw ) ); // todo! hyper-quick and dirty; just work out correct "ned" rotation, will you?
    // world.rotate( from_ned ? _quaternion( rpy.x(), rpy.y(), rpy.z() ) : _quaternion( roll, pitch, yaw ) ); // todo! hyper-quick and dirty; just work out correct "ned" rotation, will you?

    //world.translate( -center ); // world.translate( QVector3D( translation.x(), translation.y(), translation.z() ) ); // world.translate( -center ); //world.setColumn( 3, translation );
    //std::cerr << std::setprecision( 6 ) << "==> camera: set_orientation(): center: " << center << " r: " << roll << "," << pitch << "," << yaw << " get_orientation: " << get_orientation() << std::endl;
    //if( _projection.orthographic ) { update_projection(); } // todo? do we need to do it?
}

QVector3D camera_transform::get_orientation( bool to_ned ) const // todo? fix?
{
    if( to_ned )
    {
        Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
        for( unsigned int row = 0; row < 3; ++row ) { for( unsigned int col = 0; col < 3; ++col ) { m( row, col ) = world( row, col ); } }
        static const Eigen::Matrix3d ned = snark::rotation_matrix( Eigen::Vector3d( M_PI / 2, M_PI / 2, 0 ) ).rotation().inverse(); // todo! super-quick and dirty, just to get there; get the bloody correct transform!!!
        //std::cerr << "==> camera_transform::get_orientation:     rpy: " << snark::rotation_matrix::roll_pitch_yaw(m).transpose() << std::endl;
        //std::cerr << "==> camera_transform::get_orientation: (0) rpy: " << snark::rotation_matrix::roll_pitch_yaw(m*ned).transpose() << std::endl;
        //std::cerr << "==> camera_transform::get_orientation: (1) rpy: " << snark::rotation_matrix::roll_pitch_yaw(ned*m).transpose() << std::endl;
        const auto& rpy = snark::rotation_matrix::roll_pitch_yaw( ned * m ); // todo! super-quick and dirty, just to get there; get the bloody correct transform!!!
        return QVector3D( rpy.y(), rpy.x(), -rpy.z() );
    }
    QMatrix3x3 m;
    for( unsigned int row = 0; row < 3; ++row ) { for( unsigned int col = 0; col < 3; ++col ) { m( row, col ) = world( row, col ); } }
    const auto& pyr = QQuaternion::fromRotationMatrix( m ).toEulerAngles() * M_PI / 180;
    return QVector3D( pyr.z(), pyr.x(), pyr.y() );
}

void camera_transform::set_position( const QVector3D& v, bool from_ned )
{
    camera.setToIdentity();
    camera.translate( from_ned ? _from_ned( v ) : v );
    //std::cerr << "==> set_position(): " << v.x() << "," << v.y() << "," << v.z() << std::endl;
    //std::cerr << "==> camera_transform::set_position: v: " << std::setprecision( 16 ) << v << " from_ned: " << _from_ned( v ) << " get_position: " << get_position(true) << std::endl;
    if( _projection.orthographic ) { update_projection(); }
}

// void camera_transform::set_camera( const QVector3D& position, const QVector3D& orientation, bool from_ned )
// {
//     static const QQuaternion ned = QQuaternion::fromEulerAngles( QVector3D( 90, 90, 0 ) ); // quick and dirty; see https://doc.qt.io/qt-5/qquaternion.html#fromEulerAngles: QQuaternion::fromEulerAngles(pitch, yaw, roll); roll around z; pitch around x; yaw around y
//     camera.setToIdentity();
//     camera.translate( from_ned ? _from_ned( position ) : position );
//     camera.rotate( from_ned ? _quaternion( orientation ) * ned : _quaternion( orientation ) );
//     //std::cerr << "==> camera_transform::set_position: v: " << std::setprecision( 16 ) << v << " from_ned: " << _from_ned( v ) << " get_position: " << get_position(true) << std::endl;
//     if( _projection.orthographic ) { update_projection(); }
// }

QVector3D camera_transform::get_position( bool to_ned ) const
{
    //auto v = camera.column( 3 ).toVector3DAffine();
    //std::cerr << "==> get_position(): " << v.x() << "," << v.y() << "," << v.z() << std::endl;
    return to_ned ? _to_ned( camera.column(3).toVector3DAffine() ) : camera.column( 3 ).toVector3DAffine();
}

double camera_transform::distance() const { return std::abs(get_position().z()); }

void camera_transform::update_projection( const QSize& vs )
{
    if( vs != QSize( 0, 0 ) ) { _view_size = vs; }
    double aspect_ratio = double( _view_size.width() ) / _view_size.height();
    projection.setToIdentity();
    if( _projection.orthographic )
    {
        double size = 0.4 * distance();
        projection.ortho( -size * aspect_ratio, size * aspect_ratio, -size, size, -_projection.far_plane, _projection.far_plane );
    }
    else
    {
        // add camera translation (zoom)
        double fp = _projection.far_plane + distance();
        projection.perspective( _projection.field_of_view, aspect_ratio, _projection.near_plane, fp );
    }
}

} } } // namespace snark { namespace graphics { namespace qopengl {
