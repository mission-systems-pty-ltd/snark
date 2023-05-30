// Copyright (c) 2017 The University of Sydney

/// @author Navid Pirmarzdashti

#pragma once

#include <Eigen/Core>
#include <QMatrix4x4>
#include <QVector3D>

namespace snark { namespace graphics { namespace qopengl {

struct camera_transform
{
    camera_transform( bool orthographic
                    , double field_of_view
                    , const QVector3D& up = QVector3D( 0, 0, -1 )
                    , const QVector3D& center = QVector3D( 0, 0, 0 ) );
                    //, float z = -1 );

    void pan( float dx, float dy );

    /// negative numbers push camera backward (zoom out)
    void zoom( float dz );

    /// rotate world on its own x and y axis
    /// apparent rotation of view point around pivot point
    void pivot( float dx, float dy );

    QMatrix4x4 transform() const;

    void set( const QVector3D& center, const QVector3D& position, const QVector3D& orientation, bool from_ned = false );

    void set_center( const QVector3D& v, bool from_ned = false );

    void set_orientation( float roll,float pitch,float yaw, bool from_ned = false );

    void set_orientation( const QVector3D& v, bool from_ned = false ) { set_orientation( v.x(), v.y(), v.z(), from_ned ); }

    //void set_camera( const QVector3D& position, const QVector3D& orientation, bool from_ned = false );

    QVector3D get_orientation( bool to_ned = false ) const;

    /// sets camera position in world coordinate
    /// z is distance to center and (x,y) component is pan
    void set_position( const QVector3D& v, bool from_ned = false );

    QVector3D get_position( bool to_ned = false ) const;

    /// distance between camera and center
    double distance() const;

    /// updates projection matrix
    /// if called with no argument (default 0,0), it uses view_size data member
    /// call this on window resize passing widget.size(); or after updating any projection fields
    void update_projection( const QSize& view_size = QSize( 0, 0 ) );

    bool operator==( const camera_transform& rhs ) const;
    bool operator!=( const camera_transform& rhs ) const { return !operator==( rhs ); }

    QMatrix4x4 world;
    QMatrix4x4 camera;
    QMatrix4x4 projection;
    QVector3D center{0, 0, 0};
    // call update_projection if you set any of the following
    QVector3D up;   // not plugged in yet; todo
    bool orthographic;
    double near_plane;
    double far_plane;
    double field_of_view;

    QSize view_size;
};

} } } // namespace snark { namespace graphics { namespace qopengl {
