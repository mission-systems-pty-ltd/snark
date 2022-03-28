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

#include "camera.h"
#include <cmath>
#include <iostream>
#include "../../../math/rotation_matrix.h"

namespace snark { namespace graphics { namespace qopengl {

camera_transform::camera_transform(bool orthographic,double field_of_view,const QVector3D& up,const QVector3D& c,float z)
    : center(c)
    , up(up)
    , orthographic(orthographic)
    , near_plane(0.01)
    , far_plane(100)
    , field_of_view(field_of_view)
{
    // The camera always points along the z-axis. Pan moves the camera in x,y
    // coordinates and zoom moves in and out on the z-axis.
    // It starts at -1 because in OpenGL-land the transform is actually applied
    // to the world and the camera is stationary at 0,0,0.
    camera.setToIdentity();
    camera.translate(0, 0, -1);
    world.setToIdentity();
    world.translate(-center);
}
void camera_transform::pan(float dx,float dy) { camera.translate(dx,dy,0); }

void camera_transform::zoom(float dz)
{
    camera.translate(0,0,dz);
    if(orthographic) { update_projection(); }
}
void camera_transform::pivot(float dx,float dy)
{
    world.translate(center);
    QMatrix4x4 inverted_world = world.inverted();
    QVector4D x_axis = inverted_world * QVector4D(1, 0, 0, 0);
    QVector4D y_axis = inverted_world * QVector4D(0, 1, 0, 0);
    world.rotate(dy, x_axis.toVector3D());
    world.rotate(dx, y_axis.toVector3D());
    world.translate(-center);
}
void camera_transform::set_center(const QVector3D& v)
{
//     world.translate(center);
    center=v;
//     world.translate(-center);
}

// void camera_transform::set_orientation(float roll,float pitch,float yaw)
// {
//     Eigen::Quaterniond  q=snark::rotation_matrix(Eigen::Vector3d(roll,pitch,yaw)).quaternion();
// //     std::cerr<<"camera_transform::set_    orientation "<<roll<<", "<<pitch<<", "<<yaw<<std::endl;
//     world.setToIdentity();
//     world.rotate(QQuaternion(q.w(),QVector3D(q.x(),q.y(),q.z())));
//     world.translate(-center);
//}

void camera_transform::set_orientation(float roll,float pitch,float yaw)
{
    world.setToIdentity();
    world.rotate( QQuaternion::fromEulerAngles( -QVector3D( yaw, roll, pitch ) * 180 / M_PI ) ); // ??? * 180 / M_PI (documentation says: degrees...)
    world.translate(-center);
}
QVector3D camera_transform::get_orientation() const // todo? fix?
{
    Eigen::Matrix3d m=Eigen::Matrix3d::Identity();
    for(unsigned row=0;row<3;row++)
    {
        for(unsigned col=0;col<3;col++)
        {
            m(row,col)=world(row,col);
        }
    }
    auto rpy=snark::rotation_matrix::roll_pitch_yaw(m);
    double roll=rpy.x(), pitch=rpy.y(), yaw=rpy.z();
//     std::cerr<<"camera_transform::get_orientation "<<roll<<", "<<pitch<<", "<<yaw<<std::endl;
    return QVector3D(roll,pitch,yaw);
}
void camera_transform::set_position(const QVector3D& v)
{
    camera.setToIdentity();
    camera.translate(v);
    if(orthographic) { update_projection(); }
}
QVector3D camera_transform::get_position() const
{
    return camera.column(3).toVector3DAffine();
}
double camera_transform::distance()
{
    return std::abs(get_position().z());
}
void camera_transform::update_projection(const QSize& vs)
{
    if(vs!=QSize(0,0)) { view_size=vs; }
    double aspect_ratio = (double) view_size.width() / view_size.height();
    projection.setToIdentity();
    if( orthographic )
    {
        double size=0.4*distance();
        projection.ortho(-size * aspect_ratio, size * aspect_ratio, -size, size,-far_plane,far_plane);
    }
    else
    {
        // add camera translation (zoom)
        double fp=far_plane+distance();
        projection.perspective(field_of_view, aspect_ratio,near_plane,fp);
    }
}

} } } // namespace snark { namespace graphics { namespace qopengl {
