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

#pragma once

#include <Eigen/Core>

#if Qt3D_VERSION==1
#include <QVector3D>
#include <Qt3D/qcolor4ub.h>

#else
#include "../../qt5.5/qopengl/types.h"

#endif

namespace snark { namespace graphics { namespace view {
    
#if Qt3D_VERSION==1
typedef QColor4ub color_t;
#define COLOR_RED   Qt::red
#define COLOR_GREEN Qt::green
#define COLOR_BLUE  Qt::blue

#elif Qt3D_VERSION>=2

typedef snark::graphics::qopengl::color_t color_t;

struct stock
{
    const static color_t red;
    const static color_t green;
    const static color_t blue;
};

#define COLOR_RED   stock::red
#define COLOR_GREEN stock::green
#define COLOR_BLUE  stock::blue

#endif

/**
 * base class for controller. 
 * defines event handlers that are called by viewer.
 */
class controller_base
{
public:
    virtual ~controller_base() { }
    //should be called only once when opengl is up
    virtual void init()=0;
    virtual void tick()=0;
};

/// text label with 3d position
struct label_t
{
    Eigen::Vector3d position;
    color_t color;
    std::string text;
    label_t() {}
    label_t( const Eigen::Vector3d& position, const color_t& color, const std::string& text ) : position( position ), color( color ), text( text ) { }
};

#if Qt3D_VERSION==1
struct vertex_t
{
    QVector3D position;
    QColor4ub color;
    vertex_t() {}
    vertex_t( const QVector3D& position, const QColor4ub& color )
        : position( position ), color( color ) {}
    vertex_t(const Eigen::Vector3f& p,const QColor4ub& color) : position(p.x(),p.y(),p.z()), color(color) { }
    vertex_t(const Eigen::Vector3d& p,const QColor4ub& color) : position(p.x(),p.y(),p.z()), color(color) { }
};
#elif Qt3D_VERSION>=2
typedef qopengl::vertex_t vertex_t;
#endif

#if Qt3D_VERSION==1
class Viewer;
typedef Viewer viewer_t;

#elif Qt3D_VERSION>=2
namespace qopengl {
class viewer;
} // namespace qopengl {
typedef qopengl::viewer viewer_t;

#endif

} } } // namespace snark { namespace graphics { namespace view {

