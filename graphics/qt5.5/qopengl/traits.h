// Copyright (c) 2017 The University of Sydney

/// @author Navid Pirmarzdashti

#pragma once

#include "camera.h"
#include "../../traits.h"

namespace comma { namespace visiting {

template <> struct traits< snark::graphics::qopengl::camera_transform >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::qopengl::camera_transform& p, Visitor& v )
    {
        QVector3D position = p.get_position();
        v.apply( "position", position );
        QVector3D orientation = p.get_orientation();
        v.apply( "orientation", orientation );
        QVector3D center = p.center;
        v.apply( "center", center );
        v.apply( "up", p.up );
        v.apply( "orthographic", p.orthographic );
        v.apply( "near_plane", p.near_plane );
        v.apply( "far_plane", p.far_plane );
        v.apply( "field_of_view", p.field_of_view );
        p.set( center, position, orientation, false, true ); // should it be: translate_center = true?
        p.update_projection();
    }
    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::qopengl::camera_transform& p, Visitor& v )
    {
        v.apply( "position", p.get_position() ); // should it be from_ned?
        v.apply( "orientation", p.get_orientation() ); // should it be from_ned?
        v.apply( "center", p.center );
        v.apply( "up", p.up );
        v.apply( "orthographic", p.orthographic );
        v.apply( "near_plane", p.near_plane );
        v.apply( "far_plane", p.far_plane );
        v.apply( "field_of_view", p.field_of_view );
    }
};

} } // namespace comma { namespace visiting {
    
