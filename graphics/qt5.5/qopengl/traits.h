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
        QVector3D position = p.get_position(); // should it be from_ned?
        v.apply( "position", position );
        auto world = p.get_world(); // should it be from_ned?
        v.apply( "translation", world.first ); // super-bad name
        v.apply( "orientation", world.second );
        QVector3D center = p.center; // should it be from_ned?
        v.apply( "center", center );
        v.apply( "up", p.up );
        v.apply( "orthographic", p.orthographic );
        v.apply( "near_plane", p.near_plane );
        v.apply( "far_plane", p.far_plane );
        v.apply( "field_of_view", p.field_of_view );
        p.set( center, position, world.first, world.second, false ); // should it be from_ned?
        p.update_projection();
    }
    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::qopengl::camera_transform& p, Visitor& v )
    {
        v.apply( "position", p.get_position() ); // should it be to_ned?
        auto world = p.get_world(); // should it be from_ned?
        v.apply( "translation", world.first ); // super-bad name
        v.apply( "orientation", world.second ); // should it be to_ned?
        v.apply( "center", p.center ); // should it be to_ned?
        v.apply( "up", p.up );
        v.apply( "orthographic", p.orthographic );
        v.apply( "near_plane", p.near_plane );
        v.apply( "far_plane", p.far_plane );
        v.apply( "field_of_view", p.field_of_view );
    }
};

} } // namespace comma { namespace visiting {
    
