// Copyright (c) 2017 The University of Sydney

/// @author Navid Pirmarzdashti

#pragma once

#include "camera.h"
#include "../../traits.h"

namespace comma { namespace visiting {

template <> struct traits< snark::graphics::qopengl::camera_transform::config::pose >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::qopengl::camera_transform::config::pose& p, Visitor& v )
    {
        v.apply( "translation", p.translation );
        v.apply( "rotation", p.rotation );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::qopengl::camera_transform::config::pose& p, Visitor& v )
    {
        v.apply( "translation", p.translation );
        v.apply( "rotation", p.rotation );
    }
};

template <> struct traits< snark::graphics::qopengl::camera_transform::config::projection_t >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::qopengl::camera_transform::config::projection_t& p, Visitor& v )
    {
        v.apply( "up", p.up );
        v.apply( "orthographic", p.orthographic );
        v.apply( "near_plane", p.near_plane );
        v.apply( "far_plane", p.far_plane );
        v.apply( "field_of_view", p.field_of_view );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::qopengl::camera_transform::config::projection_t& p, Visitor& v )
    {
        v.apply( "up", p.up );
        v.apply( "orthographic", p.orthographic );
        v.apply( "near_plane", p.near_plane );
        v.apply( "far_plane", p.far_plane );
        v.apply( "field_of_view", p.field_of_view );
    }
};

template <> struct traits< snark::graphics::qopengl::camera_transform::config >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::qopengl::camera_transform::config& p, Visitor& v )
    {
        v.apply( "center", p.center );
        v.apply( "world", p.world );
        v.apply( "camera", p.camera );
        v.apply( "projection", p.projection );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::qopengl::camera_transform::config& p, Visitor& v )
    {
        v.apply( "center", p.center );
        v.apply( "world", p.world );
        v.apply( "camera", p.camera );
        v.apply( "projection", p.projection );
    }
};

template <> struct traits< snark::graphics::qopengl::camera_transform >
{
    template < typename Key, class Visitor >
    static void visit( Key k, snark::graphics::qopengl::camera_transform& p, Visitor& v )
    {
        auto config = p.to_config();
        traits< snark::graphics::qopengl::camera_transform::config >::visit( k, config, v );
        p = snark::graphics::qopengl::camera_transform::make( config );
    }
    template < typename Key, class Visitor >
    static void visit( Key k, const snark::graphics::qopengl::camera_transform& p, Visitor& v )
    {
        traits< snark::graphics::qopengl::camera_transform::config >::visit( k, p.to_config(), v );
    }
};

} } // namespace comma { namespace visiting {
    
