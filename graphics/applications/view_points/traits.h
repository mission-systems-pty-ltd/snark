// Copyright (c) 2017 The University of Sydney

#pragma once

#include "../../traits.h"

#if Qt3D_VERSION==1 // todo: Qt3D tear down version 1 support; it does not compile anymore anyway
#include "../../qt3d/qt3d_v1/traits.h"
#else
#include "qopengl/viewer.h"
#endif

namespace comma { namespace visiting {

template <> struct traits< snark::graphics::view::qopengl::viewer::camera::options::transitions_t >
{
    template < typename Key, class Visitor > static void visit( Key, snark::graphics::view::qopengl::viewer::camera::options::transitions_t& p, Visitor& v )
    {
        bool enabled{false};
        bool disabled{false};
        v.apply( "duration", p.duration );
        v.apply( "size", p.size );
        v.apply( "enabled", enabled );
        v.apply( "disabled", disabled );
        p.enabled = enabled ? true : disabled ? false : p.enabled;
    }

    template < typename Key, class Visitor > static void visit( Key, const snark::graphics::view::qopengl::viewer::camera::options::transitions_t& p, Visitor& v )
    {
        v.apply( "duration", p.duration );
        v.apply( "size", p.size );
        v.apply( "enabled", p.enabled );
    }
};

template <> struct traits< snark::graphics::view::qopengl::viewer::camera::options >
{
    typedef snark::graphics::qt3d::camera_options base_t;

    template < typename Key, class Visitor > static void visit( const Key& k, snark::graphics::view::qopengl::viewer::camera::options& p, Visitor& v )
    {
        comma::visiting::traits< base_t >::visit( k, static_cast< base_t& >( p ), v );
        v.apply( "transitions", p.transitions );
    }

    template < typename Key, class Visitor > static void visit( const Key& k, const snark::graphics::view::qopengl::viewer::camera::options& p, Visitor& v )
    {
        comma::visiting::traits< base_t >::visit( k, static_cast< const base_t& >( p ), v );
        v.apply( "transitions", p.transitions );
    }
};

} } // namespace comma { namespace visiting {
