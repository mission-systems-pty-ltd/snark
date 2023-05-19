// Copyright (c) 2017 The University of Sydney

#pragma once

#include <comma/base/exception.h>
#include <comma/visiting/traits.h>
#include <QVector3D>
#include <QSizeF>
#include "qt3d/camera_options.h"

namespace comma { namespace visiting {

#define COMMA_VISITING_TRAITS_QVECTOR3d
    
template <> struct traits< QVector3D >
{
    template < typename Key, class Visitor > static void visit( Key, QVector3D& p, Visitor& v )
    {
        double d;
        d= p.x(); v.apply( "x", d ); p.setX( d );
        d = p.y(); v.apply( "y", d ); p.setY( d );
        d = p.z(); v.apply( "z", d ); p.setZ( d );
    }

    template < typename Key, class Visitor > static void visit( Key, const QVector3D& p, Visitor& v )
    {
        v.apply( "x", p.x() );
        v.apply( "y", p.y() );
        v.apply( "z", p.z() );
    }
};

template <> struct traits< QSizeF >
{
    template < typename Key, class Visitor > static void visit( Key, QSizeF& p, Visitor& v )
    {
        double d;
        d = p.width(); v.apply( "width", d ); p.setWidth( d );
        d = p.height(); v.apply( "height", d ); p.setHeight( d );
    }

    template < typename Key, class Visitor > static void visit( Key, const QSizeF& p, Visitor& v )
    {
        v.apply( "width", p.width() );
        v.apply( "height", p.height() );
    }
};

template <> struct traits< snark::graphics::qt3d::camera_options >
{
    template < typename Key, class Visitor > static void visit( Key, snark::graphics::qt3d::camera_options& p, Visitor& v )
    {
        bool orthographic{false};
        bool perspective{false};
        v.apply( "orthographic", orthographic );
        v.apply( "perspective", perspective );
        COMMA_ASSERT_BRIEF( !orthographic || !perspective, "camera options: expected either orthographic or perspective; got both" );
        p.orthographic = orthographic ? true : perspective ? false : p.orthographic;
        v.apply( "fov", p.field_of_view );
        v.apply( "z_is_up", p.z_is_up );
    }

    template < typename Key, class Visitor > static void visit( Key, const snark::graphics::qt3d::camera_options& p, Visitor& v )
    {
        v.apply( "orthographic", p.orthographic );
        v.apply( "fov", p.field_of_view );
        v.apply( "z_is_up", p.z_is_up );
    }
};

} } // namespace comma { namespace visiting {
    
