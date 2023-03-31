// Copyright (c) 2011 The University of Sydney

#pragma once

#include "../../visiting/eigen.h"
#include "metashape.h"
#include "pinhole.h"
#include "stereo.h"

namespace comma { namespace visiting {

template <> struct traits< snark::camera::pinhole::config_t::distortion_t::radial_t >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::camera::pinhole::config_t::distortion_t::radial_t& p, Visitor& v )
    {
        v.apply( "k1", p.k1 );
        v.apply( "k2", p.k2 );
        v.apply( "k3", p.k3 );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, const snark::camera::pinhole::config_t::distortion_t::radial_t& p, Visitor& v )
    {
        v.apply( "k1", p.k1 );
        v.apply( "k2", p.k2 );
        v.apply( "k3", p.k3 );
    }
};

template <> struct traits< snark::camera::pinhole::config_t::distortion_t::tangential_t >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::camera::pinhole::config_t::distortion_t::tangential_t& p, Visitor& v )
    {
        v.apply( "p1", p.p1 );
        v.apply( "p2", p.p2 );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, const snark::camera::pinhole::config_t::distortion_t::tangential_t& p, Visitor& v )
    {
        v.apply( "p1", p.p1 );
        v.apply( "p2", p.p2 );
    }
};
    
template <> struct traits< snark::camera::pinhole::config_t::distortion_t >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::camera::pinhole::config_t::distortion_t& p, Visitor& v )
    {
        v.apply( "radial", p.radial );
        v.apply( "tangential", p.tangential );
        v.apply( "map", p.map_filename );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, const snark::camera::pinhole::config_t::distortion_t& p, Visitor& v )
    {
        v.apply( "radial", p.radial );
        v.apply( "tangential", p.tangential );
        v.apply( "map", p.map_filename );
    }
};
    
template <> struct traits< snark::camera::pinhole::config_t >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::camera::pinhole::config_t& p, Visitor& v )
    {
        v.apply( "sensor_size", p.sensor_size );
        v.apply( "image_size", p.image_size );
        v.apply( "focal_length", p.focal_length );
        v.apply( "principal_point", p.principal_point );
        v.apply( "distortion", p.distortion );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, const snark::camera::pinhole::config_t& p, Visitor& v )
    {
        v.apply( "sensor_size", p.sensor_size );
        v.apply( "image_size", p.image_size );
        v.apply( "focal_length", p.focal_length );
        v.apply( "principal_point", p.principal_point );
        v.apply( "distortion", p.distortion );
    }
};

template <> struct traits< snark::camera::config >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::camera::config& p, Visitor& v )
    {
        v.apply( "pinhole", p.pinhole );
        v.apply( "pose", p.pose );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, const snark::camera::config& p, Visitor& v )
    {
        v.apply( "pinhole", p.pinhole );
        v.apply( "pose", p.pose );
    }
};

template <> struct traits< snark::metashape::camera::pinhole::calibration_t >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::metashape::camera::pinhole::calibration_t& p, Visitor& v )
    {
        v.apply( "width", p.width );
        v.apply( "height", p.height );
        v.apply( "f", p.f );
        v.apply( "cx", p.cx );
        v.apply( "cy", p.cy );
        v.apply( "b1", p.b1 );
        v.apply( "b2", p.b2 );
        v.apply( "k1", p.k1 );
        v.apply( "k2", p.k2 );
        v.apply( "k3", p.k3 );
        v.apply( "k4", p.k4 );
        v.apply( "p1", p.p1 );
        v.apply( "p2", p.p2 );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, const snark::metashape::camera::pinhole::calibration_t& p, Visitor& v )
    {
        v.apply( "width", p.width );
        v.apply( "height", p.height );
        v.apply( "f", p.f );
        v.apply( "cx", p.cx );
        v.apply( "cy", p.cy );
        v.apply( "b1", p.b1 );
        v.apply( "b2", p.b2 );
        v.apply( "k1", p.k1 );
        v.apply( "k2", p.k2 );
        v.apply( "k3", p.k3 );
        v.apply( "k4", p.k4 );
        v.apply( "p1", p.p1 );
        v.apply( "p2", p.p2 );
    }
};

template <> struct traits< snark::metashape::camera::pinhole >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::metashape::camera::pinhole& p, Visitor& v )
    {
        v.apply( "calibration", p.calibration );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, const snark::metashape::camera::pinhole& p, Visitor& v )
    {
        v.apply( "calibration", p.calibration );
    }
};

} } // namespace comma { namespace visiting {
