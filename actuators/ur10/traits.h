#ifndef SNARK_ACTUATORS_ROBOT_ARM_TRAITS_H
#define SNARK_ACTUATORS_ROBOT_ARM_TRAITS_H
// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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

#include <boost/units/quantity.hpp>
#include <comma/visiting/traits.h>
#include "commands.h"
#include "units.h"
#include "config.h"
#include "data.h"

namespace comma { namespace visiting {
    
namespace robot_arm = snark::robot_arm;
using snark::robot_arm::command_base;

// Commands
template <typename C> struct traits< command_base< C > >
{
    template< typename K, typename V > static void visit( const K& k, command_base< C >& t, V& v )
    {
        v.apply( "rover_id", t.rover_id );
        v.apply( "sequence_number", t.sequence_number );
        v.apply( "name", t.name );
    }
    template< typename K, typename V > static void visit( const K& k, const command_base< C >& t, V& v )
    {
        v.apply( "rover_id", t.rover_id );
        v.apply( "sequence_number", t.sequence_number );
        v.apply( "name", t.name );
    }
};

template <> struct traits< robot_arm::move_cam >
{
    template< typename K, typename V > static void visit( const K& k, robot_arm::move_cam& t, V& v )
    {
    	traits< command_base < robot_arm::move_cam > >::visit(k, t, v);
        double p, l, h;
        v.apply( "pan", p );
        t.pan = p * robot_arm::degree;
        v.apply( "tilt", l );
        t.tilt = l * robot_arm::degree;
        v.apply( "height", h );
        t.height = h * robot_arm::meter;
    }

    template< typename K, typename V > static void visit( const K& k, const robot_arm::move_cam& t, V& v )
    {
    	traits< command_base < robot_arm::move_cam > >::visit(k, t, v);
        v.apply( "pan", t.pan.value() );
        v.apply( "tilt", t.tilt.value() );
        v.apply( "height", t.height.value() );
    }
};

template <> struct traits< robot_arm::position >
{
    template< typename K, typename V > static void visit( const K& k, robot_arm::position& t, V& v )
    {
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
    }

    template< typename K, typename V > static void visit( const K& k, const robot_arm::position& t, V& v )
    {
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
    }
};

template <> struct traits< robot_arm::move_effector >
{
    template< typename K, typename V > static void visit( const K& k, robot_arm::move_effector& t, V& v )
    {
        traits< command_base < robot_arm::move_effector > >::visit(k, t, v);
        v.apply( "offset", t.offset );
        double p, l, r;
        v.apply( "pan", p );
        t.pan = p * robot_arm::degree;
        v.apply( "tilt", l );
        t.tilt = l * robot_arm::degree;
        v.apply( "roll", r );
        t.roll = r * robot_arm::degree;
    }

    template< typename K, typename V > static void visit( const K& k, const robot_arm::move_effector& t, V& v )
    {
        traits< command_base < robot_arm::move_effector > >::visit(k, t, v);
        v.apply( "offset", t.offset );
        v.apply( "pan", t.pan.value() );
        v.apply( "tilt", t.tilt.value() );
        v.apply( "roll", t.roll.value() );
    }
};
template <> struct traits< std::vector< robot_arm::plane_angle_degrees_t > >
{
    template< typename K, typename V > static void visit( const K& k, std::vector< robot_arm::plane_angle_degrees_t >& t, V& v )
    {
        for( std::size_t i=0; i<t.size(); ++i ) {
            double d = 0;
            v.apply( boost::lexical_cast< std::string >( i ).c_str(), d );
            t[i] = d * robot_arm::degree;
        }
    }

    template< typename K, typename V > static void visit( const K& k, const std::vector< robot_arm::plane_angle_degrees_t >& t, V& v )
    {
        for( std::size_t i=0; i<t.size(); ++i ) {
            v.apply( boost::lexical_cast< std::string >( i ).c_str(), t[i].value() );
        }
    }
};

template <> struct traits< robot_arm::move_joints >
{
    template< typename K, typename V > static void visit( const K& k, robot_arm::move_joints& t, V& v )
    {
        traits< command_base < robot_arm::move_joints > >::visit(k, t, v);
        v.apply( "joints", t.joints );
    }

    template< typename K, typename V > static void visit( const K& k, const robot_arm::move_joints& t, V& v )
    {
        traits< command_base < robot_arm::move_joints > >::visit(k, t, v);
        v.apply( "joints", t.joints );
    }
};

template <> struct traits< robot_arm::joint_move >
{
    template< typename K, typename V > static void visit( const K& k, robot_arm::joint_move& t, V& v )
    {
        traits< command_base < robot_arm::joint_move > >::visit(k, t, v);
        v.apply( "joint_id", t.joint_id );
        v.apply( "dir", t.dir );
    }

    template< typename K, typename V > static void visit( const K& k, const robot_arm::joint_move& t, V& v )
    {
        traits< command_base < robot_arm::joint_move > >::visit(k, t, v);
        v.apply( "joint_id", t.joint_id );
        v.apply( "dir", t.dir );
    }
};

template <> struct traits< robot_arm::auto_init >
{
    template< typename K, typename V > static void visit( const K& k, robot_arm::auto_init& t, V& v ) {
        traits< command_base < robot_arm::auto_init > >::visit(k, t, v);
    }
    template< typename K, typename V > static void visit( const K& k, const robot_arm::auto_init& t, V& v ) {
        traits< command_base < robot_arm::auto_init > >::visit(k, t, v);
    }
};

template <> struct traits< robot_arm::enable >
{
    template< typename K, typename V > static void visit( const K& k, robot_arm::enable& t, V& v ) {
        traits< command_base < robot_arm::enable > >::visit(k, t, v);
    }
    template< typename K, typename V > static void visit( const K& k, const robot_arm::enable& t, V& v ) {
        traits< command_base < robot_arm::enable > >::visit(k, t, v);
    }
};

template <> struct traits< robot_arm::release_brakes >
{
    template< typename K, typename V > static void visit( const K& k, robot_arm::release_brakes& t, V& v ) {
        traits< command_base < robot_arm::release_brakes > >::visit(k, t, v);
    }
    template< typename K, typename V > static void visit( const K& k, const robot_arm::release_brakes& t, V& v ) {
        traits< command_base < robot_arm::release_brakes > >::visit(k, t, v);
    }
};

template <> struct traits< robot_arm::set_position >
{
    template< typename K, typename V > static void visit( const K& k, robot_arm::set_position& t, V& v )
    {
        traits< command_base < robot_arm::set_position > >::visit(k, t, v);
        v.apply( "position", t.position );
    }

    template< typename K, typename V > static void visit( const K& k, const robot_arm::set_position& t, V& v )
    {
        traits< command_base < robot_arm::set_position > >::visit(k, t, v);
        v.apply( "position", t.position );
    }
};

template <> struct traits< robot_arm::set_home >
{
    template< typename K, typename V > static void visit( const K& k, robot_arm::set_home& t, V& v )
    {
        traits< command_base < robot_arm::set_home > >::visit(k, t, v);
    }

    template< typename K, typename V > static void visit( const K& k, const robot_arm::set_home& t, V& v )
    {
        traits< command_base < robot_arm::set_home > >::visit(k, t, v);
    }
};

template <> struct traits< robot_arm::config >
{
    template< typename K, typename V > static void visit( const K& k, robot_arm::config& t, V& v )
    {
        v.apply( "home_position", t.home_position );
    }

    template< typename K, typename V > static void visit( const K& k, const robot_arm::config& t, V& v )
    {
        v.apply( "home_position", t.home_position );
    }
};

template < > struct traits< boost::array< comma::packed::big_endian_double, 6 > >
{
    template< typename K, typename V > static void visit( const K& k, const boost::array< comma::packed::big_endian_double, 6 >& t, V& v )
    {
        std::size_t i = 0;
        for( const comma::packed::big_endian_double* iter=t.cbegin(); iter!=t.cend(); ++iter ) { 
            v.apply( boost::lexical_cast< std::string >( i ).c_str(), (*iter)() ); 
            ++i;
        }
    }
};

template < > struct traits< robot_arm::joints_in_degrees >
{
    template< typename K, typename V > static void visit( const K& k, const robot_arm::joints_in_degrees& t, V& v )
    {
        for( std::size_t i=0; i<robot_arm::joints_num; ++i ) { 
            double d = t.joints[i].value();
            v.apply( boost::lexical_cast< std::string >( i ).c_str(), d ); 
        }
    }
};

template < > struct traits< robot_arm::cartesian >
{
    template< typename K, typename V > static void visit( const K& k, const robot_arm::cartesian& t, V& v )
    {
        v.apply( "x", t.x() );
        v.apply( "y", t.y() );
        v.apply( "z", t.z() );
    }
};

template <> struct traits< robot_arm::fixed_status >
{
    template< typename K, typename V > static void visit( const K& k, const  robot_arm::fixed_status& t, V& v )
    {
        v.apply( "length", t.length() );
        v.apply( "time_since_boot", t.time_since_boot() );
        
        robot_arm::joints_in_degrees positions( t.positions );
        v.apply( "positions", positions );
        v.apply( "velocities", t.velocities );
        v.apply( "currents", t.currents );
        v.apply( "coordinates", t.translation );
        v.apply( "rotation", t.rotation );
        v.apply( "temperatures", t.temperatures );
        v.apply( "robot_mode", t.robot_mode() );
        v.apply( "joint_modes", t.joint_modes );
    }
};

}} // namespace comma { namespace visiting {

#endif // SNARK_ACTUATORS_ROBOT_ARM_TRAITS_H
