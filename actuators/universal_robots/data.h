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

#ifndef SNARK_ACTUATORS_UR_ROBOTIC_ARM_DATA_H 
#define SNARK_ACTUATORS_UR_ROBOTIC_ARM_DATA_H

#include <comma/packed/packed.h>
#include <comma/math/compare.h>
#include "units.h"
#include "config.h"
#include <snark/math/applications/frame.h>

namespace comma { namespace packed {

/// This structure unpacks double according to standard IEEE754 - reverses the byte order.
/// The ntohd() is from the Windows socket library, for UNIX it is not defined in the BSD sockets library because there is more than one standard for float/double between different UNIX systems.
/// In our case the standard in use is the IEEE754 [1] standard (which all Windows systems also use).
template < typename T >
class big_endian_64 : public comma::packed::field< big_endian_64< T >, T, sizeof( T ) >
{
public:
    enum { size = sizeof( T ) };
    
    //BOOST_STATIC_ASSERT( size == 8 );
    
    typedef T type;
    
    typedef comma::packed::field< big_endian_64< T >, T, size > base_type;
    
    static type default_value() { return 0; }
    
    static void pack( char* storage, type value )
    {
        ::memcpy( storage, ( void* )&value, size );
        std::reverse(&storage, &storage + size );
    }
    
    static type unpack( const char* storage )
    {
        type value;
        ::memcpy( ( void* )&value, storage, size );
        
        char& raw = reinterpret_cast<char&>(value);
        std::reverse(&raw, &raw + size );
        return value;
    }
    
    const big_endian_64< T >& operator=( const big_endian_64< T >& rhs ) { return base_type::operator=( rhs ); }
    
    const big_endian_64< T >& operator=( type rhs ) { return base_type::operator=( rhs ); }
};

typedef big_endian_64< double > big_endian_double;

} } // namespace comma { namespace packed {

namespace snark { namespace ur { 

struct robotmode {
    enum mode { running, freedrive, ready, initializing, security_stopped, estopped, fatal_error, no_power, not_connected, shutdown, safeguard_stop };
};
struct jointmode {
    ///TODO enumerate all modes
    enum mode { power_off=239, error=242, freedrive=243, calibration=250, stopped=251, running=253, initializing=254, idle=255, other=-999 };
};

const char* robotmode_str( robotmode::mode mode );
const char* jointmode_str( jointmode::mode mode );

robotmode::mode get_robotmode( const std::string& mode );
jointmode::mode get_jointmode( const std::string& mode );
    
struct cartesian {
    comma::packed::big_endian_double x;
    comma::packed::big_endian_double y;
    comma::packed::big_endian_double z;
};

typedef boost::array< comma::packed::big_endian_double, joints_num > joints_net_t;

struct joint_modes_t 
{
    joint_modes_t( const joints_net_t& joints )
    {
        for( std::size_t i=0; i<joints_num; ++i )  { 
            modes.push_back( jointmode_str( jointmode::mode( (int)(joints[i]()) ) ) );
        }
    }
    std::vector< std::string > modes;
};

template < typename T >
void init( const T& t, boost::array< T, joints_num >& arr )
{
    for( std::size_t i=0; i<joints_num; ++i ) { arr[i] = t; }
}    

struct packet_t : public comma::packed::packed_struct< packet_t, 812  >
{
    comma::packed::big_endian_uint32 length;
    comma::packed::big_endian_double time_since_boot;
    comma::packed::string< 240 > dummy1;
    joints_net_t positions; /// actual joint positions
    joints_net_t velocities; /// actual joint velocities
    joints_net_t currents; /// actual joint currents - Amps
    comma::packed::string< 120 > dummy2;
    joints_net_t forces;    /// Force (Newton) on each joint
    comma::packed::string< 24 > dummy5;
    cartesian  translation;       ///  coordinates
    cartesian  rotation;       ///  rotation
    comma::packed::string< 56 > dummy3;
    joints_net_t  temperatures; ///  Celcius
    comma::packed::string< 16 > dummy4;
    comma::packed::big_endian_double robot_mode;
    joints_net_t joint_modes; ///  joint modes - see documents

    robotmode::mode mode() const { return  robotmode::mode( (int)this->robot_mode() ); }
    jointmode::mode jmode( int id ) const { return jointmode::mode( int(joint_modes[id]()) ); }

    std::string mode_str() const { return robotmode_str( mode() ); }
    std::string jmode_str( int id ) const { return jointmode_str( jmode( id ) ); }
};

/// Ananlog to packet_t, reads from host byte order source
struct status_t {
    typedef boost::array< double, joints_num > array_doubles_t;
    typedef boost::array< jointmode::mode, joints_num > array_jointmodes_t;
    typedef boost::array< plane_angle_t, joints_num > array_joint_angles_t;
    
    boost::posix_time::ptime timestamp;
    snark::applications::position position;     /// Tool Center Point position
    boost::array< plane_angle_t, joints_num > joint_angles;
    boost::array< double, joints_num > velocities;
    boost::array< double, joints_num > currents;
    boost::array< double, joints_num > forces;
    boost::array< double, joints_num > temperatures;
    robotmode::mode robot_mode;
    boost::array< jointmode::mode, joints_num > joint_modes;
    comma::uint32 length;
    double time_since_boot;

    void set( boost::posix_time::ptime timestamp, const packet_t& packet );
    bool is_valid() { return this->length == snark::ur::packet_t::size && this->robot_mode >= snark::ur::robotmode::running && this->robot_mode <= snark::ur::robotmode::safeguard_stop; }
    
    std::string mode_str() const { return robotmode_str( robot_mode ); }
    std::string jmode_str( int id ) const { return jointmode_str( joint_modes[id] ); }
    jointmode::mode jmode( int id ) const { return joint_modes[id]; }
    robotmode::mode mode() const { return  robot_mode; }

    /// Robotic arm must be in this state to move
    bool is_running() const;
    bool is_initialising_ready() const;
    /// Robotic arm must be in this state to power on.
    bool is_powered_off() const;
    
    bool is_stationary( double epsilon=0.05 ) const;
    
    status_t() : timestamp( boost::posix_time::microsec_clock::local_time() ), position(), robot_mode( robotmode::not_connected ), length(812), time_since_boot(-1) 
    {
        init< plane_angle_t >( 0 * radian, joint_angles );
        init< double >( 0.0, velocities );
        init< double >( 0.0, currents );
        init< double >( 0.0, forces );
        init< double >( 0.0, temperatures );
        init< jointmode::mode >( jointmode::error, joint_modes );
    }

    /// Check that the given pose ( 6 joint angles in radian ) match the current arm's physical pose
    /// Pre condition, the state must be 'running'
    bool check_pose( const boost::array< double, joints_num >& pose, const plane_angle_degrees_t& epsilon=(0.5*degree) ) const;
    typedef boost::array< plane_angle_t, joints_num > arm_position_t;
    bool check_pose( const arm_position_t& pose, const plane_angle_degrees_t& epsilon=(0.5*degree) ) const;
};

inline bool status_t::is_stationary( double epsilon ) const
{
    for( std::size_t i=0; i<joints_num; ++i ) { if( std::fabs( this->velocities[i] ) >= epsilon ) { return false; } }
    return true;
}

struct move_config_t : public boost::array< double, joints_num > {};

} } //namespace snark { namespace ur { 

#endif // SNARK_ACTUATORS_UR_ROBOTIC_ARM_DATA_H 
