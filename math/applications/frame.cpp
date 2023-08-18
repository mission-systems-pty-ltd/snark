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

#include <boost/thread/thread_time.hpp>
#include "../../math/rotation_matrix.h"
#include "frame.h"

namespace snark{ namespace applications {

frame::frame( const position& p, bool to, bool interpolate, bool rotation_present )
    : outputframe( false )
    , m_to( to )
    , m_interpolate( interpolate )
    , m_rotation( ::Eigen::Matrix3d::Identity() )
    , m_discarded( false )
    , rotation_present_( rotation_present )
{
    set_position( p );
}

frame::frame( const comma::csv::options& options, bool discardOutOfOrder, boost::optional< boost::posix_time::time_duration > maxGap, bool outputframe, bool to, bool interpolate, bool rotation_present )
    : outputframe( outputframe )
    , m_to( to )
    , m_interpolate( interpolate )
    , m_rotation( ::Eigen::Matrix3d::Identity() )
    , m_istream( new comma::io::istream( options.filename, options.binary() ? comma::io::mode::binary : comma::io::mode::ascii, comma::io::mode::blocking ) )
    , m_discardOutOfOrder( discardOutOfOrder )
    , m_maxGap( maxGap )
    , rotation_present_( rotation_present )
{
    m_is.reset( new comma::csv::input_stream< position_type >( *( *m_istream )(), options ) );
    const position_type* p = m_is->read(); // todo: maybe not very good, move to value()
    if( p == NULL ) { COMMA_THROW( comma::exception, "tried to read a new bounding frame value from '" << options.filename << "'; got end of stream/file" ); }
    m_pair.first = *p;
    set_position( p->value );
    p = m_is->read(); // todo? move to value()
    if( p == NULL ) { COMMA_THROW( comma::exception, "tried to read a new bounding frame value from '" << options.filename << "'; got end of stream/file" ); }
    m_pair.second = *p;
}

frame::~frame()
{
    if( m_istream ) { m_istream->close(); }
}

const frame::point_type* frame::converted( const point_type& rhs )
{
    if( !m_is ) { return &convert( rhs ); }
    m_discarded = false;
    while( rhs.t >= m_pair.first.t )
    {
        if( ( rhs.t <= m_pair.second.t ) )
        {
            if( m_interpolate )
            {
                if( m_pair.first.t == m_pair.second.t )
                {
                    set_position( m_pair.second.value );
                }
                else
                {
                    double factor = double( ( rhs.t - m_pair.first.t ).total_microseconds() ) / ( m_pair.second.t - m_pair.first.t ).total_microseconds();

                    Eigen::Vector3d coordinates = m_pair.first.value.coordinates * ( 1 - factor ) + m_pair.second.value.coordinates * factor;

                    Eigen::Vector3d orientation;
                    if( m_pair.first.value.orientation == m_pair.second.value.orientation )
                    {
                        orientation = m_pair.first.value.orientation;
                    }
                    else
                    {
                        rotation_matrix r1( m_pair.first.value.orientation );
                        Eigen::Quaterniond q1 = r1.quaternion();
                        rotation_matrix r2( m_pair.second.value.orientation );
                        Eigen::Quaterniond q2 = r2.quaternion();
                        Eigen::Quaterniond slerped( q1.slerp( factor, q2 ));
                        orientation = rotation_matrix( slerped ).roll_pitch_yaw();
                    }
                    position p( coordinates, orientation );
                    set_position( p );
                }
            }
            else
            {
                // keep the first point or take the second point if closer
                if( ( rhs.t - m_pair.first.t ).total_microseconds() * 2u  > ( m_pair.second.t - m_pair.first.t ).total_microseconds() )
                {
                    set_position( m_pair.second.value );
                }
            }
            return &convert( rhs );
        }
        do
        {
            const position_type* p = m_is->read();
            if( p == NULL ) { return NULL; }
            m_pair.first = m_pair.second;
            m_pair.second = *p;
            if( !m_interpolate ) { set_position( m_pair.first.value ); }
        }
        while( m_maxGap && ( m_pair.second.t - m_pair.first.t ) > *m_maxGap );
    }
    if( !m_discardOutOfOrder ) { COMMA_THROW( comma::exception, "expected timestamp not earlier than " << boost::posix_time::to_iso_string( m_pair.first.t ) << "; got " << boost::posix_time::to_iso_string( rhs.t ) << "; use --discard" ); }
    m_discarded = true;
    m_discarded_time_diff=m_pair.first.t - rhs.t;
    return NULL;
}

void frame::set_position( const position& p )
{
    m_position = p;
    m_translation.vector() = ::Eigen::Vector3d( p.coordinates );
    m_rotation = snark::rotation_matrix::rotation( p.orientation );
    m_transform = m_to
                ? m_rotation.transpose() * m_translation.inverse()
                : m_translation * m_rotation;
}


const frame::point_type& frame::convert( const point_type& rhs )
{
    m_converted.t = rhs.t;
    m_converted.value.coordinates = m_transform * rhs.value.coordinates;
    if( rotation_present_ )
    {
        Eigen::Matrix3d m = snark::rotation_matrix::rotation( rhs.value.orientation );
        if( m_to ) { m = m_rotation.transpose() * m; }
        else { m = m_rotation * m; }
        m_converted.value.orientation = snark::rotation_matrix::roll_pitch_yaw( m );
    }
    return m_converted;
}

} } // namespace snark{ namespace applications {
