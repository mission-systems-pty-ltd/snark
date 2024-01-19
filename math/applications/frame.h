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


#ifndef SNARK_POINTS_FRAME_HEADER
#define SNARK_POINTS_FRAME_HEADER

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <boost/optional.hpp>

#include <Eigen/Geometry>
#include <comma/base/exception.h>
#include <comma/csv/options.h>
#include <comma/csv/names.h>
#include <comma/csv/stream.h>
#include <comma/io/stream.h>
#include "../../visiting/eigen.h"
#include "../rotation_matrix.h"
#include "timestamped.h"

namespace snark{ namespace applications {

struct position // todo: phase out, use snark::pose
{
    position() : coordinates( Eigen::Vector3d::Zero() ), orientation( Eigen::Vector3d::Zero() ) {}
    position( const Eigen::Vector3d& p ) : coordinates( p ), orientation( Eigen::Vector3d::Zero() ) {}
    position( const Eigen::Vector3d& p, const Eigen::Vector3d& o ) : coordinates( p ), orientation( o ) {}
    Eigen::Vector3d coordinates;
    Eigen::Vector3d orientation;
};

} }

namespace comma { namespace visiting {

template <> struct traits< snark::applications::position >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::applications::position& p, Visitor& v )
    {
        v.apply( "x", p.coordinates.x() );
        v.apply( "y", p.coordinates.y() );
        v.apply( "z", p.coordinates.z() );
        v.apply( "roll", p.orientation.x() );
        v.apply( "pitch", p.orientation.y() );
        v.apply( "yaw", p.orientation.z() );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::applications::position& p, Visitor& v )
    {
        v.apply( "x", p.coordinates.x() );
        v.apply( "y", p.coordinates.y() );
        v.apply( "z", p.coordinates.z() );
        v.apply( "roll", p.orientation.x() );
        v.apply( "pitch", p.orientation.y() );
        v.apply( "yaw", p.orientation.z() );
    }
};

} }

namespace snark{ namespace applications {

class frame
{
    public:
        typedef timestamped< position, boost::posix_time::ptime > position_type;
        
        typedef position_type point_type;

        frame( const position& p, bool to = false, bool interpolate = true, bool rotation_present = false );

        frame( const comma::csv::options& options, bool discardOutOfOrder, boost::optional< boost::posix_time::time_duration > maxGap, bool outputframe, bool to = false, bool interpolate = true, bool rotation_present = false );

        ~frame();

        const point_type* converted( const point_type& rhs );

        bool discarded() const { return m_discarded; }

        const position& last() const { return m_position; }

        const bool outputframe;
        
        boost::posix_time::time_duration discarded_time_diff() const { return m_discarded_time_diff; }

    private:
        void set_position( const position& p );

        const point_type& convert( const point_type& rhs );

        bool m_to; /// to frame if true, from frame else
        bool m_interpolate; /// interpolate nav data
        position m_position;
        ::Eigen::Vector3d m_orientation; /// roll pitch yaw
        ::Eigen::Matrix3d m_rotation; ///rotation
        ::Eigen::Affine3d m_transform; /// fromframe or toframe transform
        ::Eigen::Translation3d m_translation; /// translation

        boost::scoped_ptr< comma::io::istream > m_istream;
        boost::scoped_ptr< comma::csv::input_stream< position_type > > m_is;
        std::pair< position_type, position_type > m_pair; /// nav pair
        point_type m_converted;
        bool m_discardOutOfOrder;
        bool m_discarded;
        boost::optional< boost::posix_time::time_duration > m_maxGap;
        bool rotation_present_;
        boost::posix_time::time_duration m_discarded_time_diff;
};

} } // namespace snark{ namespace applications {

#endif // SNARK_POINTS_FRAME_HEADER
