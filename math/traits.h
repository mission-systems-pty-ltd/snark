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


#pragma once

#include <boost/array.hpp>
#include <Eigen/Core>
#include "position.h"
#include "roll_pitch_yaw.h"

namespace snark { namespace math {

template < typename T >
struct traits
{
    enum { size = 1 };
    T zero() { return T( 0 ); }
    T one() { return T( 1 ); }
    T identity() { return T( 1 ); }
};

template < typename T, int Rows, int Columns >
struct traits< ::Eigen::Matrix< T, Rows, Columns > >
{
    enum { rows = Rows, columns = Columns, size = rows * columns };

    static const ::Eigen::Matrix< T, Rows, Columns >& zero()
    {
        static ::Eigen::Matrix< T, Rows, Columns > z = ::Eigen::Matrix< T, Rows, Columns >::Zero();
        return z;
    }

    static const ::Eigen::Matrix< T, Rows, Columns >& indentity()
    {
        static ::Eigen::Matrix< T, Rows, Columns > i = ::Eigen::Matrix< T, Rows, Columns >::Identity();
        return i;
    }
};

template < typename T, std::size_t Size >
class traits< boost::array< T, Size > >
{
    public:
        enum { size = Size };

        static const boost::array< T, Size >& zero() { static boost::array< T, Size > z = zero_(); return z; }

    private:
        static boost::array< T, Size > zero_()
        {
            boost::array< T, Size > z;
            for( std::size_t i = 0; i < Size; ++i ) { z[i] = math::traits< T >::zero(); }
            return z;
        }
};

template <> struct traits< snark::roll_pitch_yaw > // quick and dirty
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::roll_pitch_yaw& p, Visitor& v )
    {
        v.apply( "roll", p.x() );
        v.apply( "pitch", p.y() );
        v.apply( "yaw", p.z() );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::roll_pitch_yaw& p, Visitor& v )
    {
        v.apply( "roll", p.x() );
        v.apply( "pitch", p.y() );
        v.apply( "yaw", p.z() );
    }
};

template <> struct traits< snark::position >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::position& p, Visitor& v )
    {
        v.apply( "coordinates", p.coordinates );
        v.apply( "orientation", p.orientation );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::position& p, Visitor& v )
    {
        v.apply( "coordinates", p.coordinates );
        v.apply( "orientation", p.orientation );
    }
};

} } // namespace snark{ namespace math{
