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


/// @author Cedric Wohlleber

#ifndef SNARK_MATH_INTERVAL_H_
#define SNARK_MATH_INTERVAL_H_

#include <boost/optional.hpp>
#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include <Eigen/Core>

namespace snark { namespace math {

/// multi-dimensional interval
template < typename T, unsigned int N >
class closed_interval
{
    public:
        typedef Eigen::Matrix< T, N, 1 > vector_type;

        /// constructor
        closed_interval( const vector_type& min, const vector_type& max ) : m_interval( std::make_pair( get_min( min, max ), get_max( min, max ) ) ) { if( !less_or_equal( min, max ) ) { COMMA_THROW( comma::exception, "invalid interval on: min: " << min.transpose() << ", max: " << max.transpose() ); } }

        /// constructor
        closed_interval( const vector_type& rhs ) : m_interval( std::make_pair( rhs, rhs ) ) {}

        /// default constructor
        closed_interval() {} //interval(): m_interval( std::make_pair( vector_type::Zero(), vector_type::Zero() ) ) {}

        /// return true, if initialized
        operator bool() const { return (bool)m_interval; }

        /// return value
        const std::pair< vector_type, vector_type >& operator()() const { assert( m_interval ); return *m_interval; }

        /// return left boundary (convenience method)
        const vector_type& min() const { assert( m_interval ); return m_interval->first; }

        /// return right boundary (convenience method)
        const vector_type& max() const { assert( m_interval ); return m_interval->second; }

        /// return true, if variable belongs to the interval
        bool contains( const vector_type& rhs ) const { assert( m_interval ); return less_or_equal( m_interval->first, rhs ) && less_or_equal( rhs, m_interval->second ); }

        /// return true, if the whole rhs interval belongs to the interval
        bool contains( const closed_interval& rhs ) const { assert( m_interval && rhs ); return contains( rhs.min() ) && contains( rhs.max() ); }

        /// compute the hull of the interval and [rhs]
        closed_interval< T, N > hull( const vector_type& rhs ) const { return closed_interval( get_min( m_interval->first, rhs ), get_max( m_interval->second, rhs ) ); }

        /// compute the hull of 2 intervals
        closed_interval< T, N > hull( const closed_interval& rhs ) const { return closed_interval( get_min( m_interval->first, rhs.min() ), get_max( m_interval->second, rhs.max() ) ); }

        /// compute and assign the hull of the interval and [rhs]
        const closed_interval< T, N >& set_hull( const vector_type& rhs ) { *this = m_interval ? hull( rhs ) : closed_interval( rhs ); return *this; }

        /// compute and assign the hull of 2 intervals
        const closed_interval< T, N >& set_hull( const closed_interval& rhs ) { *this = m_interval ? hull( rhs ) : closed_interval( rhs ); return *this; }

        /// equality
        bool operator==( const closed_interval& rhs ) const { assert( m_interval && rhs ); return m_interval->first.isApprox( rhs().first ) && m_interval->second.isApprox( rhs().second ); }

        /// unequality
        bool operator!=( const closed_interval& rhs ) const { assert( m_interval && rhs ); return !operator==( rhs ); }

    private:
        // todo: use epsilon from <limits> for comparison
        static bool less_or_equal( const vector_type& lhs, const vector_type& rhs ) { return ( ( lhs.array() <= rhs.array() ).all() ); }
        static vector_type get_min( const vector_type& lhs, const vector_type& rhs ) { return rhs.array().min( lhs.array() ); }
        static vector_type get_max( const vector_type& lhs, const vector_type& rhs ) { return rhs.array().max( lhs.array() ); }

        boost::optional< std::pair< vector_type, vector_type > > m_interval;
};

} } // namespace snark { namespace math {

#endif // SNARK_MATH_INTERVAL_H_
