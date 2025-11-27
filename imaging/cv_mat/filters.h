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

#include <vector>
#include <boost/function.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <opencv2/core/core.hpp>
#include "serialization.h"

namespace snark { namespace cv_mat {
    
typedef serialization::header::buffer_t header_type;

template < typename Output = cv::Mat, typename H = boost::posix_time::ptime >
struct operation
{
    typedef std::pair< H, cv::Mat > input_type;
    typedef std::pair< H, Output > output_type;
    typedef input_type value_type; // quick and dirty for now
    operation( boost::function< output_type( value_type ) > f, bool p = true ): filter_function( f ), parallel( p ) {}
    operation( const std::pair< boost::function< output_type( value_type ) >, bool >& p ): filter_function( p.first ), parallel( p.second ) {}
    boost::function< output_type( value_type ) > filter_function;
    output_type operator()( value_type v ) { return filter_function( v ); }
    output_type operator()( value_type v ) const { return filter_function( v ); }
    bool parallel;
};

typedef operation<> filter;
typedef operation< cv::Mat, header_type > filter_with_header;

namespace impl {

template < typename H = boost::posix_time::ptime >
struct filters
{
    /// value type
    typedef std::pair< H, cv::Mat > value_type;
    typedef operation< cv::Mat, H > filter_type;
    typedef boost::function< boost::posix_time::ptime( const H& ) > get_timestamp_functor;
    
    /// return filters from name-value string
    static std::vector< filter_type > make( const std::string& how, const get_timestamp_functor& get_timestamp, bool allow_parallelisation = false );
    
    /// return filters from name-value string, backward compatible, supports H=boost::posix_time::ptime only
    static std::vector< filter_type > make( const std::string& how, bool allow_parallelisation = false );
    
    /// apply filters (a helper)
    static value_type apply( std::vector< filter_type >& filters, value_type m );
    
    /// return filter usage
    static const std::string& usage( const std::string & operation = "" );
};

} // namespace impl {

typedef impl::filters< header_type > filters_with_header; // todo: a better name

/// a helper: e.g. take CV_8UC3, return CV_8UC1
int single_channel_type( int t ); // todo: move to utils

} }  // namespace snark { namespace cv_mat {
