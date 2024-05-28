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

#include <boost/thread/condition.hpp>
#include <tbb/concurrent_queue.h>

namespace snark { namespace tbb { 

class counter
{
    public:
        /// constructor
        counter() : value_( 0 ), shutdown_( false ) {}
        
        /// increment
        unsigned int operator++()
        {
             boost::lock_guard< boost::mutex > lock( mutex_ );
             ++value_;
             condition_.notify_all();
             return value_;
        }
        
        /// decrement, if greater than 0; block, if 0
        unsigned int operator--()
        {
            boost::unique_lock< boost::mutex > lock( mutex_ );
            while( value_ == 0 && !shutdown_ ) { condition_.timed_wait( lock, boost::posix_time::milliseconds( 100 ) ); }
            if( value_ > 0 ) { --value_; }
            return value_;
        }
        
        /// block until not empty
        unsigned int wait_until_non_zero()
        {
            boost::unique_lock< boost::mutex > lock( mutex_ );
            while( value_ == 0 && !shutdown_ ) { condition_.timed_wait( lock, boost::posix_time::milliseconds( 100 ) ); }
            return value_;
        }
        
        /// unlock, if locked in operator--()
        void shutdown() { shutdown_ = true; condition_.notify_all(); }
        
    private:
        boost::condition_variable condition_;
        boost::mutex mutex_;
        unsigned int value_;
        bool shutdown_;
};

/// concurrent queue with wait function
/// @todo use some native mechanism from tbb
///       or at least tbb::vector instead of
///       queue for locking
template < typename T >
class queue
{
    public:
        queue( unsigned int capacity = 0 ) { if( capacity > 0 ) { queue_.set_capacity( capacity ); } }

        void push( const T& t ) { queue_.push( t ); ++counter_; }
        
        void pop( T& t ) { queue_.pop( t ); --counter_; }
        
        unsigned int size() const { return queue_.size(); }
        
        bool empty() const { return queue_.empty(); }
        
        void wait() { counter_.wait_until_non_zero(); }
        
        void shutdown() { counter_.shutdown(); }

    private:
        ::tbb::concurrent_bounded_queue< T > queue_;
        counter counter_;
};
    
} } // namespace snark { namespace tbb {
