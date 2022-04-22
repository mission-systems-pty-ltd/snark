// Copyright (c) 2011 The University of Sydney
// Copyright (c) 2022 Vsevolod Vlaskine

#pragma once

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <tbb/concurrent_queue.h>
#include "types.h"

namespace snark { namespace tbb {

/// the user can provide a method to validate the data,
/// the pipeline will be shut down if invalid data is received
template< typename T >
struct bursty_reader_traits
{
    static bool valid( const T& t ) { return true; }
};

/// helper class to run a tbb pipeline with bursty data
/// the pipeline has to be closed when no data is received to prevent the main thread to spin
template < typename T >
class bursty_reader
{
    public:
        /// constructor
        /// @param produce the user-provided functor to get data from the data source
        /// @param size maximum input queue size before discarding data, 0 means infinite
        /// @param capacity maximum input queue size before the reader thread blocks
        bursty_reader( boost::function0< T > produce, unsigned int size = 0, unsigned int capacity = 0 );

        ~bursty_reader();

        void stop();

        void join();

        typedef typename snark::tbb::template filter< void, T >::type filter_type;

        filter_type& filter() { return filter_; }

    private:
        T read_( ::tbb::flow_control& flow );
        void produce_loop_();

        ::tbb::concurrent_bounded_queue< T > queue_;
        unsigned int size_;
        bool running_;
        boost::function0< T > produce_;
        filter_type filter_;
        boost::scoped_ptr< boost::thread > thread_;
};

template< typename T >
bursty_reader< T >::bursty_reader( boost::function0< T > produce, unsigned int size, unsigned int capacity )
    : size_( size )
    , running_( true )
    , produce_( produce )
    , filter_( tbb::filter_mode::serial_in_order, boost::bind( &bursty_reader< T >::read_, this, _1 ) )
{
    if( capacity > 0 ) { queue_.set_capacity( capacity ); }
    thread_.reset( new boost::thread( boost::bind( &bursty_reader< T >::produce_loop_, this ) ) );
}

template< typename T >
inline bursty_reader< T >::~bursty_reader()
{
    join();
}

template < typename T >
inline void bursty_reader< T >::stop()
{
    running_ = false;
    queue_.abort();
}

template < typename T >
inline void bursty_reader< T >::join()
{
    stop();
    thread_->join();
}

template < typename T >
inline T bursty_reader< T >::read_( ::tbb::flow_control& flow )
{
    try
    {
        while( true )
        {
            T t;
            queue_.pop( t );
            if( !bursty_reader_traits< T >::valid( t ) ) { flow.stop(); return T(); }
            if( size_ == 0 || queue_.size() < size_ ) { return t; }
        }
    }
    catch( ::tbb::user_abort& ) {}
    catch( ... ) { flow.stop() ; throw ; }
    flow.stop();
    return T();
}

template < typename T >
inline void bursty_reader< T >::produce_loop_()
{
    try
    {
        while( running_ )
        {
            T t = produce_(); // attention: if produce is blocking, it may, well, block on join(), if no new data is coming... something to fix or parametrize (e.g. with timed wait)?
            if( !running_ ) { queue_.push( T() ); break; }
            queue_.push( t );
            if( !bursty_reader_traits< T >::valid( t ) ) { break; }
        }
    }
    catch( ::tbb::user_abort& ) {}
    catch( ... ) { queue_.push( T() ); throw; }
    queue_.push( T() );
}

} } // namespace snark { namespace tbb {
