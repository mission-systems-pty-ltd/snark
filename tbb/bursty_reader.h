// Copyright (c) 2011 The University of Sydney
// Copyright (c) 2022 Vsevolod Vlaskine

#pragma once

#include <boost/bind/bind.hpp>
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
        bursty_reader( boost::function0< T > produce, unsigned int size = 0, unsigned int capacity = 0, bool on_demand = false );

        ~bursty_reader();

        void stop();

        void join();

        typedef typename snark::tbb::template filter< void, T >::type filter_type;

        filter_type& filter() { return _filter; }

        const ::tbb::concurrent_bounded_queue< T >& queue() { return _queue; }

        T read();

    private:
        T _read( ::tbb::flow_control& flow );
        void _produce_loop();

        ::tbb::concurrent_bounded_queue< T > _queue;
        unsigned int _size;
        bool _on_demand{false};
        bool _running;
        boost::function0< T > _produce;
        filter_type _filter;
        boost::scoped_ptr< boost::thread > _thread;
};

template< typename T > inline bursty_reader< T >::bursty_reader( boost::function0< T > produce, unsigned int size, unsigned int capacity, bool on_demand )
    : _size( size )
    , _on_demand( on_demand )
    , _running( true )
    , _produce( produce )
    , _filter( tbb::filter_mode::serial_in_order, boost::bind( &bursty_reader< T >::_read, this, boost::placeholders::_1 ) )
{
    if( capacity > 0 ) { _queue.set_capacity( capacity ); }
    if( !on_demand ) { _thread.reset( new boost::thread( boost::bind( &bursty_reader< T >::_produce_loop, this ) ) ); }
}

template< typename T > inline bursty_reader< T >::~bursty_reader() { join(); }

template < typename T > inline void bursty_reader< T >::stop() { _running = false; _queue.abort(); }

template < typename T > inline void bursty_reader< T >::join() { stop(); _thread->join(); }

template < typename T > inline T bursty_reader< T >::_read( ::tbb::flow_control& flow )
{
    if( _on_demand && !_thread ) { _thread.reset( new boost::thread( boost::bind( &bursty_reader< T >::_produce_loop, this ) ) ); }
    try
    {
        while( true )
        {
            T t = T();
            _queue.pop( t );
            if( !bursty_reader_traits< T >::valid( t ) ) { flow.stop(); return T(); }
            if( _size == 0 || _queue.size() < _size ) { return t; }
        }
    }
    catch( ::tbb::user_abort& ) {}
    catch( ... ) { flow.stop() ; throw ; }
    flow.stop();
    return T();
}

template < typename T > inline T bursty_reader< T >::read()
{
    if( _on_demand && !_thread ) { _thread.reset( new boost::thread( boost::bind( &bursty_reader< T >::_produce_loop, this ) ) ); }
    try
    {
        while( true )
        {
            T t = T();
            _queue.pop( t );
            if( !bursty_reader_traits< T >::valid( t ) ) { stop(); return T(); }
            if( _size == 0 || _queue.size() < _size ) { return t; }
        }
    }
    catch( ::tbb::user_abort& ) {}
    catch( ... ) { stop() ; throw ; }
    stop();
    return T();
}

template < typename T > inline void bursty_reader< T >::_produce_loop()
{
    try
    {
        for( bool done{false}; _running && !done; )
        {
            T t = _produce(); // attention: if produce is blocking, it may, well, block on join(), if no new data is coming... something to fix or parametrize (e.g. with timed wait)?
            if( !_running ) { _queue.push( T() ); break; }
            _queue.push( t );
            if( !bursty_reader_traits< T >::valid( t ) ) { break; }
        }
    }
    catch( ::tbb::user_abort& ) {}
    catch( ... ) { _queue.push( T() ); throw; }
    _queue.push( T() );
}

} } // namespace snark { namespace tbb {
