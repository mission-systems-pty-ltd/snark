// Copyright (c) 2011 The University of Sydney

#pragma once

#include "../../tbb/bursty_reader.h"
#include "../../tbb/types.h"

namespace snark { namespace tbb {

/// run a tbb pipeline with bursty data using bursty_reader
/// @todo this class is too trivial; tear down?
template< typename T >
class bursty_pipeline
{
public:
    /// constructor
    /// @param number_of_threads maximum number of threads, 0 means auto
    bursty_pipeline( unsigned int number_of_threads = 0 );

    void run( bursty_reader< T >& reader, const typename tbb::filter< T, void >::type& filter );

private:
    unsigned int number_of_threads_;
};


template < typename T >
inline bursty_pipeline< T >::bursty_pipeline( unsigned int number_of_threads ) : number_of_threads_( number_of_threads > 0 ? number_of_threads : snark::tbb::default_concurrency() ) {}

template < typename T >
inline void bursty_pipeline< T >::run( bursty_reader< T >& reader, const typename tbb::filter< T, void >::type& filter ) { ::tbb::parallel_pipeline( number_of_threads_, reader.filter() & filter ); }

} } // namespace snark { namespace tbb {
