// Copyright (c) 2011 The University of Sydney

#include <boost/bind.hpp>
#include <comma/base/last_error.h>
#include "pipeline.h"

namespace snark { namespace imaging { namespace applications {

namespace filters {

/// constructor
/// @param fields csv output fields
/// @param format csv output format
/// @param filters string describing the filters
/// @param size max buffer size
/// @param mode output mode
template < typename H >
pipeline< H >::pipeline( cv_mat::serialization& output
                       , const std::string& filters
                       , tbb::bursty_reader< pair >& reader
                       , unsigned int number_of_threads )
    : m_output( output )
    , m_filters( filters_type::make( filters ) )
    , m_reader( reader )
    , m_pipeline( number_of_threads )
{
    setup_pipeline_();
}

template < typename H >
pipeline< H >::pipeline( cv_mat::serialization& output
                       , const std::vector< filter_type >& filters
                       , tbb::bursty_reader< pair >& reader
                       , unsigned int number_of_threads )
    : m_output( output )
    , m_filters( filters )
    , m_reader( reader )
    , m_pipeline( number_of_threads )
{
    setup_pipeline_();
}

/// write frame to std out
template < typename H >
void pipeline< H >::write_( pair p )
{
    if( p.second.empty() ) { m_reader.stop(); return; }
    if( !std::cout.good() ) { m_reader.stop(); }
    // We use write_to_stdout() rather than write() because we saw issues with using std::cout.
    // See serialization.cpp for details.
    try
    {
        m_output.write_to_stdout( p );
    }
    catch( const comma::last_error::exception& ex )
    {
        if( ex.value != EPIPE ) { m_error = ex.what(); }
        m_reader.stop();
    }
    catch( const comma::exception& ex )
    {
        m_error = ex.what();
        m_reader.stop();
    }
}

template < typename H >
void pipeline< H >::null_( pair p )
{
    if( p.second.empty() || !std::cout.good() ) { m_reader.stop(); }
}

/// setup the pipeline
/// @param filters name-value string describing the filters
template < typename H >
void pipeline< H >::setup_pipeline_()
{
    if( m_filters.empty() )
    {
        m_filter = typename tbb::filter< pair, void >::type( tbb::filter_mode::serial_in_order, boost::bind( &pipeline< H >::write_, this, _1 ) );
    }
    else
    {
        typename tbb::filter< pair, pair >::type all_filters;
        bool has_null = false;
        for( std::size_t i = 0; i < m_filters.size(); ++i )
        {
            tbb::filter_mode mode = m_filters[i].parallel ? tbb::filter_mode::parallel : tbb::filter_mode::serial_in_order;
            if( !m_filters[i].filter_function ) { has_null = true; break; }
            typename tbb::filter< pair, pair >::type filter( mode, boost::bind( m_filters[i].filter_function, _1 ) );
            all_filters = i == 0 ? filter : ( all_filters & filter );
        }
        m_filter = all_filters & typename tbb::filter< pair, void >::type( tbb::filter_mode::serial_in_order, boost::bind( has_null ? &pipeline< H >::null_ : &pipeline< H >::write_, this, _1 ) );
    }
}

template < typename H >
void pipeline< H >::run()
{
    m_pipeline.run( m_reader, m_filter );
    m_reader.join();
}

template class pipeline< boost::posix_time::ptime >;
template class pipeline< snark::cv_mat::serialization::header::buffer_t >;

} // namespace filters {

} } } // namespace snark { namespace imaging { namespace applications {
