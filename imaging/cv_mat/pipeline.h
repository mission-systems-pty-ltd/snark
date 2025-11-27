// Copyright (c) 2011 The University of Sydney

#pragma once

#ifdef WIN32
#include <winsock2.h>
#endif

#include "../../tbb/bursty_reader.h"
#include "../../tbb/types.h"
#include "bursty_pipeline.h"
#include "filters.h"
#include "serialization.h"

namespace snark {

namespace tbb {

template <>
struct bursty_reader_traits< std::pair< boost::posix_time::ptime, cv::Mat > >
{
    static bool valid( const std::pair< boost::posix_time::ptime, cv::Mat >& p ) { return ( !p.second.empty() ); }
};

template <>
struct bursty_reader_traits< std::pair< snark::cv_mat::serialization::header::buffer_t, cv::Mat > >
{
    static bool valid( const std::pair< snark::cv_mat::serialization::header::buffer_t, cv::Mat >& p ) { return ( !p.second.empty() ); }
};

} // namespace tbb {

namespace imaging { namespace applications {

namespace filters {

/// base class for video processing, capture images in a serarate thread, apply filters, serialize to stdout
template < typename H = boost::posix_time::ptime >
class pipeline
{
    public:
        typedef std::pair< H, cv::Mat > pair;
        typedef snark::cv_mat::impl::filters< H > filters_type;
        typedef snark::cv_mat::operation< cv::Mat, H > filter_type;
        typedef tbb::bursty_reader< pair > reader_type;

        pipeline( cv_mat::serialization& output
                , const std::string& filters
                , reader_type& reader
                , unsigned int number_of_threads = 0 );

        pipeline( cv_mat::serialization& output
                , const std::vector< filter_type >& filters
                , reader_type& reader
                , unsigned int number_of_threads = 0 );

        void run();

        void run_serially();

        const std::string& error() { return m_error; }

    protected:
        void write_( pair p );
        void null_( pair p );
        void setup_pipeline_();

        cv_mat::serialization& m_output;
        typename tbb::filter< pair, void >::type m_filter;
        std::vector< filter_type > m_filters;
        tbb::bursty_reader< pair >& m_reader;
        tbb::bursty_pipeline< pair > m_pipeline;
        std::string m_error;
};

} // namespace impl {

typedef filters::pipeline<> pipeline;
typedef filters::pipeline< snark::cv_mat::serialization::header::buffer_t > pipeline_with_header;


} }  // namespace imaging { namespace applications {

} // namespace snark
