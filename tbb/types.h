// Copyright (c) 2022 Vsevolod Vlaskine

/// @author vsevolod Vlaskine

#pragma once

#if TBB_VERSION_MAJOR >= 2021 // it sucks
#include <tbb/info.h>
#else
#include <tbb/task_scheduler_init.h>
#endif

namespace snark { namespace tbb {

#if TBB_VERSION_MAJOR >= 2021 // it sucks

template < typename In, typename Out > struct filter { typedef ::tbb::filter< In, Out > type; };

typedef ::tbb::filter_mode filter_mode;

inline unsigned int default_concurrency() { return ::tbb::info::default_concurrency(); }

#else

template < typename In, typename Out > struct filter { typedef ::tbb::filter_t< In, Out > type; };

enum filter_mode { parallel = ::tbb::filter::parallel, serial_in_order = ::tbb::filter::serial_in_order };

inline unsigned int default_concurrency() { return ::tbb::task_scheduler_init::default_num_threads(); }

#endif

} } // namespace snark { namespace tbb {
