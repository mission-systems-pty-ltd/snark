// Copyright (c) 2022 Vsevolod Vlaskine

/// @author vsevolod Vlaskine

#pragma once

#ifdef TBB_VERSION_H_EXISTS
#include <tbb/version.h>
#endif

#if defined( TBB_VERSION_MAJOR ) && TBB_VERSION_MAJOR >= 2021 // it sucks

#include <tbb/info.h>
#include <tbb/parallel_pipeline.h>
#include <tbb/task_group.h>

namespace snark { namespace tbb {

template < typename In, typename Out > struct filter { typedef ::tbb::filter< In, Out > type; };

typedef ::tbb::filter_mode filter_mode;

inline unsigned int default_concurrency() { return ::tbb::info::default_concurrency(); } // quick and dirty

inline bool cancel_group_execution() { return tbb::task_group_context().cancel_group_execution(); }

} } // namespace snark { namespace tbb {

#else // #if defined( TBB_VERSION_MAJOR ) && TBB_VERSION_MAJOR >= 2021

#include <tbb/pipeline.h>
#include <tbb/task.h>
#include <tbb/task_scheduler_init.h>
#include <tbb/tbb_exception.h>

namespace snark { namespace tbb {

template < typename In, typename Out > struct filter { typedef ::tbb::filter_t< In, Out > type; };

typedef ::tbb::filter::mode filter_mode;

inline unsigned int default_concurrency() { return ::tbb::task_scheduler_init::default_num_threads(); } // quick and dirty

inline bool cancel_group_execution() { return ::tbb::task::self().cancel_group_execution(); }

} } // namespace snark { namespace tbb {

#endif // #if defined( TBB_VERSION_MAJOR ) && TBB_VERSION_MAJOR >= 2021
