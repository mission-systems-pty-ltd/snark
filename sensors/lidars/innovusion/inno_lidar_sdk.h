// Copyright (c) 2025 Mission Systems Pty Ltd

// convenience header that loads the appropriate version of the other headers

#pragma once

#include "version.h"

#if INNOVUSION_VERSION_MAJOR == 1
#include "v1/lidar.h"
#include "v1/log.h"
#include "v1/traits.h"
#elif INNOVUSION_VERSION_MAJOR == 3
#include "v3/lidar.h"
#include "v3/log.h"
#include "v3/traits.h"
#else
#error Unsupported version of Innovusion SDK
#endif
