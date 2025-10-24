// Copyright (c) 2025 Mission Systems Pty Ltd
//
// a light wrapper around the Innovusion logging api

#pragma once

#include "sdk_types.h"

namespace snark { namespace innovusion {

class log_base
{
public:
    // define log levels in an app-centric way
    // this differs from the sdk-level definitions
    // see log.cpp in the version directories for the mapping
    enum class Level { warn, info, debug, all };

    virtual ~log_base() = default;

    virtual void set_logs() = 0;
    virtual void set_log_level( Level level ) = 0;
    bool publish_msg( sdk::msg_level level ) const { return level <= msg_level; }

protected:
    sdk::msg_level msg_level;
};

} } // namespace snark { namespace innovusion {
