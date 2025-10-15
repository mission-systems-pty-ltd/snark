// Copyright (c) 2025 Mission Systems Pty Ltd
//
// a light wrapper around the Innovusion logging api

#pragma once

#include <inno_lidar_api.h>

namespace snark { namespace innovusion {

class log
{
public:
    // define log levels in an app-centric way
    // this differs from the api-level definitions
    // see log.cpp for the mapping
    enum class Level { warn, info, debug, all };

    log();
    void set_logs();
    void set_log_level( Level level );
    bool publish_msg( inno_log_level level ) const { return level <= log_level; }

private:
    inno_log_level log_level;
};

} } // namespace snark { namespace innovusion {
