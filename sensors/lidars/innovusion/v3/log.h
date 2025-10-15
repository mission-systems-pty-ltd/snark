// Copyright (c) 2025 Mission Systems Pty Ltd
//
// a light wrapper around the Innovusion logging api

#pragma once

#include <sdk_common/inno_lidar_api.h>

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
    bool publish_msg( InnoMessageLevel level ) const { return level <= msg_level; }

private:
    InnoMessageLevel msg_level;
};

} } // namespace snark { namespace innovusion {
