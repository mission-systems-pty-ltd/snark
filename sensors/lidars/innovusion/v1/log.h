// Copyright (c) 2025 Mission Systems Pty Ltd
//
// a light wrapper around the Innovusion logging api

#pragma once

#include "../common/log.h"

namespace snark { namespace innovusion {

class log : public log_base
{
public:
    log();
    void set_logs() override;
    void set_log_level( Level level ) override;
};

} } // namespace snark { namespace innovusion {
