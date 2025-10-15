// Copyright (c) 2025 Mission Systems Pty Ltd

#include "log.h"
#include <unistd.h>

namespace snark { namespace innovusion {

log::log()
{
    set_logs();
    set_log_level( log::Level::warn );
}

void log::set_logs()
{
    // Set log handling
    //
    // As we send the log messages to stderr rather than saving to a file all
    // the log rotation parameters are not relevant
    //
    // Note that we have to call this method before we call inno_lidar_set_log_level()
    // as that method will, by default, send message output to stdout
    inno_lidar_set_logs(
        STDERR_FILENO,                  // destination fd for INFO level logs and below
        STDERR_FILENO,                  // destination fd for WARNING level logs and above
        nullptr                         // log_callback
    );
}

// There are three more log levels beyond INFO: TRACE, DETAIL and MAX
// but they log an insane amount so we'll set the debug level to be INFO
// Even INFO we'll only activate for --debug, not for --verbose
void log::set_log_level( log::Level level )
{
    switch( level )
    {
        case log::Level::warn:
            inno_lidar_set_log_level( INNO_LOG_WARNING_LEVEL );
            log_level = INNO_LOG_WARNING_LEVEL;
            break;
        case log::Level::info:
            inno_lidar_set_log_level( INNO_LOG_WARNING_LEVEL );
            log_level = INNO_LOG_WARNING_LEVEL;
            break;
        case log::Level::debug:
            inno_lidar_set_log_level( INNO_LOG_INFO_LEVEL );
            log_level = INNO_LOG_INFO_LEVEL;
            break;
        case log::Level::all:
            inno_lidar_set_log_level( INNO_LOG_EVERYTHING_LEVEL );
            log_level = INNO_LOG_EVERYTHING_LEVEL;
            break;
    }
}

} } // namespace snark { namespace innovusion {
