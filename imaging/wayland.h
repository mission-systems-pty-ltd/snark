// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2026 Vsevolod Vlaskine
// All rights reserved.

#pragma once

#include <cstdlib>
#include <string>

namespace snark { namespace wayland {

/// starting from ubuntu 26.04, wayland display server
/// does not let gui applications set window size or
/// window position (for security reasons)
/// call this function before creating a window in your
/// application
inline void use_permissive_window_management()
{
    const char* session_env = std::getenv( "XDG_SESSION_TYPE" );
    std::string session = session_env ? session_env : "";
    const char* wayland_display = std::getenv( "WAYLAND_DISPLAY" ); // todo? session type may be 'tty' if used over ssh
    if( session != "wayland" && !wayland_display ) { return; }
    setenv( "QT_QPA_PLATFORM", "xcb", 1 );
}

} } // namespace snark { namespace wayland {
