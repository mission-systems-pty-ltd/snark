// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney
// Copyright (c) 2022 Mission Systems Pty Ltd

#ifndef SNARK_SENSORS_VIMBA_SYSTEM_H_
#define SNARK_SENSORS_VIMBA_SYSTEM_H_

#include <VimbaCPP/Include/VimbaSystem.h>

#define QUOTED( arg ) #arg
#define STRINGIZED( arg ) QUOTED( arg )

namespace snark { namespace vimba {

class system
{
public:
    typedef std::tuple< int, int, int > version_t;

    system();
    ~system() { instance.Shutdown(); }

    static std::string sdk_version() { return STRINGIZED( VIMBA_SDK_DOTTED_VERSION ); }
    static VmbVersionInfo_t vmb_version();
    static version_t version();
    static AVT::VmbAPI::CameraPtrVector cameras();
    static AVT::VmbAPI::CameraPtr open_camera( const std::string& id );
    static AVT::VmbAPI::CameraPtr open_first_camera();

private:
    static AVT::VmbAPI::VimbaSystem& instance;
};

} } // namespace snark { namespace vimba {

#endif // SNARK_SENSORS_VIMBA_SYSTEM_H_
