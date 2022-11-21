// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney

#ifndef SNARK_SENSORS_VIMBA_SYSTEM_H_
#define SNARK_SENSORS_VIMBA_SYSTEM_H_

#include <VimbaCPP/Include/VimbaSystem.h>

namespace snark { namespace vimba {

class system
{
    public:
        system();
        ~system() { instance.Shutdown(); }

        static VmbVersionInfo_t version();
        static AVT::VmbAPI::CameraPtrVector cameras();
        static AVT::VmbAPI::CameraPtr open_camera( const std::string& id );
        static AVT::VmbAPI::CameraPtr open_first_camera();

    private:
        static AVT::VmbAPI::VimbaSystem& instance;
};

} } // namespace snark { namespace vimba {

#endif // SNARK_SENSORS_VIMBA_SYSTEM_H_
