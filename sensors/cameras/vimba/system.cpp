// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney

#include <comma/base/exception.h>
#include "error.h"
#include "system.h"

namespace snark { namespace vimba {

AVT::VmbAPI::VimbaSystem& system::instance = AVT::VmbAPI::VimbaSystem::GetInstance();

system::system()
{
    VmbErrorType status = instance.Startup();
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "Failed to start API", status ));
    }
}

VmbVersionInfo_t system::version()
{
    VmbVersionInfo_t version;
    VmbErrorType status = instance.QueryVersion( version );
    if( status == VmbErrorSuccess ) { return version; }
    COMMA_THROW( comma::exception, error_msg( "QueryVersion() failed", status ));
}

AVT::VmbAPI::CameraPtrVector system::cameras()
{
    AVT::VmbAPI::CameraPtrVector cameras;
    VmbErrorType status = system::instance.GetCameras( cameras ); // Fetch all cameras
    if( status == VmbErrorSuccess ) { return cameras; }
    COMMA_THROW( comma::exception, error_msg( "GetCameras() failed", status ));
}

AVT::VmbAPI::CameraPtr system::open_first_camera()
{
    AVT::VmbAPI::CameraPtrVector c = cameras();
    if( !c.empty() )
    {
        AVT::VmbAPI::CameraPtr camera = c[0];
        VmbErrorType status = camera->Open( VmbAccessModeFull );
        if( status == VmbErrorSuccess )
        {
            return camera;
        }
        else
        {
            COMMA_THROW( comma::exception, error_msg( "camera::Open() failed", status ));
        }
    }
    else
    {
        COMMA_THROW( comma::exception, "No cameras found" );
    }
}

AVT::VmbAPI::CameraPtr system::open_camera( const std::string& id )
{
    AVT::VmbAPI::CameraPtr camera;
    VmbErrorType status = system::instance.OpenCameraByID( id.c_str(), VmbAccessModeFull, camera );
    if( status == VmbErrorSuccess ) { return camera; }
    COMMA_THROW( comma::exception, error_msg( "OpenCameraById() failed", status ));
}

} } // namespace snark { namespace vimba {
