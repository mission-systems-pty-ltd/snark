// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney
// Copyright (c) 2023 Mission Systems Pty Ltd

#include "error.h"
#include "system.h"
#include <comma/base/exception.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

namespace snark { namespace vimba {

AVT::VmbAPI::VimbaSystem& system::instance = AVT::VmbAPI::VimbaSystem::GetInstance();

// In Vimba 6.1 (and probably other versions ) AVT::VmbAPI::VimbaSystem::Startup()
// outputs a '\n' byte to stdout. This breaks any sort of image processing pipeline
// (since output_frame also outputs to stdout).
//
// We fix this by redirecting stdout to /dev/null whilst calling the SDK (just here).
//
// You will know if the SDK exhibits this behaviour if (without the below fix)
// there's an extra line with --list-cameras. e.g.
// $ vimba-cat --list-cameras
//
// id="<dev-id>",name="Allied Vision 1800 C-507c",serial="<serial-num>",interface_type="CSI2",interface_id="VimbaCSIInterface_0x0"
// $
system::system()
{
    // see https://stackoverflow.com/questions/4832603/how-could-i-temporary-redirect-stdout-to-a-file-in-a-c-program
    // temporarily redirect stdout to /dev/null
    fflush( stdout );
    int orig = dup( 1 );                // save current stdout fd
    int devnull = open( "/dev/null", O_WRONLY ); // create fd for /dev/null
    dup2( devnull, 1 );                 // copy devnull fd to stdout fd (effectively assigning stdout to /dev/null)
    close( devnull );                   // we don't need this any more now that it's copied

    // calls AVT::VmbAPI::VimbaSystem::Startup() which outputs byte to stdout
    VmbErrorType status = instance.Startup();

    // restore stdout
    fflush( stdout );
    dup2( orig, 1 );                    // copy our saved fd to stdout fd (effectively restoring stdout to original setting)
    close( orig );                      // we don't need this any more

    if( status != VmbErrorSuccess ) { COMMA_THROW( comma::exception, error_msg( "Failed to start API", status )); }
}

VmbVersionInfo_t system::vmb_version()
{
    VmbVersionInfo_t version;
    VmbErrorType status = instance.QueryVersion( version );
    if( status == VmbErrorSuccess ) { return version; }
    COMMA_THROW( comma::exception, error_msg( "QueryVersion() failed", status ));
}

system::version_t system::version()
{
    VmbVersionInfo_t v = vmb_version();
    return version_t( v.major, v.minor, v.patch );
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
