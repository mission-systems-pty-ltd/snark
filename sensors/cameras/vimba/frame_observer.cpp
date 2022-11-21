// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney

#include "frame.h"
#include "frame_observer.h"

namespace snark { namespace vimba {

frame_observer::frame_observer( AVT::VmbAPI::CameraPtr camera
                              , callback_fn callback )
    : IFrameObserver( camera )
    , callback_( callback )
{}

void frame_observer::FrameReceived( const AVT::VmbAPI::FramePtr frame_ptr )
{
    frame frame( frame_ptr );
    callback_( frame );
    m_pCamera->QueueFrame( frame_ptr );
}

} } // namespace snark { namespace vimba {
