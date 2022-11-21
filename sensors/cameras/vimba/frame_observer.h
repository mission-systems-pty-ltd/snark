// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney

#ifndef SNARK_SENSORS_VIMBA_FRAME_OBSERVER_H_
#define SNARK_SENSORS_VIMBA_FRAME_OBSERVER_H_

#include <boost/function.hpp>
#include <VimbaCPP/Include/VimbaCPP.h>

namespace snark { namespace vimba {

class frame;

class frame_observer : virtual public AVT::VmbAPI::IFrameObserver
{
    public:
        typedef const boost::function< void( const vimba::frame& ) > callback_fn;

        frame_observer( AVT::VmbAPI::CameraPtr camera
                      , callback_fn callback );

        // Overridden method which is called by the Vimba library
        virtual void FrameReceived( const AVT::VmbAPI::FramePtr pFrame );

    private:
        // This holds our callback routine that will be executed on every received frame
        callback_fn callback_;
};

} } // namespace snark { namespace vimba {

#endif // SNARK_SENSORS_VIMBA_FRAME_OBSERVER_H_
