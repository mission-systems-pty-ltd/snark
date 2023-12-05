// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <boost/assign.hpp>
#include <boost/bimap.hpp>
#include <boost/bind/bind.hpp>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include "flycapture.h"
#include "helpers.h"

namespace snark{ namespace cameras{ namespace flycapture{

    void assert_ok(const FlyCapture2::Error error, const std::string& error_string)
    {
        if (error != FlyCapture2::PGRERROR_OK)
            { COMMA_THROW(comma::exception, error_string + " (" + std::string(error.GetDescription()) +")"); }
    }

    const std::string get_interface_string( FlyCapture2::InterfaceType interface )
    {
        switch (interface)
        {
            case FlyCapture2::INTERFACE_IEEE1394:
            return "IEEE1394";
            case FlyCapture2::INTERFACE_USB2:
            return "USB2";
            case FlyCapture2::INTERFACE_USB3:
            return "USB3";
            case FlyCapture2::INTERFACE_GIGE:
            return "GigE";
            default:
            return "UNKNOWN";
        }
    }

    unsigned int bits_per_pixel( const FlyCapture2::PixelFormat pixel_format )
    {
        switch( pixel_format )
        {
            case FlyCapture2::PIXEL_FORMAT_MONO8:
            case FlyCapture2::PIXEL_FORMAT_RAW8:
            return 8;
            break;
            case FlyCapture2::PIXEL_FORMAT_RAW16:
            case FlyCapture2::PIXEL_FORMAT_MONO16:
            return 16;
            case FlyCapture2::PIXEL_FORMAT_RGB:
            case FlyCapture2::PIXEL_FORMAT_BGR:
            return 24;
            case FlyCapture2::PIXEL_FORMAT_RGBU:
            case FlyCapture2::PIXEL_FORMAT_BGRU:
            return 32;
            case FlyCapture2::PIXEL_FORMAT_RGB16:
            return 48;
            case FlyCapture2::PIXEL_FORMAT_411YUV8:
            case FlyCapture2::PIXEL_FORMAT_422YUV8:
            case FlyCapture2::PIXEL_FORMAT_444YUV8:
            COMMA_THROW( comma::exception, "unsupported format " << pixel_format );
            default:
            COMMA_THROW( comma::exception, "unknown format " << pixel_format );  
            return 0;
        }
    }

    cv::Mat image_as_cvmat( const FlyCapture2::Image& frame )
    {
        int type;
        switch( bits_per_pixel(frame.GetPixelFormat() ) ) 
        {
            case 8:
            type = CV_8UC1;
            break;
            case 16:
            type = CV_16UC1;
            break;
            case 24:
            type = CV_8UC3;
            break;
            case 32:
            type = CV_8UC4;
            break;
            case 48:
            type = CV_16UC3;
            break;
            default:
            COMMA_THROW( comma::exception, "unknown format " << frame.GetPixelFormat()  );
        };

        return cv::Mat( frame.GetRows(), frame.GetCols(), type, frame.GetData() );
    }

    int get_cv_type( const FlyCapture2::Image& frame )
    {
        switch( bits_per_pixel(frame.GetPixelFormat() ) ) 
        {
            case 8: return CV_8UC1;
            case 16: return CV_16UC1;
            case 24: return CV_8UC3;
            case 32:return CV_8UC4;
            case 48: return CV_16UC3;
            default:
            COMMA_THROW( comma::exception, "unknown format " << frame.GetPixelFormat()  );
        };
    }
} } } // namespace snark{ namespace cameras{ namespace flycapture
