// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney
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

#include <comma/base/exception.h>
#include <snark/imaging/cv_mat/serialization.h>
#include "error.h"
#include "frame.h"

namespace snark { namespace vimba {

frame::frame( const AVT::VmbAPI::FramePtr& frame_ptr )
    : image_buffer_( NULL )
{
    VmbErrorType status;
    status = SP_ACCESS( frame_ptr )->GetFrameID( frame_id_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetFrameID() failed", status ));
    }
    status = SP_ACCESS( frame_ptr )->GetReceiveStatus( frame_status_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetReceiveStatus() failed", status ));
    }
    status = SP_ACCESS( frame_ptr )->GetHeight( height_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetHeight() failed", status ));
    }
    status = SP_ACCESS( frame_ptr )->GetWidth( width_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetWidth() failed", status ));
    }
    status = SP_ACCESS( frame_ptr )->GetImageSize( size_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetImageSize() failed", status ));
    }
    status = SP_ACCESS( frame_ptr )->GetImage( image_buffer_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetImage() failed", status ));
    }
    status = SP_ACCESS( frame_ptr )->GetPixelFormat( pixel_format_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetPixelFormat() failed", status ));
    }
    status = SP_ACCESS( frame_ptr )->GetTimestamp( timestamp_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetTimeStamp() failed", status ));
    }
}

const char* frame::status_as_string() const
{
    switch( frame_status_ )
    {
        case VmbFrameStatusComplete:   return "Complete";
        case VmbFrameStatusIncomplete: return "Incomplete";
        case VmbFrameStatusTooSmall:   return "Too small";
        case VmbFrameStatusInvalid:    return "Invalid";
    }
    return "";                          // Quiet gcc warning
}

frame::pixel_format_desc frame::format_desc() const
{
    switch( pixel_format_ )
    {
        // Run vimba-cat --list-attributes --verbose and search for PixelFormat
        // to see all allowed formats for a given camera

        case VmbPixelFormatBayerGR12Packed: // BayerGR12Packed
        case VmbPixelFormatBayerRG12Packed: // BayerRG12Packed
        case VmbPixelFormatBayerGB12Packed: // BayerGB12Packed
        case VmbPixelFormatBayerBG12Packed: // BayerBG12Packed
            return pixel_format_desc( CV_8UC1, 1.5 );

        case VmbPixelFormatMono8:       // Mono8
        case VmbPixelFormatBayerGR8:    // BayerGR8
        case VmbPixelFormatBayerRG8:    // BayerRG8
        case VmbPixelFormatBayerBG8:    // BayerGB8
            return pixel_format_desc( CV_8UC1, 1.0 );

        case VmbPixelFormatRgb8:        // RGB8Packed
        case VmbPixelFormatBgr8:        // BGR8Packed
            return pixel_format_desc( CV_8UC3, 1.0 );

        case VmbPixelFormatRgba8:       // RGBA8Packed
        case VmbPixelFormatBgra8:       // BGRA8Packed
            return pixel_format_desc( CV_8UC4, 1.0 );

        case VmbPixelFormatMono10:      // Mono10
        case VmbPixelFormatMono12:      // Mono12
        case VmbPixelFormatMono12Packed:// Mono12Packed
        case VmbPixelFormatMono14:      // Mono14

        case VmbPixelFormatBayerBG10:   // BayerBG10
        case VmbPixelFormatBayerGR12:   // BayerGR12
        case VmbPixelFormatBayerRG12:   // BayerRG12

                                        // RGB10Packed (no obvious mapping)
        case VmbPixelFormatRgb12:       // RGB12Packed

        case VmbPixelFormatYuv411:      // YUV411Packed
        case VmbPixelFormatYuv422:      // YUV422Packed
        case VmbPixelFormatYuv444:      // YUV444Packed

        default:
            COMMA_THROW( comma::exception, "unsupported format " << std::hex << pixel_format_ );
    }
}

} } // namespace snark { namespace vimba {