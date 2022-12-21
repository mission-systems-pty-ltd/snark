// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney
// Copyright (c) 2022 Mission Systems Pty Ltd

#include <comma/base/exception.h>
#include "../../../imaging/cv_mat/serialization.h"
#include "error.h"
#include "frame.h"

namespace snark { namespace vimba {

frame::frame( const AVT::VmbAPI::FramePtr& frame_ptr )
    : image_buffer_( NULL )
{
    VmbErrorType status;
    status = frame_ptr->GetFrameID( frame_id_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetFrameID() failed", status ));
    }
    status = frame_ptr->GetReceiveStatus( frame_status_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetReceiveStatus() failed", status ));
    }
    status = frame_ptr->GetHeight( height_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetHeight() failed", status ));
    }
    status = frame_ptr->GetWidth( width_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetWidth() failed", status ));
    }
    status = frame_ptr->GetImageSize( size_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetImageSize() failed", status ));
    }
    status = frame_ptr->GetImage( image_buffer_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetImage() failed", status ));
    }
    status = frame_ptr->GetPixelFormat( pixel_format_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetPixelFormat() failed", status ));
    }
    status = frame_ptr->GetTimestamp( timestamp_ );
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "GetTimeStamp() failed", status ));
    }
}

void frame::print_stats( std::ostream& os ) const
{
    os << "frame_id: " << frame_id_ << "\n";
    os << "frame_status: " << frame_status_ << "\n";
    os << "height: " << height_ << "\n";
    os << "width: " << width_ << "\n";
    os << "size: " << size_ << "\n";
    os << "pixel_format: " << pixel_format_ << "\n";
    os << "timestamp: " << timestamp_ << std::endl;
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
        case VmbPixelFormatMono12Packed:// Mono12Packed
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
        case VmbPixelFormatMono14:      // Mono14
        case VmbPixelFormatBayerBG10:   // BayerBG10
        case VmbPixelFormatBayerGR12:   // BayerGR12
        case VmbPixelFormatBayerRG12:   // BayerRG12
            return pixel_format_desc( CV_16UC1, 1.0 );

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
