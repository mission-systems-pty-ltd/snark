// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney
// Copyright (c) 2022 Mission Systems Pty Ltd

#ifndef SNARK_SENSORS_VIMBA_FRAME_H_
#define SNARK_SENSORS_VIMBA_FRAME_H_

#include <VimbaCPP/Include/Frame.h>
#include <iostream>

namespace snark { namespace vimba {

class frame
{
    public:
        struct pixel_format_desc
        {
            pixel_format_desc( int type_, float width_adjustment_ )
                : type( type_ )
                , width_adjustment( width_adjustment_ )
            {}
            int type;
            float width_adjustment;
        };

        frame( const AVT::VmbAPI::FramePtr& frame_ptr );

        VmbUint64_t        id() const { return frame_id_; }
        VmbFrameStatusType status() const { return frame_status_; }
        const char*        status_as_string() const;
        VmbUint32_t        height() const { return height_; }
        VmbUint32_t        width() const { return width_; }
        VmbUint32_t        size() const { return size_; }
        VmbUchar_t*        image_buffer() const { return image_buffer_; }
        VmbPixelFormatType pixel_format() const { return pixel_format_; }
        VmbUint64_t        timestamp() const { return timestamp_; }
        pixel_format_desc  format_desc() const;
        void               print_stats( std::ostream& os = std::cerr ) const;

    private:
        VmbUint64_t        frame_id_;
        VmbFrameStatusType frame_status_;
        VmbUint32_t        height_;
        VmbUint32_t        width_;
        VmbUint32_t        size_;
        VmbUchar_t*        image_buffer_;
        VmbPixelFormatType pixel_format_;
        VmbUint64_t        timestamp_;
};

} } // namespace snark { namespace vimba {

#endif // SNARK_SENSORS_VIMBA_FRAME_H_
