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

/// @author vsevolod vlaskine

#include <boost/array.hpp>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include "error.h"
#include "camera.h"
#include "stream.h"
#include "pixel_type_names.h"
#include <iostream>

namespace snark { namespace jai {

unsigned int cv_type_from_jai( uint32_t pixel_type, bool raw_output = false )
{
    if( raw_output )
    {
        switch( pixel_type )
        {
            case J_GVSP_PIX_MONO8:
            case J_GVSP_PIX_BAYRG8:
                return CV_8UC1;
            case J_GVSP_PIX_MONO10:
            case J_GVSP_PIX_MONO12:
            case J_GVSP_PIX_BAYRG10:
            case J_GVSP_PIX_BAYRG12:
                return CV_16UC1;
        }
        if( pixel_type_names.left.find( pixel_type ) != pixel_type_names.left.end() ) { COMMA_THROW( comma::exception, "conversion to OpenCV type from Jai pixel type '" << snark::jai::pixel_type_names.left.at( pixel_type ) << "' is not implemented" ); }
        else { COMMA_THROW( comma::exception, "received unrecognized pixel type " << std::hex << "0x" << std::setfill('0') << std::setw(8) << pixel_type ); }
    }
    else
    {
        if( pixel_type & J_GVSP_PIX_MONO ) { return CV_8UC1; }
        if( pixel_type & J_GVSP_PIX_RGB ) { return CV_8UC3; }
        COMMA_THROW( comma::exception, "expected pixel format, got 0 in both mono and rgb bits in: " << pixel_type );
    }
}

struct jai::stream::impl
{
    struct buffer
    {
        BUF_HANDLE handle;
        std::vector< uint8_t > data;
        
        buffer() : handle( NULL ) {}
        
        void allocate( STREAM_HANDLE stream_handle, std::size_t size )
        {
            data.resize( size );
            validate( "announcing buffer of size " + boost::lexical_cast< std::string >( size ), J_DataStream_AnnounceBuffer( stream_handle, &data[0], size, NULL, &handle ) );
            validate( "queueing buffer", J_DataStream_QueueBuffer( stream_handle, handle ) );
        }
    };
    
    class event_buffer
    {
        public:
            event_buffer( STREAM_HANDLE h, EVT_HANDLE e ) : stream_( h )
            {
                uint32_t size = sizeof( void * );
                J_Event_GetData( e, &buffer_,  &size );
            }
            
            template < typename T > void get( _J_BUFFER_INFO_CMD_TYPE what, T& t ) const
            {
                uint32_t size = sizeof( T );
                J_DataStream_GetBufferInfo( stream_, buffer_, what, &t, &size );
            }
            
            BUF_HANDLE buffer() const { return buffer_; }
            
        private:
            mutable STREAM_HANDLE stream_;
            mutable BUF_HANDLE buffer_;
    };
    
    STREAM_HANDLE handle;
    CAM_HANDLE device;
    HANDLE event;
    EVT_HANDLE event_handle;
    std::vector< buffer > buffers;
    bool output_raw_image;
    
    impl( CAM_HANDLE device, std::size_t size, unsigned int number_of_buffers = 1, bool output_raw_image = false )
        : handle( NULL )
        , device( device )
        , event( NULL )
        , buffers( number_of_buffers )
        , output_raw_image( output_raw_image )
    {
        validate( "creating data stream", J_Camera_CreateDataStream( device, 0, &handle ));
        for( unsigned int i = 0; i < buffers.size(); ++i ) { buffers[i].allocate( handle, size ); }
        if( !handle ) { COMMA_THROW( comma::exception, "creating data stream failed" ); }
        validate( "creating condition", J_Event_CreateCondition( &event ) );
        if( !event ) { COMMA_THROW( comma::exception, "creating condition failed" ); }
        J_DataStream_RegisterEvent( handle, EVENT_NEW_BUFFER, event, ( void ** )&event_handle );
        validate( "starting acquisition", J_DataStream_StartAcquisition( handle, ACQ_START_NEXT_IMAGE, 0 ) );
    }
    
    ~impl() { close(); }
    
    std::pair< boost::posix_time::ptime, cv::Mat > read()
    {
        std::pair< boost::posix_time::ptime, cv::Mat > pair;
        bool ready = false;
        while( !ready )
        {
            static const unsigned int timeout = 1000;
            J_COND_WAIT_RESULT wait_result;
            validate( "waiting for condition", J_Event_WaitForCondition( event, timeout, &wait_result ) ); // todo: will exception work with tbb?
            if( closed() ) { return std::pair< boost::posix_time::ptime, cv::Mat >(); } // todo: make thread-safe?
            switch( wait_result )
            {
                case J_COND_WAIT_SIGNAL:
                    ready = true;
                    break;
                case J_COND_WAIT_EXIT:
                    return std::pair< boost::posix_time::ptime, cv::Mat >();
                case J_COND_WAIT_TIMEOUT:
                    continue;
                case J_COND_WAIT_ERROR:
                    COMMA_THROW( comma::exception, "error on wait" ); // todo: will exception work with tbb?
                default:
                    COMMA_THROW( comma::exception, "wait returned unexpected status: " << wait_result ); // todo: will exception work with tbb?
            }
        }
        pair.first = boost::posix_time::microsec_clock::universal_time(); // todo? use timestamp from the camera?
        event_buffer e( handle, event_handle );
        J_tIMAGE_INFO raw;
        e.get( BUFFER_INFO_BASE, raw.pImageBuffer );
        e.get( BUFFER_INFO_SIZE, raw.iImageSize );
        e.get( BUFFER_INFO_PIXELTYPE, raw.iPixelType );
        e.get( BUFFER_INFO_WIDTH, raw.iSizeX );
        e.get( BUFFER_INFO_HEIGHT, raw.iSizeY );
        e.get( BUFFER_INFO_TIMESTAMP, raw.iTimeStamp );
        //e.get( BUFFER_INFO_NUM_PACKETS_MISSING, raw.iMissingPackets ); todo? do we need that?
        e.get( BUFFER_INFO_XOFFSET, raw.iOffsetX );
        e.get( BUFFER_INFO_YOFFSET, raw.iOffsetY );
        // todo? example in stream_thread.cc also sets pointer to valid and queued buffers; do we even need it?
        if( output_raw_image )
        {
            pair.second = cv::Mat( raw.iSizeY, raw.iSizeX, cv_type_from_jai( raw.iPixelType, true ) );
            ::memcpy( pair.second.data, raw.pImageBuffer, pair.second.total() * pair.second.elemSize() );
        }
        else
        {
            J_tIMAGE_INFO cooked;
            validate( "image allocation", J_Image_Malloc( &raw, &cooked ) );
            J_STATUS_TYPE r = J_Image_FromRawToImage( &raw, &cooked );
            if( r != J_ST_SUCCESS )
            {
                J_Image_Free( &cooked ); // it sucks
                COMMA_THROW( comma::exception, "conversion from raw to image failed: " << error_to_string( r ) ); // todo: will exception work with tbb?
            }
            pair.second = cv::Mat( cooked.iSizeY, cooked.iSizeX, cv_type_from_jai( cooked.iPixelType ) );
            ::memcpy( pair.second.data, cooked.pImageBuffer, cooked.iImageSize );
            J_Image_Free( &cooked ); // it sucks
        }
        J_DataStream_QueueBuffer( handle, e.buffer() );
        uint64_t pending = 0;
        uint32_t size = sizeof( uint64_t );
        if( J_DataStream_GetStreamInfo( handle, STREAM_INFO_CMD_NUMBER_OF_FRAMES_AWAIT_DELIVERY, &pending, &size ) == J_ST_SUCCESS && pending > 0 ) { J_Event_SignalCondition( event ); }
        return pair;
    }
    
    void close()
    {
        if( !handle ) { return; }
        J_DataStream_StopAcquisition( handle, ACQ_STOP_FLAG_KILL );
        J_Event_CloseCondition( event );
        J_Event_ExitCondition( event );
        J_DataStream_FlushQueue( handle, ACQ_QUEUE_INPUT_TO_OUTPUT );
        J_DataStream_FlushQueue( handle, ACQ_QUEUE_OUTPUT_DISCARD );
        J_DataStream_Close( handle );
        handle = NULL;
    }

    bool closed() const { return handle == NULL; }    
};

//jai::stream::stream::stream( const jai::camera& c, unsigned int number_of_buffers ) { new impl( c.handle(), c.width() * c.height() * J_MAX_BPP, number_of_buffers ); }

jai::stream::stream::stream( const jai::camera& c, unsigned int number_of_buffers, bool output_raw_image ) : pimpl_( new impl( c.handle(), c.width() * c.height() * J_MAX_BPP, number_of_buffers, output_raw_image ) ) {}

jai::stream::~stream() { if( pimpl_ ) { delete pimpl_; } }

std::pair< boost::posix_time::ptime, cv::Mat > jai::stream::read() { return pimpl_->read(); }

void jai::stream::close() { pimpl_->close(); }

bool jai::stream::closed() const { return pimpl_->closed(); }

} } // namespace snark { namespace jai {
