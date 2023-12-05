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
#include <comma/string/string.h>
#include "flycapture.h"
#include "helpers.h"
#include "attributes.h"

namespace snark{ namespace cameras{ namespace flycapture{
    boost::posix_time::ptime
    midpoint(boost::posix_time::ptime const& a, boost::posix_time::ptime const& b)
    { return a + (b-a)/2; }

    static const unsigned int max_retries = 15;

    bool collect_frame(cv::Mat & image, FlyCapture2::CameraBase* handle, bool & started)
    {
        FlyCapture2::Error result;
        FlyCapture2::Image frame_;
        FlyCapture2::Image raw_image;

        if( !handle ) { COMMA_THROW(comma::exception, "read from bad camera"); }
        result = handle->RetrieveBuffer(&raw_image);
        frame_.DeepCopy(&raw_image);
        raw_image.ReleaseBuffer();

        if( result == FlyCapture2::PGRERROR_OK ) 
        {
            cv::Mat cv_image( frame_.GetRows(), frame_.GetCols(), get_cv_type( frame_ ), frame_.GetData() );
            cv_image.copyTo(image);
            return true;
        }
        else if( result == FlyCapture2::PGRERROR_IMAGE_CONSISTENCY_ERROR )
        {
            // todo: no output to stderr in libraries!
//report damaged frame and try again
            std::cerr << "snark_flycapture error: " << result.GetDescription() << ". Retrying..." << std::endl;
            return false; 
        }
        else if( ( result == FlyCapture2::PGRERROR_ISOCH_START_FAILED ) 
            || ( result == FlyCapture2::PGRERROR_ISOCH_ALREADY_STARTED )
            || ( result == FlyCapture2::PGRERROR_ISOCH_NOT_STARTED ) )
        {
            // todo: no output to stderr in libraries!
//These are errors that result in a retry
            std::cerr << "snark_flycapture error: " << result.GetDescription() << ". Restarting camera." << std::endl;
            handle->StopCapture();
            started = false;
            return false;
        }
        else 
        {  
            COMMA_THROW( comma::exception, "got frame with invalid status " << result.GetType() << ": " << result.GetDescription() );
        }
    }

    class camera::impl
    {
//Note, for Point Grey, the serial number is used as ID
    public:
        impl( unsigned int id, const attributes_type& attributes, const timestamp_policy & when ) :
        handle_(nullptr), id_(id), started_( false ), when_( when )
        {
            initialize_();
            std::vector< unsigned int > camera_list = list_camera_serials();
// Check if id is in the camera enumeration
            if(std::find(camera_list.begin(), camera_list.end(), id) == camera_list.end())
            { COMMA_THROW(comma::exception, "couldn't find camera with serial " + std::to_string(id)); }
            if (!connect(id, handle_, guid_)) 
            { COMMA_THROW(comma::exception, "couldn't connect to serial " + std::to_string(id)); }
            assert_ok(
                bus_manager().GetInterfaceTypeFromGuid( &guid_, &interface_ ),
                "cannot determine interface for camera with serial " + std::to_string(id)
            );
//Get Point grey unique id (guid) from serial number. guid does not exist in CameraInfo, and so it does not appear in the camera list
            for( attributes_type::const_iterator i = attributes.begin(); i != attributes.end(); ++i )
            {
                set_attribute( handle(), i->first, i->second );
            }

            FlyCapture2::PixelFormat pixel_format;
            uint width, height;
            width = boost::lexical_cast<uint>( get_attribute( handle(), "width" ) );
            height = boost::lexical_cast<uint>( get_attribute( handle(), "height" ) );
            pixel_format = get_pixel_format_map()->right.at( get_attribute( handle(),"PixelFormat" ) );
            total_bytes_per_frame_ = width * height * bits_per_pixel( pixel_format ) / 8;
        }

        ~impl() { close(); }

        void close()
        {
            if(handle_) {
                if( handle_->IsConnected() ) {
                    handle_->StopCapture();
                    handle_->Disconnect();
                }
                id_.reset();
                handle_.reset();
            }
        }

        void test()
        {
            FlyCapture2::StrobeInfo pStrobeInfo;
            FlyCapture2::StrobeControl pStrobe;
            assert_ok(
                handle()->GetStrobeInfo( &pStrobeInfo ), 
                "no strobe info"
            );
            std::cerr
                << "source " << pStrobeInfo.source << std::endl
                << "present " << pStrobeInfo.present << std::endl
                << "readOutSupported " << pStrobeInfo.readOutSupported << std::endl
                << "onOffSupported " << pStrobeInfo.onOffSupported << std::endl
                << "polaritySupported " << pStrobeInfo.polaritySupported << std::endl
                << "minValue " << pStrobeInfo.minValue << std::endl
                << "maxValue " << pStrobeInfo.maxValue << std::endl;
            assert_ok(
                handle()->GetStrobe( &pStrobe ),
                "no strobe"
            );
            pStrobe.onOff = true;
            pStrobe.duration = 50.0;
            handle()->SetStrobe(&pStrobe);
            std::cerr
                << "source " << pStrobe.source << std::endl
                << "onOff " << pStrobe.onOff << std::endl
                << "polarity " << pStrobe.polarity << std::endl
                << "delay " << pStrobe.delay << std::endl
                << "duration " << pStrobe.duration << std::endl;
        }

        FlyCapture2::InterfaceType get_interface() const { return interface_; }

        std::pair< boost::posix_time::ptime, cv::Mat > read( )
        {
            return read( when_ );
        }

        std::pair< boost::posix_time::ptime, cv::Mat > read( const timestamp_policy & when )
        {
            cv::Mat image_;
            std::pair< boost::posix_time::ptime, cv::Mat > pair;
            bool success = false;
            unsigned int retries = 0;

            while( !success && retries < max_retries )
            {
                FlyCapture2::Error result;
                if( !started_ ) { result = handle_->StartCapture(); }
// error is not checked as sometimes the camera will start correctly but return an error
                started_ = true;
                boost::posix_time::ptime timestamp = boost::posix_time::microsec_clock::universal_time();
                success = collect_frame( image_, handle() , started_ );
                if( success ) {
                    pair.second = image_;
                    switch( when.value ) {
                        case camera::timestamp_policy::before:   pair.first = timestamp; break;
                        case camera::timestamp_policy::after:    pair.first = boost::posix_time::microsec_clock::universal_time(); break;
                        case camera::timestamp_policy::average:  pair.first = timestamp + ( boost::posix_time::microsec_clock::universal_time() - timestamp ) / 2.0; break;
                        default:
                            COMMA_THROW( comma::exception, "logical error, moment not specified" );
                    }
                }
                retries++;
            }
            if( success ) { return pair; }
            COMMA_THROW( comma::exception, "snark_flycapture: failed to read frame" << std::endl)
        }

        const FlyCapture2::CameraBase* handle() const { return &*handle_; }

        FlyCapture2::CameraBase* handle() { return &*handle_; }

        unsigned int id() const { return *id_; }

        unsigned long total_bytes_per_frame() const { return total_bytes_per_frame_; }

        static std::vector< unsigned int > list_camera_serials()
        {
            std::vector< unsigned int > list;
            unsigned int num_cameras = 0;
            assert_ok(
                bus_manager().GetNumOfCameras( &num_cameras ),
                "cannot find point grey cameras"
            );

            if ( num_cameras == 0 ) { COMMA_THROW(comma::exception, "no cameras found"); }

// USB Cameras
            for (unsigned int i = 0; i < num_cameras; ++i)
            {
                unsigned int serial_number;
                assert_ok(
                    bus_manager().GetCameraSerialNumberFromIndex(i, &serial_number ),
                    "Error getting camera serial"
                );
                list.push_back(serial_number);
            }

// GigE Cameras
            // BUG DiscoverGigECameras will sometimes crash if there are devices on a different subnet
            FlyCapture2::CameraInfo cam_info[num_cameras];
            assert_ok(
                FlyCapture2::BusManager::DiscoverGigECameras( cam_info, &num_cameras ),
                "cannot discover point grey cameras via GiGe interface"
            );
            for (unsigned int i = 0; i < num_cameras; ++i)
            { list.push_back(cam_info[i].serialNumber); }
            return list;
        }

        static void disconnect(camerabase_ptr& base)
        {
            if (base)
            {
                base->StopCapture();
                base->Disconnect();
                base.reset();
            }
            else
            { COMMA_THROW(comma::exception, "can't disconnect because camera not connected to this handle"); }
        }

        static bool connect(uint serial, camerabase_ptr& base, FlyCapture2::PGRGuid& guid)
        {
            // Instantiate the right camera type based on the interface
            FlyCapture2::InterfaceType interface;
            if (base)
            { COMMA_THROW(comma::exception, "camera already connected to this handle"); }
            assert_ok(
                bus_manager().GetCameraFromSerialNumber(serial, &guid),
                "cannot find camera with serial " + std::to_string(serial)
            );
            assert_ok(
                bus_manager().GetInterfaceTypeFromGuid( &guid, &interface ),
                "cannot determine interface for camera with serial " + std::to_string(serial)
            );
            switch(interface)
            {
                case FlyCapture2::INTERFACE_USB2:
                case FlyCapture2::INTERFACE_USB3:
                case FlyCapture2::INTERFACE_IEEE1394:
                    base.reset(dynamic_cast<FlyCapture2::CameraBase*>(new FlyCapture2::Camera()));
                    break;
                case FlyCapture2::INTERFACE_GIGE:
                    base.reset(dynamic_cast<FlyCapture2::CameraBase*>(new FlyCapture2::GigECamera()));
                    break;
                default:
                COMMA_THROW(comma::exception, "unknown interface for camera " + std::to_string(serial));
            }
            FlyCapture2::Error error;
            if( (error = base->Connect(&guid)) != FlyCapture2::PGRERROR_OK)
            {
                std::cerr << "snark_flycapture: Failed to connect (" << error.GetDescription() << ")" << std::endl;
                return false;
            }
            return true;
        }

        static const std::string describe_camera(unsigned int serial)
        {
            FlyCapture2::CameraInfo camera_info;
            camerabase_ptr camera;
            FlyCapture2::PGRGuid guid;
            FlyCapture2::InterfaceType interface;

            if (!connect(serial, camera, guid))
            { return "serial=" + std::to_string(serial) + ",ERROR"; }
            assert_ok(
                bus_manager().GetInterfaceTypeFromGuid( &guid, &interface ),
                "cannot determine interface for camera with serial " + std::to_string(serial)
            );
            assert_ok(
                camera->GetCameraInfo( &camera_info ),
                "couldn't get camera info for camera with serial " + std::to_string(serial)
            );
            disconnect(camera);

            char escape_chars[] = "\"";
            return 
                "serial=\"" + std::to_string(camera_info.serialNumber) + "\"" +
                ",interface=\"" + comma::escape(get_interface_string(interface), escape_chars) + "\"" +
                ",model=\"" + comma::escape(camera_info.modelName, escape_chars) + "\"" +
                ",vendor=\"" + comma::escape(camera_info.vendorName, escape_chars) + "\"" +
                ",sensor=\"" + comma::escape(camera_info.sensorInfo, escape_chars) + "\"" +
                ",resolution=\"" + comma::escape(camera_info.sensorResolution, escape_chars) + "\"" +
                ",version=\"" + comma::escape(camera_info.firmwareVersion, escape_chars) + "\"" +
                ",build_time=\"" + comma::escape(camera_info.firmwareBuildTime, escape_chars) + "\"";
        }

        void software_trigger(bool broadcast)
        {
            handle_->FireSoftwareTrigger(broadcast);
        }

    private:
        friend class camera::multicam::impl;
        camerabase_ptr handle_;
        boost::optional< unsigned int > id_;
        FlyCapture2::PGRGuid guid_;
        std::vector< char > buffer_;
        uint total_bytes_per_frame_;
        bool started_;
        const timestamp_policy & when_;
        FlyCapture2::InterfaceType interface_;

        static FlyCapture2::BusManager& bus_manager()
        {
           static FlyCapture2::BusManager bm;
           return bm;
        }

        static void initialize_() {} /*quick and dirty*/
    };

// multicam
    class camera::multicam::impl
    {
    public:
        impl( std::vector<camera_pair>& camera_pairs, const std::vector< unsigned int >& offsets, const timestamp_policy & when )
        : good( false ), when_( when )
        {
            for (camera_pair& pair : camera_pairs)
            {
                uint serial = pair.first;
                const attributes_type attributes = pair.second;
                cameras_.push_back(std::unique_ptr<camera::impl>(new camera::impl(serial, attributes, when)));
            }
            if (cameras_.size()) { good = true; }
            apply_offsets( offsets );
        }

        ~impl()
        {
        }

        void trigger()
        {
            if (good) { 
                for (auto& camera : cameras_)
                { camera->software_trigger(false); }
            }
        }

        void apply_offsets( const std::vector< unsigned int >& offsets )
        {
            if( offsets.empty() ) { return; }
            camera::timestamp_policy when( camera::timestamp_policy::before ); // does not matter, read return value is unused
            if( cameras_.size() != offsets.size() ) { COMMA_THROW( comma::exception, "expected offsets number equal to number of cameras: " << cameras_.size() << "; got: " << offsets.size() ); }
            for( unsigned int i = 0; i < offsets.size(); ++i )
            {
                for ( unsigned int j = 0; j < offsets[i]; ++j){ const auto pair = cameras_[i]->read( when ); }
            }
        }

        camera::multicam::frames_pair read( bool use_software_trigger )
        {
            return read( when_, use_software_trigger );
        }

        camera::multicam::frames_pair read( const camera::timestamp_policy & when, bool use_software_trigger )
        {
            if (!good) { COMMA_THROW(comma::exception, "multicam read without good cameras"); }
            camera::multicam::frames_pair image_tuple;
            boost::posix_time::ptime timestamp = boost::posix_time::microsec_clock::universal_time();
            if( use_software_trigger )
            {
                trigger();
                boost::posix_time::ptime end = boost::posix_time::microsec_clock::universal_time();
                timestamp = midpoint(timestamp, end);
            }
            for (auto& camera : cameras_)
            {
                const auto pair = camera->read( when );
                image_tuple.second.push_back(pair.second);
            }
            if ( !use_software_trigger ) {
                if ( when.value == camera::timestamp_policy::after ) { timestamp = boost::posix_time::microsec_clock::universal_time(); }
                if ( when.value == camera::timestamp_policy::average ) { timestamp = timestamp + ( boost::posix_time::microsec_clock::universal_time() - timestamp ) / 2.0; }
            }
            image_tuple.first = timestamp;
            return image_tuple;
        }

        bool good;
        std::vector<std::unique_ptr<camera::impl>> cameras_;
        const timestamp_policy & when_;
    };

    camera::timestamp_policy::timestamp_policy( const std::string & s )
        : value( s == "before" ? before : ( s == "after" ? after : ( s == "average" ? average : none ) ) )
    {
        if ( value == none ) { COMMA_THROW( comma::exception, "timestamp policy is not one of '" << list() << "'" ); }
    }

    camera::timestamp_policy::operator std::string() const
    {
        switch( value )
        {
            case none   : return "none";
            case before : return "before";
            case after  : return "after";
            case average: return "average";
        }
        return "none"; // to avoid a warning
    }
    
} } } //namespace snark{ namespace cameras{ namespace flycapture{

    namespace snark{ namespace cameras{ namespace flycapture {

// flycapture class

        camera::camera( unsigned int id, const camera::attributes_type& attributes, const timestamp_policy & when ) : pimpl_( new impl( id, attributes, when ) ) {}

        camera::~camera() { }

        FlyCapture2::InterfaceType camera::get_interface() const { return pimpl_->get_interface(); }

        std::pair< boost::posix_time::ptime, cv::Mat > camera::read( const camera::timestamp_policy & when ) { return pimpl_->read( when ); }
        std::pair< boost::posix_time::ptime, cv::Mat > camera::read( ) { return pimpl_->read( ); }

        void camera::close() { pimpl_->close(); }

        std::vector< unsigned int > camera::list_camera_serials() { return camera::impl::list_camera_serials(); }

        const std::string camera::describe_camera(unsigned int serial) { return camera::impl::describe_camera(serial); }

        unsigned int camera::id() const { return pimpl_->id(); }

        unsigned long camera::total_bytes_per_frame() const { return pimpl_->total_bytes_per_frame(); }

        camera::attributes_type camera::attributes() const { return get_attributes( pimpl_->handle() ); }

// camera::multicam class
        camera::multicam::multicam( std::vector<camera_pair>& cameras, const std::vector< unsigned int >& offsets, const timestamp_policy & when ) : pimpl_( new multicam::impl( cameras, offsets, when ) ) {}

        camera::multicam::~multicam() { delete pimpl_; }

        bool camera::multicam::good() const { return pimpl_->good; }
        
        uint camera::multicam::num_cameras() const { return pimpl_->cameras_.size(); }

        void camera::multicam::trigger()
        { pimpl_->trigger(); }

        camera::multicam::frames_pair camera::multicam::read( const camera::timestamp_policy & when, bool use_software_trigger )
        { return pimpl_->read( when, use_software_trigger ); }

        camera::multicam::frames_pair camera::multicam::read( bool use_software_trigger )
        { return pimpl_->read( use_software_trigger ); }

} } } //namespace snark{ namespace cameras{ namespace flycapture{

// FlyCapture2::BusManager snark::cameras::flycapture::camera::impl::bus_manager();
