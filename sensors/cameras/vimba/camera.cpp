// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney
// Copyright (c) 2022 Mission Systems Pty Ltd

#include <sstream>
#include <boost/bind.hpp>
#include <comma/application/command_line_options.h>
#include <comma/name_value/map.h>

#include "attribute.h"
#include "camera.h"
#include "frame.h"
#include "frame_observer.h"
#include "system.h"

namespace snark { namespace vimba {

camera::camera( const std::string& camera_id )
    : interface_type_( VmbInterfaceUnknown )
    , acquisition_mode_( ACQUISITION_MODE_UNKNOWN )
    , last_frame_id_( 0 )
{
    camera_ = system::open_camera( camera_id );
    VmbErrorType status = camera_->GetInterfaceType( interface_type_ );
    if( status != VmbErrorSuccess ) { comma::say() << error_msg( std::string("couldn't get interface type for camera ") + camera_id, status ) << std::endl; }
}

camera::camera( const AVT::VmbAPI::CameraPtr& camera_ptr )
    : camera_( camera_ptr )
    , interface_type_( VmbInterfaceUnknown )
    , acquisition_mode_( ACQUISITION_MODE_UNKNOWN )
    , last_frame_id_( 0 )
{
    VmbErrorType status = camera_->GetInterfaceType( interface_type_ );
    if( status != VmbErrorSuccess ) { std::string id; camera_->GetID( id );  comma::say() << error_msg( std::string("couldn't get interface type for camera ") + id, status ) << std::endl; }
}

camera::~camera()
{
    if( camera_ ) camera_->Close();
}

camera::name_values camera::info() const
{
    name_values name_value_pairs;

    add_name_value< std::string >( "id"
                                 , boost::bind( &AVT::VmbAPI::Camera::GetID, boost::cref( *camera_ ), _1 )
                                 , name_value_pairs );
    add_name_value< std::string >( "name"
                                 , boost::bind( &AVT::VmbAPI::Camera::GetName, boost::cref( *camera_ ), _1 )
                                 , name_value_pairs );
    add_name_value< std::string >( "model"
                                 , boost::bind( &AVT::VmbAPI::Camera::GetModel, boost::cref( *camera_ ), _1 )
                                 , name_value_pairs );
    add_name_value< std::string >( "serial_number"
                                 , boost::bind( &AVT::VmbAPI::Camera::GetSerialNumber, boost::cref( *camera_ ), _1 )
                                 , name_value_pairs );
    add_name_value< VmbInterfaceType >( "interface_type"
                                      , boost::bind( &AVT::VmbAPI::Camera::GetInterfaceType, boost::cref( *camera_ ), _1 )
                                      , name_value_pairs );
    add_name_value< std::string >( "interface_id"
                                      , boost::bind( &AVT::VmbAPI::Camera::GetInterfaceID, boost::cref( *camera_ ), _1 )
                                      , name_value_pairs );

    return name_value_pairs;
}

std::string camera::value_to_string( const std::string& value )
{
    return value;
}

std::string camera::value_to_string( VmbInterfaceType value )
{
    return VmbInterfaceType_to_string( value );
}

const char* camera::VmbInterfaceType_to_string( VmbInterfaceType type )
{
    switch( type )
    {
        case VmbInterfaceUnknown:  return "Unknown";      // Interface is not known to this version of the API
        case VmbInterfaceFirewire: return "Firewire";     // 1394
        case VmbInterfaceEthernet: return "Ethernet";     // GigE
        case VmbInterfaceUsb:      return "Usb";          // USB 3.0
        #if VIMBA_SDK_VERSION >= 020000
        case VmbInterfaceCL:       return "CL";           // Camera Link
        #endif
        #if VIMBA_SDK_VERSION >= 040200
        case VmbInterfaceCSI2:     return "CSI2";         // CSI-2
        #endif
        default:                   return "Unknown";
    }
}

std::vector< attribute > camera::attributes() const
{
    std::vector< attribute > attributes;
    AVT::VmbAPI::FeaturePtrVector features;
    comma::saymore() << "getting camera features" << std::endl;
    VmbErrorType status = camera_->GetFeatures( features );
    if( status == VmbErrorSuccess )
    {
        for( AVT::VmbAPI::FeaturePtrVector::iterator it = features.begin();
             it != features.end();
             ++it )
        {
            // Failing to read an attribute is not fatal.
            // Could use Feature::IsReadable() but there are some features that
            // pass that test but still fail to read. e.g CorrectionDataSize.
            try
            {
                attribute a( *it );
                attributes.push_back( a );
            }
            catch( comma::exception& ex ) { comma::saymore() << ex.what() << std::endl; }
        }
    }
    else
    {
        COMMA_THROW( comma::exception, error_msg( "GetFeatures() failed", status ));
    }
    return attributes;
}

// Available features and their names varies by interface type and by library version.
// Allow for requests by a generic name that will be mapped to the actual name
// for the given camera and version.

std::string camera::feature_name( const std::string& which ) const
{
    if( which == "frames_delivered" )
    {
        switch( interface_type_ )
        {
            case VmbInterfaceEthernet: return "StatFrameDelivered";
            #if VIMBA_SDK_VERSION >= 040200
            case VmbInterfaceCSI2:     return "StatFrameDelivered";
            #endif
            default:
                comma::say() << which << " not supported for " << VmbInterfaceType_to_string( interface_type_ ) << std::endl;
                return "";
        }
    }
    else
    {
        comma::say() << "unknown attribute: " << which << std::endl;
    }
    return "";
}

// return the names of the stat features, for a given interface type and library version
//
// Note that Alvium Features Reference v2.7.2 documents the StatFrame... features
// as being called StatFrames... (plural), but when you list Features for a CSI camera
// you get the spelling below. This difference between documentation and reality has
// been confirmed by support, and applies also to GigE cameras.

std::vector< std::string > camera::stat_feature_names() const
{
    switch( interface_type_ )
    {
        case VmbInterfaceEthernet:
            {
                return {
                    "StatFrameRate", "StatFrameDelivered", "StatFrameDropped", "StatFrameRescued",
                    "StatFrameShoved", "StatFrameUnderrun", "StatLocalRate", "StatPacketErrors",
                    "StatPacketMissed", "StatPacketReceived", "StatPacketRequested", "StatPacketResent",
                    "StatTimeElapsed"
                };
            }
            break;

        #if VIMBA_SDK_VERSION >= 040200
        case VmbInterfaceCSI2:
            return {
                "StatFrameRate", "StatFrameDelivered", "StatFrameCRCError", "StatFrameIncomplete", "StatFrameUnderrun"
            };
            break;
        #endif

        default:
            return std::vector< std::string >();
    }
}

boost::optional< attribute > camera::get_attribute( const std::string& name ) const
{
    boost::optional< attribute > a;
    AVT::VmbAPI::FeaturePtr feature;
    VmbErrorType status = camera_->GetFeatureByName( name.c_str(), feature );
    if( status == VmbErrorSuccess ) { a = attribute( feature ); }
    else { comma::say() << error_msg( std::string("failed to get ") + name, status ) << std::endl; }
    return a;
}

void camera::set_feature( const std::string& name, const std::string& value ) const
{
    AVT::VmbAPI::FeaturePtr feature;
    VmbErrorType status = camera_->GetFeatureByName( name.c_str(), feature );
    if( status == VmbErrorSuccess )
    {
        attribute a( feature );
        a.set( value );
    }
    else
    {
        COMMA_THROW( comma::exception, error_msg( "GetFeatureByName() failed", status ));
    }
}

void camera::set_features( const std::string& name_value_pairs ) const
{
    for( const auto& f: comma::name_value::map::as_vector( name_value_pairs ) ) { set_feature( f.first, f.second ); }
}

void camera::start_acquisition( frame_observer::callback_fn callback, boost::optional< unsigned int > num_frames ) const
{
    // Use the requested buffer_size if set, otherwise default to 3 frames unless CSI,
    // in which case use 7. See "Getting Started with GenICam for CSI"
    unsigned int buffer_size;
    if( num_frames ) { buffer_size = *num_frames; }
    #if VIMBA_SDK_VERSION >= 040200
    else if( interface_type_ == VmbInterfaceCSI2 ) { buffer_size = 7; }
    #endif
    else { buffer_size = 3; }

    comma::saymore() << "starting image acquisition using " << buffer_size << " buffers..." << std::endl;

    last_frame_id_ = 0;

    // Create a frame observer for this camera.
    // This will be wrapped in a shared_ptr so we don't delete it.
    frame_observer* fo = new frame_observer( camera_, callback );

    // Start streaming
    //
    // It's recommended to just use the StartContinuousImageAcquisition / StopContinuousImageAcquisition pair
    // and not manage buffers etc ourselves.
    //
    // AnnounceFrame is the preferred allocation mode for most cameras, but
    // AllocAndAnnounceFrame is preferred for CSI cameras. See
    // "Getting started with GenICam for CSI-3" - "Known issues and restrictions"
    #if VIMBA_SDK_VERSION >= 060000
    AVT::VmbAPI::FrameAllocationMode allocation_mode = AVT::VmbAPI::FrameAllocation_AnnounceFrame;
    if( interface_type_ == VmbInterfaceCSI2 ) { allocation_mode = AVT::VmbAPI::FrameAllocation_AllocAndAnnounceFrame; }
    VmbErrorType status = camera_->StartContinuousImageAcquisition( buffer_size, AVT::VmbAPI::IFrameObserverPtr( fo ), allocation_mode );
    #else
    // earlier Vimba versions don't have the allocationMode parameter
    VmbErrorType status = camera_->StartContinuousImageAcquisition( buffer_size, AVT::VmbAPI::IFrameObserverPtr( fo ) );
    #endif
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "StartContinuousImageAcquisition() failed", status ));
    }
    comma::saymore() << "started image acquisition" << std::endl;
}

void camera::stop_acquisition() const
{
    comma::saymore() << "stopping image acquisition..." << std::endl;
    camera_->StopContinuousImageAcquisition();
    comma::saymore() << "stopped image acquisition" << std::endl;
}

camera::timestamped_frame camera::frame_to_timestamped_frame( const snark::vimba::frame& frame, snark::vimba::ptp_status& ptps_out ) const
{
    // For the timestamp, if PTP is available use frame.timestamp(),
    // otherwise just use current time.

    // It takes about 4ms to interrogate the camera for a feature value,
    // so we can update this information as we run.

    static std::string ptp_status = "unknown";
    static bool use_ptp = false;

    // Get the current time as soon as possible after entering the callback
    boost::posix_time::ptime current_time = boost::posix_time::microsec_clock::universal_time();

    // PTP is only supported on GigE cameras. See Alvium Features Reference
    if( interface_type_ == VmbInterfaceEthernet )
    {
        boost::optional< snark::vimba::attribute > ptp_status_attribute = get_attribute( "PtpStatus" );
        if( ptp_status_attribute && ptp_status_attribute->value_as_string() != ptp_status )
        {
            ptp_status = ptp_status_attribute->value_as_string();
            comma::saymore() << "PtpStatus changed value to " << ptp_status << std::endl;
            use_ptp = ( ptp_status == "Slave" );
            comma::saymore() << ( use_ptp ? "" : "not " ) << "using PTP time source" << std::endl;
        }
    }

    boost::posix_time::ptime timestamp =
        ( use_ptp
        ? boost::posix_time::ptime( boost::gregorian::date( 1970, 1, 1 ))
              + boost::posix_time::microseconds( static_cast< long >( frame.timestamp() / 1000 ) )
        : current_time );

    ptps_out.t=timestamp;
    ptps_out.use_ptp=use_ptp;
    ptps_out.value=ptp_status;

    if( frame.status() == VmbFrameStatusComplete )
    {
        if( acquisition_mode_ == ACQUISITION_MODE_CONTINUOUS )
        {
            if( last_frame_id_ != 0 )
            {
                VmbUint64_t missing_frames = frame.id() - last_frame_id_ - 1;
                if( missing_frames > 0 )
                {
                    comma::say() << "warning - " << missing_frames << " missing frame" << ( missing_frames == 1 ? "" : "s" )
                                 << " detected" << std::endl;
                }
            }
            last_frame_id_ = frame.id();
        }

        snark::vimba::frame::pixel_format_desc fd = frame.format_desc();

        cv::Mat cv_mat( frame.height()
                      , frame.width() * fd.width_adjustment
                      , fd.type
                      , frame.image_buffer() );

        return std::make_pair( timestamp, cv_mat );
    }
    else
    {
        comma::say() << "warning - frame " << frame.id() << " status " << frame.status_as_string() << std::endl;
    }
    comma::saymore() << "returning empty frame" << std::endl;
    return timestamped_frame();
}

} } // namespace snark { namespace vimba {
