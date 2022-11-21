// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney

#include <sstream>
#include <boost/bind.hpp>
#include <comma/application/verbose.h>
#include <comma/name_value/map.h>

#include "attribute.h"
#include "camera.h"
#include "error.h"
#include "frame.h"
#include "frame_observer.h"
#include "system.h"

namespace snark { namespace vimba {

camera::camera( const std::string& camera_id )
    : acquisition_mode_( ACQUISITION_MODE_UNKNOWN )
    , last_frame_id_( 0 )
{
    camera_ = system::open_camera( camera_id );
}

camera::camera( const AVT::VmbAPI::CameraPtr& camera_ptr )
    : camera_( camera_ptr )
    , acquisition_mode_( ACQUISITION_MODE_UNKNOWN )
    , last_frame_id_( 0 )
{}

camera::~camera()
{
    if( camera_ ) camera_->Close();
}

camera::name_values camera::info() const
{
    name_values name_value_pairs;

    add_name_value( "id",            boost::bind( &AVT::VmbAPI::Camera::GetID,           boost::cref( *camera_ ), _1 ), name_value_pairs );
    add_name_value( "name",          boost::bind( &AVT::VmbAPI::Camera::GetName,         boost::cref( *camera_ ), _1 ), name_value_pairs );
    add_name_value( "model",         boost::bind( &AVT::VmbAPI::Camera::GetModel,        boost::cref( *camera_ ), _1 ), name_value_pairs );
    add_name_value( "serial_number", boost::bind( &AVT::VmbAPI::Camera::GetSerialNumber, boost::cref( *camera_ ), _1 ), name_value_pairs );
    add_name_value( "interface_id",  boost::bind( &AVT::VmbAPI::Camera::GetInterfaceID,  boost::cref( *camera_ ), _1 ), name_value_pairs );

    return name_value_pairs;
}

void camera::add_name_value( const char* label, getter_fn fn, name_values& name_value_pairs )
{
    std::string value;
    VmbErrorType status = fn( value );
    if( status == VmbErrorSuccess )
    {
        name_value_pairs[ label ] = value;
    }
    else
    {
        std::ostringstream msg;
        msg << "Count not get " << label;
        write_error( msg.str(), status );
    }
}

std::vector< attribute > camera::attributes() const
{
    std::vector< attribute > attributes;
    AVT::VmbAPI::FeaturePtrVector features;
    VmbErrorType status = camera_->GetFeatures( features );
    if( status == VmbErrorSuccess )
    {
        for( AVT::VmbAPI::FeaturePtrVector::iterator it = features.begin();
             it != features.end();
             ++it )
        {
            attribute a( *it );
            attributes.push_back( a );
        }
    }
    else
    {
        COMMA_THROW( comma::exception, error_msg( "GetFeatures() failed", status ));
    }
    return attributes;
}

boost::optional< attribute > camera::get_attribute( const std::string& name ) const
{
    boost::optional< attribute > a;
    AVT::VmbAPI::FeaturePtr feature;
    VmbErrorType status = camera_->GetFeatureByName( name.c_str(), feature );
    if( status == VmbErrorSuccess ) { a = attribute( feature ); }
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

void camera::start_acquisition( frame_observer::callback_fn callback, unsigned int num_frames ) const
{
    comma::verbose << "starting image acquisition..." << std::endl;

    last_frame_id_ = 0;

    // Create a frame observer for this camera.
    // This will be wrapped in a shared_ptr so we don't delete it.
    frame_observer* fo = new frame_observer( camera_, callback );

    // Start streaming
    VmbErrorType status = camera_->StartContinuousImageAcquisition( num_frames, AVT::VmbAPI::IFrameObserverPtr( fo ));
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "StartContinuousImageAcquisition() failed", status ));
    }
    comma::verbose << "started image acquisition" << std::endl;
}

void camera::stop_acquisition() const
{
    comma::verbose << "stopping image acquisition..." << std::endl;
    camera_->StopContinuousImageAcquisition();
    comma::verbose << "stopped image acquisition" << std::endl;
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

    boost::optional< snark::vimba::attribute > ptp_status_attribute = get_attribute( "PtpStatus" );
    if( ptp_status_attribute && ptp_status_attribute->value_as_string() != ptp_status )
    {
        ptp_status = ptp_status_attribute->value_as_string();
        comma::verbose << "PtpStatus changed value to " << ptp_status << std::endl;
        use_ptp = ( ptp_status == "Slave" );
        comma::verbose << ( use_ptp ? "" : "not " ) << "using PTP time source" << std::endl;
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
                    std::cerr << comma::verbose.app_name() << ": warning - "
                              << missing_frames << " missing frame" << ( missing_frames == 1 ? "" : "s" )
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
        std::cerr << comma::verbose.app_name() << ": warning - frame " << frame.id() << " status " << frame.status_as_string() << std::endl;
    }
    comma::verbose << "returning empty frame" << std::endl;
    return timestamped_frame();
}

} } // namespace snark { namespace vimba {
