// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney
// Copyright (c) 2022 Mission Systems Pty Ltd

#ifndef SNARK_SENSORS_VIMBA_CAMERA_H_
#define SNARK_SENSORS_VIMBA_CAMERA_H_

#include <map>
#include <vector>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/function.hpp>
#include <boost/optional.hpp>
#include <opencv2/core/core.hpp>
#include <VimbaCPP/Include/Camera.h>
#include "error.h"
#include "frame_observer.h"

namespace snark { namespace vimba {

struct ptp_status
{
    boost::posix_time::ptime t;
    bool use_ptp;
    std::string value;
};


class attribute;

class camera
{
    public:
        typedef std::pair< boost::posix_time::ptime, cv::Mat > timestamped_frame;
        typedef std::map< std::string, std::string> name_values;
        typedef enum
        {
            ACQUISITION_MODE_UNKNOWN,
            ACQUISITION_MODE_CONTINUOUS,
            ACQUISITION_MODE_SINGLE
        } acquisition_mode_t;

        camera( const std::string& camera_id );
        camera( const AVT::VmbAPI::CameraPtr& camera_ptr );
        ~camera();

        name_values info() const;
        VmbInterfaceType interface_type() const { return interface_type_; }
        std::vector< attribute > attributes() const;
        boost::optional< attribute > get_attribute( const std::string& name ) const;

        void set_feature( const std::string& name, const std::string& value = "" ) const;
        void set_features( const std::string& name_values ) const;

        void set_acquisition_mode( acquisition_mode_t acquisition_mode ) { acquisition_mode_ = acquisition_mode; }
        /// in tests double-buffering seems sufficient but we'll use three frames to
        /// allow for possible jitter in processing time
        void start_acquisition( frame_observer::callback_fn callback, unsigned int num_frames = 3 ) const;
        void stop_acquisition() const;

        timestamped_frame frame_to_timestamped_frame( const snark::vimba::frame& frame, snark::vimba::ptp_status& ptp_status_out ) const;

    private:
        template< typename T> using getter_fn = const boost::function< VmbErrorType( T& ) >;

        template< typename T > static void add_name_value( const char* label, getter_fn<T> fn, name_values& name_value_pairs );
        static std::string value_to_string( const std::string& value );
        static std::string value_to_string( VmbInterfaceType value );

        static const char* VmbInterfaceType_to_string( VmbInterfaceType type );

        AVT::VmbAPI::CameraPtr camera_;
        VmbInterfaceType interface_type_;
        acquisition_mode_t acquisition_mode_;
        mutable VmbUint64_t last_frame_id_;
};

template< typename T >
void camera::add_name_value( const char* label, getter_fn<T> fn, name_values& name_value_pairs )
{
    T value;
    VmbErrorType status = fn( value );
    if( status == VmbErrorSuccess )
    {
        name_value_pairs[ label ] = value_to_string( value );
    }
    else
    {
        std::ostringstream msg;
        msg << "Count not get " << label;
        write_error( msg.str(), status );
    }
}

} } // namespace snark { namespace vimba {

#endif // SNARK_SENSORS_VIMBA_CAMERA_H_
