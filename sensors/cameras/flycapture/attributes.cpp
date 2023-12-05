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
#include "attributes.h"
#include "helpers.h"

namespace snark{ namespace cameras{ namespace flycapture{

    static const pixel_format_map_t pixel_format_map = boost::assign::list_of< pixel_format_map_t::relation >
    (FlyCapture2::PIXEL_FORMAT_MONO8, "PIXEL_FORMAT_MONO8")
    (FlyCapture2::PIXEL_FORMAT_411YUV8, "PIXEL_FORMAT_411YUV8")     /**< YUV 4:1:1. */
    (FlyCapture2::PIXEL_FORMAT_422YUV8, "PIXEL_FORMAT_422YUV8")     /**< YUV 4:2:2. */
    (FlyCapture2::PIXEL_FORMAT_444YUV8, "PIXEL_FORMAT_444YUV8")     /**< YUV 4:4:4. */
    (FlyCapture2::PIXEL_FORMAT_RGB8   , "PIXEL_FORMAT_RGB8")        /**< R = G = B = 8 bits. */
    (FlyCapture2::PIXEL_FORMAT_MONO16 , "PIXEL_FORMAT_MONO16")      /**< 16 bits of mono information. */
    (FlyCapture2::PIXEL_FORMAT_RGB16  , "PIXEL_FORMAT_RGB16")       /**< R = G = B = 16 bits. */
    (FlyCapture2::PIXEL_FORMAT_S_MONO16, "PIXEL_FORMAT_S_MONO16")    /**< 16 bits of signed mono information. */
    (FlyCapture2::PIXEL_FORMAT_S_RGB16 , "PIXEL_FORMAT_S_RGB16")     /**< R = G = B = 16 bits signed. */
    (FlyCapture2::PIXEL_FORMAT_RAW8    , "PIXEL_FORMAT_RAW8")        /**< 8 bit raw data output of sensor. */
    (FlyCapture2::PIXEL_FORMAT_RAW16   , "PIXEL_FORMAT_RAW16")       /**< 16 bit raw data output of sensor. */
    (FlyCapture2::PIXEL_FORMAT_MONO12  , "PIXEL_FORMAT_MONO12")      /**< 12 bits of mono information. */
    (FlyCapture2::PIXEL_FORMAT_RAW12   , "PIXEL_FORMAT_RAW12")       /**< 12 bit raw data output of sensor. */
    (FlyCapture2::PIXEL_FORMAT_BGR     , "PIXEL_FORMAT_BGR")         /**< 24 bit BGR. */
    (FlyCapture2::PIXEL_FORMAT_BGRU    , "PIXEL_FORMAT_BGRU")        /**< 32 bit BGRU. */
    //     (FlyCapture2::PIXEL_FORMAT_RGB ,"PIXEL_FORMAT_RGB")         /**< 24 bit RGB. same as RGB8*/
    (FlyCapture2::PIXEL_FORMAT_RGBU    , "PIXEL_FORMAT_RGBU")        /**< 32 bit RGBU. */
    (FlyCapture2::PIXEL_FORMAT_BGR16   , "PIXEL_FORMAT_BGR16")       /**< R = G = B = 16 bits. */
    (FlyCapture2::PIXEL_FORMAT_BGRU16  , "PIXEL_FORMAT_BGRU16")      /**< 64 bit BGRU. */
    (FlyCapture2::PIXEL_FORMAT_422YUV8_JPEG, "PIXEL_FORMAT_422YUV8_JPEG")/**< JPEG compressed stream. */
    (FlyCapture2::NUM_PIXEL_FORMATS        , "NUM_PIXEL_FORMATS")        /**< Number of pixel formats. */
    (FlyCapture2::UNSPECIFIED_PIXEL_FORMAT, "UNSPECIFIED_PIXEL_FORMAT");

    const pixel_format_map_t* get_pixel_format_map()
    {
        return &pixel_format_map;
    }

    property_map_t property_map = boost::assign::list_of<property_map_t::relation>
    (FlyCapture2::BRIGHTNESS,        "brightness")
    (FlyCapture2::AUTO_EXPOSURE,     "auto_exposure")
    (FlyCapture2::SHARPNESS,         "sharpness")
    (FlyCapture2::WHITE_BALANCE,     "white_balance")
    (FlyCapture2::HUE,               "hue")
    (FlyCapture2::SATURATION,        "saturation")
    (FlyCapture2::GAMMA,             "gamma")
    (FlyCapture2::IRIS,              "iris")
    (FlyCapture2::FOCUS,             "focus")
    (FlyCapture2::ZOOM,              "zoom")
    (FlyCapture2::PAN,               "pan")
    (FlyCapture2::TILT,              "tilt")
    (FlyCapture2::SHUTTER,           "shutter")
    (FlyCapture2::GAIN,              "gain")
    //(FlyCapture2::TRIGGER_MODE,      "trigger_mode")  //Not supported properly through PropertyType
    //(FlyCapture2::TRIGGER_DELAY,     "trigger_delay") //There is a separate function set for trigger modes 
    (FlyCapture2::FRAME_RATE,        "frame_rate")
    (FlyCapture2::TEMPERATURE,       "temperature");

    //List for attributes that exist inside structs in the API, so they have to be listed directly
    static const std::vector< std::string > explicit_attributes = boost::assign::list_of
    ( "maxWidth" )
    ( "maxHeight" )
    ( "offsetHStepSize" )
    ( "offsetVStepSize" )
    ( "imageHStepSize" )
    ( "imageVStepSize" )
    ( "offsetX" )
    ( "offsetY" )
    ( "width" )
    ( "height" )
    ( "PixelFormat" )
    ( "trigger_on" )
    ( "trigger_mode" )
    ( "trigger_parameter" )
    ( "trigger_polarity" )
    ( "trigger_source" );     

    std::string get_attribute( FlyCapture2::CameraBase* handle, const std::string& key )
    {
        FlyCapture2::Error error;
        if (FlyCapture2::GigECamera* camera = dynamic_cast<FlyCapture2::GigECamera*>(handle))
        { // GigE-only properties
            //std::cerr << "trying GigECamera for key " << key << std::endl;
            FlyCapture2::GigEImageSettings image_settings;
            FlyCapture2::GigEImageSettingsInfo image_settings_info;
            assert_ok(camera->GetGigEImageSettings(&image_settings), "couldn't get GigE image settings");
            assert_ok(camera->GetGigEImageSettingsInfo(&image_settings_info), "couldn't get GigE image settings info");
            if( key == "maxWidth" ) { return std::to_string( image_settings_info.maxWidth ); }
            else if( key == "maxHeight" ) { return std::to_string( image_settings_info.maxHeight ); }
            else if( key == "offsetHStepSize" ) { std::to_string( image_settings_info.offsetHStepSize ); }
            else if( key == "offsetVStepSize" ) { std::to_string( image_settings_info.offsetVStepSize ); }
            else if( key == "imageHStepSize" ) { std::to_string( image_settings_info.imageHStepSize ); }
            else if( key == "imageVStepSize" ) { std::to_string( image_settings_info.imageVStepSize ); }
            else if( key == "offsetX" ) { return std::to_string( image_settings.offsetX ); }
            else if( key == "offsetY" ) { return std::to_string( image_settings.offsetY ); }
            else if( key == "width" ) { return std::to_string( image_settings.width ); }
            else if( key == "height" ) { return std::to_string( image_settings.height ); }
            else if( key == "PixelFormat" ) { pixel_format_map.left.at( image_settings.pixelFormat ); }
        }
        else if(FlyCapture2::Camera* camera = dynamic_cast<FlyCapture2::Camera*>(handle)) // Serial or USB Camera
        {
            //std::cerr << "trying Camera for key " << key << std::endl;
            FlyCapture2::Format7Info format_info;
            FlyCapture2::Format7ImageSettings pImageSettings;
            FlyCapture2::Error error;
            unsigned int pPacketSize;
            float pPercentage;
            bool format_supported; // , format7_is_supported;
            // assert_ok(camera->GetVideoModeAndFrameRateInfo(FlyCapture2::VIDEOMODE_FORMAT7, FlyCapture2::FRAMERATE_FORMAT7, &format7_is_supported)), "couldn't get video mode and frame rate");
            // if( !format7_is_supported ) { COMMA_THROW(comma::exception, "Only format7 cameras and GigE cameras are supported"); }
            assert_ok(
                camera->GetFormat7Info(&format_info, &format_supported),
                "couldn't get format7 info"
            );
            assert_ok(
                camera->GetFormat7Configuration( &pImageSettings, &pPacketSize, &pPercentage ),
                "couldn't get format7 config"
            );

            if( key == "maxWidth" ) { return std::to_string( format_info.maxWidth ); }
            else if( key == "maxHeight" ) { return std::to_string( format_info.maxHeight ); }
            else if( key == "offsetHStepSize" ) { return std::to_string( format_info.offsetHStepSize ); }
            else if( key == "offsetVStepSize" ) { return std::to_string( format_info.offsetVStepSize ); }
            else if( key == "imageHStepSize" ) { return std::to_string( format_info.imageHStepSize ); }
            else if( key == "imageVStepSize" ) { return std::to_string( format_info.imageVStepSize ); }
            else if( key == "offsetX" ) { return std::to_string( pImageSettings.offsetX ); }
            else if( key == "offsetY" ) { return std::to_string( pImageSettings.offsetY ); }
            else if( key == "width" ) { return std::to_string( pImageSettings.width ); }
            else if( key == "height" ) { return std::to_string( pImageSettings.height ); }
            else if( key == "PixelFormat" ) { return pixel_format_map.left.at( pImageSettings.pixelFormat ); }

            //Check the common property lists
            FlyCapture2::TriggerMode pTriggerMode;
            assert_ok(handle->GetTriggerMode( &pTriggerMode ), "couldn't get trigger mode");

            if( key == "trigger_on" ) { return std::to_string( pTriggerMode.onOff ); }
            else if( key == "trigger_polarity" ) { return std::to_string( pTriggerMode.polarity ); }
            else if( key == "trigger_source" ) { return std::to_string( pTriggerMode.source ); }
            else if( key == "trigger_mode" ) { return std::to_string( pTriggerMode.mode ); }
            else if( key == "trigger_parameter" ) { return std::to_string( pTriggerMode.parameter ); }

            if( property_map.right.find(key) != property_map.right.end() )
            {
                FlyCapture2::Property cam_prop;
                FlyCapture2::PropertyInfo cam_prop_info;
                cam_prop.type = property_map.right.at( key );
                cam_prop_info.type = property_map.right.at( key );
                handle->GetProperty( &cam_prop );
                handle->GetPropertyInfo( &cam_prop_info );

                if( !cam_prop.present ) return "N/A";    //If property is not present, it is unsupported for this camera
                if( cam_prop.autoManualMode ) return "auto";
                if( cam_prop.type == FlyCapture2::WHITE_BALANCE ) //White balance has two values, so it is a special case
                    { return std::to_string( cam_prop.valueA ) + "," + std::to_string( cam_prop.valueB ); }
                return std::to_string( cam_prop.absControl ? cam_prop.absValue : cam_prop.valueA );
            }
        }
        std::cerr << "flycapture-cat error: property '" << key << "' not found!" << std::endl;
        return "Not Found"; 
    }

    void set_attribute( FlyCapture2::CameraBase* handle, const std::string& key, const std::string& value )
    {
        //std::cerr << "trying CameraBase::set_attribute for key " << key << std::endl;
        FlyCapture2::Error error;
        FlyCapture2::TriggerMode trigger_mode;
        assert_ok(handle->GetTriggerMode(&trigger_mode), "error getting attributes from camera.");

        if( key == "trigger_on" ) 
        {
            if( value == "true" || value == "1" ) {trigger_mode.onOff = true;}
            else if( value == "false" || value == "0") {trigger_mode.onOff = false;}
            else { COMMA_THROW( comma::exception, "Error: invalid trigger_on setting. Please use true/false" ); }
        }
        else if( key == "trigger_polarity" ) 
        {
            if( value == "high" || value == "1" ) {trigger_mode.polarity = 1;}
            else if( value == "low" || value == "0" ) {trigger_mode.polarity = 0;}
            else { COMMA_THROW( comma::exception, "Error: invalid trigger_polarity setting. Please use high/low"); }
        }
        else if( key == "trigger_source" ) // 0-3 are GPIO, 4 = none
        {
            if( value == "GPIO0" || value == "0" ) {trigger_mode.source = 0;} 
            else if( value == "GPIO1" || value == "1" ) {trigger_mode.source = 1;}
            else if( value == "GPIO2" || value == "2" ) {trigger_mode.source = 2;}
            else if( value == "GPIO3" || value == "3" ) {trigger_mode.source = 3;}
            else if( value == "none"  || value == "4" ) {trigger_mode.source = 4;}
            else if( value == "software" || value == "7" ) {trigger_mode.source = 7;}
            else {COMMA_THROW( comma::exception, "Error: unknown trigger source. please use 'GPIO[0-3]', 'software' or 'none'");}
        }
        else if( key == "trigger_mode" )      {trigger_mode.mode = boost::lexical_cast<int>(value);}
        else if( key == "trigger_parameter" ) {trigger_mode.parameter = boost::lexical_cast<int>(value);}
        else if( property_map.right.find( key ) != property_map.right.end() )
        {
            FlyCapture2::Property cam_prop;
            FlyCapture2::PropertyInfo cam_prop_info;
            cam_prop.type = property_map.right.at( key );
            cam_prop_info.type = property_map.right.at( key );
            handle->GetProperty( &cam_prop );
            handle->GetPropertyInfo( &cam_prop_info );

            if( !cam_prop.present ) { return; }    //If property is not present, it is unsupported for this camera
            if( value == "auto" ) { cam_prop.autoManualMode = true; }
            else {
                cam_prop.autoManualMode = false;
                if( cam_prop.type == FlyCapture2::WHITE_BALANCE ) //White balance has two values, so it is a special case
                {
                    std::vector< std::string > v = comma::split( value, "," );
                    if(v.size() != 2)
                    {
                        COMMA_THROW( comma::exception, "Error: White Balance must be in the format of 'red,blue' or 'auto' where red and blue are integers [0,1023]" );
                    } else {
                        cam_prop.valueA = boost::lexical_cast<uint>( v[0] );
                        cam_prop.valueB = boost::lexical_cast<uint>( v[1] );
                    }
                } else {
                    cam_prop.absControl = true; //TODO: If needed, implement non-absolute control of camera
                    cam_prop.absValue = boost::lexical_cast<float>( value );             
                }
            }
            assert_ok(handle->SetProperty( &cam_prop ), "Error setting attribute " + key + " to " + value);
            return;
        } else if (FlyCapture2::GigECamera* camera = dynamic_cast<FlyCapture2::GigECamera*>(handle)){
            set_attribute(camera, key, value);
            return;
        }
        else if (FlyCapture2::Camera* camera = dynamic_cast<FlyCapture2::Camera*>(handle))
        {
            set_attribute(camera, key, value);
            return;
        } else {
            COMMA_THROW( comma::exception, "Error: property '" << key << "' not found!" );
        }
        assert_ok(handle->SetTriggerMode( &trigger_mode ), "Error setting attributes." );
    }

    void set_attribute( FlyCapture2::Camera* handle, const std::string& key, const std::string& value )
    {
        //std::cerr << "trying Camera::set_attribute for key " << key << std::endl;
        FlyCapture2::Error error;
        FlyCapture2::Format7ImageSettings image_settings;
        FlyCapture2::Format7Info image_settings_info;
        bool pSupported; 
        unsigned int packetSize;
        float percentage;

        assert_ok(
            handle->GetFormat7Info(&image_settings_info, &pSupported),
            "couldn't get format7 info"
        );
        assert_ok(
            handle->GetFormat7Configuration( &image_settings, &packetSize, &percentage ),
            "Error getting format7 config"
        );

        // if( key == "offsetHStepSize" ) { image_settings_info.offsetHStepSize = boost::lexical_cast<int>(value); }
        // else if( key == "offsetVStepSize" ) { image_settings_info.offsetVStepSize = boost::lexical_cast<int>(value); }
        /*else*/ if( key == "offsetX" )
        {
            image_settings.offsetX = boost::lexical_cast<uint>(value);
            if( image_settings.offsetX % image_settings_info.offsetHStepSize )
            {
                COMMA_THROW( comma::exception, "Error: offsetX=" << value << " is not a multiple of offset horizontal step size: " << image_settings_info.offsetHStepSize );
            }
        }
        else if( key == "offsetY" )
        {
            image_settings.offsetY = boost::lexical_cast<uint>(value);
            if( image_settings.offsetY % image_settings_info.offsetVStepSize )
            {
                COMMA_THROW( comma::exception, "Error: offsetY=" << value << " is not a multiple of offset vertical step size: " << image_settings_info.offsetVStepSize );
            }
        }
        else if( key == "width" )
        {
            if( value == "max") { image_settings.width = image_settings_info.maxWidth - image_settings.offsetX; }
            else
            {
                uint val_uint = boost::lexical_cast<uint>( value );
                if( val_uint > image_settings_info.maxWidth || val_uint < 0 ) { COMMA_THROW( comma::exception, "Error: width out of bounds" ); }
                if( val_uint % image_settings_info.imageHStepSize ) { COMMA_THROW( comma::exception, "Error: width=" << value << " is not a multiple of image horizontal step size: " << image_settings_info.imageHStepSize ); }
                image_settings.width = val_uint;
            }
        }
        else if( key == "height" )
        {
            if( value == "max") { image_settings.height = image_settings_info.maxHeight - image_settings.offsetY; }
            else
            {
                uint val_uint = boost::lexical_cast<uint>( value );
                if( val_uint > image_settings_info.maxHeight || val_uint < 0 ) { COMMA_THROW( comma::exception, "Error: height out of bounds" ); }
                if( val_uint % image_settings_info.imageVStepSize ) { COMMA_THROW( comma::exception, "Error: height=" << value << " is not a multiple of image vertical step size: " << image_settings_info.imageVStepSize ); }
                image_settings.height = val_uint;
            }
        }
        else if( key == "PixelFormat" ) 
        {
            if( pixel_format_map.right.find(value) != pixel_format_map.right.end() )
                { image_settings.pixelFormat = pixel_format_map.right.at( value ); }
            else { COMMA_THROW( comma::exception, "Error: invalid pixel format."); }
        } else {
            COMMA_THROW( comma::exception, "Error: property '" << key << "' not found!" );
        }

        bool is_valid_settings;
        FlyCapture2::Format7PacketInfo fmt7PacketInfo;
        assert_ok(
            handle->ValidateFormat7Settings(&image_settings, &is_valid_settings, &fmt7PacketInfo )
            , "couldn't validate format7 settings"
        );
        if (is_valid_settings)
        {
            assert_ok(
                handle->SetFormat7Configuration( &image_settings, fmt7PacketInfo.recommendedBytesPerPacket), 
                "Error setting format7 config");
        }
    }

    void set_attribute( FlyCapture2::GigECamera* handle, const std::string& key, const std::string& value )
    {
        //std::cerr << "trying GigECamera::set_attribute for key " << key << std::endl;
        FlyCapture2::Error error;
        FlyCapture2::GigEImageSettings image_settings;
        FlyCapture2::GigEImageSettingsInfo image_settings_info;

        assert_ok(handle->GetGigEImageSettings(&image_settings), "Error getting image settings from camera." );
        assert_ok(handle->GetGigEImageSettingsInfo(&image_settings_info), "Error getting image settings info from camera." );

        if( key == "offsetHStepSize" ) { image_settings_info.offsetHStepSize = boost::lexical_cast<int>(value); }
        else if( key == "offsetVStepSize" ) { image_settings_info.offsetVStepSize = boost::lexical_cast<int>(value); }
        else if( key == "offsetX" )
        {
            image_settings.offsetX = boost::lexical_cast<int>(value);
            if( image_settings.offsetX % image_settings_info.offsetHStepSize )
            {
                COMMA_THROW( comma::exception, "Error: offsetX=" << value << " is not a multiple of offset horizontal step size: " << image_settings_info.offsetHStepSize );
            }
        }
        else if( key == "offsetY" )
        {
            image_settings.offsetY = boost::lexical_cast<int>(value);
            if( image_settings.offsetY % image_settings_info.offsetVStepSize )
            {
                COMMA_THROW( comma::exception, "Error: offsetY=" << value << " is not a multiple of offset vertical step size: " << image_settings_info.offsetVStepSize );
            }
        }
        else if( key == "width" )
        {
            uint val_uint = boost::lexical_cast<uint>( value );
            if( val_uint > image_settings_info.maxWidth || val_uint < 0 ) { COMMA_THROW( comma::exception, "Error: width out of bounds" ); }
            if( val_uint % image_settings_info.imageHStepSize ) { COMMA_THROW( comma::exception, "Error: width=" << value << " is not a multiple of image horizontal step size: " << image_settings_info.imageHStepSize ); }
            image_settings.width = val_uint;
        }
        else if( key == "height" )
        {
            uint val_uint = boost::lexical_cast<uint>( value );
            if( val_uint > image_settings_info.maxHeight || val_uint < 0 ) { COMMA_THROW( comma::exception, "Error: height out of bounds" ); }
            if( val_uint % image_settings_info.imageVStepSize ) { COMMA_THROW( comma::exception, "Error: height=" << value << " is not a multiple of image vertical step size: " << image_settings_info.imageVStepSize ); }
            image_settings.height = val_uint;
        }
        else if( key == "PixelFormat" ) 
        {
            if( pixel_format_map.right.find(value) != pixel_format_map.right.end() )
                { image_settings.pixelFormat = pixel_format_map.right.at( value ); }
            else { COMMA_THROW( comma::exception, "Error: invalid pixel format."); }
        } else {
            COMMA_THROW( comma::exception, "Error: property '" << key << "' not found!" );
        }
        //Handle errors here, the SDK should do out of bounds checking 

        assert_ok(handle->SetGigEImageSettings( &image_settings ), "Error setting attributes." );
    }

    camera::attributes_type get_attributes( FlyCapture2::CameraBase* handle )
    {
        camera::attributes_type attributes;
        for( std::vector< std::string >::const_iterator i = explicit_attributes.begin(); i != explicit_attributes.end(); i++ )
        {
            attributes.push_back( std::make_pair( *i, get_attribute( handle, *i ) ) );
        }

        //There is an iterable list for the remaining attributes
        for( property_map_t::const_iterator i = property_map.begin(); i != property_map.end(); ++i )
        {
            attributes.push_back( std::make_pair( i->right , get_attribute( handle,i->right ) ) ); 
        }
        return attributes;
    }

} } }// namespace snark{ namespace cameras{ namespace flycapture
