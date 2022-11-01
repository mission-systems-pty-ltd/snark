// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2018 The University of Sydney
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

/// @author Dave Jennings

#include "../ocular-thermal.h"
#include "../traits.h"
#include "../../../imaging/cv_mat/serialization.h"
#include <Image.h>
#include <RobotEyeThermal.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <comma/io/select.h>
#include <opencv2/core/core.hpp>
#include <thread>

const unsigned int default_image_mode = 0;
const double default_max_speed = 10;    // 10Hz = 36000 deg/second

// Max frame rate is 7.5fps (133ms between frames) but unless the gap is
// substantially longer every second frame fails
const std::chrono::milliseconds sleep_between_frames( 300 );
// Probably a good idea also, but relatively untested
const std::chrono::milliseconds sleep_after_movement( 1000 );

static void bash_completion( unsigned const ac, char const * const * av )
{
    static const char* completion_options =
        " --help -h --verbose -v"
        " --input-fields --input-format"
        " --home --no-capture --speed --track"
        ;

    std::cout << completion_options << std::endl;
    exit( 0 );
}

#define QUOTED( arg ) #arg
#define STRINGIZED( arg ) QUOTED( arg )

void usage( bool )
{
    std::cerr << "\ncapture images from Ocular RobotEye thermal camera";
    std::cerr << "\noutput to stdout as serialized cv::Mat";
    std::cerr << "\n";
    std::cerr << "\nusage: cat <pan,tilt> | " << comma::verbose.app_name() << " <address> [<options>]";
    std::cerr << "\n    where <address> is ip address of ocular roboteye device";
    std::cerr << "\n          <pan,tilt> is pan and tilt in radians";
    std::cerr << "\n";
    std::cerr << "\noptions";
    std::cerr << "\n    --help,-h:      show help";
    std::cerr << "\n    --verbose,-v:   show detailed messages";
    std::cerr << "\n    --home:         home at the start";
    std::cerr << "\n    --image-mode=[<0-3>]: default=" << default_image_mode << "; set image mode";
    std::cerr << "\n    --input-fields: print input fields and exit";
    std::cerr << "\n    --input-format: print input format and exit";
    std::cerr << "\n    --no-capture:   don't capture images";
    std::cerr << "\n    --save=[<dir>]: save to <dir>, otherwise output to stdout";
    std::cerr << "\n    --speed=<Hz>:   default=" << default_max_speed << "; max speed (rotations/s)";
    std::cerr << "\n    --track         track input positions";
    std::cerr << "\n";
    std::cerr << "\n    where image mode is one of: 0 for None";
    std::cerr << "\n                                1 for Derotate";
    std::cerr << "\n                                2 for Circular Mask";
    std::cerr << "\n                                3 for both Derotate & Circular Mask";
    std::cerr << "\n";
    std::cerr << "\nNote that you have to set the environment variable GENICAM_ROOT_V3_0 first";
    std::cerr << "\n$ export GENICAM_ROOT_V3_0=" << STRINGIZED( OCULAR_ROBOTEYE_GENICAM_DIR );
    std::cerr << "\n";
    std::cerr << "\nexample:";
    std::cerr << "\n    io-console | control-from-console pantilt -a | " << comma::verbose.app_name() << " 192.168.1.150";
    std::cerr << "\n" << std::endl;
}

template< typename T > std::string field_names( bool full_xpath = false ) { return comma::join( comma::csv::names< T >( full_xpath ), ',' ); }
template< typename T > std::string format( bool full_xpath = false, const std::string& fields = "" ) { return comma::csv::format::value< T >( !fields.empty() ? fields : field_names< T >( full_xpath ), full_xpath ); }

using namespace snark::ocular::roboteye;

typedef std::pair< boost::posix_time::ptime, cv::Mat > timestamped_frame;

timestamped_frame image_to_timestamped_frame( const ::ocular::Image& image
                                            , const boost::posix_time::ptime& current_time )
{
    uint8_t pixel_type = image.GetPixelType();
    int opencv_pixel_type = pixel_type_to_opencv( image.GetPixelType() );
    comma::verbose << "ocular pixel type is \"" << (int)pixel_type << "\", mapping to " << opencv_pixel_type << std::endl;
    cv::Mat cv_mat( image.GetHeight()
                  , image.GetWidth()
                  , CV_8UC1
                  , (void*)image.GetDataPointer() );

    return std::make_pair( current_time, cv_mat );
}

void capture_frame( ocular::RobotEyeThermal& roboteye_thermal
                  , ocular::Image_Modes_t image_mode
                  , const boost::optional< std::string > save_dir
                  , snark::cv_mat::serialization& serialization )

{
    static unsigned int frame_num = 0;
    comma::verbose << "acquiring frame " << frame_num << std::endl;
    std::this_thread::sleep_for( sleep_between_frames );
    ::ocular::Image image;
    if( check_dev_status( roboteye_thermal.GetImage( image, image_mode ), "GetImage" ).devCode == ::ocular::NO_ERR )
    {
        boost::posix_time::ptime current_time = boost::posix_time::microsec_clock::universal_time();

        if( save_dir )
        {
            const std::string file_name = *save_dir + "/frame" + std::to_string( frame_num ) + ".png";
            if( image.SaveFrame( file_name )) { comma::verbose << "frame " << frame_num << " saved successfully." << std::endl; }
        }
        else
        {
            timestamped_frame timestamped_frame = image_to_timestamped_frame( image, current_time );
            if( !timestamped_frame.first.is_not_a_date_time() )
            {
                serialization.write( std::cout, timestamped_frame );
            }
        }
        frame_num++;
    }
}

void move( ocular::RobotEye& roboteye
         , const position_t& position
         , double max_speed
         , bool track )
{
    double pan_degrees = position.pan / M_PI * 180;
    double tilt_degrees = position.tilt / M_PI * 180;
    comma::verbose << "moving to " << pan_degrees << "," << tilt_degrees << " degrees" << std::endl;

    if( track )
    {
        check_status( roboteye.StopStabilisation(), "StopStabilisation" );
        check_status( roboteye.SetApertureAngles( pan_degrees, tilt_degrees, max_speed ), "SetApertureAngles" );
        check_status( roboteye.StartStabilisation(), "StartStabilisation" );
        double roll = 0;
        check_status( roboteye.GetStabilisedRoll( roll ), "GetStabilisedRoll" );
    }
    else
    {
        check_status( roboteye.SetApertureAngles( pan_degrees, tilt_degrees, max_speed ), "SetApertureAngles" );
    }
}

int main( int argc, char** argv )
{
    // normally we'd try to set GENICAM_ROOT_V3_0 here but it's not soon enough
    // something static in the libraries requires it before we even get here
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" )) bash_completion( argc, argv );
        if( options.exists( "--input-fields" )) { std::cout << field_names< position_t >( true ) << std::endl; return 0; }
        if( options.exists( "--input-format" )) { std::cout << format< position_t >() << std::endl; return 0; }

        comma::verbose << "GENICAM_ROOT_V3_0 set to " << ::getenv( "GENICAM_ROOT_V3_0" ) << std::endl;

        std::vector< std::string > unnamed = options.unnamed( comma::csv::options::valueless_options() + ",--verbose,-v,--track", "-.*" );
        if( unnamed.size() != 1 ) { COMMA_THROW( comma::exception, "require ip address" ); }

        std::string ip_address = unnamed[0];
        comma::verbose << "connecting to RobotEye at " << ip_address << std::endl;
        bool capture = !options.exists( "--no-capture" );
        boost::optional< std::string > save_dir;
        if( options.exists( "--save" )) { save_dir = options.value< std::string >( "--save" ); }
        double max_speed = options.value< double >( "--speed", default_max_speed );
        bool track = options.exists( "--track" );

        snark::cv_mat::serialization::options serialization_options;
        snark::cv_mat::serialization serialization( serialization_options );

        ocular::Image_Modes_t image_mode = static_cast< ocular::Image_Modes_t >( options.value< unsigned int >( "--image-mode", default_image_mode ));

        ocular::RobotEyeThermal roboteye_thermal( ip_address );
        ocular::RobotEye& roboteye = roboteye_thermal.GetRobotEye();

        std::string serial_no;
        if( roboteye.GetSerial( serial_no ) == ocular::NO_ERR )
        {
            comma::verbose << "Connected to RobotEye with serial number: " << serial_no << std::endl;
        }
        else
        {
            std::cerr << "Error getting serial number: "
                      << ocular::RobotEye::GetErrorString( roboteye.GetLastBlockingError() )
                      << std::endl;
        }

        ocular::ocular_error_t status;
        double azimuth = 0;
        double elevation = 0;
        status = check_status( roboteye.GetApertureAngles( azimuth, elevation ), "GetApertureAngles" );
        comma::verbose << "azimuth=" << azimuth << ", and elevation=" << elevation << std::endl;
        if( status == ocular::ERR_NOT_HOMED )
        {
            comma::verbose << "not homed, homing now..." << std::endl;
            status = roboteye.Home();
            if( status )
            {
                std::cout << "Home returned " << ocular::RobotEye::GetErrorString( status ) << std::endl;
                return 1;
            }
        }
        if( options.exists( "--home" ))
        {
            comma::verbose << "homing..." << std::endl;
            check_status( roboteye.Home(), "Home" );
        }

        if( capture )
        {
            comma::verbose << "setting pixel type" << std::endl;
            check_dev_status( roboteye_thermal.SetPixelType( ocular::thermal::PIXELTYPE_MONO_8 ), "SetPixelType" );
            comma::verbose << "set pixel type" << std::endl;
        }

        comma::csv::options csv( options );
        comma::csv::input_stream< position_t > istream( std::cin, csv );
        comma::io::select select;
        select.read().add( comma::io::stdin_fd );
        comma::signal_flag is_shutdown;
        bool acquiring = false;

        while( !is_shutdown && std::cin.good() )
        {
            while( select.check() && select.read().ready( comma::io::stdin_fd ) && std::cin.good() )
            {
                const position_t* position = istream.read();
                if( position )
                {
                    if( acquiring )
                    {
                        comma::verbose << "stopping frame acquisition" << std::endl;
                        check_dev_status( roboteye_thermal.StopAcquisition(), "StopAcquisition" );
                        acquiring = false;
                    }
                    move( roboteye, *position, max_speed, track );
                }
            }
            if( capture && !acquiring )
            {
                comma::verbose << "starting frame acquisition" << std::endl;
                std::this_thread::sleep_for( sleep_after_movement );
                check_dev_status( roboteye_thermal.StartAcquisition(), "StartAcquisition" );
                acquiring = true;
            }
            if( capture ) { capture_frame( roboteye_thermal, image_mode, save_dir, serialization ); }
        }
        check_status( roboteye.Stop(), "Stop" );
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << comma::verbose.app_name() << ": exception: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl;
    }
    return 1;
}
