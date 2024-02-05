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

#include <boost/algorithm/string/join.hpp>
#include <boost/bind/bind.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <tbb/concurrent_queue.h>
#include <tbb/pipeline.h>
#include <tbb/task_scheduler_init.h>
#include <opencv2/highgui/highgui.hpp>
#include <comma/application/command_line_options.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/name_value/map.h>
#include "../../../../imaging/cv_mat/serialization.h"
#include "../../../../imaging/cv_mat/traits.h"
#include "../../../../tbb/queue.h"
#include "../flycapture.h"

static bool verbose;
static unsigned int discard_more_than;
static boost::scoped_ptr< snark::cameras::flycapture::camera::multicam > multicam;
static bool running = true;

int main( int argc, char** argv )
{
    try
    {
        std::vector<std::string> camera_strings;
        std::string fields;
        std::string offsets_option;
        unsigned int discard;
        boost::program_options::options_description description( "options" );
        std::vector<snark::cameras::flycapture::camera::multicam::camera_pair> cameras;
        std::string timestamp_time_option;
        std::string use_software_trigger_option;
        std::vector< unsigned int > offsets;

        description.add_options()
            ( "help,h", "display help message" )
            ( "camera", boost::program_options::value< std::vector<std::string> >(), "a camera_string specifying the serial to connect to as well as any attributes to set" )
            ( "trigger", boost::program_options::value< std::string >( &use_software_trigger_option )->default_value( "software" ), "sets trigger type software|hardware" )
            ( "fields,f", boost::program_options::value< std::string >( &fields )->default_value( "t,rows,cols,type" ), "header fields, possible values: t,rows,cols,type,size" )
            ( "offsets,o", boost::program_options::value< std::string >( &offsets_option ), "frame offsets for each camera to synchronise images e.g. --offsets=0,1 to skip first image on second camera" )
            ( "timestamp", boost::program_options::value< std::string >( &timestamp_time_option )->default_value( "before" ), "when to take the timestamp of the frame (before or after reading data from the camera, or the average of the two); possible values before, after, average" )
            ( "list-cameras", "list all cameras and exit" )
            ( "list-attributes", "output current camera attributes" )
            ( "verbose,v", "be more verbose" )
            ( "header", "output header only" )
            ( "no-header", "output image data only" );

        boost::program_options::variables_map vm;
        boost::program_options::positional_options_description p_opts;
        p_opts.add("camera", -1);
        boost::program_options::store(
            boost::program_options::command_line_parser(argc, argv).options(description).positional(p_opts).run(),
            vm
        );
        boost::program_options::notify( vm );

        verbose = vm.count( "verbose" );
        if ( vm.count( "help" ) )
        {
            std::cerr << "Acquire images from multiple point grey flycapture2 cameras" << std::endl
                << "and append them vertically to a cv::Mat." << std::endl
                << "usage: flycapture-multicam [<options>] camera_string[,camera_string,...]" << std::endl
                << "  camera_string: serial[,attribute_pairs...]" << std::endl
                << description << std::endl
                << "Known issues:" << std::endl
                << "  * GigE discovery may fail if there are devices on a different subnets." << std::endl
                << "  * Using flycapture with 16.04 may segfault when application closes (driver issue)." << std::endl
                << "  * TBB I/O unimplemented, but works at full speed with a 7Hz 12MP camera." << std::endl
                << std::endl;
            return 1;
        }

        if( vm.count( "list-cameras" ) )
        {
            for (const uint serial : snark::cameras::flycapture::camera::list_camera_serials())
            { std::cout << snark::cameras::flycapture::camera::describe_camera(serial) << std::endl; }
            return 0;
        }

        if( vm.count( "list-attributes" ) )
        {
            for (const uint serial : snark::cameras::flycapture::camera::list_camera_serials())
            {
                std::cout << snark::cameras::flycapture::camera::describe_camera(serial) << std::endl;
                std::unique_ptr< snark::cameras::flycapture::camera > camera;
                camera.reset( new snark::cameras::flycapture::camera( serial, snark::cameras::flycapture::camera::attributes_type(), snark::cameras::flycapture::camera::timestamp_policy( snark::cameras::flycapture::camera::timestamp_policy::none ) ) );
                const auto& attributes = camera->attributes(); // quick and dirty
                for( snark::cameras::flycapture::camera::attributes_type::const_iterator it = attributes.begin(); it != attributes.end(); ++it )
                {
                    if( it != attributes.begin() ) { std::cout << std::endl; }
                    std::cout << it->first;
                    if( it->second != "" ) { std::cout << '=' << it->second; }
                }
                std::cout << std::endl << std::endl;
            }
            return 0;
        }

        if( vm.count( "header" ) + vm.count( "no-header" ) > 1 )
        { COMMA_THROW( comma::exception, "--header, and --no-header are mutually exclusive" ); }

        if (vm.count("camera") < 1)
        { COMMA_THROW(comma::exception, "specify at least one camera serial"); }
        camera_strings = vm["camera"].as< std::vector<std::string> >();
        for (auto camera_string : camera_strings)
        {
            snark::cameras::flycapture::camera::attributes_type attributes;
            auto fields = comma::split( camera_string, "," );
            if (fields.size() < 1) { COMMA_THROW(comma::exception, "expected camera serial, got none"); }
            uint serial = boost::lexical_cast<uint>(fields[0]);
            if (fields.size() >= 2) {
                std::cerr << "set " << fields[1] << std::endl;
                comma::name_value::map set_map( fields[1], ';', '=' );
                for (auto attr : set_map.get())
                { attributes.push_back( attr ); }
            }
            cameras.push_back( std::make_pair(serial, attributes) );
        }

        if( use_software_trigger_option != "software" && use_software_trigger_option != "hardware" )
        {
            std::cerr << "expected --trigger set as hardware or software, got " << use_software_trigger_option << std::endl;
            exit( 1 );
        }

        if( vm.count( "offsets" ) )
        {
            for( auto offset_str : comma::split( offsets_option, "," ) )
            {
                offsets.push_back( boost::lexical_cast< unsigned int >( offset_str ) );
            }
        }

        bool use_software_trigger;
        use_software_trigger_option == "hardware" ? use_software_trigger = false : use_software_trigger = true;
        snark::cameras::flycapture::camera::timestamp_policy when( timestamp_time_option );
        if ( use_software_trigger ) {
            if ( vm.count( "timestamp" ) ) { COMMA_THROW( comma::exception, "cannot specify timestamp capture moment when using software trigger" ); }
            when.value = snark::cameras::flycapture::camera::timestamp_policy::none;
        }

        if ( vm.count( "discard" ) ) { discard = 1; }
        discard_more_than = discard;

        comma::csv::format format;
        for (auto v : comma::split( fields, "," ))
        {
            if( v == "t" ) { format += "t"; }
            else { format += "ui"; }
        }

        boost::scoped_ptr< snark::cv_mat::serialization > serialization;
        if( vm.count( "no-header" ) )
        { serialization.reset( new snark::cv_mat::serialization( "", format ) ); }
        else
        { serialization.reset( new snark::cv_mat::serialization( fields, format, vm.count( "header" ) ) ); }

        if( verbose ) { std::cerr << "flycapture-multicam: connecting..." << std::endl; }
        multicam.reset( new snark::cameras::flycapture::camera::multicam( cameras, offsets, when ) );
        if( verbose ) { std::cerr << "flycapture-multicam: connected" << std::endl; }

        static std::unique_ptr<cv::Mat>  output_image;
        // Initialise output image, assumes all images of same type
        {
            int rows = 0, cols = 0, type = 0;
            for (auto& frame : multicam->read( when, use_software_trigger ).second)
            {
                rows += frame.rows;
                cols = std::max(cols, frame.cols);
                if (type != 0 && frame.type() != type)
                { COMMA_THROW(comma::exception, "cameras do not have common data type"); }
                type = frame.type();
            }
            output_image.reset( new cv::Mat(rows, cols, type));
        }

        while( running )
        {
            auto frames_pair = multicam->read( when, use_software_trigger );
            int y = 0;
            for (uint camera_number = 0; camera_number < frames_pair.second.size(); ++camera_number)
            {
                auto frame = frames_pair.second[camera_number];
                cv::Mat output_roi(*output_image, cv::Rect(0, y, frame.cols, frame.rows));
                frame.copyTo(output_roi);
                y += frame.rows;
            }
            serialization->write(
                std::cout,
                std::make_pair(frames_pair.first, *output_image)
            );
        }
        if( verbose ) { std::cerr << "flycapture-multicam: exited loop" << std::endl; }

        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << "flycapture-multicam: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "flycapture-multicam: unknown exception" << std::endl;
    }
    return 1;
}
