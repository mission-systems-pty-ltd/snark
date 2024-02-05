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

/// @authors john gardenier, vsevolod vlaskine

#include <cmath>
#include <Eigen/Core>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <cv.h>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/name_value/ptree.h>
#include <comma/name_value/serialize.h>
#include <comma/string/string.h>
#include "../../../../imaging/camera/pinhole.h"
#include "../../../../imaging/camera/traits.h"
#include "../../../../imaging/cv_mat/serialization.h"
#include "../../../../imaging/cv_mat/traits.h"
#include "../../../../timing/timestamped.h"
#include "../../../../timing/traits.h"
#include "../../../../visiting/traits.h"

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "kinect-cat: acquire data from kinect camera, output to stdout as ascii (default) or binary" << std::endl;
    std::cerr << std::endl;
    std::cerr << "kinect-cat: usage: kinect-cat [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "kinect-cat: options" << std::endl;
    std::cerr << "kinect-cat:     --as-binary: output binary in default format; a convenience option" << std::endl;
    std::cerr << "kinect-cat:     --output-fields; print default output fields to stdout and exit" << std::endl;
    std::cerr << "kinect-cat:     --output-format; print default output format to stdout and exit" << std::endl;
    std::cerr << "kinect-cat:     --version=[<version>]; kinect version; default: 2" << std::endl;
    std::cerr << "kinect-cat:     --verbose; more info output" << std::endl;
    std::cerr << std::endl;
    std::cerr << "kinect-cat: csv options" << std::endl;
    if( verbose ) { std::cerr << comma::csv::options::usage() << std::endl; }
    else { std::cerr << "kinect-cat:     run kinect-cat --help --verbose for details..." << std::endl; }
    std::cerr << std::endl;
}

namespace snark { namespace kinect {

struct color // quick and dirty
{
    unsigned char r, g, b;
    color( unsigned char r, unsigned char g, unsigned char b ) : r( r ), g( g ), b( b ) {}
    color() : r( 0 ), g( 0 ), b( 0 ) {}
};

struct point
{
    boost::posix_time::ptime t;
    Eigen::Vector3d coordinates;
    kinect::color color;
    Eigen::Vector2i index; // pixel index
    unsigned int block;
    point(): coordinates( Eigen::Vector3d::Zero() ), index( Eigen::Vector2i::Zero() ), block( 0 ) {}
};

struct frame // todo
{
    class const_iterator
    {
        public:
            const_iterator& operator++() { ++index_; return *this; }
            bool operator==( const const_iterator& rhs ) const { return index_ == rhs.index_; }
            bool operator!=( const const_iterator& rhs ) const { return !operator==( rhs ); }
            kinect::point operator*() const // todo: fill from frame
            {
                kinect::point p;
                p.t = frame_->t;
                p.block = frame_->block;
                p.coordinates = Eigen::Vector3d( 1, 2, 3 );
                p.color = kinect::color( 4, 5, 6 );
                p.index = Eigen::Vector2i( 7, 8 );
                return p;
            }

        private:
            friend class frame;
            unsigned int index_;
            const frame* frame_;
    };

    const_iterator begin() const { const_iterator it; it.index_ = 0; it.frame_ = this; return it; } // todo

    const_iterator end() const { const_iterator it; it.index_ = 4; it.frame_ = this; return it; } // todo

    frame() : block( 0 ) {}

    boost::posix_time::ptime t;
    unsigned int block;
};

class camera
{
    public:
        bool read() // todo: currently just a demo; read one kinect frame, whatever that means; put in current_
        {
            current_.t = boost::posix_time::microsec_clock::universal_time(); // it's better to get timestamp from the camera, if available
            return current_.block++ < 3;
        }

        const kinect::frame& frame() const { return current_; }

    private:
        kinect::frame current_;
};

} } // namespace snark { namespace kinect {

namespace comma { namespace visiting {

template <> struct traits< snark::kinect::color >
{
    template< typename K, typename V > static void visit( const K& k, const snark::kinect::color& p, V& v )
    {
        v.apply( "r", p.r );
        v.apply( "g", p.g );
        v.apply( "b", p.b );
    }
};

template <> struct traits< snark::kinect::point >
{
    template< typename K, typename V > static void visit( const K& k, const snark::kinect::point& p, V& v )
    {
        v.apply( "t", p.t );
        v.apply( "coordinates", p.coordinates );
        v.apply( "color", p.color );
        v.apply( "index", p.index );
        v.apply( "block", p.block );
    }
};



} } // namespace comma { namespace visiting {


// create a custom logger
/// [logger]
#include <fstream>
#include <cstdlib>
class CowboxLogger: public libfreenect2::Logger
{
private:
  std::string serial_;
public:
  CowboxLogger(Level level, std::string serial)
  {
    level_ = level;
    serial_ = serial;
  }
  void setSerial(std::string serial){ serial_ = serial; }
  virtual void log(Level level, const std::string &message)
  {
    std::cerr << "kinect-cat: " << serial_ << " :libfreenect2: [" << level2str(level) << "] " << message << std::endl;
  }
};
/// [logger]


std::string serial = "";

int main( int ac, char** av )
{
	CowboxLogger *logger = new CowboxLogger(libfreenect2::Logger::Debug, serial);
    try
    {
        comma::command_line_options options( ac, av, usage );
        std::string version = options.value< std::string >( "--version", "2" );
        if( version != "2" ) { std::cerr << "kinect-cat: kinect-cat: only version 2 is supported for now; got: \"" << version << "\"" << std::endl; return 1; }
        if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< snark::kinect::point >( true ), ',' ) << std::endl; return 0; }
        if( options.exists( "--output-format" ) ) { std::cout << comma::csv::format::value< snark::kinect::point >() << std::endl; return 0; }
        comma::csv::options csv( options );
        csv.full_xpath = true;
        if( !csv.binary() && options.exists( "--as-binary" ) ) { csv.format( comma::csv::format::value< snark::kinect::point >() ); }
        comma::csv::output_stream< snark::kinect::point > ostream( std::cout, csv );
        snark::kinect::camera camera;

        libfreenect2::setGlobalLogger(logger);
        /// [context]

        // struct device
        // {
        //     libfreenect2::Freenect2 freenect2;
        //     libfreenect2::Freenect2Device *dev = NULL;
        //     libfreenect2::PacketPipeline *pipeline = 0;

        //     device( ??? ) { ... }

        //     void close() { ... }
        //     ~device() { close(); /* anything else to clean up? */ }
        // };


        libfreenect2::Freenect2 freenect2;
        libfreenect2::Freenect2Device *dev = 0;
        libfreenect2::PacketPipeline *pipeline = 0;

        /// [context]
        /// [discovery]
        if(freenect2.enumerateDevices() == 0)
        {
            std::cerr << "kinect-cat: no device connected!" << std::endl;
            return 0;
        }


        if( options.exists( "--serial" ))
        {
            serial = options.value< std::string >( "--serial" );


        }

        if( options.exists( "--list-cameras" ))
        {
            int number_devices = freenect2.enumerateDevices();
            std::cerr << number_devices << " kinects found" << std::endl;
            for(int i = 0; i < number_devices; ++i){
                std::cerr << "kinect-cat: Kinect " << i << " serial: " << freenect2.getDeviceSerialNumber(i) << std::endl;

            }

            return 0;
        }

        if (serial == "")
        {
            serial = freenect2.getDefaultDeviceSerialNumber();
        }

        logger->setSerial(serial);
        int deviceId = -1;
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
        if( options.exists( "--cuda" ))
        {
            //std::cerr << "kinect-cat: here, pipeline: " << pipeline << std::endl;
            if(!pipeline)
            {
                //std::cerr << "kinect-cat: Using cuda pipeline" << std::endl;
                pipeline = new libfreenect2::CudaPacketPipeline(deviceId);
                std::cerr << "kinect-cat: " << serial << ": Using cuda pipeline" << std::endl;
            }
        }
        else if( options.exists( "--cudakde" ))
        {
            if(!pipeline)
            {
                pipeline = new libfreenect2::CudaKdePacketPipeline(deviceId);
                std::cerr << "kinect-cat: " << serial << ": Using cudakde pipeline" << std::endl;
            }
        }
        else
#endif // def LIBFREENECT2_WITH_CUDA_SUPPORT
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        if( options.exists( "--cl" ))
        {
            if(!pipeline)
            {
                pipeline = new libfreenect2::OpenCLPacketPipeline(deviceId);
                std::cerr << "kinect-cat: " << serial << ": Using cl pipeline" << std::endl;
            }
        }
        else if( options.exists( "--clkde" ))
        {
            if(!pipeline)
            {
                pipeline = new libfreenect2::OpenCLKdePacketPipeline(deviceId);
                std::cerr << "kinect-cat: " << serial << ": Using clkde pipeline" << std::endl;
            }
        }
        else
#endif // def LIBFREENECT2_WITH_OPENCL_SUPPORT
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
        if( options.exists( "--gl" ))
        {
            if(!pipeline)
            {
                pipeline = new libfreenect2::OpenGLPacketPipeline();
                std::cerr << "kinect-cat: " << serial << ": Using gl pipeline" << std::endl;
            }
        }
        else
#endif // def LIBFREENECT2_WITH_OPENGL_SUPPORT
        if(!pipeline)
        {
            pipeline = new libfreenect2::CpuPacketPipeline();
            std::cerr << "kinect-cat: " << serial << ": Using cpu pipeline" << std::endl;
        }


        /// [discovery]

        if(pipeline)
        {
        /// [open]
            dev = freenect2.openDevice(serial, pipeline);
        /// [open]
        }
        else
        {
            dev = freenect2.openDevice(serial);
        }


        if(dev == 0)
        {
            std::cerr << "kinect-cat: " << serial << ": failure opening device!" << std::endl;
            return 0;
        }


        bool enable_rgb = false;
        bool enable_depth = true;
        /// [listeners]
        int types = 0;
        if (enable_rgb)
        types |= libfreenect2::Frame::Color;
        if (enable_depth)
        types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
        libfreenect2::SyncMultiFrameListener listener(types);
        libfreenect2::FrameMap frames;

        dev->setColorFrameListener(&listener);
        dev->setIrAndDepthFrameListener(&listener);

        /// [listeners]

        /// [start]
        if (enable_rgb && enable_depth)
        {
            if (!dev->start())
            {
                return 0;
            }
        }
        else
        {
        if (!dev->startStreams(enable_rgb, enable_depth))
          return 0;
        }
        const int width_ir = 512;
        const int height_ir = 424;

        std::string        fields = "t,rows,cols,type";
        bool               header_only = false;

        comma::csv::format format( "t,3ui" );

        if( options.exists( "--no-header" ))
        {
            fields = "";
        }
        else
        {
            header_only = ( options.exists( "--header" ));
        }

        // these intrinsics are probably with respect to raw unflipped image, but not entirely sure. Commented out due to output of raw frames and undistortion external to kinect-cat or libfreenect2
        // if( options.exists( "--get-intrinsics-kinect" )){ // todo: --get-intrinsics; --get-pinhole-config
        //
        //     libfreenect2::Freenect2Device::IrCameraParams ir_params = dev->getIrCameraParams();
        //     // put these parameters in json block "kinect"
        //     std::cout << "fx=" << ir_params.fx << std::endl; ///< Focal length x (pixel)
        //     std::cout << "fy=" << ir_params.fy << std::endl; ///< Focal length y (pixel)
        //     std::cout << "cx=" << ir_params.cx << std::endl; ///< Principal point x (pixel)
        //     std::cout << "cy=" << ir_params.cy << std::endl; ///< Principal point y (pixel)

        //     std::cout << "k1=" << ir_params.k1 << std::endl; ///< Radial distortion coefficient, 1st-order
        //     std::cout << "k2=" << ir_params.k2 << std::endl; ///< Radial distortion coefficient, 2nd-order
        //     std::cout << "k3=" << ir_params.k3 << std::endl; ///< Radial distortion coefficient, 3rd-order

        //     std::cout << "p1=" << ir_params.p1 << std::endl; ///< Tangential distortion coefficient
        //     std::cout << "p2=" << ir_params.p2 << std::endl; ///< Tangential distortion coefficient

        //     return 0;
        // }
        // if( options.exists( "--get-intrinsics-pinhole" )){ // todo: --get-intrinsics; --get-pinhole-config
        //     libfreenect2::Freenect2Device::IrCameraParams ir_params = dev->getIrCameraParams();
        //     // put these parameters in json block "metric"
        //     snark::camera::pinhole::config_t config;
        //     double px_pitch = 10 * 0.000001; //10 um
        //     config.image_size = Eigen::Vector2i( width_ir, height_ir );
        //     config.sensor_size = Eigen::Vector2d( width_ir * px_pitch, height_ir * px_pitch );
        //     config.focal_length = ir_params.fx * config.sensor_size->x() / config.image_size.x();
        //     config.principal_point = Eigen::Vector2d( ir_params.cx, ir_params.cy);
        //     config.distortion = snark::camera::pinhole::config_t::distortion_t( snark::camera::pinhole::config_t::distortion_t::radial_t( ir_params.k1, ir_params.k2, ir_params.k3 ), snark::camera::pinhole::config_t::distortion_t::tangential_t( ir_params.p1, ir_params.p2 ) );
        //     comma::write_json( config, std::cout );

        //     dev->stop();
        //     dev->close();
        //     return 0;
        // }

		// these extrinsics have not been verified or tested
   //      if( options.exists( "--get-extrinsics-kinect" )){
   //          libfreenect2::Freenect2Device::ColorCameraParams color_params = dev->getColorCameraParams();


			// // These parameters are used in [a formula](https://github.com/OpenKinect/libfreenect2/issues/41#issuecomment-72022111) to map coordinates in the
			// // depth camera to the color camera.
			// // They cannot be used for matrix transformation.



   //          std::cout << "shift_d=" << color_params.shift_d << std::endl;
   //          std::cout << "shift_m=" << color_params.shift_m << std::endl;

   //          std::cout << "mx_x3y0=" << color_params.mx_x3y0 << std::endl;
   //          std::cout << "mx_x0y3=" << color_params.mx_x0y3 << std::endl;
   //          std::cout << "mx_x2y1=" << color_params.mx_x2y1 << std::endl;
   //          std::cout << "mx_x1y2=" << color_params.mx_x1y2 << std::endl;
   //          std::cout << "mx_x2y0=" << color_params.mx_x2y1 << std::endl;
   //          std::cout << "mx_x0y2=" << color_params.mx_x1y2 << std::endl;
   //          std::cout << "mx_x1y1=" << color_params.mx_x2y1 << std::endl;
   //          std::cout << "mx_x1y0=" << color_params.mx_x1y2 << std::endl;
   //          std::cout << "mx_x0y1=" << color_params.mx_x2y1 << std::endl;
   //          std::cout << "mx_x0y0=" << color_params.mx_x1y2 << std::endl;

   //          std::cout << "my_x3y0=" << color_params.mx_x3y0 << std::endl;
   //          std::cout << "my_x0y3=" << color_params.mx_x0y3 << std::endl;
   //          std::cout << "my_x2y1=" << color_params.mx_x2y1 << std::endl;
   //          std::cout << "my_x1y2=" << color_params.mx_x1y2 << std::endl;
   //          std::cout << "my_x2y0=" << color_params.mx_x2y1 << std::endl;
   //          std::cout << "my_x0y2=" << color_params.mx_x1y2 << std::endl;
   //          std::cout << "my_x1y1=" << color_params.mx_x2y1 << std::endl;
   //          std::cout << "my_x1y0=" << color_params.mx_x1y2 << std::endl;
   //          std::cout << "my_x0y1=" << color_params.mx_x2y1 << std::endl;
   //          std::cout << "my_x0y0=" << color_params.mx_x1y2 << std::endl;

   //          dev->stop();
   //          dev->close();
   //          return 0;
   //      }



        snark::cv_mat::serialization serialization( fields, format, header_only );

        // libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
        // libfreenect2::Frame depth_undistorted(512, 424, 4);
        // libfreenect2::Frame ir_undistorted(512, 424, 4);



        // int first_timestamp = true;
        //boost::posix_time::ptime libfreenect2_time_at_offset;
        //uint libfreenect2_offset = 0;
        /// [loop start]
        comma::signal_flag is_shutdown( comma::signal_flag::hard );
        while( !is_shutdown )
        {
        	//std::cerr << "--> 0: is_shutdown: " << ( is_shutdown ? "set" : "not set" ) << std::endl;
            if (!listener.waitForNewFrame(frames, 10*1000)) // 10 seconds
            {
              std::cerr << "kinect-cat: " << serial << ": timeout!" << std::endl;
              return 0;
            }

            //libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
            libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
            libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

            boost::posix_time::ptime timestamp1 = boost::posix_time::microsec_clock::universal_time();

            // registration->undistortDepth( depth, &depth_undistorted );
            // registration->undistortDepth( ir, &ir_undistorted );

            cv::Mat irMat = cv::Mat(
                    height_ir, width_ir,
                    CV_32FC1, ir->data);
            cv::Mat depthMat = cv::Mat(
                    height_ir, width_ir,
                    CV_32FC1, depth->data);


            cv::Mat depthMat_uint16 = cv::Mat(height_ir, width_ir, CV_16UC1);
            depthMat.convertTo(depthMat_uint16, CV_16UC1); //depthMat_uint16 contains depth per pixel in mm
            cv::flip(depthMat_uint16, depthMat_uint16, 1); //flip horizontally


            cv::Mat irMat_uint16 = cv::Mat(height_ir, width_ir, CV_16UC1);
            irMat.convertTo(irMat_uint16, CV_16UC1); //irMat contains intensity in integers up to 65535, so irMat_uint16 does to
           	cv::flip(irMat_uint16, irMat_uint16, 1); //flip horizontally

            cv::Mat frame(height_ir, width_ir, CV_16UC2); // 2 channel image

            std::vector<cv::Mat> channels;

            channels.push_back(irMat_uint16);
            channels.push_back(depthMat_uint16);
            cv::merge(channels, frame);

            std::pair< boost::posix_time::ptime, cv::Mat > pair;


            // if(first_timestamp){
            //     libfreenect2_time_at_offset = boost::posix_time::microsec_clock::universal_time();
            //     libfreenect2_offset = ir->timestamp;
            //     first_timestamp = false;
            // }

            // // is ir->timestamp equal to depth->timestamp?
            // boost::posix_time::ptime timestamp2 = libfreenect2_time_at_offset + boost::posix_time::microseconds( (ir->timestamp - libfreenect2_offset) * 100 ); // ir-> timestamp is uint32 counting 0.1 ms. Will rollover every 119.3 hours

            // libfreenect2 frame timestamp info: https://github.com/OpenKinect/libfreenect2/issues/721
            // ideally, we want the timestamp for each of the 9 raw ir images, which are the input for a single depth image
            // pair.first = timestamp2; // I don't trust this timestamp as I get framerates >30 Hz

            pair.first = timestamp1;
            pair.second = frame;

            // todo: investigate
            // the following is the right way of doing it (see note in imaging/cv_mat/serialization.cpp),
            // but it blocks most of the time on signal in a very strange way:
            //serialization.write_to_stdout(pair, true);
            serialization.write( std::cout, pair, true );

// content of libfreenect2::Frame
//            size_t width;           ///< Length of a line (in pixels).
//            size_t height;          ///< Number of lines in the frame.
//            size_t bytes_per_pixel; ///< Number of bytes in a pixel. If frame format is 'Raw' this is the buffer size.
//            unsigned char* data;    ///< Data of the frame (aligned). @see See Frame::Type for pixel format.
//            uint32_t timestamp;     ///< Unit: roughly or exactly 0.1 millisecond
//            uint32_t sequence;      ///< Increasing frame sequence number
//            float exposure;         ///< From 0.5 (very bright) to ~60.0 (fully covered)
//            float gain;             ///< From 1.0 (bright) to 1.5 (covered)
//            float gamma;            ///< From 1.0 (bright) to 6.4 (covered)
//            uint32_t status;        ///< zero if ok; non-zero for errors.
//            Format format;          ///< Byte format. Informative only, doesn't indicate errors.

            listener.release(frames);

        }
        // these should be closed on SIGINT
        dev->stop();
        dev->close();
        /// [stop]

//        while( camera.read() && std::cout.good() ) // todo? if too slow, parallelize reading frame and output with tbb
//        {
//            for( snark::kinect::frame::const_iterator it = camera.frame().begin(); it != camera.frame().end(); ++it ) { ostream.write( *it ); }
//        }
        return 0;

    }
    catch( std::exception& ex ) { std::cerr << "kinect-cat: " << serial << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "kinect-cat: " << serial << ": kinect-cat: unknown exception" << std::endl; }

    delete logger;
    // todo? if dev is open, do you need to stop and close it before exiting?

    return 1;
}
