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

#include <fstream>
#include <sstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <comma/csv/options.h>
#include <comma/csv/ascii.h>
#include <comma/base/exception.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <comma/io/stream.h>
#include <comma/io/publisher.h>
#include "../../../../imaging/cv_mat/serialization.h"
#include "../../../../imaging/cv_mat/traits.h"
#include "../../../../visiting/eigen.h"
#include "../realsense.h"
#include "../traits.h"

using namespace snark::realsense;
comma::signal_flag signaled;
std::vector<uchar> margin_color={255,255,255};
bool discard_margin=false;
stream_args image_args(rs::preset::best_quality);

struct list
{
    struct output_t
    {
        std::string name;
        std::string usb_port;
        std::string serial_number;
    };
    static void process(rs::context& context, const comma::csv::options& csv);
    static std::string field_names();
};

struct device_t
{
    rs::context& context;
    rs::device* device;
    std::string output;
    std::string device_options;
    device_t(rs::context& context);
    device_t(rs::context& context,const std::string& s);
    void parse_option(const std::string& s);
    rs::device* find_device(const std::string& name, const std::string& value, std::function<bool (rs::device*,const std::string& s)> fn);
    static void help(std::ostream& out, const std::string& indent="    ");
};

void usage(bool detail)
{
    std::cerr<<"    Intel realsense camera streaming" << std::endl;
    std::cerr<<"        outputs binary csv of points cloud and mapped color" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "usage: " << comma::verbose.app_name() << " [ <options> ]" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "output fields:"<< std::endl;
    std::cerr<< "    t: timestamp"<< std::endl;
    std::cerr<< "    counter: camera timestamp counter"<< std::endl;
    std::cerr<< "    block: scan block id"<< std::endl;
    std::cerr<< "    point: 3d position of point in camera frame"<< std::endl;
    std::cerr<< "    index: 2d position of pixel in image frame"<< std::endl;
    std::cerr<< "    color: rgb color of point"<< std::endl;
    std::cerr<< "    "<< std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: show help" << std::endl;
    std::cerr << "    --output-format: print output format and exit"<<std::endl;
    std::cerr << "    --output-fields: print output fields and exit"<<std::endl;
    std::cerr << "    --camera-stream,--stream=<stream>: output cv images of the specified camera stream, instead of points cloud csv" << std::endl;
    std::cerr << "        <stream>: "<<stream::name_list()<<std::endl;
    std::cerr << "    --image-format=<width>,<height>,<format>,<frame_rate>: use the specified stream args, when not specified uses rs::preset::best_quality"<<std::endl;
    std::cerr << "        <width>: width of image"<<std::endl;
    std::cerr << "        <height>: height of image"<<std::endl;
    std::cerr << "        <format>: "<<format::name_list()<<std::endl;
    std::cerr << "        <frame_rate>: camera frame rate"<<std::endl;
    std::cerr << "    --device=<param>: specify device and its parameters, multiple --device=<param> can be specified"<<std::endl;
    std::cerr << "        <param>:"<<std::endl;
    device_t::help(std::cerr,"            ");
    std::cerr << "    --list,--list-devices: output available devices and exit, csv fields: "<<list::field_names()<< std::endl;
    std::cerr << "    --log=<directory>;period=<seconds>[;index]: log points to file" << std::endl;
    std::cerr << "        <directory>: write logs to this directory" << std::endl;
    std::cerr << "        period=<seconds>: period after which to start a new log file" << std::endl;
    std::cerr << "        index: write index file containing timestamp,file_number,file_offset for each frame" << std::endl;
    std::cerr << "    --output-image-from-points: instead of points data, output the camera stream to stdout" << std::endl;
    std::cerr << "    --verbose,-v: show detailed messages" << std::endl;
    std::cerr << "    --options,--camera-options=<options>: set options for all devices, <options> is a comma serparated list of <option_name>=<value>"<<std::endl;
    std::cerr << "    --list-options: list device options and exit"<<std::endl;
    std::cerr << "        output fields: device_index,name,value,min,max,step"<<std::endl;
    std::cerr << "    --get-options=<options>: get device options and exit; <options>: comma separated option names to be retrieved"<<std::endl;
    std::cerr << "        output fields: device_index,name,value"<<std::endl;
    std::cerr << "    --margin-color=<color>: color for points that are outside image camera; <color>: <r>,<g>,<b> default: 255,255,255"<<std::endl;
    std::cerr << "    --discard-margin: don't include marginal points in the output"<<std::endl;
    std::cerr << std::endl;
    if(detail)
    {
        std::cerr << "csv options:" << std::endl;
        std::cerr<< comma::csv::options::usage() << std::endl;
        std::cerr << std::endl;
    }
    else
    {
        std::cerr << "use -v or --verbose to see more detail" << std::endl;
        std::cerr << std::endl;
    }
    std::cerr << "example" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    to view camera streams:" << std::endl;
    std::cerr << "        " << comma::verbose.app_name() << "  --stream=color --image-format=0,0,bgr8,0 | cv-cat \"resize=2;view;null\" " << std::endl;
    std::cerr << "        " << comma::verbose.app_name() << "  --stream=depth | cv-cat \"brightness=10;resize=2;view;null\" " << std::endl;
    std::cerr << std::endl;
    std::cerr << "    to view points cloud mapped with color:" << std::endl;
    std::cerr << "        " << comma::verbose.app_name() << " --binary t,ui,3d,3ub --fields t,block,point/x,point/y,point/z,color/r,color/g,color/b | csv-select --binary t,ui,3d,3ub --fields t,,x,y,z \"z;less=4\" | view-points --binary t,ui,3d,3ub --fields t,block,x,y,z,r,g,b" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    to view points cloud without color map:"<< std::endl;
    std::cerr << "        " << comma::verbose.app_name() << " --binary t,ui,3d --fields t,block,point/x,point/y,point/z | csv-select --binary t,ui,3d --fields t,b,x,y,z \"z;less=4\" | view-points --binary t,ui,3d --fields t,block,x,y,z"<< std::endl;
    std::cerr << std::endl;
    std::cerr << "    to publish points cloud and then view them:"<< std::endl;
    std::cerr << "        " << comma::verbose.app_name() << " --binary t,ui,3d,3ub --fields t,block,point/x,point/y,point/z,color/r,color/g,color/b --device \"output=tcp:12345\"" << std::endl;
    std::cerr << "        socat -u tcp:localhost:12345 - | csv-select --binary t,ui,3d,3ub --fields t,,point/x,point/y,point/z \"z;less=4\" | view-points --binary t,ui,3d,3ub --fields t,block,x,y,z,r,g,b" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    to publish points cloud from mutliple devices:"<< std::endl;
    std::cerr << "        " << comma::verbose.app_name() << " --binary t,ui,3d,3ub --fields t,block,point/x,point/y,point/z,color/r,color/g,color/b --device \"port=4-1;output=tcp:12345\" --device \"port=4-2;output=tcp:6789\"" << std::endl;
    std::cerr << std::endl;
}

struct color_t
{
    std::vector<unsigned char> buf;
    color_t() : buf(3) { }
    void set(const uchar* ptr)
    {
        memcpy(&buf[0],ptr,size());
    }
    size_t size() const { return buf.size()*sizeof(unsigned char); }
};

template<typename T>
struct writer
{
    comma::io::publisher publisher;
    std::stringstream sbuf;
    comma::csv::output_stream<T> os;
    writer(const std::string& name,const comma::csv::options& csv);
    void write(const T& t);
};

struct points_t
{
    struct output_t
    {
        boost::posix_time::ptime t;
        unsigned counter;
        unsigned block;
        Eigen::Vector3d point;
        Eigen::Vector2i pixel;
        // todo: any more point attributes available?
        //cv::Mat?
        color_t color;
        output_t() : counter(0), block(0) { }
    };
    
    rs::device& device;
    bool has_color;
    stream color;
    snark::realsense::points_cloud points_cloud;
    ::writer<output_t> writer;
    
    points_t(device_t& device,bool has_color,const comma::csv::options& csv);
    void scan(unsigned block, bool output_image);
    //process points cloud
    static void process(std::vector<device_t>& devices,const comma::csv::options& csv, bool output_image);
    static void output_format();
    static void output_fields();
};

struct logger
{
        
    comma::csv::options csv_;
    std::string directory_;
    boost::scoped_ptr< std::ofstream > filestream;
    boost::scoped_ptr< comma::csv::output_stream < points_t::output_t > > csv_stream;
    boost::posix_time::time_duration period_;
    boost::posix_time::ptime start_;
    
    struct indexer
    {
        boost::posix_time::ptime t;
        comma::uint16 file;
        comma::uint64 offset;
        boost::scoped_ptr< std::ofstream > filestream;
        boost::scoped_ptr< comma::csv::binary_output_stream< indexer > > csv_stream;
        
        indexer() : file( 0 ), offset( 0 ) {}
        
        indexer( bool enabled, const std::string& directory ) : file ( 0 ), offset( 0 )
        {
            if( !enabled ) { return; }
            std::string index_file = directory + "/index.bin";
            filestream.reset( new std::ofstream( &index_file[0] ) );
            if( !filestream->is_open() ) { COMMA_THROW( comma::exception, "failed to open \"" << index_file << "\"" ); }
            csv_stream.reset( new comma::csv::binary_output_stream< indexer >( *filestream ) );
        }
        
        ~indexer() { if ( filestream ) { filestream->close(); } }
        
        void increment_file() { ++file; offset = 0; }
        
        void write_block( const boost::posix_time::ptime& timestamp, std::size_t num_records )
        {
            t = timestamp;
            if( csv_stream ) 
            { 
                csv_stream->write( *this );
                csv_stream->flush(); 
            }
            offset += num_records * comma::csv::format(comma::csv::format::value<points_t::output_t>()).size();
        }
    };
    
    indexer index_;
    
    logger() : directory_("") {}
    
    logger( const comma::csv::options& csv, const std::string& directory, const boost::posix_time::time_duration& period, bool index ) : csv_(csv), directory_( directory ), period_( period ), index_(index, directory)
    { }
    //logger( const comma::csv::options& csv, const std::string& directory, unsigned int size, bool index ) : directory_( directory ), csv_(csv), size_( size ), count_( 0 ), index_(index, directory) { }
    
    ~logger() { if ( filestream ) { filestream->close(); } }
    
    void update_on_time_( const boost::posix_time::ptime& t )
    {
        if( start_.is_not_a_date_time() ) { start_ = t; return; }
        if( ( t - start_ ) < period_ ) { return; }
        start_ = t;
        csv_stream.reset();
        filestream->close();
        filestream.reset();
        index_.increment_file();
    }
    
    void write( const points_t::output_t& t )
    {
        if ( directory_.empty() ) { return; } // no logging directory
        //         update_on_size_();
        update_on_time_( t.t );
        if( !filestream )
        {
            std::string filename = directory_ + '/' + boost::posix_time::to_iso_string( t.t ) + ( csv_.binary() ? ".bin" : ".csv" );
            filestream.reset( new std::ofstream( &filename[0] ) );
            if( !filestream->is_open() ) { COMMA_THROW( comma::exception, "failed to open \"" << filename << "\"" ); }
            csv_stream.reset( new comma::csv::output_stream<points_t::output_t>( *filestream, csv_ ) );
        }
        csv_stream->write(t);
        csv_stream->flush();
    }
    
//     void update_on_size_()
//     {
//         if( size_ == 0 ) { return; }
//         if( count_ < size_ ) { ++count_; return; }
//         count_ = 1;
//         filestream->close();
//         filestream.reset();
//         index_.increment_file();
//     }
};

boost::scoped_ptr<logger> logging;

namespace comma { namespace visiting {
    
template <> struct traits< list::output_t >
{
    template< typename K, typename V > static void visit( const K& k, const list::output_t& p, V& v )
    {
        v.apply( "name", p.name );
        v.apply( "usb_port", p.usb_port );
        v.apply( "serial_number", p.serial_number );
    }
};

template <> struct traits< color_t >
{
    template< typename K, typename V > static void visit( const K& k, const color_t& p, V& v )
    {
        //v.apply( "", p.buf );
        v.apply( "r", p.buf[0] );
        v.apply( "g", p.buf[1] );
        v.apply( "b", p.buf[2] );
    }
};

template <> struct traits< points_t::output_t >
{
    template< typename K, typename V > static void visit( const K& k, const points_t::output_t& p, V& v )
    {
        v.apply( "t", p.t );
        v.apply( "counter", p.counter );
        v.apply( "block", p.block );
        v.apply( "point", p.point );
        v.apply( "index", p.pixel );
        v.apply( "color", p.color );
    }
};

template <> struct traits< logger::indexer >
{
    template < typename K, typename V > static void visit( const K&, const logger::indexer& t, V& v )
    {
        v.apply( "t", t.t );
        v.apply( "file", t.file );
        v.apply( "offset", t.offset );
    }

    template < typename K, typename V > static void visit( const K&, logger::indexer& t, V& v )
    {
        v.apply( "t", t.t );
        v.apply( "file", t.file );
        v.apply( "offset", t.offset );
    }
};

} } // namespace comma { namespace visiting {

struct camera_stream
{
    static void process(rs::device& device,const std::string& stream_name)
    {
        stream stream(device,stream_name);
        stream.init(image_args);
        run_stream runner(device);
        stream.start_time=runner.start_time;
        snark::cv_mat::serialization::options opt;
        snark::cv_mat::serialization serializer(opt);
        while(!signaled)
        {
            if(!device.is_streaming()) { COMMA_THROW(comma::exception, "device not streaming" ); }
            device.wait_for_frames();
            // todo: instead of using image_args, get device format and then convert to BGR
            serializer.write(std::cout,stream.get_frame());
        }
        comma::verbose<<"signaled!"<<std::endl;
    }
};

void points_t::process(std::vector<device_t>& devices,const comma::csv::options& csv, bool output_image = false)
{
    comma::verbose<<"points_t::process"<<std::endl;
    bool has_color=csv.fields.empty() || csv.has_some_of_fields("color,color/r,color/g,color/b");
    comma::verbose<<"csv.fields "<<csv.fields<<std::endl;
    if(!has_color)
    { comma::verbose<<"not processing color"<<std::endl; }
    //
    std::vector<std::unique_ptr<points_t>> all;
    for(unsigned i=0;i<devices.size();i++)
    {
        all.push_back(std::unique_ptr<points_t>(new points_t(devices[i],has_color,csv)));
    }
    //comma::verbose<<"points_t::process running "<<device.is_streaming()<< " start_time "<< boost::posix_time::to_iso_string(runner.start_time) <<std::endl;
    if(devices.size()==1)
    {
        run_stream runner(*devices[0].device);
        for(unsigned block=0;!signaled;block++)
        {
            all[0]->scan(block, output_image);
        }
    }
    else
    {
        unsigned block=0;
        while(!signaled)
        {
            for(unsigned i=0;i<all.size()&&!signaled;i++)
            {
                run_stream runner(*devices[i].device);
                all[i]->scan(block++, output_image);
            }
        }
    }
    comma::verbose<<"received a signal"<<std::endl;
}

template<typename T>
writer<T>::writer(const std::string& name,const comma::csv::options& csv) : 
    publisher(name, csv.binary()?comma::io::mode::binary : comma::io::mode::ascii),
    os(sbuf, csv)
{ }
template<typename T>
void writer<T>::write(const T& t)
{
    os.write(t);
    std::string s=sbuf.str();
    publisher.write(&s[0],s.size());
    sbuf.str(std::string());
    sbuf.clear();
}


points_t::points_t(device_t& dev,bool has_color, const comma::csv::options& csv) : 
    device(*dev.device),has_color(has_color),color(device,rs::stream::color),points_cloud(device),
    writer(dev.output,csv)
{
    if(has_color)
    {
        if(image_args.preset)
            color.init(image_args);
        else
            color.init(stream_args(image_args.width,image_args.height,rs::format::rgb8,image_args.framerate));
        if(color.format.value!=rs::format::rgb8) { COMMA_THROW( comma::exception, "expected image format "<<rs::format::rgb8<<", got "<<color.format); }
    }
    points_cloud.init(image_args, has_color ? rs::stream::color : rs::stream::depth);
    comma::verbose<<"points_t(has_color: "<<has_color<<") done "<<std::endl;
}

void points_t::scan(unsigned block, bool output_image = false)
{
    if(!device.is_streaming()) { COMMA_THROW(comma::exception, "device not streaming" ); }
    device.wait_for_frames();
    //get points
    output_t out;
    out.t=boost::posix_time::microsec_clock::universal_time();
    out.counter=points_cloud.scan();
    out.block=block;
    std::pair<boost::posix_time::ptime,cv::Mat> pair;
    if(has_color) { pair=color.get_frame(); }
    const cv::Mat& mat=pair.second;
    rs::float3 point;
    std::size_t num_output = 0;
    for(unsigned index=0;index<points_cloud.count();index++)
    {
        bool discard=false;
        if(points_cloud.get(index,point))
        {
            out.point=Eigen::Vector3d(point.x,point.y,point.z);
            if(has_color)
            {
                auto color_pixel=points_cloud.project(point);
                const int cx = (int)std::round(color_pixel.x);
                const int cy = (int)std::round(color_pixel.y);
                out.pixel=Eigen::Vector2i(cx,cy);
                if(cx < 0 || cy < 0 || cx >= mat.cols || cy >= mat.rows)
                {
                    if(discard_margin) {discard=true;}
                    out.color.set(&margin_color[0]);
                }
                else
                {
                    out.color.set(mat.ptr(cy,cx));
                }
            }
            if(!discard)
            {
                if (logging) { logging->write(out); }
                if ( !output_image ) { writer.write(out); }
                num_output++;
            }
        }
    }
    if (logging) { logging->index_.write_block(out.t, num_output); }
    if (output_image) 
    { 
        snark::cv_mat::serialization::options opt;
        snark::cv_mat::serialization serializer(opt);
        // image is always RGB in points mode, convert to BGR so the output image is consistent with cv-cat
        cv::cvtColor(pair.second, pair.second, CV_RGB2BGR); 
        serializer.write(std::cout,pair) ;
    }
}

void points_t::output_format()
{
    std::cout<<comma::csv::format::value<points_t::output_t>() << std::endl;
}
void points_t::output_fields()
{
    std::cout<<comma::join( comma::csv::names<points_t::output_t>(true), ',' ) << std::endl;
}
std::string list::field_names()
{
    return comma::join( comma::csv::names<list::output_t>(true), ',' );
}
void list::process(rs::context& context, const comma::csv::options& csv)
{
    int count=context.get_device_count();
    comma::csv::output_stream<list::output_t> os(std::cout, csv);
    for(int i=0;i<count;i++)
    {
        list::output_t out;
        rs::device* device=context.get_device(i);
        out.name=device->get_name();
        out.usb_port=device->get_usb_port_id();
        out.serial_number=device->get_serial();
        os.write(out);
    }
}
struct options_t
{
    static void get(std::vector<device_t> devices,const std::string& list)
    {
        std::vector<std::string> v=comma::split(list,",");
        for(unsigned i=0;i<devices.size();i++)
        {
            for(auto s : v)
            {
                option o(s);
                o.read(*devices[i].device);
                std::cout<<i<<","<<o.name()<<","<<o.value<<std::endl;
            }
        }
    }
    static void list(std::vector<device_t> devices)
    {
        for(unsigned i=0;i<devices.size();i++)
        {
            for(const auto a : option::get_names())
            {
                option_range o(a.second);
                try
                {
                    o.read(*devices[i].device);
                    std::cout<<i<<","<<o.name()<<","<<o.value<<","<<o.min<<","<<o.max<<","<<o.step<<std::endl;
                }
                catch(std::exception& ex)
                {
                    comma::verbose<<o.name()<<"; error: "<<ex.what()<<std::endl;
                }
            }
        }
    }
    static void set(device_t& device,const std::string& list)
    {
        std::vector<device_t> v={device};
        set(v,list);
    }
    static void set(std::vector<device_t>& devices,const std::string& list)
    {
        std::vector<std::string> v=comma::split(list,",");
        for(auto s : v)
        {
            std::vector<std::string> name_value=comma::split(s,"=");
            if(name_value.size()!=2) { COMMA_THROW( comma::exception, "expected name=value, got: "<<s); }
            option o(name_value[0]);
            o.value=boost::lexical_cast<double>(name_value[1]);
            for(auto d : devices)
            {
                o.write(*d.device);
            }
        }
    }
};
void set_margin_color(boost::optional<std::string> s)
{
    if(s)
    {
        std::vector<std::string> v=comma::split(*s,",");
        if(v.size()!=margin_color.size()) {COMMA_THROW(comma::exception,"--margin-color expected "<<margin_color.size()<<" numbers for color, got: "<<v.size());} 
        for(std::size_t i=0;i<v.size();i++)
        {
            unsigned c=boost::lexical_cast<unsigned>(v[i]);
            if(c>255) {COMMA_THROW(comma::exception,"--margin-color item, expected value between 0 and 255, got "<<c);}
            margin_color[i]=uchar(c);
        }
    }
}

void device_t::help(std::ostream& out, const std::string& indent)
{
//     std::cerr << "    --usb-port,--port=<port>: use device on given usb port" << std::endl;
//     std::cerr << "    --serial-number,--serial=<number>: use device with given serial number" << std::endl;
    out<<indent<<"semicolon separated list of options for device"<<std::endl;
    out<<indent<<"    <identifier>=<value>: <identifier> is either 'port' or 'serial' and <value> is usb port id or serial number"<<std::endl;
    out<<indent<<"    output=<stream>: stream name to publish data to, default '-' for stdout"<<std::endl;
    out<<indent<<"    options:<name_value_csv>: options to set for this device only, comma separated list of <option_name>=<value>"<<std::endl;
    out<<indent<<"example: "<<std::endl;
    out<<indent<<"    --device \"port=4-1;output=tcp:1234;options:color_brightness=50,color_contrast=75\" --device \"port=4-2\" --device \"serial=987654;output=tcp:5678\""<<std::endl;
}
device_t::device_t(rs::context& context) : context(context), device(NULL), output("-")
{
    auto count=context.get_device_count();
    if(count!=1) { COMMA_THROW( comma::exception, "multiple camera present, please specify --device"); }
    device=context.get_device(0);
    comma::verbose<<"using: "<<device->get_name()<<", usb port id: "<<device->get_usb_port_id()<<", serial: "<<device->get_serial()<<std::endl;
}
device_t::device_t(rs::context& context, const std::string& opt_str) : context(context), device(NULL), output("-")
{
    std::vector<std::string> v=comma::split(opt_str,";");
    for(const auto& s : v)
    {
        parse_option(s);
    }
    if(device==NULL)
    {
        auto count=context.get_device_count();
        if(count!=1) {COMMA_THROW(comma::exception,"multiple camera present, please specify port=<s> or serial=<s> in --device");}
        device=context.get_device(0);
        comma::verbose<<"using: "<<device->get_name()<<", usb port id: "<<device->get_usb_port_id()<<", serial: "<<device->get_serial()<<std::endl;
    }
    if(!device_options.empty()) {options_t::set(*this,device_options);}
}
void device_t::parse_option(const std::string& str)
{
    std::vector<std::string> v=comma::split(str,"=");
    if(v.size()==2)
    {
        if(v[0]=="port")
        {
            device=find_device(v[0], v[1], [](rs::device* device, const std::string& s) { return device->get_usb_port_id()==s;} );
            return;
        }
        else if(v[0]=="serial")
        {
            device=find_device(v[0], v[1], [](rs::device* device, const std::string& s) { return device->get_serial()==s;} );
            return;
        }
        else if(v[0]=="output")
        {
            output=v[1];
            return;
        }
        //else fall through
    }
    v=comma::split(str,":");
    if(v.size()!=2 || v[0]!="options") {COMMA_THROW(comma::exception,"invalid device option "<<str);}
    device_options=v[1];
}
rs::device* device_t::find_device(const std::string& name, const std::string& value, std::function<bool (rs::device*,const std::string& s)> fn)
{
    auto count=context.get_device_count();
    for(int i=0;i<count;i++)
    {
        rs::device* device=context.get_device(i);
        if(fn(device,value))
        {
            comma::verbose<<"matched: "<<device->get_name()<<", usb port id: "<<device->get_usb_port_id()<<", serial: "<<device->get_serial()<<std::endl;
            return device;
        }
    }
    COMMA_THROW(comma::exception,"no device matched "<<name<<"="<<value);
}

// rs::device* match_device(rs::context& context,std::string s)
// {
//     std::vector<std::string> v=comma::split(s,"=");
//     if(v.size()!=2) {COMMA_THROW(comma::exception,"exepcted <name>=<value>, got "<<s);}
//     int count=context.get_device_count();
//     if(v[0]=="port")
//     {
//         for(int i=0;i<count;i++)
//         {
//             rs::device* device=context.get_device(i);
//             if(v[1]==device->get_usb_port_id())
//             {
//                 comma::verbose<<"matched: "<<device->get_name()<<", usb port id: "<<device->get_usb_port_id()<<std::endl;
//                 return device;
//             }
//         }
//         COMMA_THROW(comma::exception,"no device matched port="<<v[1]);
//     }
//     else if(v[0]=="serial")
//     {
//         for(int i=0;i<count;i++)
//         {
//             rs::device* device=context.get_device(i);
//             if(v[1]==device->get_serial())
//             {
//                 comma::verbose<<"matched: "<<device->get_name()<<", serial: "<<device->get_serial()<<std::endl;
//                 return device;
//             }
//         }
//         COMMA_THROW(comma::exception,"no device matched serial="<<v[1]);
//     }
//     else {COMMA_THROW(comma::exception,"expected <name>: port or serial, got "<<v[0]);}
// }

int main( int argc, char** argv )
{
    comma::command_line_options options( argc, argv, usage );
    try
    {
        comma::csv::options csv(options);
        bool do_list=options.exists("--list-devices,--list");
        csv.full_xpath=true;
        if(options.exists("--output-format"))
        { 
             points_t::output_format();
            return 0; 
        }
        if(options.exists("--output-fields"))
        {
            points_t::output_fields();
            return 0;       
        }
        if (options.exists("--log"))
        {
            std::string args = options.value<std::string>("--log");
            std::vector <std::string> v = comma::split(args, ';');
            std::string directory = v[0];
            if (directory.empty() ) { COMMA_THROW(comma::exception, "please specify --log=directory"); }
            bool index = false;
            boost::optional <boost::posix_time::time_duration> time_duration;
            for (unsigned opt = 1; opt < v.size(); ++opt )
            {
                std::vector < std::string > w = comma::split(v[opt], '=');
                if ( w[0] == "index" ) 
                { 
                    index = true; 
                }
                else if ( w[0] == "period" )
                {
                    if (w.size() != 2 ) { COMMA_THROW(comma::exception, "please specify period=<seconds>"); }
                    double period = boost::lexical_cast< double >( w[1] );
                    comma::uint32 seconds = static_cast<int>(period);
                    time_duration = boost::posix_time::seconds( seconds ) + boost::posix_time::microseconds( static_cast< long >( period - seconds ) * 1000000 ) );
                }
            }
            if (!time_duration) { COMMA_THROW(comma::exception, "please specify period=<seconds>"); }
            logging.reset(new logger(csv, directory, *time_duration, index));
        }
        //
        rs::context context;
        if(do_list) { list::process(context,csv); return 0; }
        //
        std::string stream=options.value<std::string>("--camera-stream,--stream","");
        std::string image_format=options.value<std::string>("--image-format","");
        if(!image_format.empty())
        {
            stream_args agrs;
            image_args=comma::csv::ascii<stream_args>().get(agrs,image_format);
            comma::verbose<<"used image_format: "<<bool(image_args.preset)<<"; "<<image_args.width<<","<<image_args.height<<","<<image_args.format<<","<<image_args.framerate<<std::endl;
        }
        set_margin_color(options.optional<std::string>("--margin-color"));
        discard_margin=options.exists("--discard-margin");
        //select devices
        auto count=context.get_device_count();
        comma::verbose<<"device count: "<<count<<std::endl;
        if(count==0) { COMMA_THROW( comma::exception, "no realsense camera found!"); }
        std::vector<std::string> devices=options.values<std::string>("--device");
        std::vector<device_t> selected;
        if(devices.size()==0)
        {
            selected.push_back(device_t(context));
        }
        else
        {
            for(const std::string& s : devices)
            {
                selected.push_back(device_t(context,s));
            }
        }
        if(selected.size()==0) { COMMA_THROW( comma::exception, "no device match found"); }
        //
        std::string camera_options=options.value<std::string>("--options,--camera-options","");
        if(!camera_options.empty())
        {
            options_t::set(selected,camera_options);
        }
        if(options.exists("--list-options"))
        {
            options_t::list(selected);
            return 0;
        }
        std::string get_options=options.value<std::string>("--get-options","");
        if(!get_options.empty())
        {
            options_t::get(selected,get_options);
            return 0;
        }
        if(!stream.empty())
        {
            if(selected.size()!=1) { COMMA_THROW( comma::exception, "multiple camera not implemented yet!"); }
            camera_stream::process(*selected[0].device,stream);
        }
        else
        {
            points_t::process(selected,csv, options.exists("--output-image-from-points"));
        }
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
