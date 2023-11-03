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

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include "../../../../imaging/cv_mat/pipeline.h"
#include "../../../../imaging/cv_mat/bursty_pipeline.h"
#include <opencv2/imgproc/types_c.h>
#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigECamera.h>
#include <pylon/usb/BaslerUsbCamera.h>

static double default_timeout = 3.0;

static std::string default_exposure = "auto_continuous";
static std::string default_gain = "auto_continuous";

// These default values are determined by dumping the config from the
// Pylon Viewer app after connecting to freshly booted basler-aca1300-75gm
static float default_exposure_lower_limit = 80.0;
static float default_exposure_upper_limit = 10000.0;

static const char* possible_header_fields = "t,rows,cols,type,size,counters";
static const char* default_header_fields = "t,rows,cols,type";

static void bash_completion( unsigned const ac, char const * const * av )
{
    static const char* completion_options =
        " --help -h --help-filters --help-types --verbose -v"
        " --address --serial-number --list-cameras"
        " --test-image"
        " --fields -f"
        " --header-only --no-header"
        " --offset-x --offset-y --width --height"
        " --pixel-format"
        " --reverse-x --reverse-y"
        " --binning-horizontal --binning-vertical"
        " --frame-rate --exposure --gain"
        " --discard --buffer"
        " --frame-trigger --line-trigger --line-rate"
        " --encoder-ticks"
        " --timeout"
        " --packet-size --inter-packet-delay --num-cameras"
        ;

    std::cout << completion_options << std::endl;
    exit( 0 );
}

static void usage( bool verbose = false )
{
    std::cerr << "\nacquire images from a basler camera";
    std::cerr << "\n";
    std::cerr << "\noutput to stdout as serialized cv::Mat";
    std::cerr << "\n";
    std::cerr << "\nusage: basler-cat [<options>] [<filters>]";
    std::cerr << "\n";
    std::cerr << "\ngeneral options";
    std::cerr << "\n    --help,-h                    display help message";
    std::cerr << "\n    --help-filters               show help on filters";
    std::cerr << "\n    --help-types                 show help on image types";
    std::cerr << "\n    --address=[<address>]        camera address; default: first available";
    std::cerr << "\n    --serial-number=[<num>]      camera serial number, alternative to --address";
    std::cerr << "\n    --list-cameras               output camera list and exit";
    std::cerr << "\n                                 add --verbose for more detail";
    std::cerr << "\n    --test-image=[<num>]         output test image <num>; possible values: 1-6";
    std::cerr << "\n    --verbose,-v                 be more verbose";
    std::cerr << "\n";
    std::cerr << "\nimage options";
    std::cerr << "\n    --fields,-f=<fields>         header fields, possible values:";
    std::cerr << "\n                                 possible values: " << possible_header_fields;
    std::cerr << "\n                                 default: " << default_header_fields;
    std::cerr << "\n    --header-only                output header only";
    std::cerr << "\n    --no-header                  output image data only";
    std::cerr << "\n    --height=<pixels>            number of lines in frame (in chunk mode always 1)";
    std::cerr << "\n                                 default: max";
    std::cerr << "\n    --width=<pixels>             line width in pixels; default: max";
    std::cerr << "\n    --offset-x=<pixels>          offset in pixels in the line; default: 0";
    std::cerr << "\n                                 todo: make sure it works on images with more than 1 channel";
    std::cerr << "\n    --offset-y=<pixels>          offset in lines in the frame; default: 0";
    std::cerr << "\n                                 todo: make sure it works on images with more than 1 channel";
    std::cerr << "\n    --pixel-format=[<format>]    pixel format; lower case accepted; currently supported formats:";
    std::cerr << "\n                                     gige cameras";
    std::cerr << "\n                                         Mono8: stdout output format: ub";
    std::cerr << "\n                                         RGB8: stdout output format: 3ub";
    std::cerr << "\n                                     usb cameras";
    std::cerr << "\n                                         Mono8: stdout output format: ub";
    std::cerr << "\n                                         Mono10: stdout output format: uw";
    std::cerr << "\n                                         Mono10p: stdout output format: uw (packed format; todo)";
    std::cerr << "\n                                         Mono12: stdout output format: uw";
    std::cerr << "\n                                         Mono12p: stdout output format: 3ub (apply unpack12 to get 16-bit padded image)";
    std::cerr << "\n                                     default: use the current camera setting";
    std::cerr << "\n                                     format names correspond to enum found in basler header files";
    std::cerr << "\n    --reverse-x                  horizontal mirror image";
    std::cerr << "\n    --reverse-y                  vertical mirror image";
    std::cerr << "\n    --binning-horizontal=<pixels>[,<mode>]";
    std::cerr << "\n                                 sets the number of adjacent horizontal pixels to bin";
    std::cerr << "\n                                 <mode>: sum, average (default: sum)";
    std::cerr << "\n                                 ROI will refer to binned columns";
    std::cerr << "\n    --binning-vertical=<pixels>[,<mode>]";
    std::cerr << "\n                                 sets the number of adjacent vertical pixels to bin";
    std::cerr << "\n                                 <mode>: sum, average (default: sum)";
    std::cerr << "\n                                 ROI will refer to binned rows";
    std::cerr << "\n";
    std::cerr << "\ncamera options";
    std::cerr << "\n    --frame-rate=[<fps>]         set frame rate; limited by exposure";
    std::cerr << "\n    --exposure=[<µs>]            exposure time; \"auto_once\" or \"auto_continuous\" to automatically set; default: " << default_exposure;
    std::cerr << "\n    --exposure-lower-limit=[<µs>]  lower limit for auto exposure; default: " << default_exposure_lower_limit;
    std::cerr << "\n    --exposure-upper-limit=[<µs>]  upper limit for auto exposure; default: " << default_exposure_upper_limit;
    std::cerr << "\n    --gain=[<num>]               gain; \"auto_once\" or \"auto_continuous\" to automatically set; default: " << default_gain;
    std::cerr << "\n                                 for USB cameras units are dB";
    std::cerr << "\n";
    std::cerr << "\nacquisition options";
    std::cerr << "\n    --discard                    discard frames, if cannot keep up;";
    std::cerr << "\n                                 same as --buffer=1 (which is not a great setting)";
    std::cerr << "\n    --buffer=[<buffers>]         maximum buffer size before discarding frames";
    std::cerr << "\n                                 default: unlimited";
    std::cerr << "\n    --frame-trigger=[<type>]     'line1', 'line2', 'line3', 'encoder'";
    std::cerr << "\n    --line-trigger=[<type>]      'line1', 'line2', 'line3', 'encoder'";
    std::cerr << "\n    --line-rate=[<num>]          line acquisition rate";
    std::cerr << "\n    --encoder-ticks=[<num>]      number of encoder ticks until the count resets";
    std::cerr << "\n                                 (reused for line number in frame in chunk mode)";
    std::cerr << "\n    --timeout=<seconds>          frame acquisition timeout; default " << default_timeout << "s";
    std::cerr << "\n";
    std::cerr << "\ntransport options";
    std::cerr << "\n    run --help --verbose for details on these options";
    std::cerr << "\n    --packet-size=[<bytes>]         camera mtu size, should not be larger";
    std::cerr << "\n                                    than your lan and network interface";
    std::cerr << "\n    --inter-packet-delay=[<ticks>]  transmission delay between packets";
    std::cerr << "\n    --num-cameras=<num>             auto-set inter-packet delay";
    std::cerr << "\n";
    std::cerr << "\nfilters";
    std::cerr << "\n    See \"basler-cat --help-filters\" for a list of supported filters";
    std::cerr << "\n    or \"cv-cat --help --verbose\" much more information on filtering.";
    std::cerr << "\n";
    std::cerr << "\nBy default basler-cat will connect to the first device it finds. To choose a";
    std::cerr << "\nspecific camera use the --address or --serial-number options. For GigE cameras ";
    std::cerr << "\n<address> is the device ip address, for USB cameras it is the USB address.";
    std::cerr << "\nDetected cameras along with their addresses and serial numbers are shown by";
    std::cerr << "\n$ basler-cat --list-cameras --verbose.";
    std::cerr << "\n";
    if( verbose )
    {
        std::cerr << "\ntransport options details";
        std::cerr << "\n";
        std::cerr << "\n    If there are multiple cameras on a single host (each run from a separate";
        std::cerr << "\n    instance of basler-cat) they need space on the network to transmit.";
        std::cerr << "\n    Otherwise basler-cat will report that \"The buffer was incompletely grabbed\"";
        std::cerr << "\n    and possible exit with a timeout.";
        std::cerr << "\n";
        std::cerr << "\n    As soon as we have acquired one packet of data it is transmitted, whilst the";
        std::cerr << "\n    next packet is acquired. So it looks like:";
        std::cerr << "\n";
        std::cerr << "\n    | acquire packet 1 | acquire packet 2 | acquire packet 3 | ...";
        std::cerr << "\n                       | tx p1      |.....| tx p2      |.....| tx p3 ...";
        std::cerr << "\n";
        std::cerr << "\n    If the space between one transmission ending and the next beginning is too";
        std::cerr << "\n    small for the other cameras to transmit there will be network congestion.";
        std::cerr << "\n";
        std::cerr << "\n    To fix this we introduce a delay between the end of one packet and the";
        std::cerr << "\n    start of the next. This is set by --inter-packet-delay and should be at";
        std::cerr << "\n    least as big as the transmit window (per extra camera). So we will have:";
        std::cerr << "\n";
        std::cerr << "\n    | acquire packet 1 | acquire packet 2 | wait | acquire packet 3 | wait |";
        std::cerr << "\n                       | tx p1      | delay      | tx p2      | delay      |";
        std::cerr << "\n";
        std::cerr << "\n    As you can see, the transmit delay forces packet acquisition to wait";
        std::cerr << "\n    (there is some buffering that this ignores but the principle is the same)";
        std::cerr << "\n    so it will reduce the achievable frame rate. However this frame rate will";
        std::cerr << "\n    probably be far higher than that obtainable without the forced delay.";
        std::cerr << "\n";
        std::cerr << "\n    You can just set --num-cameras and an appropriate inter-packet-delay will";
        std::cerr << "\n    be calculated and set, or you can set --inter-packet-delay explicitly.";
        std::cerr << "\n    You might find better stability by setting it a little higher than the";
        std::cerr << "\n    value calculated by --num-cameras. Use the --verbose option to see the";
        std::cerr << "\n    calculated value.";
        std::cerr << "\n";
        std::cerr << "\n    The formula used by --num-cameras is:";
        std::cerr << "\n        inter_packet_delay = ( num_cameras - 1 ) * ( packet_size + 18 )";
        std::cerr << "\n    where 18 represents the ethernet overhead (header plus checksum).";
        std::cerr << "\n";
        std::cerr << "\n    The formula assumes a one byte pixel format, so if you have something";
        std::cerr << "\n    different you will need to adjust appropriately (scale up).";
        std::cerr << "\n";
        std::cerr << "\nFor help on filters or image types try:";
        std::cerr << "\n    basler-cat --help-filters";
        std::cerr << "\n    basler-cat --help-types";
        std::cerr << "\n";
    }
    std::cerr << "\nExample:";
    std::cerr << "\n    $ basler-cat \"resize=0.5;timestamp;view;null\"";
    std::cerr << "\n    $ basler-cat --packet-size=9000 --pixel-format=Mono8 --frame-rate=120";
    std::cerr << "\n    $ basler-cat --serial-number=21991077 --height=480 --width=1280";
    std::cerr << "\n    $ basler-cat | cv-cat \"timestamp;view;null\"";

    std::cerr << "\n" << std::endl;
}

struct ChunkData
{
    boost::posix_time::ptime timestamp;
    comma::uint32 frames;
    comma::uint64 ticks; // timestamp; 1 tick = 8ns
    comma::uint32 line_trigger_ignored;
    comma::uint32 frame_trigger_ignored;
    comma::uint32 line_trigger_end_to_end;
    comma::uint32 frame_trigger;
    comma::uint32 frames_per_trigger;
};

struct Counters
{
    boost::posix_time::ptime adjusted_timestamp;
    comma::uint64 ticks; // number of 8ns ticks
    comma::uint64 line_count; // overall line count
    comma::uint32 line; // line number in one motor revolution

    Counters() : ticks( 0 ), line_count( 0 ), line( 0 ) {}
};

struct Header // quick and dirty
{
    snark::cv_mat::serialization::header header;
    Counters counters;

    Header() {}
    Header( const snark::cv_mat::serialization::header& header ) : header( header ) {}
};

static snark::cv_mat::serialization::options cv_mat_options;
static snark::cv_mat::serialization::header cv_mat_header;
static bool is_packed; // quick and dirty
static comma::csv::options csv;
static unsigned int timeout;
static Pylon::IChunkParser* parser = NULL;
static unsigned int encoder_ticks;

namespace comma { namespace visiting {

template <> struct traits< Counters >
{
    template < typename K, typename V >
    static void visit( const K&, Counters& t, V& v )
    {
        v.apply( "adjusted-t", t.adjusted_timestamp );
        v.apply( "ticks", t.ticks );
        v.apply( "line-count", t.line_count );
        v.apply( "line", t.line );
    }

    template < typename K, typename V >
    static void visit( const K&, const Counters& t, V& v )
    {
        v.apply( "adjusted-t", t.adjusted_timestamp );
        v.apply( "ticks", t.ticks );
        v.apply( "line-count", t.line_count );
        v.apply( "line", t.line );
    }
};

template <> struct traits< Header >
{
    template < typename K, typename V >
    static void visit( const K&, Header& t, V& v )
    {
        v.apply( "header", t.header );
        v.apply( "counters", t.counters );
    }

    template < typename K, typename V >
    static void visit( const K&, const Header& t, V& v )
    {
        v.apply( "header", t.header );
        v.apply( "counters", t.counters );
    }
};

} } // namespace comma { namespace visiting {

typedef std::pair< boost::posix_time::ptime, cv::Mat > pair_t;
typedef std::pair< ChunkData, cv::Mat > chunk_pair_t;

template < typename T >
static void set( boost::posix_time::ptime& timestamp, boost::posix_time::ptime& t, const Pylon::GrabResult&, T& ) { timestamp = t; }

template < typename T >
static void set( ChunkData& d, boost::posix_time::ptime& t, const Pylon::GrabResult& result, T& camera )
{
    parser->AttachBuffer( ( unsigned char* ) result.Buffer(), result.GetPayloadSize() );
    d.timestamp = t;
    d.frames = camera.ChunkFramecounter();
    d.ticks = camera.ChunkTimestamp();
    d.line_trigger_ignored = camera.ChunkLineTriggerIgnoredCounter();
    d.frame_trigger_ignored = camera.ChunkFrameTriggerIgnoredCounter();
    d.line_trigger_end_to_end = camera.ChunkLineTriggerEndToEndCounter();
    d.frame_trigger = camera.ChunkFrameTriggerCounter();
    d.frames_per_trigger = camera.ChunkFramesPerTriggerCounter();
    parser->DetachBuffer();
}

static void output_result_status( const Pylon::GrabResult& result )
{
    std::cerr << "basler-cat: " << result.GetErrorDescription()
              << " (0x" << std::hex << result.GetErrorCode() << std::dec << ")" << std::endl;
    std::cerr << "            status: " << ( result.Status() == Pylon::Idle ? "idle" :
                                             result.Status() == Pylon::Queued ? "queued" :
                                             result.Status() == Pylon::Grabbed ? "grabbed" :
                                             result.Status() == Pylon::Canceled ? "canceled" :
                                             result.Status() == Pylon::Failed ? "failed" : "unknown" ) << std::endl;
}

struct trigger_source
{
    typedef Basler_GigECameraParams::TriggerSourceEnums type_t;

    static type_t from_string( const std::string& source )
    {
        if( boost::algorithm::to_lower_copy( source ) == "line1" ) { return Basler_GigECameraParams::TriggerSource_Line1; }
        if( boost::algorithm::to_lower_copy( source ) == "line2" ) { return Basler_GigECameraParams::TriggerSource_Line2; }
        if( boost::algorithm::to_lower_copy( source ) == "line3" ) { return Basler_GigECameraParams::TriggerSource_Line3; }
        if( boost::algorithm::to_lower_copy( source ) == "encoder" ) { return Basler_GigECameraParams::TriggerSource_ShaftEncoderModuleOut; }
        COMMA_THROW( comma::exception, "trigger source \"" << source << "\" not implemented" );
    }

    static const char* to_string( type_t source )
    {
        switch( source )
        {
            case Basler_GigECameraParams::TriggerSource_Line1: return "line1";
            case Basler_GigECameraParams::TriggerSource_Line2: return "line2";
            case Basler_GigECameraParams::TriggerSource_Line3: return "line3";
            case Basler_GigECameraParams::TriggerSource_ShaftEncoderModuleOut: return "encoder";
            default: return "unknown";
        }
    }
};

template < typename T > struct exposure_mode;

template <> struct exposure_mode< Pylon::CBaslerGigECamera >
{
    typedef Basler_GigECameraParams::ExposureModeEnums type_t;

    static const char* to_string( type_t mode )
    {
        switch( mode )
        {
            case Basler_GigECameraParams::ExposureMode_Off: return "off";
            case Basler_GigECameraParams::ExposureMode_Timed: return "timed";
            case Basler_GigECameraParams::ExposureMode_TriggerWidth: return "trigger width";
            case Basler_GigECameraParams::ExposureMode_TriggerControlled: return "trigger controlled";
            default: return "unknown";
        }
    }
};

template <> struct exposure_mode< Pylon::CBaslerUsbCamera >
{
    typedef Basler_UsbCameraParams::ExposureModeEnums type_t;

    static const char* to_string( type_t mode )
    {
        switch( mode )
        {
            case Basler_UsbCameraParams::ExposureMode_Timed: return "timed";
            case Basler_UsbCameraParams::ExposureMode_TriggerWidth: return "trigger width";
            default: return "unknown";
        }
    }
};

template < typename T > struct exposure_auto;

template <> struct exposure_auto< Pylon::CBaslerGigECamera >
{
    typedef Basler_GigECameraParams::ExposureAutoEnums type_t;

    static type_t from_string( const std::string& auto_enum )
    {
        if( boost::algorithm::to_lower_copy( auto_enum ) == "auto_once" ) { return Basler_GigECameraParams::ExposureAuto_Once; }
        if( boost::algorithm::to_lower_copy( auto_enum ) == "auto_continuous" ) { return Basler_GigECameraParams::ExposureAuto_Continuous; }
        unsigned int exposure_time;
        if( boost::conversion::try_lexical_convert< unsigned int >( auto_enum, exposure_time )) { return Basler_GigECameraParams::ExposureAuto_Off; }
        COMMA_THROW( comma::exception, "exposure \"" << auto_enum << "\" not implemented" );
    }

    static const char* to_string( type_t auto_enum )
    {
        switch( auto_enum )
        {
            case Basler_GigECameraParams::ExposureAuto_Off: return "off";
            case Basler_GigECameraParams::ExposureAuto_Once: return "once";
            case Basler_GigECameraParams::ExposureAuto_Continuous: return "continuous";
            default: return "unknown";
        }
    }
};

template <> struct exposure_auto< Pylon::CBaslerUsbCamera >
{
    typedef Basler_UsbCameraParams::ExposureAutoEnums type_t;

    static type_t from_string( const std::string& auto_enum )
    {
        if( boost::algorithm::to_lower_copy( auto_enum ) == "auto_once" ) { return Basler_UsbCameraParams::ExposureAuto_Once; }
        if( boost::algorithm::to_lower_copy( auto_enum ) == "auto_continuous" ) { return Basler_UsbCameraParams::ExposureAuto_Continuous; }
        unsigned int exposure_time;
        if( boost::conversion::try_lexical_convert< unsigned int >( auto_enum, exposure_time )) { return Basler_UsbCameraParams::ExposureAuto_Off; }
        COMMA_THROW( comma::exception, "exposure \"" << auto_enum << "\" not implemented" );
    }

    static const char* to_string( type_t auto_enum )
    {
        switch( auto_enum )
        {
            case Basler_UsbCameraParams::ExposureAuto_Off: return "off";
            case Basler_UsbCameraParams::ExposureAuto_Once: return "once";
            case Basler_UsbCameraParams::ExposureAuto_Continuous: return "continuous";
            default: return "unknown";
        }
    }
};

template < typename T > struct gain_auto;

template <> struct gain_auto< Pylon::CBaslerGigECamera >
{
    typedef Basler_GigECameraParams::GainAutoEnums type_t;

    static type_t from_string( const std::string& auto_enum )
    {
        if( boost::algorithm::to_lower_copy( auto_enum ) == "auto_once" ) { return Basler_GigECameraParams::GainAuto_Once; }
        if( boost::algorithm::to_lower_copy( auto_enum ) == "auto_continuous" ) { return Basler_GigECameraParams::GainAuto_Continuous; }
        unsigned int gain;
        if( boost::conversion::try_lexical_convert< unsigned int >( auto_enum, gain )) { return Basler_GigECameraParams::GainAuto_Off; }
        COMMA_THROW( comma::exception, "gain \"" << auto_enum << "\" not implemented" );
    }

    static const char* to_string( type_t auto_enum )
    {
        switch( auto_enum )
        {
            case Basler_GigECameraParams::GainAuto_Off: return "off";
            case Basler_GigECameraParams::GainAuto_Once: return "once";
            case Basler_GigECameraParams::GainAuto_Continuous: return "continuous";
            default: return "unknown";
        }
    }
};

template <> struct gain_auto< Pylon::CBaslerUsbCamera >
{
    typedef Basler_UsbCameraParams::GainAutoEnums type_t;

    static type_t from_string( const std::string& auto_enum )
    {
        if( boost::algorithm::to_lower_copy( auto_enum ) == "auto_once" ) { return Basler_UsbCameraParams::GainAuto_Once; }
        if( boost::algorithm::to_lower_copy( auto_enum ) == "auto_continuous" ) { return Basler_UsbCameraParams::GainAuto_Continuous; }
        unsigned int gain;
        if( boost::conversion::try_lexical_convert< unsigned int >( auto_enum, gain )) { return Basler_UsbCameraParams::GainAuto_Off; }
        COMMA_THROW( comma::exception, "gain \"" << auto_enum << "\" not implemented" );
    }

    static const char* to_string( type_t auto_enum )
    {
        switch( auto_enum )
        {
            case Basler_UsbCameraParams::GainAuto_Off: return "off";
            case Basler_UsbCameraParams::GainAuto_Once: return "once";
            case Basler_UsbCameraParams::GainAuto_Continuous: return "continuous";
            default: return "unknown";
        }
    }
};

template < typename T > struct pixel_format;

template <> struct pixel_format< Pylon::CBaslerUsbCamera > // todo: support more formats
{
    typedef Basler_UsbCameraParams::PixelFormatEnums type_t;
    
    static type_t from_string( const std::string& name )
    {
        if( boost::algorithm::to_lower_copy( name ) == "mono8" ) { return Basler_UsbCameraParams::PixelFormat_Mono8; }
        if( boost::algorithm::to_lower_copy( name ) == "mono10" ) { return Basler_UsbCameraParams::PixelFormat_Mono10; }
        if( boost::algorithm::to_lower_copy( name ) == "mono10p" ) { return Basler_UsbCameraParams::PixelFormat_Mono10p; }
        if( boost::algorithm::to_lower_copy( name ) == "mono12" ) { return Basler_UsbCameraParams::PixelFormat_Mono12; }
        if( boost::algorithm::to_lower_copy( name ) == "mono12p" ) { return Basler_UsbCameraParams::PixelFormat_Mono12p; }
        COMMA_THROW( comma::exception, "pixel format \"" << name << "\" not implemented" );
    }
    
    static const char* to_string( type_t format )
    {
        switch( format )
        {
            case Basler_UsbCameraParams::PixelFormat_Mono8: return "Mono8";
            case Basler_UsbCameraParams::PixelFormat_Mono10: return "Mono10";
            case Basler_UsbCameraParams::PixelFormat_Mono10p: return "Mono10p";
            case Basler_UsbCameraParams::PixelFormat_Mono12: return "Mono12";
            case Basler_UsbCameraParams::PixelFormat_Mono12p: return "Mono12p";
            default: COMMA_THROW( comma::exception, "pixel format " << format << " not implemented" );
        }
    }
    
    static unsigned int channels( type_t format )
    {
        switch( format )
        {
            case Basler_UsbCameraParams::PixelFormat_Mono8: return 1;
            case Basler_UsbCameraParams::PixelFormat_Mono10: return 1;
            case Basler_UsbCameraParams::PixelFormat_Mono10p: return 1;
            case Basler_UsbCameraParams::PixelFormat_Mono12: return 1;
            case Basler_UsbCameraParams::PixelFormat_Mono12p: return 1;
            default: COMMA_THROW( comma::exception, "pixel format " << format << " not implemented" );
        }
    }
    
    static bool is_packed( type_t format )
    {
        switch( format )
        {
            case Basler_UsbCameraParams::PixelFormat_Mono8: return false;
            case Basler_UsbCameraParams::PixelFormat_Mono10: return false;
            case Basler_UsbCameraParams::PixelFormat_Mono12: return false;
            case Basler_UsbCameraParams::PixelFormat_Mono12p: return true;
            default: COMMA_THROW( comma::exception, "pixel format " << format << " not implemented" );
        }
    }
    
    static const char* to_image_type_string( type_t format )
    {
        switch( format )
        {
            case Basler_UsbCameraParams::PixelFormat_Mono8: return "ub";
            case Basler_UsbCameraParams::PixelFormat_Mono10: return "uw";
            //case Basler_UsbCameraParams::PixelFormat_Mono10p: return "uw";
            case Basler_UsbCameraParams::PixelFormat_Mono12: return "uw";
            case Basler_UsbCameraParams::PixelFormat_Mono12p: return "ub"; // quick and dirty
            default: COMMA_THROW( comma::exception, "pixel format " << format << " not implemented" );
        }
    }
};

template <> struct pixel_format< Pylon::CBaslerGigECamera > // todo: support more formats
{
    typedef Basler_GigECameraParams::PixelFormatEnums type_t;
    
    static type_t from_string( const std::string& name )
    {
        if( boost::algorithm::to_lower_copy( name ) == "mono8" ) { return Basler_GigECameraParams::PixelFormat_Mono8; }
        if( boost::algorithm::to_lower_copy( name ) == "rgb8packed" ) { return Basler_GigECameraParams::PixelFormat_RGB8Packed; }
        COMMA_THROW( comma::exception, "pixel format \"" << name << "\" not implemented" );
    }
    
    static const char* to_string( type_t format )
    {
        switch( format )
        {
            case Basler_GigECameraParams::PixelFormat_Mono8: return "Mono8";
            case Basler_GigECameraParams::PixelFormat_RGB8Packed: return "RGB8Packed";
            default: COMMA_THROW( comma::exception, "pixel format " << format << " not implemented" );
        }
    }
    
    static unsigned int channels( type_t format )
    {
        switch( format )
        {
            case Basler_GigECameraParams::PixelFormat_Mono8: return 1;
            case Basler_GigECameraParams::PixelFormat_RGB8Packed: return 3;
            default: COMMA_THROW( comma::exception, "pixel format " << format << " not implemented" );
        }
    }
    
    static bool is_packed( type_t format )
    {
        switch( format )
        {
            case Basler_GigECameraParams::PixelFormat_Mono8: return false;
            case Basler_GigECameraParams::PixelFormat_RGB8Packed: return false;
            default: COMMA_THROW( comma::exception, "pixel format " << format << " not implemented" );
        }
    }
    
    static const char* to_image_type_string( type_t format )
    {
        switch( format )
        {
            case Basler_GigECameraParams::PixelFormat_Mono8: return "ub";
            case Basler_GigECameraParams::PixelFormat_RGB8Packed: return "3ub";
            default: COMMA_THROW( comma::exception, "pixel format " << format << " not implemented" );
        }
    }
};

template< typename T, typename P >
static void load_default_settings( T& camera, P user_set_selector_default )
{
    camera.UserSetSelector.SetValue( user_set_selector_default );
    camera.UserSetLoad();
}
static void load_default_settings( Pylon::CBaslerGigECamera& camera ) { load_default_settings( camera, Basler_GigECameraParams::UserSetSelector_Default ); }
static void load_default_settings( Pylon::CBaslerUsbCamera& camera ) { load_default_settings( camera, Basler_UsbCameraParams::UserSetSelector_Default ); }

template < typename T, typename P >
static void set_pixel_format( T& camera, P type )
{
    if( camera.PixelFormat() == type ) { return; }
    GenApi::NodeList_t entries;
    camera.PixelFormat.GetEntries( entries );
    bool supported = false;
    comma::verbose << "supported pixel format(s): ";
    for( std::size_t i = 0; i < entries.size(); ++i )
    {
        GenApi::INode* node = entries[i]; // bloody voodoo
        if( !IsAvailable( node->GetAccessMode() ) ) { continue; }
        GenApi::IEnumEntry* e = dynamic_cast< GenApi::IEnumEntry* >( node );
        if( comma::verbose ) { std::cerr << ( i > 0 ? ", " : "" ) << e->GetSymbolic(); }
        supported = supported || pixel_format< T >::to_string( type ) != e->GetSymbolic().c_str();
    }
    if( comma::verbose ) { std::cerr << std::endl; }
    if( !supported ) { COMMA_THROW( comma::exception, "pixel format \"" << pixel_format< T >::to_string( type ) << "\" is not supported by the camera" ); }
    comma::verbose << "setting pixel format..." << std::endl;
    // the voodoo theory is that in the continuous mode the camera settings
    // cannot be changed, while the camera is acquiring a frame. now, you may think:
    // well, just stop acquisition! see below: somehow, stopping acquisition does not
    // work; therefore (even in the native basler viewer) you need to try to set the
    // pixel format between the frames, just trying it until you succeed...
    // and then you try again...
    // but even that does not seem to work... well, power-cycling helps...
    static const unsigned int retries = 100; // quick and dirty: arbitrary
    for( unsigned int i = 0; i < retries; ++i )
    {
        camera.PixelFormat = type;
        boost::thread::sleep( boost::posix_time::microsec_clock::universal_time() + boost::posix_time::milliseconds( 10 ) );
    }
    if( camera.PixelFormat() != type ) { COMMA_THROW( comma::exception, "failed to set pixel format after " << retries << " attempts; try again or power-cycle the camera" ); }
    comma::verbose << "pixel format set to " << pixel_format< T >::to_string( type ) << std::endl;
    return;
}

template < typename T >
static void set_pixel_format( T& camera, const comma::command_line_options& options )
{
    std::string pixel_format_string = options.value< std::string >( "--pixel-format", "" );
    typename ::pixel_format< T >::type_t pixel_format = pixel_format_string.empty() ? camera.PixelFormat() : ::pixel_format< T >::from_string( pixel_format_string );
    set_pixel_format( camera, pixel_format );
    cv_mat_options.type = ::pixel_format< T >::to_image_type_string( pixel_format );
    is_packed = ::pixel_format< T >::is_packed( pixel_format );
}

static std::string get_address( const Pylon::CDeviceInfo& device_info )
{
    std::string full_name = device_info.GetFullName().c_str();
    std::string device_class = device_info.GetDeviceClass().c_str();
    if( device_class == "BaslerGigE" )
    {
        // It would be nice to use Pylon::CBaslerGigEDeviceInfo::GetAddress()
        // but casting to that class appears not to work
        boost::regex address_regex( ".*#([0-9.]+):.*", boost::regex::extended );
        boost::smatch match;
        if( boost::regex_match( full_name, match, address_regex )) { return match[1]; }
    }
    else if( device_class == "BaslerUsb" ) { return full_name; }
    return "";
}

static void list_cameras()
{
    Pylon::CTlFactory& factory = Pylon::CTlFactory::GetInstance();
    Pylon::DeviceInfoList_t devices;
    factory.EnumerateDevices( devices );
    Pylon::DeviceInfoList_t::const_iterator it;
    for( it = devices.begin(); it != devices.end(); ++it )
    {
        if( comma::verbose )
        {
            std::cerr << "\nVendor:     " << it->GetVendorName();
            std::cerr << "\nModel:      " << it->GetModelName();
            std::cerr << "\nVersion:    " << it->GetDeviceVersion();
            std::cerr << "\nType:       " << it->GetDeviceClass();
            std::cerr << "\nSerial no.: " << it->GetSerialNumber();
            std::cerr << "\nAddress:    " << get_address( *it );
            std::cerr << "\nFull name:  " << it->GetFullName();
            std::cerr << std::endl;
        }
        else { std::cerr << it->GetFullName() << std::endl; }
    }
    if( comma::verbose && !devices.empty() ) { std::cerr << std::endl; }
}

static bool is_ip_address( std::string str )
{
    boost::regex ip_regex( "[0-9]+\\.[0-9]+\\.[0-9]+\\.[0-9]+" ); // somewhat naive
    return boost::regex_match( str, ip_regex );
}

static Pylon::IPylonDevice* create_device( const std::string& address, const std::string& serial_number )
{
    Pylon::CTlFactory& factory = Pylon::CTlFactory::GetInstance();
    if( !serial_number.empty() )
    {
        Pylon::CDeviceInfo device_info;
        device_info.SetSerialNumber( serial_number.c_str() );
        return factory.CreateDevice( device_info );
    }
    else if( !address.empty() )
    {
        if( is_ip_address( address ))
        {
            Pylon::CBaslerGigEDeviceInfo gige_device_info;
            gige_device_info.SetIpAddress( address.c_str() );
            return factory.CreateDevice( gige_device_info );
        }
        else
        {
            Pylon::CDeviceInfo device_info;
            device_info.SetFullName( address.c_str() );
            return factory.CreateDevice( device_info );
        }
    }
    else
    {
        Pylon::DeviceInfoList_t devices;
        factory.EnumerateDevices( devices );
        std::cerr << "basler-cat: ";
        switch( devices.size() )
        {
            case 0:
                std::cerr << "no camera found" << std::endl;
                return NULL;
            case 1:
                std::cerr << "found 1 device: " << devices[0].GetFullName() << std::endl;
                break;
            default:
                std::cerr << "will connect to the first of " << devices.size() << " found devices:" << std::endl;
                Pylon::DeviceInfoList_t::const_iterator it;
                for( it = devices.begin(); it != devices.end(); ++it ) { std::cerr << "    " << it->GetFullName() << std::endl; }
        }
        return factory.CreateDevice( devices[0] );
    }
}

static bool chunk_mode = false;
static std::string filters;

static bool configure_trigger( Pylon::CBaslerUsbCamera& camera, const comma::command_line_options& options ) { return true; }

static bool configure_trigger( Pylon::CBaslerGigECamera& camera, const comma::command_line_options& options )
{
    GenApi::IEnumEntry* acquisition_start = camera.TriggerSelector.GetEntry( Basler_GigECameraParams::TriggerSelector_AcquisitionStart );
    GenApi::IEnumEntry* trigger_source_software=camera.TriggerSource.GetEntry(Basler_GigECameraParams::TriggerSource_Software);
    std::string frame_trigger = options.value< std::string >( "--frame-trigger", "" );
    
//     comma::verbose<<"IsAvailable TriggerSelector :"<<GenApi::IsAvailable(camera.TriggerSelector)<<std::endl;
//     comma::verbose<<"IsAvailable TriggerMode :"<<GenApi::IsAvailable(camera.TriggerMode)<<std::endl;
//     comma::verbose<<"IsAvailable TriggerSource :"<<GenApi::IsAvailable(camera.TriggerSource)<<std::endl;
//     comma::verbose<<"IsWritable TriggerSource :"<<GenApi::IsWritable(camera.TriggerSource)<<std::endl;
//     comma::verbose<<"IsAvailable trigger_source_software "<<GenApi::IsAvailable(trigger_source_software)<<std::endl;
//     comma::verbose<<"IsWritable trigger_source_software "<<GenApi::IsWritable(trigger_source_software)<<std::endl;
//     if(GenApi::IsAvailable(camera.TriggerSelector)) comma::verbose<<"camera.TriggerSelector "<<camera.TriggerSelector()<<std::endl;
//     if(GenApi::IsAvailable(camera.TriggerMode)) comma::verbose<<"camera.TriggerMode "<<camera.TriggerMode()<<std::endl;
//     if(GenApi::IsAvailable(camera.TriggerSource)) comma::verbose<<"camera.TriggerSource "<<camera.TriggerSource()<<std::endl;
    
    if( GenApi::IsWritable(trigger_source_software) && acquisition_start && GenApi::IsAvailable( acquisition_start ) )
    {
        camera.TriggerSelector = Basler_GigECameraParams::TriggerSelector_AcquisitionStart;
        camera.TriggerMode = ( frame_trigger.empty() ? Basler_GigECameraParams::TriggerMode_Off : Basler_GigECameraParams::TriggerMode_On );
        camera.TriggerSource = Basler_GigECameraParams::TriggerSource_Software;
    }
    GenApi::IEnumEntry* frame_start = camera.TriggerSelector.GetEntry( Basler_GigECameraParams::TriggerSelector_FrameStart );
    if( frame_start && GenApi::IsAvailable( frame_start ) )
    {
        //if( frame_trigger.empty() ) { frame_trigger = line_trigger; }
        if( frame_trigger.empty() )
        {
            camera.TriggerSelector = Basler_GigECameraParams::TriggerSelector_FrameStart;
            camera.TriggerMode = Basler_GigECameraParams::TriggerMode_Off;
        }
        else
        {
            camera.TriggerSelector = Basler_GigECameraParams::TriggerSelector_FrameStart;
            camera.TriggerMode = Basler_GigECameraParams::TriggerMode_On;
            camera.TriggerSource = trigger_source::from_string( frame_trigger );
            camera.TriggerActivation = Basler_GigECameraParams::TriggerActivation_RisingEdge;
            camera.TriggerSelector = Basler_GigECameraParams::TriggerSelector_LineStart;
            camera.TriggerMode = Basler_GigECameraParams::TriggerMode_On;
            camera.TriggerActivation = Basler_GigECameraParams::TriggerActivation_RisingEdge;

            if( camera.TriggerSource() == Basler_GigECameraParams::TriggerSource_ShaftEncoderModuleOut )
            {
                // todo: make configurable
                camera.ShaftEncoderModuleLineSelector = Basler_GigECameraParams::ShaftEncoderModuleLineSelector_PhaseA;
                camera.ShaftEncoderModuleLineSource = Basler_GigECameraParams::ShaftEncoderModuleLineSource_Line1;
                camera.ShaftEncoderModuleLineSelector = Basler_GigECameraParams::ShaftEncoderModuleLineSelector_PhaseB;
                camera.ShaftEncoderModuleLineSource = Basler_GigECameraParams::ShaftEncoderModuleLineSource_Line2;
                camera.ShaftEncoderModuleCounterMode = Basler_GigECameraParams::ShaftEncoderModuleCounterMode_FollowDirection;
                camera.ShaftEncoderModuleMode = Basler_GigECameraParams::ShaftEncoderModuleMode_ForwardOnly;
                camera.ShaftEncoderModuleCounterMax = encoder_ticks - 1;
                /// @todo compensate for mechanical jitter, if needed
                ///       see Runner_Users_manual.pdf, 8.3, Case 2
                camera.ShaftEncoderModuleReverseCounterMax = 0;
                camera.ShaftEncoderModuleCounterReset();
                camera.ShaftEncoderModuleReverseCounterReset();
            }
        }
    }
    GenApi::IEnumEntry* line_start = camera.TriggerSelector.GetEntry( Basler_GigECameraParams::TriggerSelector_LineStart );
    if( line_start && GenApi::IsAvailable( line_start ) )
    {
        std::string line_trigger = options.value< std::string >( "--line-trigger", "" );
        if( line_trigger.empty() )
        {
            camera.TriggerSelector = Basler_GigECameraParams::TriggerSelector_LineStart;
            camera.TriggerMode = Basler_GigECameraParams::TriggerMode_Off;
        }
        else
        {
            camera.TriggerSelector = Basler_GigECameraParams::TriggerSelector_LineStart;
            camera.TriggerMode = Basler_GigECameraParams::TriggerMode_On;
            camera.TriggerSource = trigger_source::from_string( line_trigger );
            camera.TriggerActivation = Basler_GigECameraParams::TriggerActivation_RisingEdge;
            camera.TriggerSelector = Basler_GigECameraParams::TriggerSelector_LineStart;
            camera.TriggerMode = Basler_GigECameraParams::TriggerMode_On;
            camera.TriggerActivation = Basler_GigECameraParams::TriggerActivation_RisingEdge;
        }
    }
    return true;
}

static void configure_chunk_mode( Pylon::CBaslerUsbCamera& camera ) { COMMA_THROW( comma::exception, "chunk mode not supported for USB cameras" ); }

static void configure_chunk_mode( Pylon::CBaslerGigECamera& camera )
{
    camera.ChunkSelector = Basler_GigECameraParams::ChunkSelector_Framecounter;
    camera.ChunkEnable = true;
    camera.ChunkSelector = Basler_GigECameraParams::ChunkSelector_Timestamp;
    camera.ChunkEnable = true;
    camera.ChunkSelector = Basler_GigECameraParams::ChunkSelector_LineTriggerIgnoredCounter;
    camera.ChunkEnable = true;
    camera.ChunkSelector = Basler_GigECameraParams::ChunkSelector_FrameTriggerIgnoredCounter;
    camera.ChunkEnable = true;
    camera.ChunkSelector = Basler_GigECameraParams::ChunkSelector_LineTriggerEndToEndCounter;
    camera.ChunkEnable = true;
    camera.ChunkSelector = Basler_GigECameraParams::ChunkSelector_FrameTriggerCounter;
    camera.ChunkEnable = true;
    camera.ChunkSelector = Basler_GigECameraParams::ChunkSelector_FramesPerTriggerCounter;
    camera.ChunkEnable = true;
}

static bool exposure_is_auto( const Pylon::CBaslerGigECamera& camera ) { return( camera.ExposureAuto() != Basler_GigECameraParams::ExposureAuto_Off ); }
static double get_exposure_time( const Pylon::CBaslerGigECamera& camera ) { return camera.ExposureTimeAbs(); }
static double get_exposure_lower_limit( const Pylon::CBaslerGigECamera& camera ) { return camera.AutoExposureTimeAbsLowerLimit(); }
static double get_exposure_upper_limit( const Pylon::CBaslerGigECamera& camera ) { return camera.AutoExposureTimeAbsUpperLimit(); }
static bool gain_is_auto( const Pylon::CBaslerGigECamera& camera ) { return( camera.GainAuto() != Basler_GigECameraParams::GainAuto_Off ); }
static double get_gain( const Pylon::CBaslerGigECamera& camera ) { return camera.GainRaw(); }
static std::string gain_units( const Pylon::CBaslerGigECamera& ) { return ""; }
static double get_frame_rate_set( const Pylon::CBaslerGigECamera& camera ) { return camera.AcquisitionFrameRateAbs(); }
static double get_frame_rate_max( const Pylon::CBaslerGigECamera& camera ) { return camera.ResultingFrameRateAbs(); }

static bool exposure_is_auto( const Pylon::CBaslerUsbCamera& camera ) { return( camera.ExposureAuto() != Basler_UsbCameraParams::ExposureAuto_Off ); }
static double get_exposure_time( const Pylon::CBaslerUsbCamera& camera ) { return camera.ExposureTime(); }
static double get_exposure_lower_limit( const Pylon::CBaslerUsbCamera& camera ) { return camera.AutoExposureTimeLowerLimit(); }
static double get_exposure_upper_limit( const Pylon::CBaslerUsbCamera& camera ) { return camera.AutoExposureTimeUpperLimit(); }
static bool gain_is_auto( const Pylon::CBaslerUsbCamera& camera ) { return( camera.GainAuto() != Basler_UsbCameraParams::GainAuto_Off ); }
static double get_gain( const Pylon::CBaslerUsbCamera& camera ) { return camera.Gain(); }
static std::string gain_units( const Pylon::CBaslerUsbCamera& ) { return "dB"; }
static double get_frame_rate_set( const Pylon::CBaslerUsbCamera& camera ) { return camera.AcquisitionFrameRate(); }
static double get_frame_rate_max( const Pylon::CBaslerUsbCamera& camera ) { return camera.ResultingFrameRate(); }

static void set_frame_rate( Pylon::CBaslerGigECamera& camera, double frame_rate )
{
    camera.AcquisitionFrameRateEnable = true;
    camera.AcquisitionFrameRateAbs = frame_rate;
}

static void set_frame_rate( Pylon::CBaslerUsbCamera& camera, double frame_rate )
{
    camera.AcquisitionFrameRateEnable = true;
    camera.AcquisitionFrameRate = frame_rate;
}

static void set_exposure_mode_timed( Pylon::CBaslerGigECamera& camera )
{
    camera.ExposureMode = Basler_GigECameraParams::ExposureMode_Timed;
}

static void set_exposure_mode_timed( Pylon::CBaslerUsbCamera& camera )
{
    camera.ExposureMode = Basler_UsbCameraParams::ExposureMode_Timed;
}

static void set_exposure_time( Pylon::CBaslerGigECamera& camera, unsigned int exposure_time )
{
    camera.ExposureTimeAbs = exposure_time;
}

static void set_exposure_time( Pylon::CBaslerUsbCamera& camera, unsigned int exposure_time )
{
    camera.ExposureTime = exposure_time;
}

static void set_exposure_limits( Pylon::CBaslerGigECamera& camera, float lower_limit, float upper_limit )
{
    camera.AutoExposureTimeAbsLowerLimit = lower_limit;
    camera.AutoExposureTimeAbsUpperLimit = upper_limit;
}

static void set_exposure_limits( Pylon::CBaslerUsbCamera& camera, float lower_limit, float upper_limit )
{
    camera.AutoExposureTimeLowerLimit = lower_limit;
    camera.AutoExposureTimeUpperLimit = upper_limit;
}

template< typename T >
static void set_exposure( T& camera, const comma::command_line_options& options )
{
    set_exposure_mode_timed( camera );

    std::string exposure = options.value( "--exposure", default_exposure );
    if(GenApi::IsAvailable(camera.ExposureAuto))
    {
        camera.ExposureAuto = exposure_auto< T >::from_string( exposure );
        if( exposure_is_auto( camera ))
        {
            set_exposure_limits( camera
                            , options.value< float >( "--exposure-lower-limit", default_exposure_lower_limit )
                            , options.value< float >( "--exposure-upper-limit", default_exposure_upper_limit ));
        }
        else { set_exposure_time( camera, boost::lexical_cast< unsigned int >( exposure )); }
    }
    else
    {
        comma::verbose<<"ExposureAuto not available"<<std::endl;
        set_exposure_time( camera, boost::lexical_cast< unsigned int >( exposure ));
    }
}

static void set_gain_selector( Pylon::CBaslerGigECamera& camera )
{
    camera.GainSelector = Basler_GigECameraParams::GainSelector_All;
}

static void set_gain_selector( Pylon::CBaslerUsbCamera& camera )
{
    camera.GainSelector = Basler_UsbCameraParams::GainSelector_All;
}

static void set_gain_value( Pylon::CBaslerGigECamera& camera, unsigned int gain )
{
    // the docs say that GainAbs (in dB) is available, but the camera disagrees
    camera.GainRaw = gain;
}

static void set_gain_value( Pylon::CBaslerUsbCamera& camera, unsigned int gain )
{
    camera.Gain = gain;
}

template< typename T >
static void set_gain( T& camera, const comma::command_line_options& options )
{
    set_gain_selector( camera );
    std::string gain = options.value( "--gain", default_gain );
    if(GenApi::IsAvailable(camera.GainAuto))
    {
        camera.GainAuto = gain_auto< T >::from_string( gain );

        if( !gain_is_auto( camera )) { set_gain_value( camera, boost::lexical_cast< unsigned int >( gain )); }
    }
    else
    {
        comma::verbose<<"GainAuto not available"<<std::endl;
        set_gain_value( camera, boost::lexical_cast< unsigned int >( gain ));
    }
}

static void set_line_rate( Pylon::CBaslerGigECamera& camera, unsigned int line_rate ) { camera.AcquisitionLineRateAbs = line_rate; }
static void set_line_rate( Pylon::CBaslerUsbCamera& camera, unsigned int ) { COMMA_THROW( comma::exception, "--line-rate not supported for USB cameras" ); }
static void set_socket_buffer_size( Pylon::CBaslerGigECamera::StreamGrabber_t& grabber, unsigned int socket_buffer_size ) { grabber.SocketBufferSize = socket_buffer_size; }
static void set_socket_buffer_size( Pylon::CBaslerUsbCamera::StreamGrabber_t&, unsigned int ) {}

static void set_test_image( Pylon::CBaslerGigECamera& camera, unsigned int test_image_num )
{
    switch( test_image_num )
    {
        case 0: camera.TestImageSelector = Basler_GigECameraParams::TestImageSelector_Off; break;
        case 1: camera.TestImageSelector = Basler_GigECameraParams::TestImageSelector_Testimage1; break;
        case 2: camera.TestImageSelector = Basler_GigECameraParams::TestImageSelector_Testimage2; break;
        case 3: camera.TestImageSelector = Basler_GigECameraParams::TestImageSelector_Testimage3; break;
        case 4: camera.TestImageSelector = Basler_GigECameraParams::TestImageSelector_Testimage4; break;
        case 5: camera.TestImageSelector = Basler_GigECameraParams::TestImageSelector_Testimage5; break;
        case 6: camera.TestImageSelector = Basler_GigECameraParams::TestImageSelector_Testimage6; break;
        default: COMMA_THROW( comma::exception, "test image " << test_image_num << " is not supported. Choose a number from 1 to 6" );
    }
}

static void set_test_image( Pylon::CBaslerUsbCamera& camera, unsigned int test_image_num )
{
    switch( test_image_num )
    {
        case 0: camera.TestImageSelector = Basler_UsbCameraParams::TestImageSelector_Off; break;
        case 1: camera.TestImageSelector = Basler_UsbCameraParams::TestImageSelector_Testimage1; break;
        case 2: camera.TestImageSelector = Basler_UsbCameraParams::TestImageSelector_Testimage2; break;
        case 3: camera.TestImageSelector = Basler_UsbCameraParams::TestImageSelector_Testimage3; break;
        case 4: camera.TestImageSelector = Basler_UsbCameraParams::TestImageSelector_Testimage4; break;
        case 5: camera.TestImageSelector = Basler_UsbCameraParams::TestImageSelector_Testimage5; break;
        case 6: camera.TestImageSelector = Basler_UsbCameraParams::TestImageSelector_Testimage6; break;
        default: COMMA_THROW( comma::exception, "test image " << test_image_num << " is not supported. Choose a number from 1 to 6" );
    }
}

template< typename T, typename P >
static void set_acquisition_mode( T& camera, P acquisition_mode )
{
    if( camera.AcquisitionMode() == acquisition_mode ) { return; }
    if( GenApi::IsWritable( camera.AcquisitionMode ) ) { camera.AcquisitionMode = acquisition_mode; }
    else { COMMA_THROW( comma::exception, "unable to set acquisition mode, parameter is not writable" ); }
}

static void set_continuous_acquisition_mode( Pylon::CBaslerGigECamera& camera ) { set_acquisition_mode( camera, Basler_GigECameraParams::AcquisitionMode_Continuous ); }
static void set_continuous_acquisition_mode( Pylon::CBaslerUsbCamera& camera ) { set_acquisition_mode( camera, Basler_UsbCameraParams::AcquisitionMode_Continuous ); }

template< typename T, typename P > static void set_binning_horizontal_mode( T& camera, P mode ) { camera.BinningHorizontalMode = mode; }
static void set_binning_horizontal_mode_sum( Pylon::CBaslerGigECamera& camera ) { set_binning_horizontal_mode( camera, Basler_GigECameraParams::BinningHorizontalMode_Sum ); }
static void set_binning_horizontal_mode_average( Pylon::CBaslerGigECamera& camera ) { set_binning_horizontal_mode( camera, Basler_GigECameraParams::BinningHorizontalMode_Average ); }
static void set_binning_horizontal_mode_sum( Pylon::CBaslerUsbCamera& camera ) { set_binning_horizontal_mode( camera, Basler_UsbCameraParams::BinningHorizontalMode_Sum ); }
static void set_binning_horizontal_mode_average( Pylon::CBaslerUsbCamera& camera ) { set_binning_horizontal_mode( camera, Basler_UsbCameraParams::BinningHorizontalMode_Average ); }

template< typename T, typename P > static void set_binning_vertical_mode( T& camera, P mode ) { camera.BinningVerticalMode = mode; }
static void set_binning_vertical_mode_sum( Pylon::CBaslerGigECamera& camera ) { set_binning_vertical_mode( camera, Basler_GigECameraParams::BinningVerticalMode_Sum ); }
static void set_binning_vertical_mode_average( Pylon::CBaslerGigECamera& camera ) { set_binning_vertical_mode( camera, Basler_GigECameraParams::BinningVerticalMode_Average ); }
static void set_binning_vertical_mode_sum( Pylon::CBaslerUsbCamera& camera ) { set_binning_vertical_mode( camera, Basler_UsbCameraParams::BinningVerticalMode_Sum ); }
static void set_binning_vertical_mode_average( Pylon::CBaslerUsbCamera& camera ) { set_binning_vertical_mode( camera, Basler_UsbCameraParams::BinningVerticalMode_Average ); }

static void set_transport_options( Pylon::CBaslerGigECamera& camera, const comma::command_line_options& options )
{
    camera.GevStreamChannelSelector = Basler_GigECameraParams::GevStreamChannelSelector_StreamChannel0;

    unsigned int initial_packet_size = camera.GevSCPSPacketSize();
    unsigned int packet_size = options.value< unsigned int >( "--packet-size", initial_packet_size );
    if( packet_size != initial_packet_size ) { camera.GevSCPSPacketSize = packet_size;  }

    unsigned int initial_inter_packet_delay = camera.GevSCPD();
    unsigned int num_cameras = options.value< unsigned int >( "--num-cameras", 1 );
    // If there are other cameras then we need to allow network space for them to transmit.
    // Each additional camera will require time for ( <packet_size> + 18 ) bytes
    // (being 14 byte ethernet header and 4 byte ethernet CRC ).
    // For single byte pixel formats this is ( <packet_size> + 18 ) ticks
    // TODO: support other pixel formats
    unsigned int inter_packet_delay = ( num_cameras - 1 ) * ( packet_size + 18 );
    // An explicit --inter-packet-delay overrides the calculated value
    inter_packet_delay = options.value< unsigned int >( "--inter-packet-delay", inter_packet_delay );
    if( inter_packet_delay != initial_inter_packet_delay ) { camera.GevSCPD = inter_packet_delay;  }

    // Note that the Pylon Viewer app sets:
    //   camera.GevSCBWR = 17; camera.GevSCBWRA = 1;
    // but I was getting a lot of "The buffer was incompletely grabbed"
    // compared to the default values of:
    //   camera.GevSCBWR = 10; camera.GevSCBWRA = 10;
}

static void set_transport_options( Pylon::CBaslerUsbCamera&, const comma::command_line_options& options )
{
    for( std::string const& option
             : std::vector< std::string > { "--packet-size", "--inter-packet-delay", "--num-cameras" } )
    {
        if( options.exists( option ))
        {
            COMMA_THROW( comma::exception, option << " not supported for USB cameras" );
        }
    }
}

template< typename T >
static void show_config( const T& camera )
{
    std::cerr << "basler-cat:       exposure: ";
    if( GenApi::IsAvailable(camera.ExposureAuto) && exposure_is_auto( camera ))
    {
        std::cerr << "auto " << exposure_auto< T >::to_string( camera.ExposureAuto() );
        std::cerr << " (min: " << get_exposure_lower_limit( camera ) << "µs, max: " << get_exposure_upper_limit( camera ) << "µs)";
    }
    else { std::cerr << get_exposure_time( camera ) << "µs"; }
    std::cerr << std::endl;

    std::cerr << "basler-cat:           gain: ";
    if( GenApi::IsAvailable(camera.GainAuto) && gain_is_auto( camera )) { std::cerr << "auto " << gain_auto< T >::to_string( camera.GainAuto() ); }
    else { std::cerr << get_gain( camera ) << gain_units( camera ); }
    std::cerr << std::endl;

    std::cerr << "basler-cat:     frame rate: ";
    if( GenApi::IsAvailable(camera.AcquisitionFrameRateEnable) && camera.AcquisitionFrameRateEnable() ) { std::cerr << get_frame_rate_set( camera ) << " fps"; }
    else { std::cerr << "unset"; }
    std::cerr << std::endl;

    std::cerr << "basler-cat: max frame rate: " << get_frame_rate_max( camera ) << " fps"
              << " (based on AoI, exposure et al.)" << std::endl;

    std::cerr << "basler-cat:   payload size: " << camera.PayloadSize() << " bytes" << std::endl;
    std::cerr << "basler-cat:   pixel format: " << pixel_format< T >::to_string( camera.PixelFormat() ) << std::endl;
}

static std::string trigger_config( Pylon::CBaslerGigECamera& camera, Basler_GigECamera::TriggerSelectorEnums trigger_selector )
{
    GenApi::IEnumEntry* entry = camera.TriggerSelector.GetEntry( trigger_selector );
    if( entry && GenApi::IsAvailable( entry ) )
    {
        camera.TriggerSelector = trigger_selector;
        if( camera.TriggerMode() == Basler_GigECameraParams::TriggerMode_Off ) { return "off"; }
        else { return trigger_source::to_string( camera.TriggerSource() ); }
    }
    else { return "unavailable"; }
}

static void show_trigger_config( Pylon::CBaslerGigECamera& camera )
{
    std::cerr << "basler-cat: acquisition start trigger mode: " << trigger_config( camera, Basler_GigECameraParams::TriggerSelector_AcquisitionStart ) << std::endl;
    std::cerr << "basler-cat:       frame start trigger mode: " << trigger_config( camera, Basler_GigECameraParams::TriggerSelector_FrameStart ) << std::endl;
    std::cerr << "basler-cat:        line start trigger mode: " << trigger_config( camera, Basler_GigECameraParams::TriggerSelector_LineStart ) << std::endl;
}

static void show_trigger_config( Pylon::CBaslerUsbCamera& camera )
{
}

static void show_transport_config( Pylon::CBaslerGigECamera& camera )
{
    std::cerr << "basler-cat:        packet size: " << camera.GevSCPSPacketSize() << " bytes" << std::endl;
    std::cerr << "basler-cat: inter-packet delay: " << camera.GevSCPD() << " ticks" << std::endl;
}

static void show_transport_config( Pylon::CBaslerUsbCamera& camera )
{
}

static void show_config( Pylon::CBaslerGigECamera::StreamGrabber_t& grabber )
{
    std::cerr << "basler-cat: socket buffer size: " << grabber.SocketBufferSize() << " kB" << std::endl;
    std::cerr << "basler-cat:        max buffers: " << grabber.MaxNumBuffer() << std::endl;
    std::cerr << "basler-cat:    max buffer size: " << grabber.MaxBufferSize() << " bytes" << std::endl;
}

static void show_config( Pylon::CBaslerUsbCamera::StreamGrabber_t& grabber )
{
    std::cerr << "basler-cat:     max buffers: " << grabber.MaxNumBuffer() << std::endl;
    std::cerr << "basler-cat: max buffer size: " << grabber.MaxBufferSize() << " bytes" << std::endl;
}

boost::scoped_ptr< snark::tbb::bursty_reader< pair_t > > simple_reader;
boost::scoped_ptr< snark::tbb::bursty_reader< chunk_pair_t > > chunky_reader;

template< typename T >
struct reader_type{ typedef snark::tbb::bursty_reader< T >* type; };

template< typename T >
typename reader_type< T >::type get_reader();

template<>
typename reader_type< pair_t >::type get_reader< pair_t >() { return simple_reader.get(); }

template<>
typename reader_type< chunk_pair_t >::type get_reader< chunk_pair_t >() { return chunky_reader.get(); }

template < typename T, typename P >
static P capture( T& camera, typename T::StreamGrabber_t& grabber )
{
    /// @todo if color spatial correction implemented, mind the following:
    ///
    /// Runner_Users_manual.pdf, 8.2.2.3:
    ///
    /// If you are using a color camera, you have spatial correction enabled
    /// and you have the frame start trigger mode set to off, you must discard
    /// the first n x 2 lines from the first frame transmitted by the camera
    /// after an acquisition start command is issued (where n is the absolute
    /// value of the current spatial correction parameter setting).
    ///
    /// If you have spatial correction enabled and you have the frame start
    /// trigger mode set to on, you must discard the first n x 2 lines from
    /// each frame transmitted by the camera.

    static const unsigned int retries = 10; // quick and dirty: arbitrary
    for( unsigned int i = 0; i < retries; ++i )
    {
        static comma::signal_flag is_shutdown;
        if( is_shutdown )
        {
            comma::verbose << "caught signal" << std::endl;
            comma::verbose << "stopping reader" << std::endl;
            get_reader< P >()->stop();
            comma::verbose << "reader stopped" << std::endl;
            return P();
        }
        Pylon::GrabResult result;
        //camera.AcquisitionStart(); // acquire single image (since acquisition mode set so)
        if( !grabber.GetWaitObject().Wait( timeout ) ) // quick and dirty: arbitrary timeout
        {
            std::cerr << "basler-cat: timeout" << std::endl;
            grabber.CancelGrab();
            while( grabber.RetrieveResult( result ) ); // get all buffers back
            // we just return here rather than continue and try again because
            // experience has shown that once it times out it never recovers
            return P();
        }
        boost::posix_time::ptime current_time = boost::get_system_time();
        if( grabber.RetrieveResult( result ))
        {
            if( !result.Succeeded() )
            {
                std::cerr << "basler-cat: acquisition failed" << std::endl;
                output_result_status( result );
                show_transport_config( camera );
                grabber.QueueBuffer( result.Handle() ); // requeue buffer
                continue;
            }
            P pair;
            pair.second = cv::Mat( result.GetSizeY(), is_packed ? ( ( result.GetSizeX() * 3 ) / 2 ) : result.GetSizeX(), cv_mat_header.type ); // todo: seriously quick and dirty, does not scale to Mono10p; implement packed bytes layout properly 
            ::memcpy( pair.second.data, reinterpret_cast< const char* >( result.Buffer() ), pair.second.dataend - pair.second.datastart );
            // quick and dirty for now: rgb are not contiguous in basler camera frame
            if( cv_mat_header.type == CV_8UC3 || cv_mat_header.type == CV_16UC3 ) { cv::cvtColor( pair.second, pair.second, CV_RGB2BGR ); }
            set< T >( pair.first, current_time, result, camera );
            grabber.QueueBuffer( result.Handle() ); // requeue buffer
            return pair;
        }
        else
        {
            std::cerr << "basler-cat: failed to retrieve result" << std::endl;
            continue;
        }
    }
    return P();
}

static void write( const chunk_pair_t& p )
{
    if( p.second.empty() || !std::cout.good() ) { return; }
    static comma::csv::binary_output_stream< Header > ostream( std::cout, csv );
    static Header header( cv_mat_header );
    header.header.timestamp = p.first.timestamp;
    header.counters.ticks = p.first.ticks;
    static ChunkData first_chunk_data;
    if( first_chunk_data.timestamp.is_not_a_date_time() )
    {
        first_chunk_data = p.first;
        header.counters.adjusted_timestamp = p.first.timestamp;
    }
    else
    {
        static const double factor = 8.0 / 1000; // 8ns per tick
        header.counters.adjusted_timestamp = first_chunk_data.timestamp + boost::posix_time::microseconds( static_cast< int >( factor * first_chunk_data.ticks ) ); // todo: factor in network delay?
    }
    header.counters.line_count += p.first.line_trigger_ignored + 1;
    header.counters.line = header.counters.line_count % encoder_ticks;
    ostream.write( header );
    std::cout.write( ( const char* )( p.second.datastart ), p.second.dataend - p.second.datastart );
    std::cout.flush();
}

static bool run_chunk_pipeline( Pylon::CBaslerGigECamera& camera
                              , Pylon::CBaslerGigECamera::StreamGrabber_t& grabber
                              , unsigned int max_queue_size
                              , unsigned int max_queue_capacity )
{
    chunky_reader.reset( new snark::tbb::bursty_reader< chunk_pair_t >
                           ( boost::bind( &capture< Pylon::CBaslerGigECamera, chunk_pair_t >
                                        , boost::ref( camera )
                                        , boost::ref( grabber ))
                           , max_queue_size, max_queue_capacity ));
    tbb::filter_t< chunk_pair_t, void > writer( tbb::filter::serial_in_order, boost::bind( &write, _1 ));
    snark::tbb::bursty_pipeline< chunk_pair_t > pipeline;
    camera.AcquisitionStart();
    comma::verbose << "running in chunk mode..." << std::endl;
    pipeline.run( *chunky_reader, writer );
    comma::verbose << "shutting down..." << std::endl;
    camera.AcquisitionStop();
    comma::verbose << "acquisition stopped" << std::endl;
    chunky_reader->join();
    camera.DestroyChunkParser( parser );
    return true;
}

static bool run_chunk_pipeline( Pylon::CBaslerUsbCamera& camera
                              , Pylon::CBaslerUsbCamera::StreamGrabber_t& grabber
                              , unsigned int max_queue_size
                              , unsigned int max_queue_capacity )
{
    COMMA_THROW( comma::exception, "chunk mode not supported for USB cameras" );
}

template< typename C, typename G >
static bool run_simple_pipeline( C& camera, G& grabber, unsigned int max_queue_size, unsigned int max_queue_capacity )
{
    bool error = false;
    snark::cv_mat::serialization serialization( cv_mat_options );
    simple_reader.reset( new snark::tbb::bursty_reader< pair_t >
                           ( boost::bind( &capture< C, pair_t >
                                        , boost::ref( camera )
                                        , boost::ref( grabber ))
                           , max_queue_size, max_queue_capacity ));
    snark::imaging::applications::pipeline pipeline( serialization, filters, *simple_reader );
    camera.AcquisitionStart();
    comma::verbose << "running..." << std::endl;
    pipeline.run();
    if( !pipeline.error().empty() ) { std::cerr << "basler-cat: \"" << pipeline.error() << "\"" << std::endl; error = true; }
    comma::verbose << "shutting down..." << std::endl;
    camera.AcquisitionStop();
    comma::verbose << "acquisition stopped" << std::endl;
    simple_reader->join();
    return !error;
}

template< typename C, typename G >
static bool run_pipeline( C& camera, G& grabber, bool chunk_mode, unsigned int max_queue_size, unsigned int max_queue_capacity )
{
    if( chunk_mode ) { return run_chunk_pipeline( camera, grabber, max_queue_size, max_queue_capacity ); }
    else { return run_simple_pipeline( camera, grabber, max_queue_size, max_queue_capacity ); }
}

template< typename T >
static int run( T& camera, const comma::command_line_options& options )
{
    typedef T camera_t;
    cv_mat_options.header_only = options.exists( "--header-only" );
    cv_mat_options.no_header = options.exists( "--no-header" );
    timeout = options.value< double >( "--timeout", default_timeout ) * 1000.0;
    comma::verbose << "initialized camera" << std::endl;
    comma::verbose << "opening camera " << camera.GetDevice()->GetDeviceInfo().GetFullName() << "..." << std::endl;
    camera.Open();
    comma::verbose << "opened camera " << camera.GetDevice()->GetDeviceInfo().GetFullName() << std::endl;    
    load_default_settings( camera );
    set_pixel_format( camera, options );
    cv_mat_header = cv_mat_options.get_header();
    typename camera_t::StreamGrabber_t grabber( camera.GetStreamGrabber( 0 ) );
    grabber.Open();
    camera.OffsetX = 0;                 // reset before we get the maximum width
    unsigned int max_width = camera.Width.GetMax();
    unsigned int offset_x = options.value< unsigned int >( "--offset-x", 0 );
    if( offset_x >= max_width ) { std::cerr << "basler-cat: expected --offset-x less than " << max_width << ", got " << offset_x << std::endl; return 1; }
    unsigned int width = options.value< unsigned int >( "--width", max_width );
    if(( width + offset_x ) > max_width ) { width = max_width - offset_x; }
    camera.Width = width;
    camera.OffsetX = offset_x;          // but set _after_ we set the actual width
    camera.OffsetY = 0;                 // reset before we get the maximum height
    unsigned int max_height = camera.Height.GetMax();
    unsigned int offset_y = options.value< unsigned int >( "--offset-y", 0 );
    if( offset_y >= max_height ) { std::cerr << "basler-cat: expected --offset-y less than " << max_height << ", got " << offset_y << std::endl; return 1; }
    unsigned int height = options.value< unsigned int >( "--height", max_height );

    // todo: is the colour line 2098 * 3 or ( 2098 / 3 ) * 3 ?
    //unsigned int channels = num_channels( camera, cv_mat_header.type );
    //offset_y *= channels;
    //height *= channels;

    if(( height + offset_y ) > max_height ) { height = max_height - offset_y; }
    camera.Height = height;
    camera.OffsetY = offset_y;          // but set _after_ we set the actual height
    comma::verbose << "set width,height to " << width << "," << height << std::endl;

    set_transport_options( camera, options );

    if( options.exists( "--binning-horizontal" ) )
    {
        std::string s = options.value< std::string >( "--binning-horizontal" );
        std::vector< std::string > v = comma::split( s, ',' );
        camera.BinningHorizontal = boost::lexical_cast< unsigned int >( v[0] );
        for( unsigned int i = 1 ; i < v.size(); ++i )
        {
            if( v[i] == "sum" ) { set_binning_horizontal_mode_sum( camera ); }
            else if( v[i] == "average" ) { set_binning_horizontal_mode_average( camera ); }
            else { std::cerr << "basler-cat: invalid mode for --binning-horizontal: " << v[i] << std::endl; return 1; }
        }
    }
    if( options.exists( "--binning-vertical" ) )
    {
        std::string s = options.value< std::string >( "--binning-vertical" );
        std::vector< std::string > v = comma::split( s, ',' );
        camera.BinningVertical = boost::lexical_cast< unsigned int >( v[0] );
        for( unsigned int i = 1 ; i < v.size(); ++i )
        {
            if( v[i] == "sum" ) { set_binning_vertical_mode_sum( camera ); }
            else if( v[i] == "average" ) { set_binning_vertical_mode_average( camera ); }
            else { std::cerr << "basler-cat: invalid mode for --binning-vertical: " << v[i] << std::endl; return 1; }
        }
    }

    if( options.exists( "--reverse-x" ) ) { camera.ReverseX = true; }
    if( options.exists( "--reverse-y" ) ) { camera.ReverseY = true; }
    
    // todo: giving up... the commented code throws, but failure to stop acquisition, if active
    //       seems to lead to the following scenario:
    //       - power-cycle camera
    //       - view colour images: it works
    //       - view grey-scale images: it works
    //       - view colour images: it still displays grey-scale
    //comma::verbose << "getting acquisition status... (frigging voodoo...)" << std::endl;
    //GenApi::IEnumEntry* acquisition_status = camera.AcquisitionStatusSelector.GetEntry( Basler_GigECameraParams::AcquisitionStatusSelector_AcquisitionActive );
    //if( acquisition_status && GenApi::IsAvailable( acquisition_status ) && camera.AcquisitionStatus() )
    //{
    //    comma::verbose << "stopping acquisition..." << std::endl;
    //    camera.AcquisitionStop();
    //    comma::verbose << "acquisition stopped" << std::endl;
    //}

    if( !configure_trigger( camera, options )) { return 1; }
    if( chunk_mode )
    {
        std::cerr << "basler-cat: setting chunk mode..." << std::endl;
        if( !GenApi::IsWritable( camera.ChunkModeActive )) { std::cerr << "basler-cat: camera does not support chunk features" << std::endl; camera.Close(); return 1; }
        camera.ChunkModeActive = true;
        configure_chunk_mode( camera );
        parser = camera.CreateChunkParser();
        if( !parser ) { std::cerr << "basler-cat: failed to create chunk parser" << std::endl; camera.Close(); return 1; }
        std::cerr << "basler-cat: set chunk mode" << std::endl;
    }
    if( options.exists( "--frame-rate" )) { set_frame_rate( camera, options.value< double >( "--frame-rate" )); }
    else if(GenApi::IsAvailable(camera.AcquisitionFrameRateEnable)) { camera.AcquisitionFrameRateEnable = false; }
    set_exposure( camera, options );
    set_gain( camera, options );
    if( options.exists( "--line-rate" )) { set_line_rate( camera, options.value< unsigned int >( "--line-rate" )); }
    if( GenApi::IsAvailable( camera.TestImageSelector ) ) { set_test_image( camera, options.value< unsigned int >( "--test-image", 0 )); }
    else { if( options.exists( "--test-image" )) { COMMA_THROW( comma::exception, "test image is not supported by this camera" ); } }
    std::vector< std::vector< char > > buffers( 2 ); // todo? make number of buffers configurable
    for( std::size_t i = 0; i < buffers.size(); ++i ) { buffers[i].resize( camera.PayloadSize() ); }
    grabber.MaxBufferSize = buffers[0].size();
    grabber.MaxNumBuffer = buffers.size(); // todo: use --buffer value for number of buffered images
    set_socket_buffer_size( grabber, 127 );
    if( comma::verbose )
    {
        show_config( camera );
        show_trigger_config( camera );
        show_transport_config( camera );
        show_config( grabber );
    }
    grabber.PrepareGrab(); // image size now must not be changed until FinishGrab() is called.
    std::vector< Pylon::StreamBufferHandle > buffer_handles( buffers.size() );
    for( std::size_t i = 0; i < buffers.size(); ++i )
    {
        buffer_handles[i] = grabber.RegisterBuffer( &buffers[i][0], buffers[i].size() );
        grabber.QueueBuffer( buffer_handles[i] );
    }
    unsigned int max_queue_size = options.value< unsigned int >( "--buffer", options.exists( "--discard" ));
    int return_value = 0;
    set_continuous_acquisition_mode( camera );
    try { if( !run_pipeline( camera, grabber, chunk_mode, max_queue_size, max_queue_size * 3 ) ) { return_value = 1; } }
    catch( std::exception& ex ) { std::cerr << "basler-cat: " << ex.what() << std::endl; return_value = 1; }
    grabber.FinishGrab();
    Pylon::GrabResult result;
    while( grabber.RetrieveResult( result ) ); // get all buffers back
    for( std::size_t i = 0; i < buffers.size(); ++i ) { grabber.DeregisterBuffer( buffer_handles[i] ); }
    grabber.Close();
    camera.Close();
    return return_value;
}

#define QUOTED( arg ) #arg
#define STRINGIZED( arg ) QUOTED( arg )

int main( int argc, char** argv )
{
    ::setenv( "PYLON_ROOT", STRINGIZED( BASLER_PYLON_DIR ), 0 );
    ::setenv( "GENICAM_ROOT_V2_1", STRINGIZED( BASLER_PYLON_GENICAM_DIR ), 0 );
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" )) bash_completion( argc, argv );
        if( options.exists( "--help-filters" )) { std::cerr << snark::cv_mat::impl::filters<>::usage(); return 0; }
        if( options.exists( "--help-types" )) { std::cerr << snark::cv_mat::serialization::options::type_usage(); return 0; }

        Pylon::PylonAutoInitTerm auto_init_term;
        if( options.exists( "--list-cameras" ) ) { list_cameras(); return 0; }
        comma::verbose << "PYLON_ROOT=" << ::getenv( "PYLON_ROOT" ) << std::endl;
        comma::verbose << "GENICAM_ROOT_V2_1=" << ::getenv( "GENICAM_ROOT_V2_1" ) << std::endl;
        comma::verbose << "initializing camera..." << std::endl;
        std::string address = options.value< std::string >( "--address", "" );
        std::string serial_number = options.value< std::string >( "--serial-number", "" );
        Pylon::IPylonDevice* device = create_device( address, serial_number );
        if( !device )
        {
            std::cerr << "basler-cat: unable to open camera";
            if( !address.empty() ) { std::cerr << " for address " << address; }
            std::cerr << std::endl;
            return 1;
        }
        filters = comma::join( options.unnamed( "--help,-h,--verbose,-v,--discard,--list-cameras,--header-only,--no-header", "-.*" ), ';' );
        csv = comma::csv::options( argc, argv );
        chunk_mode =    csv.has_field( "counters" ) // quick and dirty
                     || csv.has_field( "adjusted-t" )
                     || csv.has_field( "line" )
                     || csv.has_field( "line-count" )
                     || csv.has_field( "ticks" )
                     || csv.has_field( "counters/adjusted-t" )
                     || csv.has_field( "counters/line" )
                     || csv.has_field( "counters/line-count" )
                     || csv.has_field( "counters/ticks" );
        if( chunk_mode )
        {
            if( !options.exists( "--encoder-ticks" )) { std::cerr << "basler-cat: chunk mode, please specify --encoder-ticks" << std::endl; return 1; }
            encoder_ticks = options.value< unsigned int >( "--encoder-ticks" );
            if( !filters.empty() ) { std::cerr << "basler-cat: chunk mode, cannot handle filters; use: basler-cat | cv-cat <filters> instead" << std::endl; return 1; }
            unsigned int height = options.value< unsigned int >( "--height", 1 );
            if( height != 1 ) { std::cerr << "basler-cat: only --height=1 implemented in chunk mode" << std::endl; return 1; }
            std::vector< std::string > v = comma::split( csv.fields, ',' );
            std::string format;
            for( unsigned int i = 0; i < v.size(); ++i )
            {
                if( v[i] == "t" ) { v[i] = "header/" + v[i]; format += "t"; }
                else if( v[i] == "rows" || v[i] == "cols" || v[i] == "size" || v[i] == "type" ) { v[i] = "header/" + v[i]; format += "ui"; }
                else if( v[i] == "adjusted-t" ) { v[i] = "counters/" + v[i]; format += "t"; }
                else if( v[i] == "line-count" || v[i] == "ticks" ) { v[i] = "counters/" + v[i]; format += "ul"; }
                else if( v[i] == "line" ) { v[i] = "counters/" + v[i]; format += "ui"; }
                else { std::cerr << "basler-cat: expected field, got '" << v[i] << "'" << std::endl; return 1; }
            }
            csv.fields = comma::join( v, ',' );
            csv.full_xpath = true;
            csv.format( format );
        }
        Pylon::String_t device_class = device->GetDeviceInfo().GetDeviceClass();
        if ( device_class == "BaslerGigE" )
        {
            Pylon::CBaslerGigECamera camera;
            camera.Attach( device );
            return run( camera, options );
        }
        if( device_class == "BaslerUsb" )
        {
            Pylon::CBaslerUsbCamera camera;
            camera.Attach( device );
            return run( camera, options );
        }
        std::cerr << "basler-cat: unsupported device type of " << device_class << std::endl;
        return 1;
    }
#if ( PYLON_VERSION_MAJOR >= 5 )
    catch(const Pylon::GenericException& e) { std::cerr << "basler-cat: pylon exception: " << e.what() << std::endl; }
#endif
    catch( std::exception& ex ) { std::cerr << "basler-cat: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "basler-cat: unknown exception" << std::endl; }
    return 1;
}
