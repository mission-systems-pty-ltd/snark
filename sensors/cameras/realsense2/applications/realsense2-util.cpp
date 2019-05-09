// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2019 The University of Sydney
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
#include <comma/io/stream.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/name_value/parser.h>
#include <librealsense2/rs.hpp>

namespace {

//comma::signal_flag signaled;

void bash_completion( unsigned const ac, char const * const * av )
{
    static char const * const arguments[] =
    {
        " configure list reset",
        " --device",
        " --sensor",
        " --operations",
        " --output-fields",
        " --output-format",
        " --verbose"
    };
    std::cout << arguments << std::endl;
    exit( 0 );
}

void operations( unsigned const indent_count = 0 )
{
    auto const indent = std::string( indent_count, ' ' );
    std::cerr << indent << "configure; configure sensor options from stdin (fields: index,value)." << std::endl;
    std::cerr << indent << "list; list devices." << std::endl;
    std::cerr << indent << "reset; reset devices." << std::endl;
}

void usage( bool const verbose )
{
    static const char* const indent="    ";

    std::cerr << std::endl;
    std::cerr << "Read the data from realsense camera and convert into cv format." << std::endl;
    std::cerr << std::endl;
    std::cerr << "Usage:" << std::endl;
    std::cerr << indent << comma::verbose.app_name() << " <operation> [<options>...]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Operations:" << std::endl;
    operations(4);
    std::cerr << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << "    common:" << std::endl;
    std::cerr << "        --device=<serial>; serial number(s) of device(s)." << std::endl;
    std::cerr << std::endl;
    std::cerr << "    configure:" << std::endl;
    std::cerr << "        --sensor=<index>; serial number(s) of device(s)." << std::endl;
    std::cerr << std::endl;
    std::cerr << "Info:" << std::endl;
    std::cerr << "    --operations; print operations and exit." << std::endl;
    std::cerr << "    --output-fields; print operation-dependent output fields to stdout and exit." << std::endl;
    std::cerr << "    --output-format; print operation-dependent output format to stdout and exit." << std::endl;
    std::cerr << std::endl;
    std::cerr << "Examples:" << std::endl;
    std::cerr << indent << comma::verbose.app_name() << " list" << std::endl;
    std::cerr << std::endl;
    std::cerr << indent << comma::verbose.app_name() << " reset --device 1234 --device 4321" << std::endl;
    std::cerr << std::endl;
    std::cerr << indent << comma::verbose.app_name() << " configure --sensor=1 <<< '11,0'" << std::endl;
    std::cerr << std::endl;
}

void handle_info_options( comma::command_line_options const& options )
{
    if( options.exists( "--operations" ) ) { operations(); exit( 0 ); }
}

std::string get_operation( comma::command_line_options const& options )
{
    std::vector< std::string > operations = options.unnamed( "", "--device,--sensor,--output-fields,--output-format,--operations,--verbose" );
    if( operations.size() == 1 ) { return operations.front(); }
    COMMA_THROW( comma::exception, "expected one operation, got " << operations.size() << ": " << comma::join( operations, ' ' ) );
}

void list_sensors( rs2::device const& device )
{
    auto sensors = device.query_sensors();
    for( auto& sensor : sensors )
    {
        std::cerr << "    Sensor: " << sensor.get_info( RS2_CAMERA_INFO_NAME ) << std::endl;
        std::cout << "        Options( index,name,description,default,min,max,current ):" << std::endl;
        for( int oi = 0; oi < static_cast< int >( RS2_OPTION_COUNT ); oi++ )
        {
            rs2_option option = static_cast< rs2_option >( oi );
            if( sensor.supports( option ) )
            {
                auto range = sensor.get_option_range( option );
                std::cerr << "            " << oi << ',' << option << ',' << sensor.get_option_description( option )
                    << ','<< range.def << ',' << range.min << ',' << range.max << ',' << sensor.get_option( option ) << std::endl;
            }
        }

        //std::cout << "        Stream Profiles( id, index, name, type):" << std::endl;
        //auto stream_profiles = sensor.get_stream_profiles();
        //for( auto const& sp : stream_profiles )
        //{
        //    std::cerr << "            " << sp.unique_id() << ',' << sp.stream_index() << ',' << sp.stream_name() << ',' << sp.stream_type() << std::endl;
        //}
    }
}

}

namespace configure
{

struct input_t
{
    int index;
    float value;
    input_t( void ) : index( 0 ), value( 0 ) {}
};

}

namespace comma { namespace visiting {

template <> struct traits< configure::input_t >
{
    template < typename K, typename V > static void visit ( K const&, configure::input_t& t, V& v )
    {
        v.apply( "index", t.index );
        v.apply( "value", t.value );
    }

    template < typename K, typename V > static void visit ( K const&, configure::input_t const& t, V& v )
    {
        v.apply( "index", t.index );
        v.apply( "value", t.value );
    }
};

}}


int main( int ac, char* av[] )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--bash-completion" ) ) bash_completion( ac, av );
        handle_info_options( options );

        auto const verbose = options.exists( "--verbose" );
        auto operation = get_operation( options );
        auto device_ids = options.values< std::string >( "--device" );

        if( "configure" == operation )
        {
            rs2::context context;
            auto devices = context.query_devices();
            if( 0 == devices.size() ) { std::cerr << comma::verbose.app_name() << ": no device present." << std::endl; return 1; }
            if( 1 < device_ids.size() ) { std::cerr << comma::verbose.app_name() << ": only one device can be configured at a time." << std::endl; return 1; }
            rs2::device device;
            if( 0 == device_ids.size() ) { device = devices[ 0 ]; }
            else
            {
                for( auto const& dev : devices )
                {
                    auto device_id = std::string( dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) );
                    if( device_id == device_ids[ 0 ] ) { device = dev; break; }
                }
                if( !device ) { std::cerr << comma::verbose.app_name() << ": device with serial "<< device_ids[ 0 ] << std::endl; return 1; }
            }
            auto sensors = device.query_sensors();
            auto const sensor_index = options.value< unsigned >( "--sensor" );
            if( sensors.size() <= sensor_index )
            { std::cerr << comma::verbose.app_name() << ": sensor index " << sensor_index << " greater than maximum: "<< sensors.size() << std::endl; return 1; }
            auto sensor = sensors[ sensor_index ];
            //auto const sensor_name = sensor.get_info( RS2_CAMERA_INFO_NAME );

            comma::csv::options csv;
            csv.fields = "index,value";
            comma::csv::input_stream< configure::input_t > istrm( std::cin, csv );
            while( istrm.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                auto record = istrm.read(); if( !record ) break;
                rs2_option option = static_cast< rs2_option >( record->index );
                //if( !sensor.supports( option ) )
                //{
                //    std::cerr << comma::verbose.app_name() << ": sensor '" << sensor_name << "' at index " << sensor_index
                //        << " does not support option " << option << " with index" << record->index << std::endl;
                //}
                //auto range = sensor.get_option_range( option );
                //if( range.min > record->value || range.max < record->value )
                //{
                //    std::cerr << comma::verbose.app_name() << ": given value " << record->value << " is out of range (" << range.min << ',' << range.max
                //        << "for option " << option << " with index" << record->index << std::endl;
                //}
                sensor.set_option( option, record->value );

            }

        }
        else if( "list" == operation )
        {
            rs2::context context;
            auto devices = context.query_devices();
            for( auto const& dev : devices )
            {
                auto device_id = std::string( dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) );
                if( device_ids.empty() || device_ids.end() != std::find( device_ids.begin(), device_ids.end(), device_id ) )
                {
                    std::cout << dev.get_info(RS2_CAMERA_INFO_NAME)
                        << ','<< device_id
                        << ','<< dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT) << std::endl;
                    
                    if( verbose ) { list_sensors( dev ); }
                }
            }
            if( !verbose ) { std::cerr << comma::verbose.app_name() << ": pass --verbose for sensor information." << std::endl; }
        }
        else if( "reset" == operation )
        {
            rs2::context context;
            auto devices = context.query_devices();
            for( auto dev : devices )
            {
                auto device_id = std::string( dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) );
                if( device_ids.empty() || device_ids.end() != std::find( device_ids.begin(), device_ids.end(), device_id ) )
                {
                    dev.hardware_reset();
                }
            }
        }
        else
        {
            std::cerr << comma::verbose.app_name() << ": unknown operation '" << operation << '\'' << std::endl;
        }
        
        return 0;
    }
    catch( rs2::error& ex ) { std::cerr << comma::verbose.app_name() << ": realsense exception: " << ex.what() << std::endl; }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": unknown exception" << std::endl; }
    return 1;
}

