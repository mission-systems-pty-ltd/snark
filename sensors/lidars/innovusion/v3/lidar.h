// Copyright (c) 2025 Mission Systems Pty Ltd

#pragma once

#include "../common/lidar.h"
#include <sdk_common/inno_lidar_api.h>
#include <comma/base/last_error.h>
#include <comma/csv/stream.h>
#include <iostream>
#include <string>

namespace snark { namespace innovusion {

const std::string default_address( "172.168.1.10" );
const unsigned int default_port( 8010 );
const unsigned int default_udp_port( 8010 );
const unsigned int default_max_latency( 0 );

// thin wrapper around the Innovusion API

class lidar
{
public:
    lidar();
    ~lidar();

    void init( const std::string& name
             , const std::string& address
             , unsigned int port
             , unsigned int udp_port
             , InnoMessageCallback message_callback
             , InnoDataPacketCallback data_callback
             , InnoStatusPacketCallback status_callback
             , void* context = nullptr );

    void start();
    void set_timeframe_offset_us( int64_t timeframe_offset_us_ ) { timeframe_offset_us = timeframe_offset_us_; }

private:
    int handle;
};

template< typename T >
struct writer
{
    // We roll the output routine by hand rather than use comma::csv::binary_output_stream
    // so that we can use a custom write() routine that writes to the stdout file descriptor
    // rather than using iostreams std::cout.
    //
    // This is because we were seeing output errors under load. Similar to that
    // seen in other high data rate multithreaded applications.
    static void output( const InnoDataPacket* frame )
    {
        // Implement these few lines as a lambda so they only get executed on
        // the first run through when the static "os" is set.
        // Making this a separate method would complicate the template
        // overloading for raw_output and null_output
        auto make_binary = []()
        {
            comma::csv::options output_csv;
            output_csv.format( comma::csv::format::value< T >() );
            return comma::csv::binary< T >( output_csv.format().string(), output_csv.fields, output_csv.full_xpath, T() );
        };
        static comma::csv::binary< T > binary = make_binary();
        static const size_t record_size = binary.format().size();

        // buf is large enough to contain one complete frame.
        //
        // The size of a frame can vary slightly from one to the next, and will
        // vary from one device to another, so we dynamically size once we
        // receive some data. This typically happens just once upon receipt of
        // the first frame.
        static std::vector< char > buf;

        comma::verbose << "received " << frame->item_number << " points" << std::endl;
        if( frame->item_number * record_size > buf.size() )
        {
            std::cerr << "innovusion-cat: received " << frame->item_number << " points, resizing output buffer" << std::endl;
            buf.resize( static_cast< size_t >( frame->item_number * record_size * 1.05 ));     // 5% buffer to minimise resizes
        }

        for( unsigned int i = 0; i < frame->item_number; i++ )
        {
            binary.put( T( frame, i, timeframe_offset_us ), &buf[ i * record_size ] );
        }
        write( &buf[0], frame->item_number * record_size );
    }

    static void write( const char* buf, unsigned int count )
    {
        while( count > 0 )
        {
            ssize_t bytes_written = ::write( 1, buf, count );
            if( bytes_written == -1 ) { COMMA_THROW( comma::last_error::exception, "error" ); }
            if( bytes_written == 0 ) { COMMA_THROW( comma::exception, "write() wrote 0 bytes" ); } // shouldn't occur with stdout
            count -= bytes_written;
            buf += bytes_written;
        }
    }
};

template<> void writer< snark::innovusion::null_output >::output( const InnoDataPacket* frame ) {}

template<> void writer< snark::innovusion::raw_output >::output( const InnoDataPacket* frame )
{
    std::cout.write( (const char*)&frame->xyz_points[0], frame->item_number * sizeof( InnoXyzPoint ));
}

} } // namespace snark { namespace innovusion {
