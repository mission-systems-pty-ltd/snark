// Copyright (c) 2021,2025 Mission Systems Pty Ltd

#pragma once

#include "../common/lidar.h"
#include <inno_lidar_api.h>
#include <comma/base/last_error.h>
#include <comma/csv/stream.h>
#include <string>

namespace snark { namespace innovusion {

const std::string default_address( "172.168.1.10" );
const unsigned int default_port( 8001 );

// TODO: probably could do this better
extern int64_t timeframe_offset_us;

// thin wrapper around the Innovusion API

class lidar
{
public:
    lidar();
    ~lidar();

    void init( const std::string& name
             , const std::string& address, unsigned int port
             , inno_lidar_alarm_callback_t alarm_callback
             , inno_lidar_frame_callback_t frame_callback
             , void* context = nullptr );

    void start();

private:
    int handle;
};

template< typename T >
struct writer
{
    static void process()
    {
        // while( !shutdown_requested )
        // {
        //     inno_frame* frame;
        //     // non-blocking, as opposed to concurrent_bounded_queue::pop()
        //     while( queue.try_pop( frame ) && !shutdown_requested )
        //     {
        //         // latest_frame is the latest frame received on the queue
        //         // frame is the frame we are processing now
        //         inno_timestamp_us_t latency = latest_frame_start_time - frame->ts_us_start;
        //         if( comma::verbose )
        //         {
        //             if( latency > 0 ) { std::cerr << comma::verbose.app_name() << ": latency = " << latency << "µs"; }
        //             if( latency > max_latency ) { std::cerr << "...dropping frame"; }
        //             if( latency > 0 ) { std::cerr << std::endl; }
        //         }
        //         if( latency <= max_latency ) { output( frame ); }
        //         free( (void*)frame );
        //     }
        //     std::this_thread::sleep_for( std::chrono::milliseconds( 40 ));
        // }
    }

    // We roll the output routine by hand rather than use comma::csv::binary_output_stream
    // so that we can use a custom write() routine that writes to the stdout file descriptor
    // rather than using iostreams std::cout.
    //
    // This is because we were seeing output errors under load. Similar to that
    // seen in other high data rate multithreaded applications.
    static void output( inno_frame* frame )
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

        if( frame->points_number * record_size > buf.size() )
        {
            std::cerr << "innovusion-cat: received " << frame->points_number << " points, resizing output buffer" << std::endl;
            buf.resize( static_cast< size_t >( frame->points_number * record_size * 1.05 ));     // 5% buffer to minimise resizes
        }

        for( unsigned int i = 0; i < frame->points_number; i++ )
        {
            binary.put( T( frame, i, timeframe_offset_us ), &buf[ i * record_size ] );
        }
        write( &buf[0], frame->points_number * record_size );
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

template<> void writer< snark::innovusion::null_output >::output( inno_frame* frame ) {}

template<> void writer< snark::innovusion::raw_output >::output( inno_frame* frame )
{
    std::cout.write( (const char*)&frame->points[0], frame->points_number * sizeof( inno_point ));
}

} } // namespace snark { namespace innovusion {
