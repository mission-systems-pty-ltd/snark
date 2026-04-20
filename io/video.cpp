// Copyright (c) 2024 Mission Systems Pty Ltd

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <comma/base/exception.h>
#include <comma/timing/epoch.h>
#include <comma/timing/kernel.h>
#include "video.h"

namespace snark { namespace io { namespace video {

static int xioctl( int fd, int request, void* arg )
{
    int r;
    do { r = ::ioctl( fd, request, arg ); } while( r == -1 && errno == EINTR ); // todo? microsleep? select?
    return r;
}

stream::stream( const std::string& name, unsigned int width, unsigned int height, unsigned int number_of_buffers, int pixel_format, bool use_v4l2_time )
    : _name( name )
    , _width( width )
    , _height( height )
    , _buffers( number_of_buffers )
    , _use_v4l2_time( use_v4l2_time )
{
    _file = std::fopen( &name[0], "r+" );
    COMMA_ASSERT( _file, "failed to open '" << name << "'" );
    _fd = ::fileno( _file );
    if(!_use_v4l2_time) { _time_strategy = timestamp_strategy::userspace_explicit; }
    v4l2_capability capability{};
    v4l2_format format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = pixel_format;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    COMMA_ASSERT( xioctl( _fd, VIDIOC_QUERYCAP, &capability ) != -1, "'" << name << "': " << ( errno == EINVAL ? "not a v4l2 device" : "ioctl error: VIDIOC_QUERYCAP" ) );
    COMMA_ASSERT( capability.capabilities & V4L2_CAP_VIDEO_CAPTURE, "'" << name << "': is not a video capture device" );
    COMMA_ASSERT( capability.capabilities & V4L2_CAP_STREAMING, "'" << name << "': does not support streaming i/o" );
    COMMA_ASSERT( xioctl( _fd, VIDIOC_S_FMT, &format ) != -1, "'" << name << "': ioctl error: VIDIOC_S_FMT" );
    v4l2_requestbuffers request_buffers{};
    request_buffers.count = number_of_buffers;
    request_buffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request_buffers.memory = V4L2_MEMORY_MMAP;
    COMMA_ASSERT( xioctl( _fd, VIDIOC_REQBUFS, &request_buffers ) != -1, "'" << name << "': " << ( errno == EINVAL ? "not a v4l2 device" : "ioctl error: VIDIOC_REQBUFS" ) << "; " << ::strerror( errno ) << "(" << errno << ")" );
    COMMA_ASSERT( request_buffers.count == _buffers.size(), "'" << name << "': insufficient buffer memory for " << _buffers.size() << " buffers; maximum available " << request_buffers.count << " buffers" );
    for( unsigned int i = 0; i < number_of_buffers; ++i ) // todo? make exception-safe: munmap on buffers already mapped buffers
    {
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;
        COMMA_ASSERT( xioctl( _fd, VIDIOC_QUERYBUF, &buffer ) != -1, "'" << name << "': on buffer " << i << ": ioctl error: VIDIOC_QUERYBUF, " << ::strerror( errno ) << "(" << errno << ")" );
        _size = buffer.length;
        _buffers[i].data = ::mmap( nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, _fd, buffer.m.offset );
        COMMA_ASSERT( _buffers[i].data != MAP_FAILED, "'" << name << "': mmap failed on buffer index " << i );
    }
}

stream::~stream()
{
    for( auto buffer: _buffers ) { ::munmap( buffer.data, _size ); }
    std::fclose( _file );
}

void stream::start()
{
    if( _started ) { return; }
    for( size_t i = 0; i < _buffers.size(); ++i )
    {
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;
        COMMA_ASSERT( xioctl( _fd, VIDIOC_QBUF, &buffer ) != -1, "'" << _name << "': ioctl error: VIDIOC_QBUF" );
    }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    COMMA_ASSERT( xioctl( _fd, VIDIOC_STREAMON, &type ) != -1, "'" << _name << "': ioctl error: VIDIOC_STREAMON" );
    _started = true;
}

void stream::stop()
{
    if( !_started ) { return; }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    COMMA_ASSERT( xioctl( _fd, VIDIOC_STREAMOFF, &type ) != -1, "'" << _name << "': failed to stop streaming: ioctl error: VIDIOC_STREAMOFF" );
    _started = false;
}

stream::record stream::read( float timeout_seconds, unsigned int attempts )
{
    fd_set fds{}; // todo? use comma::select?
    FD_ZERO( &fds );
    FD_SET( _fd, &fds );
    timeval timeout{};
    unsigned int microseconds = timeout_seconds * 1e6;
    timeout.tv_sec = microseconds / 1000000;
    timeout.tv_usec = microseconds % 1000000;
    unsigned int attempts_remaining = attempts;
    bool forever = attempts == 0;
    while( true )
    {
        int r = select( _fd + 1, &fds, nullptr, nullptr, &timeout );
        if( r == -1 )
        {
            if( errno == EINTR ) { return record(); }
            COMMA_THROW( comma::exception, "'" << _name << ": select error: " << strerror( errno ) << "(" << errno << ")" );
        }
        if( r == 0 )
        {
            COMMA_ASSERT( forever || attempts_remaining > 0, "'" << _name << ": select timeout" );
            --attempts_remaining;
            continue;
        }
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        COMMA_ASSERT( xioctl( _fd, VIDIOC_DQBUF, &buffer ) != -1, "'" << _name << "': " << ( errno == EAGAIN ? "EAGAIN" : "VIDIOC_DQBUF" ) << ": " << ::strerror( errno ) << "(" << errno << ")" );
        COMMA_ASSERT( buffer.index < _buffers.size(), "'" << _name << "': expected buffer index less than number of buffers (" << _buffers.size() << "); got: " << buffer.index );
        COMMA_ASSERT( xioctl( _fd, VIDIOC_QBUF, &buffer ) != -1, "'" << _name << "': ioctl error: VIDIOC_QBUF" );
        _index = buffer.index;
        if( _use_v4l2_time )
        {
            if( _time_strategy == timestamp_strategy::unset )
            {
                uint32_t timestamp_type = buffer.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK;
                switch( timestamp_type )
                {
                    // Standard modern V4L2 behavior
                    case V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC:
                        _time_strategy = timestamp_strategy::monotonic;
                        break;
                    // Legacy drivers use CLOCK_REALTIME (epoch)
                    // Note: Subject to NTP adjustments.
                    case V4L2_BUF_FLAG_TIMESTAMP_UNKNOWN:
                        _time_strategy = timestamp_strategy::epoch;
                        break;
                    // Timestamp was copied from some other source. Unknown conversion to
                    // system time so fallback to userspace time to be safe
                    case V4L2_BUF_FLAG_TIMESTAMP_COPY:
                    default:
                        _time_strategy = timestamp_strategy::userspace_fallback;
                        break;
                }
            }

            if( _time_strategy == timestamp_strategy::monotonic )
            {
                static const boost::posix_time::ptime boot_time = comma::timing::kernel::boot_time();
                _buffers[_index].t = boot_time + boost::posix_time::seconds( buffer.timestamp.tv_sec ) + boost::posix_time::microseconds( buffer.timestamp.tv_usec );
            }
            else if( _time_strategy == timestamp_strategy::epoch )
            {
                _buffers[_index].t = comma::timing::epoch_time() + boost::posix_time::seconds( buffer.timestamp.tv_sec ) + boost::posix_time::microseconds( buffer.timestamp.tv_usec );
            }
            else
            {
                _buffers[_index].t = boost::posix_time::microsec_clock::universal_time();
            }
        }
        else
        {
            _buffers[_index].t = boost::posix_time::microsec_clock::universal_time();
        }
        return stream::record( ++_count, _buffers[_index] );
    }
}

} } } // namespace snark { namespace io { namespace video {
