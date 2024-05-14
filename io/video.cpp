// Copyright (c) 2024 Mission Systems Pty Ltd

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <comma/base/exception.h>
#include "video.h"

namespace snark { namespace io { namespace video {

static int _ioctl( int fd, int request, void* arg )
{
    int r;
    do { r = ::ioctl( fd, request, arg ); } while( r == -1 && errno == EINTR ); // todo? microsleep? select?
    return r;
}

static void _query_capabilities( int fd, const std::string& name, unsigned int width, unsigned int height )
{
    v4l2_capability capability{};
    v4l2_format format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_SRGGB8;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    COMMA_ASSERT( _ioctl( fd, VIDIOC_QUERYCAP, &capability ) != -1, "'" << name << "': " << ( errno == EINVAL ? "not a v4l2 device" : "ioctl error: VIDIOC_QUERYCAP" ) );
    COMMA_ASSERT( capability.capabilities & V4L2_CAP_VIDEO_CAPTURE, "'" << name << "': is not a video capture device" );
    COMMA_ASSERT( capability.capabilities & V4L2_CAP_STREAMING, "'" << name << "': does not support streaming i/o" );
    COMMA_ASSERT( _ioctl( fd, VIDIOC_S_FMT, &format ) != -1, "'" << name << "': ioctl error: VIDIOC_S_FMT" );
}

static std::FILE* _open( const std::string& name )
{
    std::FILE* f = std::fopen( &name[0], "r+" );
    COMMA_ASSERT( f, "failed to open '" << name << "'" );
    return f;
}

stream::stream( const std::string& name, unsigned int width, unsigned int height, unsigned int number_of_buffers )
    : _name( name )
    , _width( width )
    , _height( height )
    , _file( _open( name ) )
    , _fd( ::fileno( _file ) )
    , _buffers( number_of_buffers )
{
    _query_capabilities( _fd, name, width, height );
    v4l2_requestbuffers request_buffers{};
    request_buffers.count = number_of_buffers;
    request_buffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request_buffers.memory = V4L2_MEMORY_MMAP;
    COMMA_ASSERT( _ioctl( _fd, VIDIOC_REQBUFS, &request_buffers ) != -1, "'" << name << "': " << ( errno == EINVAL ? "not a v4l2 device" : "ioctl error: VIDIOC_REQBUFS" ) << "; " << ::strerror( errno ) << "(" << errno << ")" );
    COMMA_ASSERT( request_buffers.count == _buffers.size(), "'" << name << "': insufficient buffer memory for " << _buffers.size() << " buffers; maximum available " << request_buffers.count << " buffers" );
    for( unsigned int i = 0; i < number_of_buffers; ++i )
    {
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;
        COMMA_ASSERT( _ioctl( _fd, VIDIOC_QUERYBUF, &buffer ) != -1, "'" << name << "': on buffer " << i << ": ioctl error: VIDIOC_QUERYBUF, " << ::strerror( errno ) << "(" << errno << ")" );
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
    for( size_t i = 0; i < _buffers.size(); ++i )
    {
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;
        COMMA_ASSERT( _ioctl( _fd, VIDIOC_QBUF, &buffer ) != -1, "'" << _name << "': ioctl error: VIDIOC_QBUF" );
    }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    COMMA_ASSERT( _ioctl( _fd, VIDIOC_STREAMON, &type ) != -1, "'" << _name << "': ioctl error: VIDIOC_STREAMON" );
}

void stream::stop()
{
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    COMMA_ASSERT( _ioctl( _fd, VIDIOC_STREAMOFF, &type ) != 0, "'" << _name << "': failed to stop streaming: ioctl error: VIDIOC_STREAMOFF" );
}

std::pair< unsigned int, snark::timestamped< void* > > stream::read()
{
    while( true )
    {
        fd_set fds{}; // todo? use comma::select?
        FD_ZERO( &fds );
        FD_SET( _fd, &fds );
        timeval timeout{};
        timeout.tv_sec = 2;
        int r = select( _fd + 1, &fds, nullptr, nullptr, &timeout );
        if( r == -1 )
        {
            if( errno == EINTR ) { continue; } // todo? why continue on interrupted system call? should not we return nullptr instead on signal?
            COMMA_THROW( comma::exception, "'" << _name << ": select error: " << strerror( errno ) << "(" << errno << ")" );
        }
        COMMA_ASSERT( r != 0, "'" << _name << ": select timeout" );
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        COMMA_ASSERT( _ioctl( _fd, VIDIOC_DQBUF, &buffer ) != -1, "'" << _name << "': " << ( errno == EAGAIN ? "EAGAIN" : "VIDIOC_DQBUF" ) << ": " << ::strerror( errno ) << "(" << errno << ")" );
        COMMA_ASSERT( buffer.index < _buffers.size(), "'" << _name << "': expected buffer index less than number of buffers (" << _buffers.size() << "); got: " << buffer.index );
        COMMA_ASSERT( _ioctl( _fd, VIDIOC_QBUF, &buffer ) != -1, "'" << _name << "': ioctl error: VIDIOC_QBUF" );
        _index = buffer.index;
        _buffers[_index].t = boost::posix_time::microsec_clock::universal_time();
        return std::make_pair( ++_count, _buffers[_index] );
    }
}

} } } // namespace snark { namespace io { namespace video {
