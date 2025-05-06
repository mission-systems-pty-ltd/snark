// Copyright (c) 2024 Mission Systems Pty Ltd

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <comma/base/exception.h>
#include <comma/application/command_line_options.h>
#include <comma/timing/traits.h>
#include "video.h"

namespace snark { namespace io { namespace video {

static int xioctl( int fd, int request, void* arg )
{
    int r;
    do { r = ::ioctl( fd, request, arg ); } while( r == -1 && errno == EINTR ); // todo? microsleep? select?
    return r;
}

stream::stream( const std::string& name, unsigned int width, unsigned int height, std::vector< void* > user_buffers, int pixel_format )
    : _name( name )
    , _width( width )
    , _height( height )
    , _buffers( user_buffers.size() )
{   
    COMMA_ASSERT(user_buffers.size() != 0, "user_buffers must be provided for IO_USERPTR mode");
    _io_method = IO_USERPTR;

    initialise_stream( name, width, height, user_buffers.size(), pixel_format);
    
    for( unsigned int i = 0; i < user_buffers.size(); ++i )
    {
        _buffers[i].data = user_buffers[i];
    }
    // unsigned int page_size = getpagesize ();
    // _size = (_width * _height + page_size - 1) & ~(page_size - 1); 
    _size = _width * _height;
}

stream::stream( const std::string& name, unsigned int width, unsigned int height, unsigned int number_of_buffers, int pixel_format )
    : _name( name )
    , _width( width )
    , _height( height )
    , _buffers( number_of_buffers )
{
    _io_method = IO_MMAP;

    initialise_stream( name, width, height, number_of_buffers, pixel_format);

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
    if(_io_method == IO_MMAP) 
    {   
        for( auto& buffer: _buffers ) { ::munmap( buffer.data, _size ); }
    }
    std::fclose( _file );
}

void stream::initialise_stream(const std::string& name, const unsigned int width, const unsigned int height, const unsigned int number_of_buffers, const int pixel_format)
{
    _file = std::fopen( &name[0], "r+" );
    COMMA_ASSERT( _file, "failed to open '" << _name << "'" );
    _fd = ::fileno( _file );
    v4l2_capability capability{};
    v4l2_format format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = pixel_format;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    COMMA_ASSERT( xioctl( _fd, VIDIOC_QUERYCAP, &capability ) != -1, "'" << name << "': " << ( errno == EINVAL ? "not a v4l2 device" : "ioctl error: VIDIOC_QUERYCAP" ) );
    COMMA_ASSERT( capability.capabilities & V4L2_CAP_VIDEO_CAPTURE, "'" << name << "': is not a video capture device" );
    if(_io_method == IO_MMAP) { COMMA_ASSERT( capability.capabilities & V4L2_CAP_STREAMING, "'" << name << "': does not support streaming i/o" ); }
    COMMA_ASSERT( xioctl( _fd, VIDIOC_S_FMT, &format ) != -1, "'" << name << "': ioctl error: VIDIOC_S_FMT" );
    v4l2_requestbuffers request_buffers{};
    request_buffers.count = number_of_buffers;
    request_buffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request_buffers.memory = (_io_method == IO_MMAP) ? V4L2_MEMORY_MMAP : V4L2_MEMORY_USERPTR;
    COMMA_ASSERT( xioctl( _fd, VIDIOC_REQBUFS, &request_buffers ) != -1, "'" << name << "': " << ( errno == EINVAL ? "not a v4l2 device" : "ioctl error: VIDIOC_REQBUFS" ) << "; " << ::strerror( errno ) << "(" << errno << ")" );
    COMMA_ASSERT( request_buffers.count == _buffers.size(), "'" << name << "': insufficient buffer memory for " << _buffers.size() << " buffers; maximum available " << request_buffers.count << " buffers" );
}

void stream::start()
{
    if( _started ) { return; }
    for( size_t i = 0; i < _buffers.size(); ++i )
    {
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.index = i;
        switch (_io_method )
        {
            case (IO_MMAP):
            {   
                buffer.memory = V4L2_MEMORY_MMAP;
                break;
            } case (IO_USERPTR):
            {   
                buffer.memory = V4L2_MEMORY_USERPTR;
                buffer.m.userptr = (unsigned long) _buffers[i].data;
                buffer.length = _size;
                break;
            } default:
            {
                COMMA_THROW( comma::exception, "we shouldnt be here at all; _io_method: " << _io_method );
            }
        }
        
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
    using clock = std::chrono::high_resolution_clock;
    std::chrono::time_point<std::chrono::high_resolution_clock> end_total, start_total;
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
        start_total = clock::now();
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = (_io_method == IO_MMAP) ? V4L2_MEMORY_MMAP : V4L2_MEMORY_USERPTR;
        COMMA_ASSERT( xioctl( _fd, VIDIOC_DQBUF, &buffer ) != -1, "'" << _name << "': " << ( errno == EAGAIN ? "EAGAIN" : "VIDIOC_DQBUF" ) << ": " << ::strerror( errno ) << "(" << errno << ")" );
        COMMA_ASSERT( buffer.index < _buffers.size(), "'" << _name << "': expected buffer index less than number of buffers (" << _buffers.size() << "); got: " << buffer.index );
        COMMA_ASSERT( xioctl( _fd, VIDIOC_QBUF, &buffer ) != -1, "'" << _name << "': ioctl error: VIDIOC_QBUF" );
        _index = buffer.index;
        _buffers[_index].t = boost::posix_time::microsec_clock::universal_time();
        end_total = clock::now();
        std::ostringstream oss;
        oss << "read CSI data into buffer " 
                    << std::chrono::duration<double, std::milli>(end_total - start_total).count() 
                    << " ms";
        COMMA_SAY_DEBUG(oss.str());
        return stream::record( ++_count, _buffers[_index] );
    }
}

} } } // namespace snark { namespace io { namespace video {
