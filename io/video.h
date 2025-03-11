// Copyright (c) 2024 Mission Systems Pty Ltd

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <linux/videodev2.h>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/optional.hpp>
#include <comma/base/none.h>
#include "../timing/timestamped.h"

namespace snark { namespace io { namespace video {

class stream
{
    public:
        struct record
        {
            std::uint32_t count{0};
            snark::timestamped< void* > buffer{nullptr};

            record( std::uint32_t count = 0, void* ptr = nullptr ): count( count ), buffer( ptr ) {}
            record( std::uint32_t count, const snark::timestamped< void* >& buffer ): count( count ), buffer( buffer ) {}
            operator bool() const { return buffer.data != nullptr; }
        };
        stream( const std::string& name, unsigned int width, unsigned int height, std::vector< snark::timestamped< void* > >* user_buffers, int pixel_format ); // USERPTR constructor
        stream( const std::string& name, unsigned int width, unsigned int height, unsigned int number_of_buffers, int pixel_format = V4L2_PIX_FMT_Y16 ); // IOMAP constructor
        ~stream();
        const unsigned int width() const { return _width; }
        const unsigned int height() const { return _height; }
        unsigned int count() const { return _count; }
        void start();
        void stop();
        record read( float timeout = 1, unsigned int attempts = 1 );
        const std::vector< snark::timestamped< void* > >& buffers() const 
        {   
            if(_buffers == nullptr) { COMMA_THROW(comma::exception, "_buffers is nullptr"); }
            return *_buffers; 
        }

    private:
        enum io_method_t { IO_MMAP, IO_USERPTR }; // todo: add support for DMABUF?
        io_method_t _io_method = IO_MMAP;   // preserve existing functionality by assuming IO_MMAP is default
        std::string _name;
        unsigned int _width{0};
        unsigned int _height{0};
        std::FILE* _file{nullptr};
        int _fd{-1};
        std::vector< snark::timestamped< void* > >* _buffers{nullptr};
        unsigned int _size{0};
        unsigned int _index{0};
        unsigned int _count{0};
        bool _started{false};

        void initialise_stream( const std::string& name, const unsigned int width, const unsigned int height, const unsigned int number_of_buffers, const int pixel_format );


};

} } } // namespace snark { namespace io { namespace video {
