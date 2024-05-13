// Copyright (c) 2024 Mission Systems Pty Ltd

#pragma once

#include <string>
#include <vector>
#include <boost/date_time/posix_time/ptime.hpp>
#include "../timing/timestamped.h"

namespace snark { namespace io { namespace video {

class stream
{
    public:
        stream( const std::string& name, unsigned int width, unsigned int height, unsigned int number_of_buffers );
        ~stream();
        const unsigned int width() const { return _width; }
        const unsigned int height() const { return _height; }
        const snark::timestamped< void* >& current() const { return _buffers[_index]; }
        unsigned int count() const { return _count; }
        void start();
        void stop();
        void read_once();

    private:
        std::string _name;
        unsigned int _width{0};
        unsigned int _height{0};
        std::FILE* _file{nullptr};
        int _fd{0};
        std::vector< snark::timestamped< void* > > _buffers;
        unsigned int _size{0};
        unsigned int _index{0};
        unsigned int _count{0};

};

} } } // namespace snark { namespace io { namespace video {
