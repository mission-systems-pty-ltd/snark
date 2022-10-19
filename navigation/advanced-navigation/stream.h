// Copyright (c) 2017 The University of Sydney
// Copyright (c) 2022 Mission Systems Pty Ltd

#pragma once

#include <comma/io/stream.h>
#include <boost/asio.hpp>

namespace snark { namespace navigation { namespace advanced_navigation {

class eois_exception : public std::runtime_error
{
public:
    eois_exception( const std::string& msg ) : std::runtime_error( msg ) {}
};

/// stream/device options for advanced navigation
struct options
{
    int baud_rate;
    options( int baud_rate=115200 ) : baud_rate( baud_rate ) {}
};

struct stream
{
    // buf_size: size of buffer (max read size)
    // read_size: may block until read at least this much (min read size)
    // return: number of bytes read; may return 0 without reading anything
    virtual std::size_t read_some( char* buf, std::size_t buf_size, std::size_t read_size=0 ) = 0;
    virtual std::size_t write( const char* buf, std::size_t size ) = 0;
    virtual comma::io::file_descriptor fd() = 0;
};

struct serial_stream : public stream
{
    boost::asio::io_service service;
    boost::asio::serial_port port;
    serial_stream( const std::string& name, const advanced_navigation::options& options );
    std::size_t read_some( char* buf, std::size_t buf_size, std::size_t read_size );
    std::size_t write( const char* buf, std::size_t to_write );
    comma::io::file_descriptor fd();
};

// invoke as:
// io_stream< comma::io::iostream > for tcp/udp streams
// io_stream< comma::io::istream >  for stdin streams
template< typename T >
struct io_stream : public stream
{
    T ios;

    io_stream( const std::string& name )
        : ios( name, comma::io::mode::binary, comma::io::mode::non_blocking )
    {}

    std::size_t read_some( char* buf, std::size_t buf_size, std::size_t read_size )
    {
        if( !ios->good() ) { throw eois_exception( std::string( "end of file on iostream ") + ios.name() ); }
        ios->read( buf, read_size );
        return ios->gcount();
    }

    std::size_t write( const char* buf, std::size_t to_write );

    comma::io::file_descriptor fd() { return ios.fd(); }
};

} } } //namespace snark { namespace navigation { namespace advanced_navigation {
