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


#include <iostream>
#include <type_traits>
#include <boost/array.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>
#include <boost/optional.hpp>
#include <boost/scoped_ptr.hpp>
#include <comma/base/exception.h>
#include <comma/base/last_error.h>
#include <comma/io/select.h>
#include <comma/string/string.h>
#include "commands.h"
#include "packet.h"
#include "protocol.h"

namespace snark { namespace quickset { namespace ptcr {

class transport // todo: quick and dirty; decouple later, once needed
{
    public:
        transport( const std::string& name )
        {
            std::vector< std::string > v = comma::split( name, ':' );
            if( v[0] == "tcp" )
            {
                if( v.size() != 3 ) { COMMA_THROW( comma::exception, "expected tcp:address:port, got " << name ); }
                boost::asio::ip::tcp::resolver resolver( service_ );
                boost::asio::ip::tcp::resolver::query query( v[1], v[2] );
                boost::asio::ip::tcp::resolver::iterator it = resolver.resolve( query );
                socket_.reset( new boost::asio::ip::tcp::socket( service_ ) );
                socket_->connect( it->endpoint() );
                socket_->set_option( boost::asio::ip::tcp::no_delay( true ) );
                if( !socket_->is_open() ) { COMMA_THROW( comma::exception, "failed to open socket " << name ); }
            }
            else
            {
                serial_.reset( new boost::asio::serial_port( service_, name ) );
                serial_->set_option( boost::asio::serial_port::baud_rate() ); // autobaud
                serial_->set_option( boost::asio::serial_port::parity( boost::asio::serial_port::parity::none ) );
                serial_->set_option( boost::asio::serial_port::stop_bits( boost::asio::serial_port::stop_bits::one ) );
                serial_->set_option( boost::asio::serial_port::character_size( 8 ) );
                if( !serial_->is_open() ) { COMMA_THROW( comma::exception, "failed to open serial port " << name ); }
            }
        }

        void close() { if( serial_ ) { serial_->close(); } else { socket_->close(); } };

        comma::io::file_descriptor fd() const { return serial_ ? serial_->native_handle() : socket_->native_handle(); }

    private:
        boost::asio::io_service service_;
        boost::scoped_ptr< boost::asio::serial_port > serial_;
        boost::scoped_ptr< boost::asio::ip::tcp::socket > socket_;
};

class protocol::impl
{
    public:
        impl( const std::string& name )
            : transport_( name )
        {
            select_.read().add( transport_ );
        }

        template < typename C >
        const packet< typename C::response >* send( const C& command, bool debug )
        {
            if( command_pending ) { COMMA_THROW( comma::exception, "cannot send command 0x" << std::hex << ( 0xff & C::id ) << std::dec << " since command 0x" << std::hex << ( 0xff & *command_pending ) << std::dec << " is pending" ); }
            static_assert( packet< C >::size < 64, "expected size less than 64" );
            static_assert( packet< typename C::response >::size < 64, "expected size less than 64" );
            packet< C > p( command );
            escape_( p );
            if( debug ) print_buf_( "send", txbuf_.data(), txbuf_.size(), constants::etx );
            send_();
            command_pending = C::id;
            bool ok = receive_();
            command_pending.reset();
            if( debug ) print_buf_( "recv", rxbuf_.data(), rxbuf_.size(), constants::etx );
            if( !ok ) { return NULL; }
            if( !unescape_< packet< typename C::response > >() ) { return NULL; }
            const packet< typename C::response >* r = reinterpret_cast< const packet< typename C::response >* >( &response_[0] );
            if( r->packet_header.id() != C::id ) { return NULL; } 
            if( r->packet_footer.etx() != constants::etx ) { return NULL; }
            if( !r->packet_footer.footer_lrc.ok() ) { return NULL; } 
            return r;
        }

        bool receive( boost::posix_time::time_duration timeout ) { return receive_( timeout ); }

        void close() { transport_.close(); }

    private:
        boost::optional< unsigned char > command_pending;
        transport transport_;
        boost::array< char, 128 > txbuf_; // quick and dirty: arbitrary size
        std::size_t txsize_;
        boost::array< char, 128 > rxbuf_; // quick and dirty: arbitrary size
        boost::array< char, 64 > response_; // quick and dirty: arbitrary size
        comma::io::select select_;

        static boost::array< bool, 256 > init_escaped()
        {
            boost::array< bool, 256 > e;
            for( unsigned int i = 0; i < e.size(); e[ i++ ] = false );
            e[ constants::ack ] = true;
            e[ constants::nak ] = true;
            e[ constants::stx ] = true;
            e[ constants::etx ] = true;
            e[ constants::esc ] = true;
            return e;
        }

        static boost::array< bool, 256 > escaped_;

        template < typename P >
        void escape_( const P& p )
        {
            txbuf_[0] = constants::stx;
            const char* begin = p.packet_header.address.data();
            const char* end = p.packet_footer.etx.data();
            char* cur = &txbuf_[1];
            for( const char* s = begin; s < end; ++s, ++cur )
            {
                if( escaped_[ static_cast< unsigned char >( *s ) ] )
                {
                    *cur++ = constants::esc;
                    *cur = ( *s ) | 0x80;
                }
                else
                {
                    *cur = *s;
                }
            }
            *cur++ = constants::etx;
            txsize_ = cur - &txbuf_[0];
        }

        template < typename P >
        bool unescape_()
        {
            if( rxbuf_[0] != constants::ack && rxbuf_[0] != constants::nak ) { return false; }
            char* cur = &response_[0];
            for( const char* s = &rxbuf_[0]; *s != constants::etx; ++s, ++cur )
            {
                *cur = *s == constants::esc ? *( ++s ) & 0x7f : *s;
                if( *s == constants::esc ) { std::cerr << std::hex << "--> unescaped " << ( 0xff & *s ) << std::dec << std::endl; }
            }
            *cur = constants::etx;
            return true;
        }

        void send_()
        {
            const char* begin = &txbuf_[0];
            const char* end = &txbuf_[0] + txsize_;
            unsigned int sent = 0;
            for( const char* t = begin; t < end; )
            {
                int s = ::write( transport_.fd(), t, txsize_ - sent );
                if( s < 0 ) { COMMA_THROW( comma::exception, "failed to send: " << comma::last_error::to_string() ); }
                t += s;
                sent += s;
            }
        }

        // todo: quick and dirty, reading 1 byte at time may be slow - watch
        bool receive_( boost::posix_time::time_duration timeout = boost::posix_time::seconds( 1 ))
        {
            for( char* cur = &rxbuf_[0]; select_.wait( timeout ) != 0; ++cur )
            {
                int r = ::read( transport_.fd(), cur, 1 );
                if( r < 0 ) { COMMA_THROW( comma::exception, "connection failed" ); }
                if( r == 0 ) { COMMA_THROW( comma::exception, "connection closed" ); }
                if( *cur == constants::etx ) { return true; } // quick and dirty, don't care for ack/nak byte for now
            }
            return false;
        }

        void print_buf_( const char* label, char* buf, size_t size, char stop_char )
        {
            fprintf( stderr, "%s:", label );
            for( size_t i = 0; i < size; i++ )
            {
                fprintf( stderr, " %.2x", static_cast<unsigned char>( buf[i] ));
                if( buf[i] == stop_char ) break;
            }
            fprintf( stderr, "\n" );
        }
};

boost::array< bool, 256 > protocol::impl::escaped_ = protocol::impl::init_escaped();

protocol::protocol( const std::string& name ) : pimpl_( new impl( name ) ) {}

protocol::~protocol() { close(); delete pimpl_; }

void protocol::close() { pimpl_->close(); }

template < typename C >
const packet< typename C::response >* protocol::send( const C& command, bool debug )
{
    return pimpl_->send( command, debug );
}

template const packet< commands::get_status::response >* protocol::send< commands::get_status >( const commands::get_status&, bool debug );
template const packet< commands::move_to::response >* protocol::send< commands::move_to >( const commands::move_to&, bool debug );
template const packet< commands::move_to_delta::response >* protocol::send< commands::move_to_delta >( const commands::move_to_delta&, bool debug );
template const packet< commands::get_limits::response >* protocol::send< commands::get_limits >( const commands::get_limits&, bool debug );
template const packet< commands::set_limits::response >* protocol::send< commands::set_limits >( const commands::set_limits&, bool debug );
template const packet< commands::set_camera::response >* protocol::send< commands::set_camera >( const commands::set_camera&, bool debug );

bool protocol::receive( boost::posix_time::time_duration timeout ) { return pimpl_->receive( timeout ); }

} } } // namespace snark { namespace quickset { namespace ptcr {
