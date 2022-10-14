// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2017 The University of Sydney
// Copyright (c) 2021,2022 Mission Systems Pty Ltd

#include "device.h"
#include <comma/application/verbose.h>

namespace snark { namespace navigation { namespace advanced_navigation {

device::device( const std::string& name, const advanced_navigation::options& options )
    : buf( 2600 ), index( 0 ), head( 0 ), msg_header( NULL )
{
    if( name.find("/dev") == 0 ) { stream.reset( new serial_stream( name, options )); }
    else if( name == "-" ) { stream.reset( new io_stream< comma::io::istream >( name )); }
    else { stream.reset( new io_stream< comma::io::iostream >( name )); }
}

comma::io::file_descriptor device::fd() { return stream->fd(); }

/*
  |                       buf                        |
  |     |       |<---   remaining_buffer_space   --->|
      head    index

  index is the end of the current data
  head points to the start of the message header when we find it

  remaining_buffer_space is the remaining space in the buffer (or: buf-index)
  when the index goes past halfway through the buffer the data from head to
  index is moved to the start of the buffer
*/
void device::debug_internal_state()
{
    comma::verbose << "head=" << head << "; index=" << index << ": got header: " << ( msg_header ? "yes" : "no" ) << std::endl;
}

// try to read at least "at_least" bytes, but not more than "at_most" bytes
void device::read_data( unsigned int at_least, unsigned int at_most )
{
    unsigned int read_size = stream->read_some( &buf[index], at_most, at_least );
    //comma::verbose << "device::process() read " << read_size << " bytes" << std::endl;
    if( read_size == 0 ) { return; }
    // this should never happen but read_some() might not enforce not reading more than at_most size
    if( read_size > at_most ) { std::cerr << "read long " << read_size << " vs " << at_most << std::endl; }
    index += read_size;
}

void device::process()
{
    static messages::header* skipper = NULL;
    static unsigned int debug_count = 0;

    //debug_internal_state();

    if( head > 0 && index > buf.size() / 2 )
    {
        if( index - head > 0 )
        {
            // relocate
            memmove( &buf[0], &buf[head], index - head );
            index -= head;
            head = 0;
        }
        else
        {
            index = head = 0;
        }
        msg_header = NULL;
    }
    unsigned int remaining_buffer_space = buf.size() - index;
    if( remaining_buffer_space > 0 )
    {
        // if we have a header, attempt to read the rest of the message
        // otherwise, read enough to get a header but no more (so we don't read in to the following header)
        unsigned int to_read = msg_header
                             ? msg_header->len() - ( index - head - messages::header::size )
                             : messages::header::size * 2;
        //comma::verbose << ( msg_header ? "already have" : "don't have" ) << " header, reading " << to_read << " bytes" << std::endl;
        read_data( to_read, remaining_buffer_space );
    }
    // Keep looping through reading messages until the index lies within a header (no more messages)
    // In practice there should always be only one message, because that's how we're reading the data above.
    while( index >= head + messages::header::size )
    {
        if( !msg_header )
        {
            // find the header by stepping through the buffer looking for a valid sequence of bytes
            for( ; head + messages::header::size <= index; head++ )
            {
                msg_header = reinterpret_cast< messages::header* >( &buf[head] );
                if( msg_header->is_valid() )
                {
                    break;
                }
                if( !skipper )
                    skipper = msg_header;
                debug_count++;
                msg_header = NULL;
            }
        }
        if( msg_header )
        {
            //comma::verbose << "found message header for " << msg_header->len() << " byte payload" << std::endl;
            //debug_internal_state();
            unsigned int msg_start = head + messages::header::size;

            if( msg_start + msg_header->len() > index )
            {
                // we don't have the whole message yet, try to read the rest
                unsigned int remaining_bytes = msg_header->len() - ( index - head - messages::header::size );
                read_data( remaining_bytes, remaining_buffer_space );
            }

            if( msg_header->check_crc( &buf[msg_start] ))
            {
                handle_raw( msg_header, &buf[msg_start], msg_header->len() );
                comma::verbose << "got message id " << static_cast< unsigned int >( msg_header->id() ) << std::endl;
                switch( msg_header->id() )
                {
                case messages::acknowledgement::id:
                    handle( reinterpret_cast< messages::acknowledgement* >( &buf[msg_start] ));
                    break;
                case messages::system_state::id:
                    handle( reinterpret_cast< messages::system_state* >( &buf[msg_start] ));
                    break;
                case messages::position_standard_deviation::id:
                    handle( reinterpret_cast< messages::position_standard_deviation* >( &buf[msg_start] ));
                    break;
                case messages::velocity_standard_deviation::id:
                    handle( reinterpret_cast< messages::velocity_standard_deviation* >( &buf[msg_start] ));
                    break;
                case messages::orientation_standard_deviation::id:
                    handle( reinterpret_cast< messages::orientation_standard_deviation* >( &buf[msg_start] ));
                    break;
                case messages::raw_sensors::id:
                    handle( reinterpret_cast< messages::raw_sensors* >( &buf[msg_start] ));
                    break;
                case messages::satellites::id:
                    handle( reinterpret_cast< messages::satellites* >( &buf[msg_start] ));
                    break;
                case messages::filter_options::id:
                    handle( reinterpret_cast< messages::filter_options* >( &buf[msg_start] ));
                    break;
                case messages::magnetic_calibration_status::id:
                    handle( reinterpret_cast< messages::magnetic_calibration_status* >( &buf[msg_start] ));
                    break;
                default:
//                     comma::verbose<<"unhandled msg id: "<<int(msg_header->id())<<" len "<<msg_header->len()<<" "<<head<<" "<<index<<std::endl;
                    break;
                }
                if( debug_count )
                {
                    if( !skipper )
                        comma::verbose << " skipped " << debug_count << std::endl;
                    else
                        comma::verbose << " skipped " << debug_count << "; " << (unsigned int)( skipper->LRC() ) << " "
                                       << (unsigned int)( skipper->id() ) << " " << skipper->len() << std::endl;
                    debug_count = 0;
                    skipper = NULL;
                }
            }
            else
            {
                comma::verbose << "crc failed " << (unsigned int)( msg_header->LRC() ) << " "
                               << (unsigned int)( msg_header->id() ) << " " << msg_header->len() << std::endl;
            }
            head += msg_header->len() + messages::header::size;
            msg_header = NULL;
        }
    }
}

void device::send_ntrip( const std::vector<char>& buf )
{
//     comma::verbose<<"send_ntrip "<<buf.size()<<std::endl;
    unsigned int index = 0;
    while( index < buf.size() )
    {
        unsigned int size = std::min< unsigned int >( buf.size() - index, 255 );
        messages::rtcm_corrections msg( &buf[index], size );
        index += size;
//         comma::verbose<<"rtcm_corrections "<<size<<std::endl;
        std::size_t to_write = size + messages::header::size;
        std::size_t written = stream->write( msg.data(), to_write );
        if( written != to_write ) { std::cerr << "writing ntrip msg failed (expected " << to_write << " actual " << written << " )" << std::endl; }
    }
}

void device::send( const messages::command& command )
{
    comma::verbose << "sending message id " << static_cast< unsigned int >( command.header.id() ) << std::endl;
    std::size_t to_write = command.header.len() + messages::header::size;
    std::size_t written = stream->write( command.data(), to_write );
    if( written != to_write )
    {
        std::cerr << "writing command msg failed (expected " << to_write << " actual " << written
                  << " id " << (unsigned int)command.header.id() << ")" << std::endl;
    }
}

} } } //namespace snark { namespace navigation { namespace advanced_navigation {
