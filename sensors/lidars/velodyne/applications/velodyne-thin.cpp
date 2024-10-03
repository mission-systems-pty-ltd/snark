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

#include <cstring>
#include <pcap.h>
#include <boost/array.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/optional.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/shared_ptr.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <comma/csv/ascii.h>
#include <comma/io/server.h>
#include <comma/math/compare.h>
#include <comma/name_value/parser.h>
#include <comma/string/string.h>
#include "../../../../timing/time.h"
#include "../../../../visiting/traits.h"
#include "../stream.h"
#include "../thin/thin.h"
#include "../impl/pcap_reader.h"
#include "../impl/proprietary_reader.h"
#include "../impl/stream_reader.h"
#include "../impl/stream_traits.h"
#include "../impl/udp_reader.h"
#include "../thin/scan.h"

using namespace snark;

static void usage()
{
    std::cerr << std::endl;
    std::cerr << "Takes velodyne packets on stdin and outputs thinned packets to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Usage: cat velodyne*.bin | velodyne-thin <options>" << std::endl;
    std::cerr << "       netcat shrimp.littleboard 12345 | velodyne-thin <options>" << std::endl;
    std::cerr << std::endl;
    std::cerr << "velodyne options" << std::endl;
    std::cerr << "    --db <velodyne db.xml file>: default /usr/local/etc/db.xml" << std::endl;
    std::cerr << std::endl;
    std::cerr << "data flow options" << std::endl;
    std::cerr << "    --output-raw: if present, output uncompressed thinned packets" << std::endl;
    std::cerr << "    --pcap: if present, velodyne data is read from pcap packets" << std::endl;
    std::cerr << "             e.g: cat velo.pcap | velodyne-thin <options> --pcap" << std::endl;
    std::cerr << "    --proprietary,-q : read velodyne data directly from stdin using the proprietary protocol" << std::endl;
    std::cerr << "        <header, 16 bytes><timestamp, 12 bytes><packet, 1206 bytes><footer, 4 bytes>" << std::endl;
    std::cerr << "    default input format: <timestamp, 8 bytes><packet, 1206 bytes>" << std::endl;
    std::cerr << "    --publish=<address>: if present, publish on given address (see io-publish -h for address syntax)" << std::endl;
    std::cerr << "    --verbose,-v" << std::endl;
    std::cerr << std::endl;
    std::cerr << "filtering options" << std::endl;
    std::cerr << "    --udp-port <port>: if present, read raw velodyne packets from udp and timestamp them" << std::endl;
    std::cerr << "    --rate <rate>: thinning rate between 0 and 1" << std::endl;
    std::cerr << "                    default 1: send all valid datapoints" << std::endl;
    std::cerr << "    --scan-rate <rate>: scan thin rate between 0 and 1" << std::endl;
    std::cerr << "    --focus --region <options>: focus on particular region" << std::endl;
    std::cerr << "        <options>" << std::endl;
    std::cerr << "            sector: e.g: sector;range=10;bearing=0;ken=30" << std::endl;
    std::cerr << "                    default: bearing: 0, ken: 360" << std::endl;
    std::cerr << "            extents;<min>,<max>: e.g: extents;0,0,0,10,10,5" << std::endl;
    std::cerr << "        examples" << std::endl;
    std::cerr << "            e.g. at choosen --rate in the direction of" << std::endl;
    std::cerr << "            0 degrees 30 degrees wide not farther than 10 metres" << std::endl;
    std::cerr << "            output 80% points in the focus region and 20% the rest" << std::endl;
    std::cerr << "            --focus=\"sector;range=10;bearing=0;ken=30;ratio=0.8\"" << std::endl;
    std::cerr << std::endl;
    exit( -1 );
}

static bool verbose = false;
static bool outputRaw = false;
static boost::optional< float > rate;
static boost::optional< double > scan_rate;
static boost::optional< double > angularSpeed_;
static boost::optional< velodyne::hdl64::db > db;
static boost::scoped_ptr< velodyne::thin::focus > focus;
static velodyne::thin::scan scan;
static boost::scoped_ptr< comma::io::oserver > publisher;

// todo: quick and dirty
#if (BOOST_VERSION >= 106600)
static boost::scoped_ptr< boost::asio::io_context > publisher_udp_service;
#else
static boost::scoped_ptr< boost::asio::io_service > publisher_udp_service;
#endif
static boost::scoped_ptr< boost::asio::ip::udp::socket > publisher_udp_socket;
static unsigned int udp_port;
boost::asio::ip::udp::endpoint udp_destination;

static double angularSpeed( const snark::velodyne::hdl64::packet& packet )
{
    if( angularSpeed_ ) { return *angularSpeed_; }
    double da = double( packet.blocks[0].rotation() - packet.blocks[11].rotation() ) / 100;
    double dt = snark::velodyne::impl::time_span();
    return da / dt;
}

static velodyne::thin::focus* make_focus( const std::string& options, double rate ) // quick and dirty
{
    std::string type = comma::name_value::map( options, "type" ).value< std::string >( "type" );
    double ratio = comma::name_value::map( options ).value( "ratio", 1.0 );
    velodyne::thin::region* region;
    if( type == "sector" ) { region = new velodyne::thin::sector( comma::name_value::parser().get< velodyne::thin::sector >( options ) ); }
    else if( type == "extents" ) { region = new velodyne::thin::extents( comma::csv::ascii< snark::math::closed_interval< double, 3 > >().get( comma::split( options, ';' )[1] ) ); }
    else { COMMA_THROW( comma::exception, "expected type (sector), got " << type ); }
    velodyne::thin::focus* focus = new velodyne::thin::focus( rate, ratio );
    focus->insert( 0, region );
    return focus;
}

template < typename S >
void run( S* stream )
{
    static const unsigned int timeSize = 12;
    boost::mt19937 generator;
    boost::uniform_real< float > distribution( 0, 1 );
    boost::variate_generator< boost::mt19937&, boost::uniform_real< float > > random( generator, distribution );
    comma::uint64 count = 0;
    comma::uint64 dropped_count = 0;
    double compression = 0;
    velodyne::hdl64::packet packet;
    comma::signal_flag isShutdown;
    velodyne::scan_tick tick;
    comma::uint32 scan_id = 0;
    while( !isShutdown && std::cin.good() && !std::cin.eof() && std::cout.good() && !std::cout.eof() )
    {
        const char* p = velodyne::impl::stream_traits< S >::read( *stream, sizeof( velodyne::hdl64::packet ) );
        if( p == NULL ) { break; }
        std::memcpy( ( char* )( &packet ), p, velodyne::hdl64::packet::size );
        boost::posix_time::ptime timestamp = stream->timestamp();
        if( tick.is_new_scan( packet, timestamp ).first ) { ++scan_id; } // quick and dirty
        if( scan_rate ) { scan.thin( packet, *scan_rate, angularSpeed( packet ) ); }
        if( !scan_rate || !scan.empty() )
        {
            if( focus ) { velodyne::thin::thin( packet, *focus, *db, angularSpeed( packet ), random ); }
            if( rate ) { velodyne::thin::thin( packet, *rate, random ); }
        }
        const boost::posix_time::ptime base( snark::timing::epoch );
        const boost::posix_time::time_duration d = timestamp - base;
        comma::int64 seconds = d.total_seconds();
        comma::int32 nanoseconds = static_cast< comma::int32 >( d.total_microseconds() % 1000000 ) * 1000;
        if( outputRaw ) // real quick and dirty
        {
            static boost::array< char, 16 + timeSize + velodyne::hdl64::packet::size + 4 > buf;
            static const boost::array< signed char, 2 > start = {{ -78, 85 }}; // see QLib::Bytestreams::GetDefaultStartDelimiter()
            static const boost::array< signed char, 2 > end = {{ 117, -97 }}; // see QLib::Bytestreams::GetDefaultStartDelimiter()
            ::memcpy( &buf[0], &start[0], 2 );
            ::memcpy( &buf[0] + buf.size() - 2, &end[0], 2 );
            ::memcpy( &buf[0] + 16, &seconds, 8 );
            ::memcpy( &buf[0] + 16 + 8, &nanoseconds, 4 );
            ::memcpy( &buf[0] + 16 + 8 + 4, &packet, velodyne::hdl64::packet::size );
            if( publisher ) { publisher->write( &buf[0], buf.size() ); }
            else if( publisher_udp_socket ) { publisher_udp_socket->send_to( boost::asio::buffer( &buf[0], buf.size() ), udp_destination ); }
            else { std::cout.write( &buf[0], buf.size() ); }
        }
        else
        {
            // todo: certainly rewrite with the proper header using comma::packed
            static char buf[ timeSize + sizeof( comma::uint16 ) + velodyne::thin::maxBufferSize ];
            comma::uint16 size = velodyne::thin::serialize( packet, buf + timeSize + sizeof( comma::uint16 ), scan_id );
            bool empty = size == ( sizeof( comma::uint32 ) + 1 ); // todo: atrocious... i.e. packet is not empty; refactor!!!
            if( !empty )
            {
                size += timeSize;
                ::memcpy( buf, &size, sizeof( comma::uint16 ) );
                size += sizeof( comma::uint16 );
                ::memcpy( buf + sizeof( comma::uint16 ), &seconds, sizeof( comma::int64 ) );
                ::memcpy( buf + sizeof( comma::uint16 ) + sizeof( comma::int64 ), &nanoseconds, sizeof( comma::int32 ) );
                if( publisher ) { publisher->write( buf, size ); }
                else if( publisher_udp_socket ) { publisher_udp_socket->send_to( boost::asio::buffer( buf, size ), udp_destination ); }
                else { std::cout.write( buf, size ); }
            }
            else
            {
                ++dropped_count;
            }
            if( verbose )
            {
                ++count;
                compression = 0.9 * compression + 0.1 * ( empty ? 0.0 : double( size + sizeof( comma::int16 ) ) / ( velodyne::hdl64::packet::size + timeSize ) );
                if( count % 10000 == 0 ) { std::cerr << "velodyne-thin: processed " << count << " packets; dropped " << ( double( dropped_count ) * 100. / count ) << "% full packets; compression rate " << compression << std::endl; }
            }
        }
    }
    if( publisher ) { publisher->close(); }
    std::cerr << "velodyne-thin: " << ( isShutdown ? "signal received" : "no more data" ) << "; shutdown" << std::endl;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av );
        if( options.exists( "--help,-h" ) ) { usage(); }
        outputRaw = options.exists( "--output-raw" );
        rate = options.optional< float >( "--rate" );

        scan_rate = options.optional< double >( "--scan-rate" );

        if( options.exists( "--publish" ) )
        {
            std::string how = options.value< std::string >( "--publish" );
            if( comma::split( how, ':' )[0] == "udp" ) // quick and dirty
            {
                udp_port = boost::lexical_cast< unsigned short >( comma::split( how, ':' )[1] );
#if (BOOST_VERSION >= 106600)
                publisher_udp_service.reset( new boost::asio::io_context() );
#else
                publisher_udp_service.reset( new boost::asio::io_service() );
#endif
                publisher_udp_socket.reset( new boost::asio::ip::udp::socket ( *publisher_udp_service, boost::asio::ip::udp::v4() ) );
                boost::system::error_code error;
                publisher_udp_socket->set_option( boost::asio::ip::udp::socket::broadcast( true ), error );
                if( error ) { std::cerr << "velodyne-thin: failed to set broadcast option on port " << udp_port << std::endl; return 1; }
                publisher_udp_socket->set_option( boost::asio::ip::udp::socket::reuse_address( true ), error );
                if( error ) { std::cerr << "velodyne-thin: failed to set reuse address option on port " << udp_port << std::endl; return 1; }
                udp_destination = boost::asio::ip::udp::endpoint( boost::asio::ip::address_v4::broadcast(), udp_port );
            }
            else
            {
                publisher.reset( new comma::io::oserver( how, comma::io::mode::binary ) );
            }
        }
        options.assert_mutually_exclusive( "--focus,--region,--subtract-by-age,--subtract-max-range,--subtract" );
        if( options.exists( "--focus,--region,--subtract-by-age,--subtract-max-range,--subtract" ) )
        {
            db = velodyne::hdl64::db( options.value< std::string >( "--db", "/usr/local/etc/db.xml" ) );
        }
        if( options.exists( "--focus,--region" ) )
        {
            focus.reset( make_focus( options.value< std::string >( "--focus,--region" ), rate ? *rate : 1.0 ) );
            std::cerr << "velodyne-thin: rate in focus: " << focus->rate_in_focus() << "; rate out of focus: " << focus->rate_out_of_focus() << "; coverage: " << focus->coverage() << std::endl;
        }
        verbose = options.exists( "--verbose,-v" );
        #ifdef WIN32
        _setmode( _fileno( stdin ), _O_BINARY );
        _setmode( _fileno( stdout ), _O_BINARY );
        #endif
        options.assert_mutually_exclusive( "--pcap,--udp-port,--proprietary,-q" );
        boost::optional< unsigned short > port = options.optional< unsigned short >( "--udp-port" );
        if( port ) { run( new snark::udp_reader( *port ) ); }
        else if( options.exists( "--pcap" ) ) { run( new snark::pcap_reader ); }
        else if( options.exists( "--proprietary,-q" ) )
        {
            run( new snark::proprietary_reader );
        }
        else
        {
            run( new snark::stream_reader );
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "velodyne-thin: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "velodyne-thin: unknown exception" << std::endl; }
    usage();
}
