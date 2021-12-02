// Copyright (c) 2016 The University of Sydney

#include <iostream>
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <boost/thread/thread.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/io/select.h>
#include <comma/io/stream.h>
#include <comma/string/string.h>

// To check headers try:
// ntrip-cat -v httpbin.org:80 -m bytes/50 --nmea=nmea-sentence -u user -p pass

static std::string default_port = "2101";
static std::string default_protocol_version = "1.0";

static void bash_completion( unsigned const ac, char const* const* av )
{
    static const char* completion_options =
        " --help -h --verbose -v"
        " --user -u --password -p --mountpoint -m --nmea --protocol --timeout"
        ;

    std::cout << completion_options << std::endl;
    exit( 0 );
}

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "ntrip protocol client" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: ntrip-cat <host[:port]> [<options>]" << std::endl;
    std::cerr << "       echo nmea | ntrip-cat <host[:port]> [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "port defaults to " << default_port << std::endl;
    std::cerr << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --help,-h:                 output help" << std::endl;
    std::cerr << "    --mountpoint,-m=[<mount>]: station id" << std::endl;
    std::cerr << "    --username,-u=[<name>]:    user name" << std::endl;
    std::cerr << "    --password,-p=[<pass>]:    login password" << std::endl;
    std::cerr << "    --timeout=[<secs>]:        exit if no data for timeout secs" << std::endl;
    std::cerr << "    --nmea=[<sentence>]:       send nmea string" << std::endl;
    std::cerr << "    --protocol=[<version>]:    protocol version; default=" << default_protocol_version << std::endl;
    std::cerr << "    --separator-repeats=<repeats>:  how many times to repeat \\r\\n separator; default: 1" << std::endl;
    std::cerr << "                                    you may need 2 in some cases" << std::endl;
    std::cerr << "    --verbose,-v:              more output to stderr" << std::endl;
    std::cerr << std::endl;
    std::cerr << "If required, the nmea sentence can either be defined as a parameter," << std::endl;
    std::cerr << "when it is sent once, or input on stdin." << std::endl;
    std::cerr << std::endl;
    std::cerr << "If mountpoint is not given the sourcetable for the service provider will be" << std::endl;
    std::cerr << "returned. This generally will not require a username and password." << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples:" << std::endl;
    std::cerr << "    --- protocol 1.0 request with single NMEA sentence ---" << std::endl;
    std::cerr << "    ntrip-cat -v www.smartnetaus.com:12103 -m TMBL_NB -u <user> -p <pass>" << std::endl;
    std::cerr << "        --nmea='$GPGGA,062147,3300.0000,S,14900.0000,E,7,00,0,100,M,0,M,,*7C'" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    --- observe the data ---" << std::endl;
    std::cerr << "    ntrip-cat -v <host> -m <mount> -u <user> -p <pass> | hexdump" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    --- download sourcetable ---" << std::endl;
    std::cerr << "    ntrip-cat www.smartnetaus.com:12103" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

std::string base64( std::string s )
{
    using namespace boost::archive::iterators;
    typedef base64_from_binary< transform_width< std::string::const_iterator, 6, 8 > > it_base64_t;

    unsigned int num_pad_chars = ( 3 - s.length() % 3 ) % 3;
    std::string b64( it_base64_t( s.begin() ), it_base64_t( s.end() ));
    b64.append( num_pad_chars, '=' );
    return b64;
}

// Filter out \r before writing to stderr. It confuses journald.
std::string strip_cr( std::string s )
{
    s.erase( std::remove( s.begin(), s.end(), '\r' ), s.end() );
    return s;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--bash-completion" ) ) bash_completion( ac, av );
        bool verbose = options.exists( "--verbose,-v" );
        const std::vector< std::string >& unnamed = options.unnamed( "--verbose,-v", "-.+" );
        if( unnamed.size() != 1 ) { COMMA_THROW( comma::exception, "server required" ); }
        std::vector< std::string > address_components = comma::split( unnamed[0], ':' );
        if( address_components.size() == 1 ) { address_components.push_back( default_port ); }
        address_components.insert( address_components.begin(), "tcp" );
        std::string address = comma::join< std::vector< std::string > >( address_components, ':' );
        std::string mountpoint = options.value< std::string >( "--mountpoint,-m", "" );
        std::string protocol_version = options.value< std::string >( "--protocol", default_protocol_version );
        std::vector< std::string > protocol_version_components = comma::split( protocol_version, '.' );
        unsigned int separator_repeats = options.value( "--separator-repeats", 1 );
        std::string separator_string = "";
        for( unsigned int i = 0; i < separator_repeats; ++i ) { separator_string += "\r\n"; } // uber-quick and dirty for now
        const char* separator = &separator_string[0];
        unsigned int protocol_version_major = boost::lexical_cast< unsigned int >( protocol_version_components[0] );
        comma::signal_flag is_shutdown;
        std::string nmea_line;
        bool done = false;
        while( !is_shutdown && !done )
        {
            comma::io::select select;
            select.read().add( comma::io::stdin_fd );
            comma::verbose << "opening " << address << std::endl;
            comma::io::iostream ios( address );
            select.read().add( ios.fd() );
            std::ostringstream request;
            request << "GET /" << mountpoint << " HTTP/1.1\r\n";
            request << "Host: " << address_components[1] << "\r\n";
            request << "User-Agent: NTRIP client/" << protocol_version << "\r\n";
            if( protocol_version_major > 1 ) { request << "Ntrip-Version: Ntrip/" << protocol_version << "\r\n"; }
            request << "Accept: */*\r\n";
            request << "Connection: close\r\n";
            if( options.exists( "--username,-u" ))
            {
                std::string userpass( options.value< std::string >( "--username,-u" ));
                userpass.push_back( ':' );
                userpass.append( options.value< std::string >( "--password,-p" ));
                request << "Authorization: Basic " << base64( userpass ) << "\r\n";
            }
            if( options.exists( "--nmea" ))
            {
                std::string nmea = options.value< std::string >( "--nmea" );
                if( protocol_version_major > 1 ) { request << "Ntrip-GGA: " << nmea << "\r\n"; }
                else { request << "\r\n" << nmea << "\r\n"; }
            }
            request << "\r\n";
            if( verbose ) { std::cerr << strip_cr( request.str() ) << std::flush; }
            *ios << request.str() << std::flush;
            // If we have an nmea line that we've previously sent we must have
            // lost connection and restarted, in which case, resend the last line.
            if( !nmea_line.empty() )
            {
                if( verbose ) { std::cerr << nmea_line << std::endl; }
                *ios << nmea_line << "\r\n" << std::flush;
            }
            bool ntrip_stream_closed = false;
            bool in_header = true;
            unsigned int total_bytes_read = 0;
            std::vector< char > buffer( 4096 );
            boost::posix_time::ptime last_data_time = boost::posix_time::microsec_clock::universal_time();
            boost::optional< boost::posix_time::time_duration > data_timeout;
            if( options.exists( "--timeout" )) { data_timeout.reset(  boost::posix_time::seconds( options.value< int >( "--timeout" ))); }
            while( !is_shutdown && ios->good() && !ntrip_stream_closed )
            {
                if( data_timeout )
                {
                    // wait until the next timeout
                    select.wait( last_data_time + *data_timeout - boost::posix_time::microsec_clock::universal_time() );

                    if( boost::posix_time::microsec_clock::universal_time() > last_data_time + *data_timeout )
                    {
                        std::cerr << "ntrip-cat: last_data_time=" << last_data_time << std::endl;
                        std::cerr << "ntrip-cat: current time = " << boost::posix_time::microsec_clock::universal_time() << std::endl;
                        std::cerr << "ntrip-cat: timeout = " << *data_timeout << std::endl;
                        std::cerr << "ntrip-cat: timed out waiting for ntrip data" << std::endl;
                        return 1;
                    }
                }
                else
                {
                    select.wait();
                }
                // input ready
                while( select.check() && select.read().ready( comma::io::stdin_fd ) && std::cin.good() )
                {
                    std::getline( std::cin, nmea_line );
                    if( nmea_line.empty() ) { continue; }
                    if( verbose ) { std::cerr << nmea_line << std::endl; }
                    *ios << nmea_line << "\r\n" << std::flush;
                }
                // ntrip data ready
                while( select.check() && select.read().ready( ios.fd() ) && ios->good() )
                {
                    if( data_timeout ) { last_data_time = boost::posix_time::microsec_clock::universal_time(); }
                    std::size_t available = ios.available_on_file_descriptor();
                    if( available == 0 ) { ntrip_stream_closed = true; break; }
                    if( in_header )
                    {
                        unsigned int size = std::min( available, buffer.size() - total_bytes_read );
                        ios->read( &buffer[ total_bytes_read ], size );
                        std::size_t actual_bytes_read = ios->gcount();
                        total_bytes_read += actual_bytes_read;
                        if( actual_bytes_read < size ) { ntrip_stream_closed = true; break; }
                        std::vector< char >::const_iterator separator_start =
                            std::search( buffer.begin(), buffer.begin() + total_bytes_read
                                       , separator, separator + strlen( separator ));
                        if( separator_start != buffer.begin() + total_bytes_read )
                        {
                            in_header = false;
                            // Write out any data that came after the header
                            unsigned int data_start = separator_start - buffer.begin() + strlen( separator );
                            if( verbose )
                            {
                                std::string header( &buffer[0], data_start );
                                std::cerr << strip_cr( header );
                            }
                            // Reset total_bytes_read to now represent data bytes
                            total_bytes_read -= data_start;
                            std::cout.write( &buffer[ data_start ], total_bytes_read );
                        }
                    }
                    else
                    {
                        unsigned int size = std::min( available, buffer.size() );
                        ios->read( &buffer[0], size );
                        std::size_t actual_bytes_read = ios->gcount();
                        std::cout.write( &buffer[0], actual_bytes_read );
                        std::cout.flush();
                        total_bytes_read += actual_bytes_read;
                        if( actual_bytes_read < size ) { ntrip_stream_closed = true; break; }
                    }
                }
            }
            comma::verbose << "shutdown: " << ( is_shutdown ? "Y" : "N" )
                           << "; ios good: " << ( ios->good() ? "Y" : "N" )
                           << "; ntrip stream closed: " << ( ntrip_stream_closed ? "Y" : "N" )
                           << std::endl;
            if( mountpoint.empty() )
            {
                done = true;    // we only want one block of data (the sourcetable)
            }
            else
            {
                // if the ntrip stream closed, wait a short while before we try to re-open it
                if( ntrip_stream_closed ) { boost::this_thread::sleep( boost::posix_time::seconds( 5 )); }
            }
        }
        if( verbose ) { std::cerr << std::endl; }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "ntrip-cat: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "ntrip-cat: unknown exception" << std::endl; }
    return 1;
}
