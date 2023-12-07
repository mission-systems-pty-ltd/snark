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


#include <cmath>
#include <cstring>
#include <comma/base/exception.h>
#include "../../../../timing/ntp.h"
#include "packets.h"

namespace snark {  namespace sick { namespace ibeo {
    
boost::array< unsigned char, 4 > header::sentinel_value = { { 0xAF, 0xFE, 0xC0, 0xC2 } };

header::header() { std::memcpy( ( char* )( &sentinel ), ( const char* )( &sentinel_value[0] ), sizeof( comma::uint32 ) ); }
    
bool header::valid() const { return std::memcmp( ( char* )( sentinel.data() ), ( char* )( &sentinel_value[0] ), sizeof( comma::uint32 ) ) == 0; }

double scan::angle_as_radians( short angle ) const { return ( M_PI * ( 2 * angle ) ) / scan_header.steps(); }

double scan::angle_as_radians( const scan::point& point ) const { return angle_as_radians( static_cast< comma::int16 >( point.angle() ) ); }

unsigned int scan::point::id::echo() const { return ( ( data()[0] ) & 0xf0 ) >> 4; }

unsigned int scan::point::id::layer() const { return ( data()[0] ) & 0x0f; }

double scan::point::elevation() const
{
    // elevation angles are middles of the aperture for each of 4 planes
    static double step = 0.8 * M_PI / 180;
    static double base = -step * 3 / 2;
    static boost::array< double, 4 > elevations = { { base, base + step, base + 2 * step, base + 3 * step } };
    return elevations[ id.layer() ];
}

scan::point* scan::points() { return reinterpret_cast< scan::point* >( scan_header.data() + scan::header::size ); }

const scan::point* scan::points() const { return reinterpret_cast< const scan::point* >( scan_header.data() + scan::header::size ); }

scan::timestamps::timestamps( const scan& scan )
    : m_scan( scan )
    , m_start( snark::timing::from_ntp_time( scan.scan_header.start.seconds(), scan.scan_header.start.fractions() ) )
    , m_finish( snark::timing::from_ntp_time( scan.scan_header.finish.seconds(), scan.scan_header.finish.fractions() ) )
    , m_elapsed( m_finish - m_start )
    , m_start_angle( static_cast< short >( scan.scan_header.start_angle() ) )
    , m_finish_angle( static_cast< short >( scan.scan_header.finish_angle() ) )
    , m_diff( m_finish_angle - m_start_angle )
    , m_steps( scan.scan_header.steps() )
{
    if( m_diff < 0 ) { m_diff += m_steps; }
}

boost::posix_time::ptime scan::timestamps::operator[]( std::size_t i ) const
{
    short diff = static_cast< short >( m_scan.points()[i].angle() );
    if( diff < 0 ) { diff += m_steps; }
    return m_start + m_elapsed * ( double( diff ) / m_diff );
}

std::size_t commands::response_header::payload_size() const
{
    switch( id() & 0x3fff )
    {
        case get_status::id: return get_status::response::size;
        case save_configuration::id: return save_configuration::response::size;
        case set::id: return set::response::size;
        case get::id: return get::response::size;
        case reset::id: return reset::response::size;
        case start::id: return start::response::size;
        case stop::id: return stop::response::size;
        case set_ntp_seconds::id: return set_ntp_seconds::response::size;
        case set_ntp_fractions::id: return set_ntp_fractions::response::size;
		default: COMMA_THROW( comma::exception, "expected reponse id, got 0x" << std::hex << ( id() & 0x3fff ) << std::dec );
    };
}

bool fault::fatal() const { return faultRegister1() != 0 || faultRegister2() != 0; }

std::ostream& operator<<( std::ostream& os, const fault& rhs )
{
    // todo: quick and dirty, interpret bits
    if( rhs.fatal() )
    { 
        os << "faults: " << std::hex
           << ( 0xff & rhs.faultRegister1.data()[0] ) << " "
           << ( 0xff & rhs.faultRegister1.data()[1] ) << " "
           << ( 0xff & rhs.faultRegister2.data()[0] ) << " "
           << ( 0xff & rhs.faultRegister2.data()[1] ) << " " << std::dec;
    }
    os << "warnings: " << std::hex
       << ( 0xff & rhs.warningRegister1.data()[0] ) << " "
       << ( 0xff & rhs.warningRegister1.data()[1] ) << " "
       << ( 0xff & rhs.warningRegister2.data()[0] ) << " "
       << ( 0xff & rhs.warningRegister2.data()[1] ) << std::dec;
    return os;
}

std::ostream& operator<<( std::ostream& os, const commands::get_status::response& rhs )
{
    os << "firmware-version=" << std::hex << ( rhs.firmwareVersion() >> 24 ) << "." << ( ( rhs.firmwareVersion() & 0x0f00 ) >> 24 ) << "." << ( rhs.firmwareVersion() & 0x00ff ) << std::dec;
    os << ",fpga-version=" << std::hex << ( rhs.fpgaVersion() >> 24 ) << "." << ( ( rhs.fpgaVersion() & 0x0f00 ) >> 24 ) << "." << ( rhs.fpgaVersion() & 0x00ff ) << std::dec;
    // todo: scanner status
    os << ",temperature=" << ( ( 579.2364 - rhs.temperature() ) / 3.63 );
    os.width( 4 );
    os << ",serial-number=" << std::hex << rhs.serialNumber0() << std::dec;
    os.width();
    os << ",serial-number-counter=" << rhs.serialNumber1();
    // todo: os << ",fpga-version-date=" << // boost::array< Packed::LittleEndianUInt16, 3 > fpgaVersionDate; // cryptic: YYYY MM DD HH MM FPGA S
    // todo: os << ",fpga-version-date=" << // boost::array< Packed::LittleEndianUInt16, 3 > dspVersionDate; // cryptic: YYYY MM DD HH MM
    return os;
}

std::ostream& operator<<( std::ostream& os, const commands::get::response& rhs )
{
    switch( rhs.index() )
    {
        case commands::set::ip_address:
            os << "address=" << static_cast< unsigned int >( static_cast< unsigned char >( rhs.value.data()[3] ) ) << "."
                             << static_cast< unsigned int >( static_cast< unsigned char >( rhs.value.data()[2] ) ) << "."
                             << static_cast< unsigned int >( static_cast< unsigned char >( rhs.value.data()[1] ) ) << "."
                             << static_cast< unsigned int >( static_cast< unsigned char >( rhs.value.data()[0] ) );
            break;
        case commands::set::tcp_port:
            os << "port=" << rhs.value(); // test
            break;
        case commands::set::subnet_mask:
            os << "subnet=" << ( unsigned int )( rhs.value.data()[3] ) << "." << ( unsigned int )( rhs.value.data()[2] ) << "." << ( unsigned int )( rhs.value.data()[1] ) << "." << ( unsigned int )( rhs.value.data()[0] );
            break;
        case commands::set::gateway:
            os << "gateway=" << rhs.value(); // debug
            break;
        case commands::set::data_output_flag:
            os << "flags=" << rhs.value(); // debug
            break;
        default:
            COMMA_THROW( comma::exception, "expected parameter type, got 0x" << std::hex << rhs.index() << std::dec );
    }
    return os;
}

commands::set_ntp_seconds::set_ntp_seconds() {}

commands::set_ntp_seconds::set_ntp_seconds( comma::uint32 s ) { seconds = s; }

commands::set_ntp_fractions::set_ntp_fractions() {}

commands::set_ntp_fractions::set_ntp_fractions( comma::uint32 f ) { fractions = f; }

} } } // namespace snark {  namespace sick { namespace ibeo {
