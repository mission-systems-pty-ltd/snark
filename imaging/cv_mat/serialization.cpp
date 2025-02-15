// Copyright (c) 2011 The University of Sydney

#include <stdio.h>
#include <sstream>
#include <comma/application/verbose.h>
#include <comma/base/exception.h>
#include <comma/base/last_error.h>
#include <comma/csv/options.h>
#include <comma/csv/traits.h>
#include <comma/name_value/parser.h>
#include <comma/string/string.h>
#include <boost/bimap.hpp>
#include "utils.h"
#include "serialization.h"
#include "traits.h"

namespace snark { namespace cv_mat {

serialization::header::header() : rows( 0 ), cols( 0 ), type( 0 ), size( 0 ) {}

serialization::header::header( const cv::Mat& m ):
    rows( m.rows ),
    cols( m.cols ),
    type( m.type() ),
    size( m.dataend - m.datastart )
{        
}

serialization::header::header( const std::pair< boost::posix_time::ptime, cv::Mat >& p ) :
    timestamp( p.first ),
    rows( p.second.rows ),
    cols( p.second.cols ),
    type( p.second.type() ),
    size( p.second.dataend - p.second.datastart )
{
}

serialization::header::header( const boost::posix_time::ptime& t, const cv::Mat & m ) :
    timestamp( t ),
    rows( m.rows ),
    cols( m.cols ),
    type( m.type() ),
    size( m.dataend - m.datastart )
{
}

serialization::serialization() :
    m_binary( comma::csv::binary< header >( header::default_format(), header::default_fields(), false ) ),
    m_binary_no_timestamp( comma::csv::binary< header >( header::default_format(), ",rows,cols,type", false ) ),
    m_buffer( m_binary->format().size() ),
    m_headerOnly( false )
{
    m_binary->put( m_header, &m_buffer[0]); // make sure that the default timestamp (if not read from input) is not-a-date-time, and not 19700101T00000
}

serialization::serialization( const std::string& fields, const comma::csv::format& format, bool headerOnly, const header& default_header, bool set_timestamp )
    : m_buffer( format.size() )
    , m_headerOnly( headerOnly )
    , m_header( default_header )
    , _set_timestamp( set_timestamp )
{
    if( !fields.empty() )
    { 
        m_binary = comma::csv::binary< header >( format.string(), fields, false, default_header );
        std::vector< std::string > v = comma::split(fields, ',');
        for( unsigned int i = 0; i < v.size(); ++i ) { if( v[i] == "t" ) { v[i] = ""; } }
        std::string no_timestamp_fields = comma::join( v, ',' );
        if( !no_timestamp_fields.empty() ) { m_binary_no_timestamp.reset( comma::csv::binary< header >( format.string(), comma::join( v, ',' ), false, m_header ) );  }
    }
    
    // make sure that the default timestamp (if not read from input) is not-a-date-time, and not 19700101T00000
    ( !fields.empty() ? *m_binary : comma::csv::binary< serialization::header >( format.string(), fields ) ).put( m_header, &m_buffer[0]);
}

serialization::serialization( const serialization::options& options )
    : _set_timestamp( options.timestamp )
{
    if( options.no_header && options.header_only ) { COMMA_THROW( comma::exception, "cannot have no-header and header-only at the same time" ); }
    std::string fields = options.fields.empty() ? header::default_fields() : options.fields;
    std::vector< std::string > v = comma::split( fields, "," );
    comma::csv::format format;
    if( options.format.elements().empty() ) { for( unsigned int i = 0; i < v.size(); ++i ) { if( v[i] == "t" ) { format += "t"; } else { format += "ui"; } } }
    else { format = options.format; } 
    m_buffer.resize( format.size() );
    m_headerOnly = options.header_only;
    m_header = options.get_header();
    if( options.no_header )
    {
        _no_header_binary.reset( comma::csv::binary< header >( "t,3ui", "t,rows,cols,type", false, m_header ) );
    }
    else
    { 
        m_binary.reset( comma::csv::binary< header >( format.string(), fields, false, m_header ) );
        for( unsigned int i = 0; i < v.size(); ++i ) { if( v[i] == "t" ) { v[i] = ""; } } 
        std::string no_timestamp_fields = comma::join( v, ',' );
        if( !no_timestamp_fields.empty() ) { m_binary_no_timestamp.reset( comma::csv::binary< header >( format.string(), no_timestamp_fields, false, m_header ) ); }
    }
    // make sure that the default timestamp (if not read from input) is not-a-date-time, and not 19700101T00000
    ( options.no_header ? comma::csv::binary< serialization::header >( format.string(), fields ) : *m_binary ).put( m_header, &m_buffer[0]);
}

const comma::csv::binary< serialization::header >* serialization::header_binary() const { return m_binary ? &( *m_binary ) : NULL; }

bool serialization::no_header() const { return !static_cast< bool >( m_binary ); }

std::size_t serialization::put( const std::pair< boost::posix_time::ptime, cv::Mat >& p, char* buf ) const
{
    const auto& b = m_binary ? m_binary : _no_header_binary;
    if( b )
    {
        header h( p );
        b->put( h, buf );
    }
    std::size_t size = p.second.dataend - p.second.datastart;
    ::memcpy( buf + m_binary->format().size(), p.second.datastart, size );
    return m_binary->format().size() + size;
}

std::size_t serialization::get( std::pair< boost::posix_time::ptime, cv::Mat >& p, const char* buf ) const
{
    header h = get_header( buf );
    unsigned int headerSize = 0;
    if( m_binary ) { headerSize = m_binary->format().size(); }
    cv::Mat m( h.rows, h.cols, h.type, const_cast< char* >( buf + headerSize ) );
    m.copyTo( p.second );
    return size( p.second );
}

std::pair< boost::posix_time::ptime, cv::Mat > serialization::get( const char* buf ) const
{
    std::pair< boost::posix_time::ptime, cv::Mat > p;
    get( p, buf );
    return p;
}

serialization::header serialization::get_header( const char* buf ) const
{
    if( !m_binary ) { return m_header; }
    header h;
    m_binary->get( h, buf );
    return h;
}

const std::vector< char >& serialization::header_buffer() const { return m_buffer; }

std::size_t serialization::size( const cv::Mat& m ) const
{
    unsigned int headerSize = 0;
    if( m_binary ) { headerSize = m_binary->format().size(); }
    return headerSize + ( m.dataend - m.datastart );
}

std::size_t serialization::size( const std::pair< serialization::header::buffer_t, cv::Mat >& m ) const { return size( m.second ); }

std::size_t serialization::size( const std::pair< boost::posix_time::ptime, cv::Mat >& m ) const { return size( m.second ); }

const std::string& serialization::last_error() const { return _last_error; }

template <> std::pair< serialization::header::buffer_t, cv::Mat > serialization::read< serialization::header::buffer_t >( std::istream& is ) noexcept
{
    try
    {
        std::pair< serialization::header::buffer_t, cv::Mat > p;
        if( m_binary )
        {
            is.read( &m_buffer[0], m_buffer.size() );
            int count = is.gcount();
            if( count <= 0 ) { return p; }
            if( count < int( m_buffer.size() ) ) { COMMA_THROW( comma::exception, "expected " << m_buffer.size() << " bytes, got " << count ); }        
            m_binary->get( m_header, &m_buffer[0] );
        }
        if( _set_timestamp ) // quick and dirty; todo: watch performance!
        {
            m_header.timestamp = boost::posix_time::microsec_clock::universal_time();
            ( m_binary ? m_binary : _no_header_binary )->put( m_header, &m_buffer[0] );
        }
        p.first = m_buffer;
        p.second = cv::Mat( m_header.rows, m_header.cols, m_header.type );
        std::size_t size = p.second.dataend - p.second.datastart;
        // todo: accumulate
        is.read( const_cast< char* >( reinterpret_cast< const char* >( p.second.datastart ) ), size ); // quick and dirty
        int count = is.gcount();
        return count < int( size ) ? std::pair< serialization::header::buffer_t, cv::Mat >() : p;
    }
    catch( std::exception& ex ) { _last_error = ex.what(); }
    catch( ... ) { _last_error = "snark::cv_mat::serialization::read(): unknow exception caught"; }
    return std::pair< serialization::header::buffer_t, cv::Mat >();
}

template <> std::pair< boost::posix_time::ptime, cv::Mat > serialization::read< boost::posix_time::ptime >( std::istream& is ) noexcept
{
    std::pair< serialization::header::buffer_t, cv::Mat > p = read< serialization::header::buffer_t >( is );
    return std::make_pair( _set_timestamp ? boost::posix_time::microsec_clock::universal_time() : m_header.timestamp, p.second );
}

void serialization::write( std::ostream& os, const std::pair< boost::posix_time::ptime, cv::Mat >& m, bool flush )
{
    if( m_binary )
    { 
        m_binary->put( serialization::header( m ), &m_buffer[0] );
        os.write( &m_buffer[0], m_buffer.size() );
    }
    if( !m_headerOnly ) { os.write( reinterpret_cast< const char* >( m.second.datastart ), m.second.dataend - m.second.datastart ); }
    if( flush ) { os.flush(); }
}

void serialization::write( std::ostream& os, const std::pair< header::buffer_t, cv::Mat >& m, bool flush )
{
    if( m_binary )
    {
        m_buffer = m.first;     // This forces the output fields to be the same as the input fields
        if( m_binary_no_timestamp ) { m_binary_no_timestamp->put( serialization::header( m.second ), &m_buffer[0] ); }
        os.write( &m_buffer[0], m_buffer.size() );
    }
    if( !m_headerOnly ) { os.write( reinterpret_cast< const char* >( m.second.datastart ), m.second.dataend - m.second.datastart ); }
    if( flush ) { os.flush(); }
}

// In testing basler-cat with the Pika XC2 camera we saw issues with the write() method above.
//
// Specifically, using a command line like:
//     basler-cat "log=log.bin" > stdout.bin
// we would witness stdout.bin as corrupted, log.bin would be fine.
//
// Note that on the Pika XC2 the frame rate is high (150fps) and the image size large
// (2354176 bytes) which might contribute to the problem. The target was an SSD drive.
//
// In detail:
// * sometimes (every few seconds), the header (20 bytes) would be written but the body would not
// * os.write( body) was called but os.tellp() would indicate that no frames were written
// * os.fail() and os.bad() were always false
// * strangely, writing a large (1M) amount of random data to another diskfile stopped the problem
// * we tried various combinations of flushing data with no success
// * switching from std::cout.write() to write( 1, ... ) was successful
//
// We don't understand why std::cout would exhibit this behaviour so for now we
// will have a second method (below) that uses the c-style write() function.
// This is used by pipeline::write_()

static void write_( int fd, const char* buf, size_t count )
{
    while( count > 0 )
    {
        ssize_t bytes_written = ::write( fd, buf, count );
        if( bytes_written == -1 ) { COMMA_THROW( comma::last_error::exception, "error" ); }
        if( bytes_written == 0 ) { COMMA_THROW( comma::exception, "write() wrote 0 bytes" ); } // shouldn't occur with stdout
        count -= bytes_written;
        buf += bytes_written;
    }
}

void serialization::write_to_stdout(const std::pair< serialization::header::buffer_t, cv::Mat >& m, bool flush)
{
    if( m_binary )
    {
        m_buffer = m.first;     // This forces the output fields to be the same as the input fields
        if( m_binary_no_timestamp ) { m_binary_no_timestamp->put( serialization::header( m.second ), &m_buffer[0] ); }
        write_( 1, &m_buffer[0], m_buffer.size() );
    }
    if( !m_headerOnly ) { write_( 1, reinterpret_cast< const char* >( m.second.datastart ), m.second.dataend - m.second.datastart ); }
    if( flush ) { ::fflush( stdout ); }
}

void serialization::write_to_stdout( const std::pair< boost::posix_time::ptime, cv::Mat >& m, bool flush )
{
    if( m_binary )
    { 
        m_binary->put( serialization::header( m ), &m_buffer[0] );
        write_( 1, &m_buffer[0], m_buffer.size() );
    }
    if( !m_headerOnly ) { write_( 1, reinterpret_cast< const char* >( m.second.datastart ), m.second.dataend - m.second.datastart ); }
    if( flush ) { ::fflush( stdout ); }
}

unsigned type_from_string( const std::string& t )
{
    if( t == "CV_8UC1" || t == "ub" ) { return CV_8UC1; }
    else if( t == "CV_8UC2" || t == "2ub" ) { return CV_8UC2; }
    else if( t == "CV_8UC3" || t == "3ub" ) { return CV_8UC3; }
    else if( t == "CV_8UC4" || t == "4ub" ) { return CV_8UC4; }
    else if( t == "CV_8SC1" || t == "b" ) { return CV_8SC1; }
    else if( t == "CV_8SC2" || t == "2b" ) { return CV_8SC2; }
    else if( t == "CV_8SC3" || t == "3b" ) { return CV_8SC3; }
    else if( t == "CV_8SC4" || t == "4b" ) { return CV_8SC4; }
    else if( t == "CV_16UC1" || t == "uw" ) { return CV_16UC1; }
    else if( t == "CV_16UC2" || t == "2uw" ) { return CV_16UC2; }
    else if( t == "CV_16UC3" || t == "3uw" ) { return CV_16UC3; }
    else if( t == "CV_16UC4" || t == "4uw" ) { return CV_16UC4; }
    else if( t == "CV_16SC1" || t == "w" ) { return CV_16SC1; }
    else if( t == "CV_16SC2" || t == "2w" ) { return CV_16SC2; }
    else if( t == "CV_16SC3" || t == "3w" ) { return CV_16SC3; }
    else if( t == "CV_16SC4" || t == "4w" ) { return CV_16SC4; }
    else if( t == "CV_32SC1" || t == "i" ) { return CV_32SC1; }
    else if( t == "CV_32SC2" || t == "2i" ) { return CV_32SC2; }
    else if( t == "CV_32SC3" || t == "3i" ) { return CV_32SC3; }
    else if( t == "CV_32SC4" || t == "4i" ) { return CV_32SC4; }
    else if( t == "CV_32FC1" || t == "f" ) { return CV_32FC1; }
    else if( t == "CV_32FC2" || t == "2f" ) { return CV_32FC2; }
    else if( t == "CV_32FC3" || t == "3f" ) { return CV_32FC3; }
    else if( t == "CV_32FC4" || t == "4f" ) { return CV_32FC4; }
    else if( t == "CV_64FC1" || t == "d" ) { return CV_64FC1; }
    else if( t == "CV_64FC2" || t == "2d" ) { return CV_64FC2; }
    else if( t == "CV_64FC3" || t == "3d" ) { return CV_64FC3; }
    else if( t == "CV_64FC4" || t == "4d" ) { return CV_64FC4; }
    COMMA_THROW( comma::exception, "expected type, got '" << t << "'" );
}

// std::string type_as_string( int type ) { return type_as_string( type ); }

std::string format_from_type( unsigned type )
{
    switch( type )
    {
        case CV_8UC1: return std::string( "ub" );
        case CV_8UC2: return std::string( "2ub" );
        case CV_8UC3: return std::string( "3ub" );
        case CV_8UC4: return std::string( "4ub" );
        case CV_8SC1: return std::string( "b" );
        case CV_8SC2: return std::string( "2b" );
        case CV_8SC3: return std::string( "3b" );
        case CV_8SC4: return std::string( "4b" );
        case CV_16UC1: return std::string( "uw" );
        case CV_16UC2: return std::string( "2uw" );
        case CV_16UC3: return std::string( "3uw" );
        case CV_16UC4: return std::string( "4uw" );
        case CV_16SC1: return std::string( "w" );
        case CV_16SC2: return std::string( "2w" );
        case CV_16SC3: return std::string( "3w" );
        case CV_16SC4: return std::string( "4w" );
        case CV_32SC1: return std::string( "i" );
        case CV_32SC2: return std::string( "2i" );
        case CV_32SC3: return std::string( "3i" );
        case CV_32SC4: return std::string( "4i" );
        case CV_32FC1: return std::string( "f" );
        case CV_32FC2: return std::string( "2f" );
        case CV_32FC3: return std::string( "3f" );
        case CV_32FC4: return std::string( "4f" );
        case CV_64FC1: return std::string( "d" );
        case CV_64FC2: return std::string( "2d" );
        case CV_64FC3: return std::string( "3d" );
        case CV_64FC4: return std::string( "4d" );
    }
    COMMA_THROW( comma::exception, "type: '" << type << "' is not valid" );
}

std::string all_image_types()
{
    std::stringstream stream;
    stream << "CV_8UC1,ub," << CV_8UC1 << "\n";
    stream << "CV_8UC2,2ub," << CV_8UC2 << "\n";
    stream << "CV_8UC3,3ub," << CV_8UC3 << "\n";
    stream << "CV_8UC4,4ub," << CV_8UC4 << "\n";
    stream << "CV_8SC1,b," << CV_8SC1 << "\n";
    stream << "CV_8SC2,2b," << CV_8SC2 << "\n";
    stream << "CV_8SC3,3b," << CV_8SC3 << "\n";
    stream << "CV_8SC4,4b," << CV_8SC4 << "\n";
    stream << "CV_16UC1,uw," << CV_16UC1 << "\n";
    stream << "CV_16UC2,2uw," << CV_16UC2 << "\n";
    stream << "CV_16UC3,3uw," << CV_16UC3 << "\n";
    stream << "CV_16UC4,4uw," << CV_16UC4 << "\n";
    stream << "CV_16SC1,w," << CV_16SC1 << "\n";
    stream << "CV_16SC2,2w," << CV_16SC2 << "\n";
    stream << "CV_16SC3,3w," << CV_16SC3 << "\n";
    stream << "CV_16SC4,4w," << CV_16SC4 << "\n";
    stream << "CV_32SC1,i," << CV_32SC1 << "\n";
    stream << "CV_32SC2,2i," << CV_32SC2 << "\n";
    stream << "CV_32SC3,3i," << CV_32SC3 << "\n";
    stream << "CV_32SC4,4i," << CV_32SC4 << "\n";
    stream << "CV_32FC1,f," << CV_32FC1 << "\n";
    stream << "CV_32FC2,2f," << CV_32FC2 << "\n";
    stream << "CV_32FC3,3f," << CV_32FC3 << "\n";
    stream << "CV_32FC4,4f," << CV_32FC4 << "\n";
    stream << "CV_64FC1,d," << CV_64FC1 << "\n";
    stream << "CV_64FC2,2d," << CV_64FC2 << "\n";
    stream << "CV_64FC3,3d," << CV_64FC3 << "\n";
    stream << "CV_64FC4,4d," << CV_64FC4 << "\n";
    return stream.str();
}

serialization::header serialization::options::get_header() const
{
    header h;
    h.cols = cols;
    h.rows = rows;
    h.size = cols * rows;
    if( !type.empty() ) { h.type = type_from_string( type ); }
    return h;
}

std::string serialization::options::usage()
{
    std::stringstream stream;
    stream << "cv::mat serialization options:\n";
    stream << "    ';'-separated name=value options\n";
    stream << "    fields=<fields>; default: t,cols,rows,type\n";
    stream << "        note: shall describe only the image header, not the data\n";
    stream << "    binary=<format>; default: t,3ui\n";
    stream << "        note: shall describe only the image header, not the data\n";
    stream << "    header-only: if present, output only header (output only)\n";
    stream << "    no-header: if present, no header\n";
    stream << "    header: if present, image has header\n";
    stream << "            e.g. if input has no header, but you want output to have header,\n";
    stream << "            run something like: cv-cat --input 'rows=400;cols=600;type=ub;no-header' --output 'header'\n";
    stream << "    rows=<rows>: default number of rows (input only)\n";
    stream << "    cols=<cols>: default number of columns (input only)\n";
    stream << "    type=<type>: default image type (input only)\n";
    stream << "    timestamp: if present, set to timestamp to current UTC time\n";
    stream << type_usage();
    return stream.str();
}

std::string serialization::options::type_usage()
{
    std::stringstream stream;
    stream << "    image types (see opencv cv::Mat and cxtypes.hpp):\n";
    stream << "        CV_8UC1  or ub  (" << CV_8UC1 << ")\n";
    stream << "        CV_8UC2  or 2ub (" << CV_8UC2 << ")\n";
    stream << "        CV_8UC3  or 3ub (" << CV_8UC3 << ")\n";
    stream << "        CV_8UC4  or 4ub (" << CV_8UC4 << ")\n";
    stream << "        CV_8SC1  or b   (" << CV_8SC1 << ")\n";
    stream << "        CV_8SC2  or 2b  (" << CV_8SC2 << ")\n";
    stream << "        CV_8SC3  or 3b  (" << CV_8SC3 << ")\n";
    stream << "        CV_8SC4  or 4b  (" << CV_8SC4 << ")\n";
    stream << "        CV_16UC1 or uw  (" << CV_16UC1 << ")\n";
    stream << "        CV_16UC2 or 2uw (" << CV_16UC2 << ")\n";
    stream << "        CV_16UC3 or 3uw (" << CV_16UC3 << ")\n";
    stream << "        CV_16UC4 or 4uw (" << CV_16UC4 << ")\n";
    stream << "        CV_16SC1 or w   (" << CV_16SC1 << ")\n";
    stream << "        CV_16SC2 or 2w  (" << CV_16SC2 << ")\n";
    stream << "        CV_16SC3 or 3w  (" << CV_16SC3 << ")\n";
    stream << "        CV_16SC4 or 4w  (" << CV_16SC4 << ")\n";
    stream << "        CV_32SC1 or i   (" << CV_32SC1 << ")\n";
    stream << "        CV_32SC2 or 2i  (" << CV_32SC2 << ")\n";
    stream << "        CV_32SC3 or 3i  (" << CV_32SC3 << ")\n";
    stream << "        CV_32SC4 or 4i  (" << CV_32SC4 << ")\n";
    stream << "        CV_32FC1 or f   (" << CV_32FC1 << ")\n";
    stream << "        CV_32FC2 or 2f  (" << CV_32FC2 << ")\n";
    stream << "        CV_32FC3 or 3f  (" << CV_32FC3 << ")\n";
    stream << "        CV_32FC4 or 4f  (" << CV_32FC4 << ")\n";
    stream << "        CV_64FC1 or d   (" << CV_64FC1 << ")\n";
    stream << "        CV_64FC2 or 2d  (" << CV_64FC2 << ")\n";
    stream << "        CV_64FC3 or 3d  (" << CV_64FC3 << ")\n";
    stream << "        CV_64FC4 or 4d  (" << CV_64FC4 << ")\n";
    return stream.str();
}

static serialization::options _handle_fields_and_format( const comma::csv::options& csv, serialization::options s )
{
    COMMA_ASSERT( csv.fields.empty() || s.fields.empty(), "expected --fields or --input/--output=...,fields,...; got both" );
    COMMA_ASSERT( !csv.binary() || s.format.elements().empty(), "please set binary format in --binary or --input/--output, not both");
    if( !csv.fields.empty() && s.fields.empty() ) { s.fields = csv.fields; }
    if( csv.binary() && s.format.string().empty() ) { s.format = csv.format(); }
    return s;
}

std::pair< serialization::options, serialization::options > serialization::options::make( const comma::command_line_options& o, bool ignore_csv_options )
{
    const serialization::options parsed = comma::name_value::parser( ';', '=' ).get< serialization::options >( o.value< std::string >( "--input", "" ) );
    serialization::options input_options = ignore_csv_options ? parsed : _handle_fields_and_format( comma::csv::options( o ), parsed );
    std::string output_options_string = o.value< std::string >( "--output", "" );
    serialization::options output_options = output_options_string.empty() ? input_options : comma::name_value::parser( ';', '=' ).get< serialization::options >( output_options_string );
    if( input_options.no_header && !output_options.fields.empty() && input_options.fields != output_options.fields ) { COMMA_ASSERT( output_options.fields == serialization::header::default_fields(), "when --input has no-header option, --output fields can only be fields='" << serialization::header::default_fields() << "', got: '" << output_options.fields << "'" ); }
    else { COMMA_ASSERT( output_options.fields.empty() || input_options.fields == output_options.fields, "customised output header fields not supported (todo); got: input fields: '" << input_options.fields << "' output fields: '" << output_options.fields << "'" ); }
    if( output_options.fields.empty() ) { output_options.fields = input_options.fields; } // output fields and format will be empty when the user specifies only --output no-header or --output header-only
    COMMA_ASSERT( output_options.format.elements().empty() || input_options.format.string() == output_options.format.string(), "customised output header format not supported (todo); got: input format: '" << input_options.format.string() << "' output format: '" << output_options.format.string() << "'" );
    if( output_options.format.elements().empty() ) { output_options.format = input_options.format; };
    return { input_options, output_options };
}

std::pair< serialization, serialization > serialization::make( const comma::command_line_options& o, bool ignore_csv_options )
{
    const auto& p = serialization::options::make( o, ignore_csv_options );
    return { serialization( p.first ), serialization( p.second ) };
}

} } // namespace snark{ namespace cv_mat {
