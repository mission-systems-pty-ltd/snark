// Copyright (c) 2011 The University of Sydney

#include <iostream>
#include <memory>
#include <unordered_set>
#include <vector>
#include <boost/functional/hash.hpp>
#include <boost/test/utils/nullstream.hpp>
#include <comma/application/command_line_options.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/name_value/parser.h>
#include <comma/csv/options.h>
#include <comma/string/split.h>
#include "../cv_mat/serialization.h"

static const char* app_name="image-to-csv";
static boost::onullstream nullstream;
static std::ostream* cverbose=&nullstream;
static snark::cv_mat::serialization::options input_options;
static snark::cv_mat::serialization::header header;
static comma::csv::options csv;
static std::size_t channels = 3;
static int depth = 0;
static std::vector< std::vector< float > > discard;
static std::vector< std::vector< char > > discard_pixels;
static bool discard_any;
static bool unique;

struct app_t
{
    virtual ~app_t(){}
    virtual void output_fields() = 0;
    virtual void output_format() = 0;
    virtual bool process_image() = 0;
};

template< typename T >
struct output_t: public app_t
{
    boost::posix_time::ptime t;
    comma::uint32 block;
    comma::uint32 x;
    comma::uint32 y;
    std::vector< T > channels;
    output_t() : block( 0 ), x( 0 ), y( 0 ), channels( ::channels ) {}
    virtual void output_fields();
    virtual void output_format();
    virtual bool process_image();
};

namespace comma { namespace visiting {

template < typename T > struct traits< output_t< T > >
{
    template < typename K, typename V > static void visit( const K&, const output_t< T >& r, V& v )
    {
        v.apply( "t", r.t );
        v.apply( "block", r.block );
        v.apply( "x", r.x );
        v.apply( "y", r.y );
        v.apply( "channels", r.channels );
    }
};

} } // namespace comma { namespace visiting {

static void usage( bool detail )
{
    std::cerr << std::endl;
    std::cerr << "read images with header from stdin and output pixels x,y and data to stdout" << std::endl;
    if( !detail ) { std::cerr << "see --help --verbose for more details" << std::endl; }
    std::cerr << std::endl;
    std::cerr<< "usage: " << app_name << " <options>" << std::endl
                    << std::endl
                    << "output fields" << std::endl
                    << "    t: input image timestamp" << std::endl
                    << "    x,y: pixel coordinates" << std::endl
                    << "    channels: image channels; can be defined individually, e.g. as channels[0],channels[1],channels[2]" << std::endl
                    << "    block: image number" << std::endl
                    << "    default: t,x,y,channels" << std::endl
                    << std::endl
                    << "options" << std::endl
                    << "    --help,-h: show help" << std::endl
                    << "    --verbose,-v: show detailed messages" << std::endl
                    << "    --discard=[<pixel-value>]: discard given pixel values, e.g:" << std::endl
                    << "                                --discard=-1, or --discard=0,0,255, or --discard='255,0,0;0,255,0;0,0,255'" << std::endl
                    << "    --discard-any: makes sense only for multichannel images" << std::endl
                    << "                   example: for --discard=-1,0,0, discard pixels" << std::endl
                    << "                            with first channel equal to -1 or second channel" << std::endl
                    << "                            equal to 0 or third channel equal to 0" << std::endl
                    << "                   todo: support empty values" << std::endl
                    << "    --discard-zero,--output-non-zero: output only values with non-zero data, same as --discard=0" << std::endl
                    << "    --input: input options if image has no header (see details)" << std::endl
                    << "    --output-fields: output fields and exit need to feed in image header through stdin or specify --input options" << std::endl
                    << "    --output-format: output format string and exit need to feed in image header through stdin or specify --input options" << std::endl
                    << "    --output-header-fields: output header fields and exit" << std::endl
                    << "    --output-header-format: output header format and exit" << std::endl
                    << "    --some,--unique,-u: output unique pixel for each colour present in image" << std::endl
                    << std::endl
                    << "deprecated" << std::endl
                    << "    --channels=<n>: number of channels, example: " << std::endl
                    << "        deprecated: --input=\"no-header;type=0\" --channels=3" << std::endl
                    << "        new usage: --input=\"no-header;type=3ub\"" << std::endl
                    << "    " << std::endl
                    << std::endl;
    std::cerr << "serialization and csv options" << std::endl;
    if( detail )
    {
        std::cerr<< snark::cv_mat::serialization::options::usage() << std::endl << std::endl;
        std::cerr<< "output stream csv options:" << std::endl << csv.usage() << std::endl << std::endl;
    }
    else
    {
        std::cerr << "    run image-to-csv --help --verbose for more..." << std::endl;
    }
    std::cerr << std::endl;
    std::cerr << "example" << std::endl << "      cv-cat --file image.jpg | " << app_name << " > pixels.csv" << std::endl << std::endl;
    exit( 0 );
}

static void read_header()
{
    comma::csv::options csvh;
    csvh.full_xpath = false;
    csvh.fields=input_options.fields;
    csvh.format(comma::csv::format::value< snark::cv_mat::serialization::header >(input_options.fields, false));
    comma::csv::input_stream<snark::cv_mat::serialization::header> is(std::cin, csvh);
    const snark::cv_mat::serialization::header* p=is.read();
    if( !p ) { exit( 0 ); }
    header = *p;
}

template< typename T > void output_t<T>::output_fields() { std::cout<<comma::join( comma::csv::names< output_t< T > >( csv.fields ), ','  ) << std::endl; }

template< typename T > void output_t<T>::output_format() { std::cout<<comma::csv::format::value< output_t< T > >( csv.fields, false ) << std::endl; }

template< typename T >
static void set_discard_pixels()
{
    if( discard.empty() ) { return; }
    discard_pixels.resize( discard.size() );
    for( unsigned int i = 0; i < discard.size(); ++i )
    {
        discard_pixels[i].resize( sizeof( T ) * channels );
        for( unsigned int j = 0; j < channels; ++j ) { *( reinterpret_cast< T* >( &discard_pixels[i][ j * sizeof( T ) ] ) ) = static_cast< T >( discard[i][j] ); }
    }
}

template < typename T >
struct vector_hash
{
    std::size_t operator()( const std::vector< T >& v ) const
    {
        std::size_t seed = 0;
        for( auto e: v ) { boost::hash_combine( seed, e ); }
        return seed;
    }
};

template< typename T >
static bool keep( const char* p )
{
    static unsigned int size = sizeof( T ) * channels;
    if( unique )
    {
        static std::unordered_set< std::vector< T >, vector_hash< T > > colours;
        static std::vector< T > pixel( channels );
        std::memcpy( reinterpret_cast< char* >( &pixel[0] ), p, size ); // todo: quick and dirty for now; watch performance
        if( colours.find( pixel ) != colours.end() ) { return false; }
        colours.insert( pixel );
    }
    if( discard.empty() ) { return true; }
    if( discard_any )
    {
        for( const auto& d: discard_pixels )
        {
            for( unsigned int i = 0; i < size; i += sizeof( T ) )
            {
                if( std::memcmp( p + i, &d[i], sizeof( T ) ) == 0 ) { return false; }
            }
        }
        return true;
    }
    for( const auto& d: discard_pixels ) { if( std::memcmp( p, &d[0], d.size() ) == 0 ) { return false; } }
    return true;
}

template< typename T >
bool output_t< T >::process_image()
{
    static comma::csv::output_stream< output_t< T > > os( std::cout, csv );
    static output_t< T > out;
    out.channels.resize( ::channels );
    out.t = header.timestamp;
    std::size_t size = ::channels * sizeof( T );
    set_discard_pixels< T >();
    static std::vector< char > buffer;
    buffer.resize( header.cols * size );
    for( out.y = 0; out.y < header.rows && std::cin.good(); ++out.y )
    {
        std::cin.read( &buffer[0], buffer.size() );
        if( std::size_t( std::cin.gcount() ) != buffer.size() ) { return false; }
        char* ptr = &buffer[0];
        for( out.x = 0; out.x < header.cols; ++out.x )
        {
            if( keep< T >( ptr ) )
            {
                std::memcpy( &out.channels[0], ptr, size );
                os.write( out );
            }
            ptr += size;
        }
    }
    if( csv.flush ) { os.flush(); }
    ++out.block;
    return true;
}

struct get_app
{
    app_t* app;
    get_app(): app( NULL )
    {
        switch( depth )
        {
            case CV_8U: app = new output_t< unsigned char >(); break;
            case CV_8S: app = new output_t< signed char >(); break;
            case CV_16U: app = new output_t< comma::uint16 >(); break;
            case CV_16S: app = new output_t< comma::int16 >(); break;
            case CV_32S: app = new output_t< comma::int32 >(); break;
            case CV_32F: app = new output_t< float >(); break;
            case CV_64F: app = new output_t< double >(); break;
            default: COMMA_THROW( comma::exception, "expected depth; got: " << depth );
        }
    }
    ~get_app() { if( app != NULL ) { delete app; } }
};

static void process()
{
    #ifdef WIN32
    _setmode( _fileno( stdin ), _O_BINARY );
    #endif
    while( std::cin.good() && std::cin.peek() != -1 )
    {
        if( !input_options.no_header ) 
        {
            read_header();
            channels = CV_MAT_CN( header.type );
            depth = CV_MAT_DEPTH( header.type );
        }
        if( !discard.empty() && discard[0].size() > 1 && discard[0].size() < channels ) { std::cerr << "image-to-csv: input image has " << channels << " channels, but --discard specifies only " << discard.size() << std::endl; }
        *cverbose << app_name << ": timestamp: " << header.timestamp << " cols: " << header.cols << " rows: " << header.rows << " type: " << header.type << std::endl;
        *cverbose << app_name << ": channels: " << channels << " depth: " << depth << std::endl;
        get_app g; //we need new app_t because channel/depth may change after reading header
        if( !g.app->process_image() ) { break; }
    }
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        cverbose =  options.exists( "--verbose,-v" ) ? &std::cerr : &nullstream;
        input_options = comma::name_value::parser( ';', '=' ).get< snark::cv_mat::serialization::options >( options.value< std::string >( "--input", "" ) );
        if( input_options.fields.empty() ) { input_options.fields = "t,rows,cols,type"; }
        csv = comma::csv::options( options );
        if( csv.fields.empty() ) { csv.fields = "t,x,y,channels"; }
        csv.full_xpath = false;
        header = input_options.get_header();
        channels = options.value< int >( "--channels", CV_MAT_CN( header.type ) );
        depth = CV_MAT_DEPTH( header.type );
        unique = options.exists( "--unique,-u" );
        discard_any = options.exists( "--discard-any" );
        *cverbose << app_name << ": type: " << header.type << " channels: " << channels << " depth: " << int( CV_MAT_DEPTH( header.type ) ) << std::endl;
        std::vector< std::string > unnamed = options.unnamed( "--output-fields,--output-format,--verbose,-v,--output-header-fields,--output-header-format,--discard-zero,--output-non-zero,--flush,--some,--unique,-u,--discard-any", 
                                                              "--input,--channels,--binary,-b,--fields,-f,--precision,--quote,--delimiter,-d,--discard");
        if( !unnamed.empty() ) { std::cerr<<"invalid option"<<((unnamed.size()>1)?"s":"")<<": "<< comma::join(unnamed, ',') <<std::endl; return 1; }
        get_app g;
        if( options.exists( "--output-fields") ) { g.app->output_fields(); return 0; }
        if( options.exists( "--output-format") ) { g.app->output_format(); return 0; }
        if( options.exists( "--output-header-fields") ) { std::cout<<comma::join( comma::csv::names< snark::cv_mat::serialization::header >( input_options.fields ), ','  ) << std::endl; return 0; }
        if( options.exists( "--output-header-format") ) { std::cout<<comma::csv::format::value< snark::cv_mat::serialization::header >( input_options.fields, false ) << std::endl; return 0; }
        options.assert_mutually_exclusive( "--discard-zero,--output-non-zero", "--discard" );
        if( options.exists( "--discard-zero,--output-non-zero" ) ) { discard.push_back( std::vector< float >( 4, 0 ) ); }
        const auto& d = comma::split( options.value< std::string >( "--discard", "" ), ';', true );
        for( const auto& v: d )
        { 
            discard.push_back( comma::split_as< float >( v, ',' ) );
            if( discard.back().size() == 1 ) { discard.back() = std::vector< float >( 4, discard.back()[0] ); } // quick and dirty
        }
        process();
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "image-to-csv: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "image-to-csv: unknown exception" << std::endl; }
    return 1;
}
