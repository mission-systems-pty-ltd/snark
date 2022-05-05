// Copyright (c) 2011 The University of Sydney
// Copyright (c) 2022 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <memory>
#include <Eigen/Core>
#include <comma/application/command_line_options.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/name_value/serialize.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include "../../../../visiting/eigen.h"
#include "../packets.h"
#include "../traits.h"

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "convert las (laser exchange format) to csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "cat points.las | las-to-csv <what> [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "what" << std::endl;
    std::cerr << "    points: output points" << std::endl;
    std::cerr << "    header: output header as json and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: help; --help --verbose: more help" << std::endl;
    std::cerr << "    --no-offset; do not apply cartesian offset to points" << std::endl;
    std::cerr << "    --output-fields: output fields for a given point format (0-5) and exit; if --type not given, las data on stdin used" << std::endl;
    std::cerr << "    --output-format: output format for a given point format (0-5) and exit; if --type not given, las data on stdin used" << std::endl;
    std::cerr << "    --type,--point-format=<point format>: enforce point format" << std::endl;
    std::cerr << "    --verbose,-v: more output" << std::endl;
    if( verbose ) { std::cerr << comma::csv::options::usage() << std::endl; }
    std::cerr << std::endl;
    exit( 0 );
}

struct color
{
    comma::uint16 red;
    comma::uint16 green;
    comma::uint16 blue;
    color() : red( 0 ), green( 0 ), blue( 0 ) {};
    color( comma::uint16 red, comma::uint16 green, comma::uint16 blue ) : red( red ), green( green ), blue( blue ) {};
};

template < unsigned int I > struct point;

template <> struct point< 0 >
{
    Eigen::Vector3d coordinates;
    comma::uint16 intensity;
    unsigned char return_number;
    unsigned char number_of_returns;
    bool scan_direction;
    bool edge_of_flight_line;
    unsigned char classification;
    char scan_angle;
    unsigned char user_data;
    comma::uint16 point_source_id;

    point< 0 >() {}
    template < typename P > point< 0 >( const P& p, const Eigen::Vector3d& factor, const Eigen::Vector3d& offset )
        : coordinates( Eigen::Vector3d( factor.x() * p.coordinates.x()
                                      , factor.y() * p.coordinates.y()
                                      , factor.z() * p.coordinates.z() ) + offset )
        , intensity( p.intensity() )
        , return_number( p.returns().number )
        , number_of_returns( p.returns().size )
        , scan_direction( p.returns().scan_direction )
        , edge_of_flight_line( p.returns().edge_of_flight_line )
        , classification( p.classification() )
        , scan_angle( p.scan_angle() )
        , user_data( p.user_data() )
        , point_source_id( p.point_source_id() )
    {
    }
};

template <> struct point< 1 > : public point< 0 >
{
    double gps_time;

    point() {}
    point( const snark::las::point< 1 >& p, const Eigen::Vector3d& factor, const Eigen::Vector3d& offset ) : point< 0 >( p, factor, offset ), gps_time( p.gps_time() ) {}
};

template <> struct point< 2 > : public point< 0 >
{
    ::color color;

    point() {}
    point( const snark::las::point< 2 >& p, const Eigen::Vector3d& factor, const Eigen::Vector3d& offset ) : point< 0 >( p, factor, offset ), color( p.color.red(), p.color.green(), p.color.blue() ) {}
};

template <> struct point< 3 > : public point< 0 >
{
    double gps_time;
    ::color color;

    point() {}
    point( const snark::las::point< 3 >& p, const Eigen::Vector3d& factor, const Eigen::Vector3d& offset ) : point< 0 >( p, factor, offset ), gps_time( p.gps_time() ), color( p.color.red(), p.color.green(), p.color.blue() ) {}
};

// todo: template <> struct point< 4 >

// todo: template <> struct point< 5 >

template <> struct point< 6 > : public point< 0 >
{
    double gps_time;

    point() {}
    point( const snark::las::point< 6 >& p, const Eigen::Vector3d& factor, const Eigen::Vector3d& offset ) : point< 0 >( p, factor, offset ), gps_time( p.gps_time() ) {}
};

template <> struct point< 7 > : public point< 0 >
{
    double gps_time;
    ::color color;

    point() {}
    point( const snark::las::point< 7 >& p, const Eigen::Vector3d& factor, const Eigen::Vector3d& offset ) : point< 0 >( p, factor, offset ), gps_time( p.gps_time() ), color( p.color.red(), p.color.green(), p.color.blue() ) {}
};

namespace comma { namespace visiting {

template <> struct traits< point< 0 > >
{
    template< typename K, typename V > static void visit( const K&, const point< 0 >& t, V& v ) // todo
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "intensity", t.intensity );
        v.apply( "return_number", t.return_number );
        v.apply( "number_of_returns", t.number_of_returns );
        v.apply( "scan_direction", t.scan_direction );
        v.apply( "edge_of_flight_line", t.edge_of_flight_line );
        v.apply( "classification", t.classification );
        v.apply( "scan_angle", t.scan_angle );
        v.apply( "user_data", t.user_data );
        v.apply( "point_source_id", t.point_source_id );
    }
};

template <> struct traits< color >
{
    template< typename K, typename V > static void visit( const K&, const color& t, V& v ) // todo
    {
        v.apply( "red", t.red );
        v.apply( "green", t.green );
        v.apply( "blue", t.blue );
    }
};

template <> struct traits< point< 1 > >
{
    template< typename K, typename V > static void visit( const K& k, const point< 1 >& t, V& v ) // todo
    {
        traits< point< 0 > >::visit( k, t, v );
        v.apply( "gps_time", t.gps_time );
    }
};

template <> struct traits< point< 2 > >
{
    template< typename K, typename V > static void visit( const K& k, const point< 2 >& t, V& v ) // todo
    {
        traits< point< 0 > >::visit( k, t, v );
        v.apply( "color", t.color );
    }
};

template <> struct traits< point< 3 > >
{
    template< typename K, typename V > static void visit( const K& k, const point< 3 >& t, V& v ) // todo
    {
        traits< point< 0 > >::visit( k, t, v );
        v.apply( "gps_time", t.gps_time );
        v.apply( "color", t.color );
    }
};

template <> struct traits< point< 6 > >
{
    template< typename K, typename V > static void visit( const K& k, const point< 6 >& t, V& v ) // todo
    {
        traits< point< 0 > >::visit( k, t, v );
        v.apply( "gps_time", t.gps_time );
    }
};

template <> struct traits< point< 7 > >
{
    template< typename K, typename V > static void visit( const K& k, const point< 7 >& t, V& v ) // todo
    {
        traits< point< 0 > >::visit( k, t, v );
        v.apply( "gps_time", t.gps_time );
        v.apply( "color", t.color );
    }
};

} } // namespace comma { namespace visiting {

template < unsigned int I > static void check_point_data_version( const comma::command_line_options& options ) {}
template <> void check_point_data_version< 6 >( const comma::command_line_options& options ) { if( !options.exists( "--force" ) ) { COMMA_THROW( comma::exception, "point data format 6: not all packet fields get extracted (todo); please run with --force" ); } }
template <> void check_point_data_version< 7 >( const comma::command_line_options& options ) { if( !options.exists( "--force" ) ) { COMMA_THROW( comma::exception, "point data format 7: not all packet fields get extracted (todo); please run with --force" ); } }

template < unsigned int I > static int read_points( const snark::las::header& header, const comma::command_line_options& options )
{
    check_point_data_version< I >( options );
    comma::csv::options csv( options );
    csv.full_xpath = false;
    Eigen::Vector3d factor( header.scale_factor.x(), header.scale_factor.y(), header.scale_factor.z() );
    auto offset = options.exists( "--no-offset" ) ? Eigen::Vector3d::Zero() : Eigen::Vector3d( header.offset.x(), header.offset.y(), header.offset.z() );
    comma::csv::output_stream< point< I > > os( std::cout, csv );
    while( std::cin.good() && !std::cin.eof() )
    {
        snark::las::point< I > p;
        std::cin.read( reinterpret_cast< char* >( &p ), snark::las::point< I >::size ); // todo: watch performance
        int count = std::cin.gcount();
        if( count == 0 ) { break; }
        if( count < snark::las::point< I >::size ) { std::cerr << "las-to-csv: expected las point record format " << I << " of " << snark::las::point< I >::size << " bytes, got only: " << count << std::endl; return 1; }
        os.write( point< I >( p, factor, offset ) );
    }
    return 0;
}

static void _read( std::istream& is, char* buf, unsigned int size, const std::string& name = "buffer" )
{
    is.read( reinterpret_cast< char* >( buf ), size );
    int count = is.gcount();
    if( count < int( size ) ) { COMMA_THROW( comma::exception, "las-to-csv: expected " << name << " of " << snark::las::variable_length_records::header::size << " bytes, got only: " << count << std::endl ); }
}

static snark::las::header read_header( std::istream& is )
{
    snark::las::header header;
    _read( is, reinterpret_cast< char* >( &header ), snark::las::header::size, "las header" );
    return header;
}

static snark::las::variable_length_records::record read_variable_length_record( std::istream& is )
{
    snark::las::variable_length_records::record record;
    _read( is, reinterpret_cast< char* >( &record.header ), snark::las::variable_length_records::header::size, "las variable length header" );
    std::vector< char > body( record.header.record_length_after_header() );
    _read( is, &body[0], record.header.record_length_after_header(), "variable length record body" );
    if( record.header.user_id() == "LASF_Projection" )
    {
        switch( record.header.record_id() )
        {
            case 34735:
                // todo
                break;
            case 34737:
                record.body.reset( new snark::las::variable_length_records::geo_ascii_params_tag::body( std::string( &body[0], record.header.record_length_after_header() ) ) );
                break;
            default:
                break;
        }
    }
    return record;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        boost::optional< unsigned int > point_format = options.optional< unsigned int >( "--type,--point-format" );
        if( ac < 2 ) { usage(); }
        if( options.exists( "--output-fields" ) )
        {
            if( !point_format ) { point_format = read_header( std::cin ).point_data_format(); }
            switch( *point_format )
            {
                case 0: std::cout << comma::join( comma::csv::names< point< 0 > >( false ), ',' ) << std::endl; return 0;
                case 1: std::cout << comma::join( comma::csv::names< point< 1 > >( false ), ',' ) << std::endl; return 0;
                case 2: std::cout << comma::join( comma::csv::names< point< 2 > >( false ), ',' ) << std::endl; return 0;
                case 3: std::cout << comma::join( comma::csv::names< point< 3 > >( false ), ',' ) << std::endl; return 0;
                case 4:
                case 5:
                    std::cerr << "las-to-csv: output fields for point data format " << *point_format << ": todo" << std::endl;
                    return 1;
                case 6: std::cout << comma::join( comma::csv::names< point< 6 > >( false ), ',' ) << std::endl; return 0;
                case 7: std::cout << comma::join( comma::csv::names< point< 7 > >( false ), ',' ) << std::endl; return 0;
                default:
                    std::cerr << "las-to-csv: expected point data format between 0 and 7, got: " << *point_format << std::endl;
                    return 1;
            }
            return 0;
        }
        if( options.exists( "--output-format" ) )
        {
            if( !point_format ) { point_format = read_header( std::cin ).point_data_format(); }
            switch( *point_format )
            {
                case 0: std::cout << comma::csv::format::value< point< 0 > >() << std::endl; return 0;
                case 1: std::cout << comma::csv::format::value< point< 1 > >() << std::endl; return 0;
                case 2: std::cout << comma::csv::format::value< point< 2 > >() << std::endl; return 0;
                case 3: std::cout << comma::csv::format::value< point< 3 > >() << std::endl; return 0;
                case 4:
                case 5:
                    std::cerr << "las-to-csv: output fields for point data format " << *point_format << ": todo" << std::endl;
                    return 1;
                case 6: std::cout << comma::csv::format::value< point< 6 > >() << std::endl; return 0;
                case 7: std::cout << comma::csv::format::value< point< 7 > >() << std::endl; return 0;
                default:
                    std::cerr << "las-to-csv: expected point data format between 0 and 7, got: " << *point_format << std::endl;
                    return 1;
            }
            return 0;
        }
        std::string what = av[1];
        snark::las::header header = read_header( std::cin );
        if( !point_format ) { point_format = header.point_data_format(); }
        if( what == "header" ) { comma::write_json( header, std::cout ); return 0; }
        if( what == "variable-length-records" )
        {
            std::cerr << "las-to-csv: variable-length-records: todo" << std::endl; exit( 1 );
            for( unsigned int i = 0; i < header.number_of_variable_length_records(); ++i ) { read_variable_length_record( std::cin ); } // todo! .to_json(); }
        }
        if( what == "points" )
        {
            std::vector< char > offset( header.offset_to_point_data() - snark::las::header::size );
            std::cin.read( &offset[0], offset.size() );
            int count = std::cin.gcount();
            if( count < int( offset.size() ) ) { std::cerr << "las-to-csv: expected " << offset.size() << " bytes, got only: " << count << std::endl; return 1; }
            switch( *point_format )
            {
                case 0: return read_points< 0 >( header, options );
                case 1: return read_points< 1 >( header, options );
                case 2: return read_points< 2 >( header, options );
                case 3: return read_points< 3 >( header, options );
                case 4:
                case 5:
                    std::cerr << "las-to-csv: point data format " << point_format << ": todo" << std::endl;
                    return 1;
                case 6: return read_points< 6 >( header, options );
                case 7: return read_points< 7 >( header, options );
                default:
                    std::cerr << "las-to-csv: expected point data format between 0 and 7, got: " << point_format << std::endl;
                    return 1;
            }
            return 0;
        }
        std::cerr << "las-to-csv: expected operation, got: \"" << what << "\"" << std::endl;
        return 1;
    }
    catch( std::exception& ex ) { std::cerr << "las-to-csv: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "las-to-csv: unknown exception" << std::endl; }
    return 1;
}
