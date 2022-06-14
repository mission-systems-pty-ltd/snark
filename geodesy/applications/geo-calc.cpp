// Copyright (c) 2011 The University of Sydney

#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include "../../math/spherical_geometry/traits.h"
#include "../../visiting/eigen.h"
#include "../detail/GeographicConversions/Redfearn.h"
#include "../geoids.h"

static void usage( bool more = false )
{
    std::cerr << std::endl;
    std::cerr << "take coordinates in degrees on geoid from stdin, perform calculations, append result and output to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage examples" << std::endl;
    std::cerr << "    geo-calc <operation> [<options>]" << std::endl;
    std::cerr << "    cat arcs.csv | geo-calc distance [<options>] > results.csv" << std::endl;
    std::cerr << "    cat circular-arcs.csv | geo-calc discretize arc [<options>] > results.csv" << std::endl;
    std::cerr << "    cat circle.csv | geo-calc discretize circle [<options>]  > results.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations" << std::endl;
    std::cerr << "    convert    : convert between coordinates and north-east-down frame" << std::endl;
    std::cerr << "    distance   : output length of ellipsoid arc" << std::endl;
    std::cerr << "    discretize : output a discretized shape (e.g. circle) with a given resolution" << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations: details" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    convert" << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --geoid=<geoid>: geoid; default: wgs84" << std::endl;
    std::cerr << "                geoids: wgs84, agd84; support more geoids: todo, low-hanging fruit" << std::endl;
    std::cerr << "            --to=<what>: append the converted values and zone to output" << std::endl;
    std::cerr << "                <what>" << std::endl;
    std::cerr << "                    coordinates: convert from north-east-down to coordinates; default input fields: x,y,z" << std::endl;
    std::cerr << "                    north-east-down,ned: convert from coordinates to north-east-down; default input fields: latitude,longitude,z" << std::endl;
    std::cerr << "            --hemisphere=<what>: either 'north' or 'south', if --coordinates" << std::endl;
    std::cerr << "            --north; same as hemisphere=north" << std::endl;
    std::cerr << "            --south; same as hemisphere=south" << std::endl;
    std::cerr << "            --zone=<zone>; MGA zone" << std::endl;
    std::cerr << "                           --to coordinates: ; e.g. 56 for Sydney: default zone, in case zone field is not on stdin input" << std::endl;
    std::cerr << "                           --to ned: enforce using a given zone rather than calculating zone from latitude/longitude" << std::endl;
    std::cerr << "            --zone-use-first; use zone from the first latitude/longitude pair for all the subsequent input" << std::endl;
    std::cerr << "        info options" << std::endl;
    std::cerr << "            --input-fields; print input fields for a given conversion to stdout and exit" << std::endl;
    std::cerr << "            --output-fields; print output fields for a given conversion to stdout and exit" << std::endl;
    std::cerr << "            --output-format; print binary output format for a given conversion to stdout and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    distance: ellipsoid arc distance in meters; if --binary, output as double" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: " << comma::join( comma::csv::names< snark::spherical::ellipsoid::arc >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "    discretize <shape>: output a discretized shape with a given resolution" << std::endl;
    std::cerr << "                multiple output lines per input line" << std::endl;
    std::cerr << "                the result will be appended to the line (see example)" << std::endl;
    std::cerr << "          shape: circle, arc" << std::endl;
    std::cerr << "          input fields" << std::endl;
    std::cerr << "              circle: centre,radius" << std::endl;
    std::cerr << "                  centre: latitude,longitude in degrees" << std::endl;
    std::cerr << "                  radius: metric distance from centre" << std::endl;
    std::cerr << "              arc: circle/centre,circle/radius,begin,end" << std::endl;
    std::cerr << "                  circle/centre: latitude,longitude in degrees" << std::endl;
    std::cerr << "                  circle/radius: metric distance from centre" << std::endl;
    std::cerr << "                  begin: begin bearing, degrees" << std::endl;
    std::cerr << "                  end: end bearing, degrees" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    info: get geoid's information; output format is name,description,major_semiaxis,minor_semiaxis,eccentricity" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --resolution=<degrees>" << std::endl;
    std::cerr << "            --circle-size,--size=<number>: number of points in the discretized circle (circle or arc only)" << std::endl;
    std::cerr << "                                           if both --resolution and --cricle-size present, whatever gives finer resolution will be choosen" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: show help; --help --verbose for more help" << std::endl;
    std::cerr << "    --verbose,-v: more info" << std::endl;
    std::cerr << "    --geoid: <geoid name> (case insensitive); default: WGS84 " << std::endl;
    std::cerr << "        geoids:" << std::endl;
    std::cerr << "            WGS84 (default): standard used in GPS" << std::endl;
    std::cerr << "            AGD84: Australian standard" << std::endl;
    std::cerr << "            GRS67: Historical standard" << std::endl;
    std::cerr << "            sphere: major=minor" << std::endl;
    std::cerr << "            unit: scaled down WGS84 (~radian)" << std::endl;

    if ( more )
    {
        std::cerr << "        (name: description (major semiaxis; minor semiaxis); inverse eccentricity)" << std::endl;
        std::cerr << snark::geodesy::wgs84::help();
        std::cerr << snark::geodesy::agd84::help();
        std::cerr << snark::geodesy::grs67::help();
        std::cerr << snark::geodesy::geoids::help();
    }
    else
    {
        std::cerr << "           run geo-calc --help --verbose for more details..." << std::endl;
    }
    if( more ) { std::cerr << std::endl << "csv options" << std::endl << comma::csv::options::usage() << std::endl; }
    std::cerr << std::endl;
    std::cerr << "examples (try them)" << std::endl;
    std::cerr << "    convert" << std::endl;
    std::cerr << "        echo -34,154,10 | geo-calc convert --to ned" << std::endl;
    std::cerr << "        echo 6237393.34007,592349.603343,-10,56 | geo-calc convert --to coordinates --zone 56 --south" << std::endl;
    std::cerr << "        echo 6237393.34007,592349.603343,-10,56 | geo-calc convert --to coordinates --fields=x,y,z,zone --south" << std::endl;
    std::cerr << "        echo 6237393.34007,592349.603343,-10,56,1 | geo-calc convert --to coordinates --fields=x,y,z,zone,is_south" << std::endl;
    std::cerr << "    discretize" << std::endl;
    std::cerr << "        cat circular-arcs.csv | geo-calc discretize arc --fields=circle/centre/latitude,circle/centre/longitude,circle/radius,begin,end --circle-size=32 > results.csv" << std::endl;
    std::cerr << "        cat circle.csv | geo-calc discretize circle --fields=centre,radius --resolution=0.1 | column -ts," << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

template < typename S > int discretize( const comma::csv::options &csv, const snark::spherical::ellipsoid &ellipsoid, const boost::optional< double > &resolution, const boost::optional< unsigned int > &circle_size )
{
    comma::csv::input_stream< S > istream( std::cin, csv );
    comma::csv::ascii< snark::spherical::coordinates > ascii;
    comma::csv::binary< snark::spherical::coordinates > binary;
    while ( istream.ready() || std::cin.good() )
    {
        const S *a = istream.read();
        if ( !a ) { break; }
        const std::vector< snark::spherical::coordinates > &v = a->discretize( ellipsoid, resolution, circle_size );
        for ( unsigned int i = 0; i < v.size(); ++i )
        {
            if ( csv.binary() )
            {
                std::cout.write( istream.binary().last(), istream.binary().binary().format().size() );
                std::vector< char > buf( binary.format().size() );
                binary.put( v[i], &buf[0] );
                std::cout.write( reinterpret_cast< const char * >( &buf[i] ), buf.size() );
            }
            else
            {
                std::cout << comma::join( istream.ascii().last(), csv.delimiter );
                std::cout << csv.delimiter << ascii.put( v[i] );
                std::cout << std::endl;
            }
        }
    }
    return 0;
}

namespace convert_ {

struct ned_
{
    Eigen::Vector3d coordinates;
    comma::int32 zone; // todo: is it uint or int?
    bool is_south;

    ned_() : coordinates( 0, 0, 0 ), zone( 0 ), is_south( false ) {}
};

struct coordinates_
{
    snark::spherical::coordinates coordinates;
    double z;

    coordinates_() : z( 0 ) {}
};

} // namespace convert_ {

namespace comma { namespace visiting {

template <> struct traits< convert_::coordinates_ >
{
    template < typename Key, class Visitor > static void visit( const Key&, convert_::coordinates_& p, Visitor& v )
    {
        v.apply( "coordinates", p.coordinates );
        v.apply( "z", p.z );
    }

    template < typename Key, class Visitor > static void visit( const Key&, const convert_::coordinates_& p, Visitor& v )
    {
        v.apply( "coordinates", p.coordinates );
        v.apply( "z", p.z );
    }
};

template <> struct traits< convert_::ned_ >
{
    template < typename Key, class Visitor > static void visit( const Key&, convert_::ned_& p, Visitor& v )
    {
        v.apply( "coordinates", p.coordinates );
        v.apply( "zone", p.zone );
        v.apply( "is_south", p.is_south );
    }

    template < typename Key, class Visitor > static void visit( const Key&, const convert_::ned_& p, Visitor& v )
    {
        v.apply( "coordinates", p.coordinates );
        v.apply( "zone", p.zone );
        v.apply( "is_south", p.is_south );
    }
};

} } // namespace comma { namespace visiting {

int main( int ac, char **av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options csv( options );
        if( !csv.binary() ) { std::cout.precision( options.value( "--precision,-p", 16 ) ); } // do we need it?
        csv.full_xpath = true;
        std::string geoid_name = options.value< std::string >( "--geoid", "wgs84" );
        const snark::spherical::ellipsoid& geoid = snark::geodesy::geoids::select( geoid_name );
        const std::vector<std::string> &operations = options.unnamed( "--verbose,-v,--degrees,--north,--south", "-.*" );
        if( operations.empty() ) { std::cerr << "geo-calc: please specify operation" << std::endl; return 1; }
        if( operations[0] == "convert" )
        {
            std::string to = options.value< std::string >( "--to" );
            if( options.exists( "--input-fields" ) )
            {
                if( to == "north-east-down" || to == "ned" ) { std::cout << comma::join( comma::csv::names< convert_::coordinates_ >( false ), ',' ) << std::endl; return 0; }
                if( to == "coordinates" ) { std::cout << comma::join( comma::csv::names< convert_::ned_ >( false ), ',' ) << std::endl; return 0; }
                std::cerr << "geo-calc: expected --what=<to>; got --what='" << to << "'" << std::endl;
                return 1;
            }
            if( options.exists( "--output-fields" ) )
            {
                if( to == "north-east-down" || to == "ned" ) { std::cout << comma::join( comma::csv::names< convert_::ned_ >( false ), ',' ) << std::endl; return 0; }
                if( to == "coordinates" ) { std::cout << comma::join( comma::csv::names< convert_::coordinates_ >( false ), ',' ) << std::endl; return 0; }
                std::cerr << "geo-calc: expected --what=<to>; got --what='" << to << "'" << std::endl;
                return 1;
            }
            if( options.exists( "--output-format" ) )
            {
                if( to == "north-east-down" || to == "ned" ) { std::cout << comma::csv::format().value< convert_::ned_ >() << std::endl; return 0; }
                if( to == "coordinates" ) { std::cout << comma::csv::format().value< convert_::coordinates_ >() << std::endl; return 0; }
                std::cerr << "geo-calc: expected --what=<to>; got --what='" << to << "'" << std::endl;
                return 1;
            }
            snark::detail::GeographicGeodeticRectangular::CRedfearn credfearn(
                  geoid_name == "agd84" ? snark::detail::Ellipsoid::AUSTRALIAN_NATIONAL
                : geoid_name == "wgs84" ? snark::detail::Ellipsoid::WGS84
                                        : snark::detail::Ellipsoid::WGS84
                , snark::detail::MapGrid::MGA );
            csv.full_xpath = false; // quick and dirty
            comma::csv::options ocsv;
            ocsv.flush = options.exists( "--flush" );
            if( to == "north-east-down" || to == "ned" )
            {
                if( csv.fields.empty() ) { csv.fields = "latitude,longitude,z"; }
                comma::csv::input_stream< convert_::coordinates_ > is( std::cin, csv );
                if( csv.binary() ) { ocsv.format( comma::csv::format().value< convert_::ned_ >() ); }
                comma::csv::output_stream< convert_::ned_ > os( std::cout, ocsv );
                comma::csv::tied< convert_::coordinates_, convert_::ned_ > tied( is, os );
                auto zone = options.optional< unsigned int >( "--zone" );
                bool zone_use_first = options.exists( "--zone-use-first" );
                while ( is.ready() || ( std::cin.good() && !std::cin.eof() ) )
                {
                    const convert_::coordinates_* p = is.read();
                    if( !p ) { break; }
                    convert_::ned_ q;
                    double east, north, scale, convergence;
                    credfearn.GetGridCoordinates( ( p->coordinates.latitude * 180 / M_PI ) // todo: quick and dirty, refactor redfearn
                                                , ( p->coordinates.longitude * 180 / M_PI )
                                                , q.zone
                                                , east
                                                , north
                                                , convergence
                                                , scale
                                                , zone );
                    if( !zone && zone_use_first ) { zone = q.zone; }
                    q.is_south = p->coordinates.latitude < 0;
                    if( !q.is_south ) { north -= 10000000; }
                    q.coordinates = Eigen::Vector3d( north, east, -p->z );
                    tied.append( q );
                }
                return 0;
            }
            if( to == "coordinates" )
            {
                std::string hemisphere = options.value< std::string >( "--hemisphere", "" );
                bool north = options.exists( "--north" ) || hemisphere == "north";
                bool south = options.exists( "--south" ) || hemisphere == "south";
                if( !csv.has_field( "is_south" ) )
                {
                    if( !north && !south ) { std::cerr << "geo-calc: convert: for --what=coordinates, please specify either --north or --south" << std::endl; return 1; }
                    if( north && south ) { std::cerr << "geo-calc: convert: for --what=coordinates, expected either --north or --south; got both" << std::endl; return 1; }
                }
                if( csv.fields.empty() ) { csv.fields = "x,y,z"; }
                convert_::ned_ default_input;
                if( !csv.has_field( "zone" ) ) { default_input.zone = options.value< unsigned int >( "--zone" ); }
                default_input.is_south = south;
                comma::csv::input_stream< convert_::ned_ > is( std::cin, csv, default_input );
                if( csv.binary() ) { ocsv.format( comma::csv::format::value< convert_::coordinates_ >() ); }
                comma::csv::output_stream< convert_::coordinates_ > os( std::cout, ocsv );
                comma::csv::tied< convert_::ned_, convert_::coordinates_ > tied( is, os );
                while ( is.ready() || ( std::cin.good() && !std::cin.eof() ) )
                {
                    const convert_::ned_* p = is.read();
                    if( !p ) { break; }
                    convert_::coordinates_ q;
                    double latitude, longitude, scale, convergence;
                    credfearn.GetGeographicCoordinates( p->zone
                                                      , p->coordinates.y()
                                                      , p->coordinates.x() + ( p->is_south ? 0 : 10000000 )
                                                      , latitude
                                                      , longitude
                                                      , convergence
                                                      , scale );
                    q.coordinates.latitude = latitude * M_PI / 180;
                    q.coordinates.longitude = longitude * M_PI / 180;
                    q.z = -p->coordinates.z();
                    tied.append( q );
                }
                return 0;
            }
            std::cerr << "geo-calc: expected --what=<to>; got --what='" << to << "'" << std::endl;
            return 1;
        }
        if( operations[0] == "distance" )
        {
            comma::csv::input_stream< snark::spherical::ellipsoid::arc > istream( std::cin, csv );
            while ( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const snark::spherical::ellipsoid::arc* a = istream.read();
                if ( !a ) { break; }
                double distance = geoid.distance( a->begin, a->end );
                if ( csv.binary() )
                {
                    std::cout.write( istream.binary().last(), istream.binary().binary().format().size() );
                    std::cout.write( reinterpret_cast< const char * >( &distance ), sizeof( double ) );
                }
                else
                {
                    std::cout << comma::join( istream.ascii().last(), csv.delimiter ) << csv.delimiter << distance << std::endl;
                }
            }
            return 0;
        }
        if( operations[0] == "discretize" )
        {
            boost::optional< double > resolution = options.optional< double >( "--resolution" );
            if( resolution ) { resolution = *resolution * M_PI / 180; }
            boost::optional< unsigned int > circle_size = options.optional< unsigned int >( "--circle-size,--size" );
            if( operations.size() != 2 ) { std::cerr << "geo-calc: expected shape for operation discretize" << std::endl; return 1; }
            if( operations[1] == "circle" ) { return discretize< snark::spherical::ellipsoid::circle >( csv, geoid, resolution, circle_size ); }
            else if ( operations[1] == "arc" ) { return discretize<snark::spherical::ellipsoid::circle::arc>( csv, geoid, resolution, circle_size ); }
            std::cerr << "geo-calc: unknown shape for discretize: \"" << operations[1] << "\"" << std::endl;
        }
        if( operations[0] == "info" ) { std::cerr << snark::geodesy::geoids::info( geoid_name ); return 0; }
        std::cerr << "geo-calc: unknown operation: \"" << operations[0] << "\"" << std::endl;
    }
    catch ( std::exception &ex ) { std::cerr << "geo-calc: " << ex.what() << std::endl; }
    catch ( ... ) { std::cerr << "geo-calc: unknown exception" << std::endl; }
    return 1;
}
