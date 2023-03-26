// Copyright (c) 2016 The University of Sydney
// Copyright (c) 2021 Mission Systems Pty Ltd
// Copyright (c) 2021 Vsevolod Vlaskine

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <boost/lexical_cast.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include "../../math/range_bearing_elevation.h"
#include "../../visiting/eigen.h"

namespace snark { namespace points_make {

template < typename T > struct operation_t
{
    static std::string output_fields() { return comma::join( comma::csv::names< T >( false ), ',' ); }
    static std::string output_format() { return comma::csv::format::value< T >(); }
};

class random
{
    public:
        random( int seed, bool true_random )
        {
            if( true_random )
            {
                device_.reset( new std::random_device );
            }
            else
            {
                engine_.reset( std::mt19937_64( seed ) );
                distribution_.reset( std::uniform_real_distribution< double >( 0, 1 ) );
            }
        }
        
        random( const std::string& how ): random( how == "true" ? 0 : boost::lexical_cast< int >( how ), how == "true" ) {}
        
        double operator()() { return device_ ? double( ( *device_ )() ) / std::numeric_limits< unsigned int >::max() : ( *distribution_ )( *engine_ ); }
        
    private:
        std::unique_ptr< std::random_device > device_;
        boost::optional< std::mt19937_64 > engine_;
        boost::optional< std::uniform_real_distribution< double > > distribution_;
};

static Eigen::Vector3d tensor_product( const Eigen::Vector3d& s, const Eigen::Vector3d& t ) { return Eigen::Vector3d( s[0] * t[0], s[1] * t[1], s[2] * t[2] ); }

struct box : public operation_t< Eigen::Vector3d >
{
    static void usage()
    {
        std::cerr << "\n    box: output point cloud of rectangular box shape; surface only or filled";
        std::cerr << "\n         current limitation: extent on a given dimension will be shrank to";
        std::cerr << "\n                             multiple of resolution of that dimension";
        std::cerr << "\n        --fill";
        std::cerr << "\n        --extents,-e=<point>";
        std::cerr << "\n        --origin,o=<point>; default=0,0,0";
        std::cerr << "\n        --random; output random points";
        std::cerr << "\n        --random-seed,--seed,-s=<seed>; if present, output uniform random sample on sphere";
        std::cerr << "\n                                        instead of regular reasonably uniform sample";
        std::cerr << "\n                                        <seed>: number for repeatable pseudo-random numbers or 'true' for true random";
        std::cerr << "\n        --resolution,-r=<resolution>; <resolution>: <number> or <point>; default=1";
        std::cerr << "\n        --size=<n>; desired number of points, overrides --resolution";
        std::cerr << "\n        --width,-w=<width>; shortcut for --extents, assumes cube";
    }

    static void examples()
    {
        std::cerr << "\n    points-make box --width 100 | view-points '-;color=yellow'";
        std::cerr << "\n    points-make box --width 10 --fill --binary 3d | view-points '-;color=yellow;binary=3d'";
    }

    static const char* completion_options() { return " box --fill --extents -e --origin -o --resolution -r --random-seed --random --seed -s --width -w"; }

    static int run( const comma::command_line_options& options )
    {
        options.assert_mutually_exclusive( "--extents,-e", "--width,-w" );
        bool fill = options.exists( "--fill" );
        auto origin = comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--origin,-o", "0,0,0" ) );
        Eigen::Vector3d extents;
        if( !options.exists( "--extents,-e,--width,-w" ) ) { COMMA_THROW( comma::exception, "box: please specify either --extents or --width" ); }
        if( options.exists( "--extents,-e" ) ) { extents = comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--extents,-e" ) ); }
        else { double w = options.value< double >( "--width,-w" ); extents = Eigen::Vector3d( w, w, w ); }
        Eigen::Vector3d resolution;
        std::string s = options.value< std::string >( "--resolution,-r", "1,1,1" );
        if( comma::split( s, ',' ).size() == 1 ) { auto r = boost::lexical_cast< double >( s ); resolution = Eigen::Vector3d( r, r, r ); }
        else { resolution = comma::csv::ascii< Eigen::Vector3d >().get( s ); }
        auto size = options.optional< unsigned int >( "--size" );
        auto seed = options.optional< std::string >( "--random-seed,--seed,-s" );
        if( options.exists( "--random" ) && !seed ) { seed = "0"; } // quick and dirty
        comma::csv::output_stream< Eigen::Vector3d > ostream( std::cout, comma::csv::options( options ) );
        auto p = origin;
        auto end = origin + extents;
        if( fill )
        {
            if( seed )
            {
                if( !size ) { size = ( extents[0] * extents[1] * extents[2] ) / ( resolution[0] * resolution[1] * resolution[2] ); }
                points_make::random rd( *seed );
                for( unsigned int i = 0; i < *size; ++i ) { ostream.write( origin + tensor_product( extents, Eigen::Vector3d( rd(), rd(), rd() ) ) ); }
            }
            else
            {
                //if( size ) { double f = std::pow( extents[0] * extents[1] * extents[2] / *size, 1. / 3 ); resolution = Eigen::Vector3d( f, f, f ); }
                if( size ) { COMMA_THROW( comma::exception, "box: --fill --size: todo" ); } // the formula above does not guarantee the exact number of points
                for( p[0] = origin[0]; p[0] < end[0]; p[0] += resolution[0] )
                {
                    for( p[1] = origin[1]; p[1] < end[1]; p[1] += resolution[1] )
                    {
                        for( p[2] = origin[2]; p[2] < end[2]; p[2] += resolution[2] ) { ostream.write( p ); }
                    }
                }
            }
        }
        else
        {
            if( seed )
            {
                std::array< double, 3 > area = {{ extents[1] * extents[2], extents[0] * extents[2], extents[0] * extents[1] }};
                if( !size ) { size = 2 * ( area[0] / resolution[1] * resolution[2] + area[1] / resolution[0] * resolution[2] + area[2] / resolution[0] * resolution[1] ); }
                points_make::random rd( *seed );
                std::mt19937_64 e;
                std::uniform_real_distribution< double > r( 0, area[0] + area[1] + area[2] );
                std::uniform_int_distribution< int > l( 0, 1 );
                auto index = [&]( double s ) -> unsigned int { return s < area[0] ? 0 : s < ( area[0] + area[1] ) ? 1 : 2; };
                for( unsigned int i = 0; i < *size; ++i )
                {
                    Eigen::Vector3d v( rd(), rd(), rd() );
                    v[ index( r( e ) ) ] = l( e );
                    ostream.write( origin + tensor_product( extents, v ) );
                }
            }
            else
            {
                //if( size ) { double f = std::sqrt( ( extents[0] * extents[1] + extents[1] * extents[2] + extents[0] * extents[2] ) * 2 / * size ); resolution = Eigen::Vector3d( f, f, f ); }
                if( size ) { COMMA_THROW( comma::exception, "box: --size: todo" ); } // the formula above does not guarantee the exact number of points
                auto output_filled = [&]()
                {
                    for( p[0] = origin[0]; p[0] < end[0]; p[0] += resolution[0] )
                    {
                        for( p[1] = origin[1]; p[1] < end[1]; p[1] += resolution[1] ) { ostream.write( p ); }
                    }
                };
                auto last = p - resolution;  // quick and dirty
                auto output_unfilled = [&]()
                {
                    p[1] = origin[1];
                    for( p[0] = origin[0]; p[0] < end[0]; p[0] += resolution[0] ) { ostream.write( p ); }
                    for( p[1] = origin[1] + resolution[1]; p[1] < end[1] - resolution[1]; p[1] += resolution[0] )
                    {
                        ostream.write( Eigen::Vector3d( origin[0], p[1], p[2] ) );
                        ostream.write( Eigen::Vector3d( last[0], p[1], p[2] ) );
                    }
                    if( p[1] < end[1] ) { for( p[0] = origin[0]; p[0] < end[0]; p[0] += resolution[0] ) { ostream.write( p ); } }
                };
                output_filled();
                p[2] += resolution[2];
                for( ; p[2] < end[2] - resolution[2]; p[2] += resolution[2] ) { output_unfilled(); }
                if( p[2] < end[2] ) { output_filled(); }
            }
        }
        return 0;
    }
};

struct sphere : public operation_t< Eigen::Vector3d >
{
    static void usage()
    {
        std::cerr << "\n    sphere: output point cloud of spherical shape; surface only or filled";
        std::cerr << "\n        --fill; todo";
        std::cerr << "\n        --radius=<radius>";
        std::cerr << "\n        --center,-c=<point>; default=0,0,0";
        std::cerr << "\n        --fibonacci; fibonacci spiral-based sphere, a reasonably uniform distribution";
        std::cerr << "\n                     todo: a more accurate uniform distribution";
        std::cerr << "\n        --resolution,-r=<resolution>; roughly how far points need to be from each other; default=1";
        std::cerr << "\n        --random; output random points";
        std::cerr << "\n        --random-seed,--seed,-s=<seed>; if present, output uniform random sample on sphere";
        std::cerr << "\n                                        instead of regular reasonably uniform sample";
        std::cerr << "\n                                        <seed>: number for repeatable pseudo-random numbers or 'true' for true random";
        std::cerr << "\n        --size=<n>; desired number of points, overrides resolution";
    }

    static void examples()
    {
        std::cerr << "\n    points-make sphere --radius 10 | view-points '-;color=yellow'";
        std::cerr << "\n    points-make sphere --radius 10 --size 10000 --random | view-points '-;color=yellow'";
    }

    static const char* completion_options() { return " sphere --radius --center -c --resolution -r --random-seed --random --seed -s"; }

    static int run( const comma::command_line_options& options )
    {
        options.assert_mutually_exclusive( "--resolution,-r", "--size" );
        options.assert_mutually_exclusive( "--fibonacci", "--fill,--random,--seed,--random-seed,-s" );
        options.assert_exists_if( "--fibonacci", "--size" );
        bool fill = options.exists( "--fill" );
        auto center = comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--center,-c", "0,0,0" ) );
        double radius = options.value< double >( "--radius" );
        auto resolution = options.value( "--resolution,-r", 1.0 );
        auto size = options.optional< unsigned int >( "--size" );
        bool fibonacci = options.exists( "--fibonacci" );
        if( fibonacci && !size ) { COMMA_THROW( comma::exception, "for --fibonacci, please specify --size" ); }
        auto seed = options.optional< std::string >( "--random-seed,--seed,-s" );
        if( options.exists( "--random" ) && !seed ) { seed = "0"; } // quick and dirty
        comma::csv::output_stream< Eigen::Vector3d > ostream( std::cout, comma::csv::options( options ) );
        if( seed )
        {
            if( size ) { COMMA_THROW( comma::exception, "sphere: --size for --random: todo; meanwhile use --resolution" ); }
            points_make::random rd( *seed );
            unsigned int n = size ? *size
                                  : fill ? ( ( radius * radius * radius * 4. / 3. * M_PI ) / ( resolution * resolution * resolution / ( 6. * std::sqrt( 2 ) ) ) ) * 4 / 20  // quick and dirty: assuming resolution much less than radius
                                         : ( ( radius * radius * 4. * M_PI ) / ( resolution * resolution * std::sqrt( 3. ) / 2. ) ) * 3. / 6.; // quick and dirty: assuming resolution much less than radius
            for( unsigned int i = 0; i < n; ++i )
            {
                double a = ( rd() * 2 - 1 ) * M_PI;
                double e = std::asin( rd() * 2 - 1 ); // todo: improve; even though it'sfibo reasonably uniform, there are local irregularities
                double r = ( fill ? std::pow( rd(), 1. / 3 ) : 1 ) * radius;
                ostream.write( snark::range_bearing_elevation( r, a, e ).to_cartesian() + center );
            }
        }
        else
        {
            if( fill )
            {
                COMMA_THROW( comma::exception, "sphere: --fill: implemented only for --random" );
            }
            else
            {
                if( fibonacci )
                {
                    double phi = M_PI * ( 3 - std::sqrt( 5. ) );
                    double theta = 0;
                    for( unsigned int i = 0; i < *size; ++i, theta += phi )
                    {
                        double y = 1 - double( i * 2 ) / ( *size - 1 );
                        double r = std::sqrt( 1 - y * y );
                        Eigen::Vector3d p;
                        ostream.write( Eigen::Vector3d( std::cos( theta ) * r, y, std::sin( theta ) * r ) * radius + center );
                    }
                }
                else
                {
                    double as = std::asin( ( resolution / 2 ) / radius ) * 2;
                    for( double a = 0; a < M_PI * 2; a += as ) { ostream.write( snark::range_bearing_elevation( radius, a, 0 ).to_cartesian() + center ); }
                    double es = std::asin( resolution / radius * std::sqrt( 3 ) / 2 );
                    double t = 0;
                    for( double e = es; e < M_PI / 2; e += es )
                    {
                        t = 0.5 - t; // quick and dirty
                        double s = std::asin( ( resolution / 2 ) / ( radius * std::cos( e ) ) ) * 2;
                        for( double a = s * t; a < M_PI * 2; a += s )
                        {
                            auto p = snark::range_bearing_elevation( radius, a, e ).to_cartesian();
                            ostream.write( p + center );
                            p[2] = -p[2];
                            ostream.write( p + center );
                        }
                    }
                }
            }
        }
        return 0;
    }
};

namespace test_pattern {

struct color_t
{
    boost::array< unsigned char, 4 > rgba = {{ 0, 0, 0, 255 }};
    color_t() {}
    color_t( unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha = 255 ): rgba( {{ red, green, blue, alpha }} ) {}

    unsigned char red() const   { return rgba[0]; }
    unsigned char green() const { return rgba[1]; }
    unsigned char blue() const  { return rgba[2]; }
    unsigned char alpha() const { return rgba[3]; }
};

// Define our own vertex type using our colour type.
// A more flexible alternative is to use snark::graphics::qt3d::vertex_t
// but this type works better with view-points.
struct vertex_t
{
    Eigen::Vector3f position;
    color_t color;

    vertex_t(): position( 0, 0, 0 ) {}
    vertex_t( const Eigen::Vector3f& position, const color_t& color ): position( position ), color( color ) {}
};

struct cube : public operation_t< vertex_t >
{
    static const unsigned int default_num_points = 100000;
    static constexpr float default_width = 1.0f;
    static constexpr float default_thickness = 0.01f;

    static void usage()
    {
        std::cerr << "\n    test-cube";
        std::cerr << "\n        --number-of-points,--num,-n=<val>:     number of points in cube (default: " << default_num_points << ")";
        std::cerr << "\n        --width,-w=<val>:   cube width (default: " << default_width << ")";
        std::cerr << "\n        --thickness=<val>:  cube thickness (default: " << default_thickness << ")";
    }

    static void examples()
    {
        std::cerr << "\n    " << "points-make test-cube | view-points --fields x,y,z,r,g,b,a";
    }

    static const char* completion_options() { return " test-cube --num -n --width -w --thickness"; }

    static int run( const comma::command_line_options& options )
    {
        unsigned seed = options.value< unsigned int >( "--random-seed,--seed,-s", time( 0 ) );
        srand( seed );
        unsigned int num_points = options.value< unsigned int >( "--number-of-points,--num,-n", default_num_points );
        float width = options.value< float >( "--width,-w", default_width );
        float thickness = options.value< float >( "--thickness", default_thickness );

        comma::csv::options output_csv( options );
        output_csv.full_xpath = false;
        if( options.exists( "--binary,-b" )) { output_csv.format( comma::csv::format::value< vertex_t >() ); }
        comma::csv::output_stream< vertex_t > ostream( std::cout, output_csv );

        for( unsigned int i = 0; i < num_points; i++ )
        {
            Eigen::Vector3f p = make_point( width, thickness );
            vertex_t vertex( p, color_t( point_to_color( p.x(), width, thickness )
                                       , point_to_color( p.y(), width, thickness )
                                       , point_to_color( p.z(), width, thickness )
                                       , 255 ));
            ostream.write( vertex );
        }
        return 0;
    }

private:
    static unsigned char point_to_color( float p, float width, float thickness ) { return (unsigned char)( fabs( p / ( width + thickness )) * 256 ); }
    static float random( float min, float max ) { return (float)::random() / RAND_MAX * ( max - min ) + min; }
    static int random_sign() { return ::random() % 2 * 2 - 1; }

    static Eigen::Vector3f make_point( float width, float thickness )
    {
        float min = width - thickness;
        float max = width + thickness;
        float x_min = 0;
        float y_min = 0;
        float z_min = 0;
        switch( ::random() % 3 )
        {
            case 0: x_min = min; break;
            case 1: y_min = min; break;
            default: case 2: z_min = min; break;
        }
        float x = random_sign() * random( x_min, max );
        float y = random_sign() * random( y_min, max );
        float z = random_sign() * random( z_min, max );
        return Eigen::Vector3f( x, y, z );
    }
};

struct grid : public operation_t< Eigen::Vector3f >
{
    static constexpr float default_width = 6.0f;
    static constexpr float default_spacing = 1.0f;
    static constexpr float default_z_offset = 0.0f;

    static void usage()
    {
        std::cerr << "\n    test-grid";
        std::cerr << "\n        --width,-w=<val>:   grid width; default: " << default_width;
        std::cerr << "\n        --spacing,-s=<val>: grid spacing; default: " << default_spacing;
        std::cerr << "\n        --z-offset=<val>:   grid z offset; default: " << default_z_offset;
    }

    static void examples()
    {
        std::cerr << "\n    " << "points-make test-grid | view-points --shape lines";
    }

    static const char* completion_options() { return " test-grid --width -w --spacing -s --z-offset"; }

    static int run( const comma::command_line_options& options )
    {
        unsigned seed = options.value< unsigned int >( "--random-seed,--seed,-s", time( 0 ) );
        srand( seed );
        float width = options.value< float >( "--width,-w", default_width );
        float spacing = options.value< float >( "--spacing,-s", default_spacing );
        float z_offset = options.value< float >( "--z-offset", default_z_offset );

        comma::csv::options output_csv( options );
        output_csv.full_xpath = false;
        if( options.exists( "--binary,-b" )) { output_csv.format( comma::csv::format::value< Eigen::Vector3f >() ); }

        comma::csv::output_stream< Eigen::Vector3f > ostream( std::cout, output_csv );

        int sign = 1;
        float offset = width / 2.0f;
        for( float y = -offset; y <= offset; y += spacing )
        {
            ostream.write( Eigen::Vector3f( -offset * sign, y, z_offset ));
            ostream.write( Eigen::Vector3f(  offset * sign, y, z_offset ));
            sign *= -1;
        }
        sign = 1;
        for( float x = -offset; x <= offset; x += spacing )
        {
            ostream.write( Eigen::Vector3f( x,  offset * sign, z_offset ));
            ostream.write( Eigen::Vector3f( x, -offset * sign, z_offset ));
            sign *= -1;
        }
        return 0;
    }
};

} } // namespace test_pattern {

} // namespace snark { namespace points_make {

namespace comma { namespace visiting {

template <> struct traits< snark::points_make::test_pattern::color_t >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::points_make::test_pattern::color_t& p, Visitor& v )
    {
        v.apply( "r", p.rgba[0] );
        v.apply( "g", p.rgba[1] );
        v.apply( "b", p.rgba[2] );
        v.apply( "a", p.rgba[3] );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::points_make::test_pattern::color_t& p, Visitor& v )
    {
        v.apply( "r", p.red() );
        v.apply( "g", p.green() );
        v.apply( "b", p.blue() );
        v.apply( "a", p.alpha() );
    }
};

template <> struct traits< snark::points_make::test_pattern::vertex_t >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::points_make::test_pattern::vertex_t& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "color", p.color );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::points_make::test_pattern::vertex_t& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "color", p.color );
    }
};

} } // namespace comma { namespace visiting {

template < typename Operation > static int run( const comma::command_line_options& options )
{
    if( options.exists( "--output-fields" ) ) { std::cout << Operation::output_fields() << std::endl; return 0; }
    if( options.exists( "--output-format" ) ) { std::cout << Operation::output_format() << std::endl; return 0; }
    return Operation::run( options );
}

static void bash_completion( unsigned const ac, char const * const * av )
{
    std::cout << "--help -h --output-fields --output-format --binary -b --random-seed --seed -s"
              << snark::points_make::test_pattern::cube::completion_options()
              << snark::points_make::test_pattern::grid::completion_options()
              << std::endl;
    exit( 0 );
}

static void usage( bool verbose )
{
    std::cerr << "\noutput point clouds";
    std::cerr << "\n";
    std::cerr << "\nusage: " << "points-make <operation> [<options>]";
    std::cerr << "\n";
    std::cerr << "\n<operation>";
    std::cerr << "\n    cube: todo: document...";
    std::cerr << "\n    grid: todo: document...";
    std::cerr << "\n";
    std::cerr << "\n<options>";
    std::cerr << "\n    --help,-h:       show this help, --help --verbose for more help";
    std::cerr << "\n    --output-fields: show output fields and exit";
    std::cerr << "\n    --output-format: show binary output format and exit";
    std::cerr << "\n";
    std::cerr << "\ntest-* options";
    std::cerr << "\n    --binary,-b:     output in binary (default is ascii)";
    std::cerr << "\n    --random-seed,--seed,-s=<seed>: seed for random generator (see srand)";
    std::cerr << "\n                                    <seed>: seed for random generator; default: current time";
    std::cerr << "\n";
    std::cerr << "\ncsv options\n";
    std::cerr << comma::csv::options::usage( verbose );
    std::cerr << "\n";
    snark::points_make::box::usage();
    std::cerr << "\n";
    snark::points_make::sphere::usage();
    std::cerr << "\n";
    snark::points_make::test_pattern::cube::usage();
    std::cerr << "\n";
    snark::points_make::test_pattern::grid::usage();
    std::cerr << "\n";
    std::cerr << "\nexamples";
    if( verbose )
    {
        snark::points_make::box::examples();
        snark::points_make::sphere::examples();
        snark::points_make::test_pattern::cube::examples();
        snark::points_make::test_pattern::grid::examples();
    }
    else
    {
        std::cerr << "\n    run points-make --help --verbose to see examples...";
    }
    std::cerr << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" ) ) { bash_completion( argc, argv ); }
        std::vector< std::string > unnamed = options.unnamed( "--help,-h,--fibonacci,--fill,--flush,--random,--verbose,-v", "--.*" );
        if( unnamed.empty() ) { std::cerr << "points-make: please specify operation" << std::endl; return 1; }
        std::string operation = unnamed[0];
        if( operation == "box" ) { return run< snark::points_make::box >( options ); }
        if( operation == "sphere" ) { return run< snark::points_make::sphere >( options ); }
        if( operation == "test-cube" ) { return run< snark::points_make::test_pattern::cube >( options ); }
        else if( operation == "test-grid" ) { return run< snark::points_make::test_pattern::grid >( options ); }
        std::cerr << "points-make: expected operation, got: '" << operation << "'" << std::endl;
    }
    catch( std::exception& ex ) { std::cerr << "points-make: " << ex.what() << std::endl;  }
    catch( ... ) { std::cerr << "points-make: unknown exception" << std::endl; }
    return 1;
}
