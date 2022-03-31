// Copyright (c) 2018 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <sstream>
#include <unordered_map>
#include <Eigen/Core>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include "../../../visiting/eigen.h"
#include "../../voxel_map.h"
#include "thin.h"

namespace snark { namespace points_calc { namespace thin {

std::string traits::usage()
{
    std::ostringstream oss;
    oss << "    thin" << std::endl;
    oss << "        read input points and thin it down" << std::endl;
    oss << "        input fields: " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    oss << std::endl;
    oss << "        thins data uniformly across space by creating voxels and thinning" << std::endl;
    oss << "        within each; if block field is present, clears voxel grid on a new block" << std::endl;
    oss << std::endl;
    oss << "        options" << std::endl;
    oss << "            --block-pass: output only the points with the block of the point first to hit a voxel;" << std::endl;
    oss << "                          if id field present, for each id output only the points of the same block" << std::endl;
    oss << "                          of the point first with this id to hit a voxel" << std::endl;
    oss << "            --id-filter: output only the first point with a given id in a voxel" << std::endl;
    oss << "                         if no id field present, equivalent to --points-per-voxel=1" << std::endl;
    oss << "            --id-pass: output only the points with the id of the point first to hit a voxel," << std::endl;
    oss << "                       if no id field present, passes all points through" << std::endl;
    oss << "            --points-per-voxel=[<n>]: number of points to retain in each voxel" << std::endl;
    oss << "                                      takes the first <n> points" << std::endl;
    oss << "            --rate: rate of thinning, between 0.0 and 1.0" << std::endl;
    oss << "            --resolution=<value>; e.g. either --resolution=0.1 or for non-cubic voxels: --resolution=0.1,0.2,0.3" << std::endl;
    oss << "                                  if --linear (DEPRECATED), minimum distance between points" << std::endl;
    oss << std::endl;
    oss << "      --linear: DEPRECATED; use trajectory-thin" << std::endl;
    oss << std::endl;
    return oss.str();
}

struct point
{
    Eigen::Vector3d coordinates;
    comma::uint32 block;
    comma::uint32 id;
    point() : coordinates( Eigen::Vector3d::Zero() ), block( 0 ), id( 0 ) {}
};

typedef point input;

} } } // namespace snark { namespace points_calc { namespace thin {

namespace comma { namespace visiting {

template <> struct traits< snark::points_calc::thin::input >
{
    template< typename K, typename V > static void visit( const K&, const snark::points_calc::thin::input& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "block", t.block );
        v.apply( "id", t.id );
    }

    template< typename K, typename V > static void visit( const K&, snark::points_calc::thin::input& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "block", t.block );
        v.apply( "id", t.id );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace points_calc { namespace thin {

std::string traits::input_fields() { return comma::join( comma::csv::names< input >( true ), ',' ); }

std::string traits::input_format() { return comma::csv::format::value< input >(); }

std::string traits::output_fields() { return ""; } // appends no fields

std::string traits::output_format() { return ""; } // appends no fields

class proportional
{
    public:
        proportional( double rate ) : increment_( 1.0 / rate ) { count_ = ( std::isinf( increment_ ) ? 0.0 : increment_ / 2.0 ); }

        bool keep( const point& )
        {
            count_ += 1.0;
            if( count_ < increment_ ) { return false; }
            count_ -= increment_;
            return true;
        }

        const proportional& updated( const point& ) const { return *this; }

    private:
        double increment_;
        double count_;
};

class points_per_voxel
{
    public:
        points_per_voxel( unsigned int points_per_voxel ): available_( points_per_voxel ) {}

        bool keep( const point& )
        {
            if( available_ == 0 ) { return false; }
            --available_;
            return true;
        }

        const points_per_voxel& updated( const point& ) const { return *this; }

    private:
        unsigned int available_;
};

class id_filter
{
    public:
        id_filter(): keep_( true ) {}
        bool keep( const point& ) const { bool k = keep_; keep_ = false; return k; } // quick and dirty
        id_filter updated( const point& p ) const { return *this; }

    private:
        mutable bool keep_; // quick and dirty
};

class id_pass
{
    public:
        id_pass(): id_( 0 ) {}
        id_pass( comma::uint32 id ): id_( id ) {}
        bool keep( const point& p ) const { return p.id == id_; }
        id_pass updated( const point& p ) const { return id_pass( p.id ); }

    private:
        comma::uint32 id_;
};

class block_pass
{
    public:
        block_pass(): block_( 0 ) {}
        block_pass( comma::uint32 block ): block_( block ) {}
        bool keep( const point& p ) const { return p.block == block_; }
        block_pass updated( const point& p ) const { return block_pass( p.block ); }

    private:
        comma::uint32 block_;
};

static comma::csv::options csv;
static Eigen::Vector3d resolution;

template < typename T >
static snark::voxel_map< T, 3 >& update_id( std::unordered_map< comma::uint32, snark::voxel_map< T, 3 > >& grids, const point& p )
{
    auto it = grids.find( p.id );
    if( it == grids.end() ) { it = grids.insert( std::make_pair( p.id, snark::voxel_map< T, 3 >( resolution ) ) ).first; }
    return it->second;
}

template < typename T >
static void update_block( std::unordered_map< comma::uint32, snark::voxel_map< T, 3 > >& grids, const point& p )
{
    static comma::uint32 block = 0;
    if( block != p.block ) { for( auto& g: grids ) { g.second.clear(); } }
    block = p.block;
}

template <> void update_block< block_pass >( std::unordered_map< comma::uint32, snark::voxel_map< block_pass, 3 > >&, const point& ) {}

template <> snark::voxel_map< id_pass, 3 >& update_id( std::unordered_map< comma::uint32, snark::voxel_map< id_pass, 3 > >& grids, const point& )
{
    if( grids.empty() ) { grids.insert( std::make_pair( 0, snark::voxel_map< id_pass, 3 >( resolution ) ) ); } // quick and dirty
    return grids.begin()->second;
}

template < typename T >
static int process( const T& thinner )
{
    std::unordered_map< comma::uint32, snark::voxel_map< T, 3 > > grids;
    comma::csv::input_stream< point > istream( std::cin, csv );
    comma::csv::passed< point > passed( istream, std::cout );
    while( istream.ready() || std::cin.good() )
    {
        const point* p = istream.read();
        if( !p ) { break; }
        update_block< T >( grids, *p );
        T* t = &update_id< T >( grids, *p ).insert( p->coordinates, thinner.updated( *p ) ).first->second;
        if( !t->keep( *p ) ) { continue; }
        passed.write();
        if( csv.flush ) { std::cout.flush(); }
    }
    return 0;
}

int traits::run( const comma::command_line_options& options )
{
    csv = comma::csv::options( options );
    if( options.exists( "--linear" ) ) { std::cerr << "points-calc: thin: --linear: DEPRECATED; use trajectory-thin instead" << std::endl; return 1; }
    if( csv.fields.empty() ) { csv.fields = "x,y,z"; }
    csv.full_xpath = false;
    options.assert_mutually_exclusive( "--block-pass,--id-filter,-id-pass,--points-per-voxel,--rate" );
    auto s = options.value< std::string >( "--resolution" );
    switch( comma::split( s, ',' ).size() )
    {
        case 1: { double v = boost::lexical_cast< double >( s ); resolution = Eigen::Vector3d( v, v, v ); break; }
        case 3: resolution = comma::csv::ascii< Eigen::Vector3d >().get( s ); break;
        default: std::cerr << "points-calc: thin: expected --resolution as <value> or <x>,<y>,<z>; got: '" << s << "'" << std::endl; return 1;
    }
    if( options.exists( "--rate" ) ) { return process( thin::proportional( options.value< double >( "--rate" ) ) ); }
    if( options.exists( "--points-per-voxel" ) ) { return process( thin::points_per_voxel( options.value< unsigned int >( "--points-per-voxel" ) ) ); }
    if( options.exists( "--id-filter" ) ) { return process( thin::id_filter() ); }
    if( options.exists( "--id-pass" ) ) { return process( thin::id_pass() ); }
    if( options.exists( "--block-pass" ) ) { return process( thin::block_pass() ); }
    std::cerr << "points-calc: thin: please specify either --rate, or --points-per-voxel, or --id-filter, or --id-pass" << std::endl;
    return 1;
}

} } } // namespace snark { namespace points_calc { namespace thin {
