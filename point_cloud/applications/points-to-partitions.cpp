// Copyright (c) 2011 The University of Sydney

/// @author vsevolod vlaskine

#ifdef WIN32
#include <stdio.h>
#include <fcntl.h>
#include <io.h>
#endif
#include <deque>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <vector>
#include <tbb/version.h>
#if TBB_VERSION_MAJOR < 2021
#include <tbb/pipeline.h>
#else
#include <tbb/parallel_pipeline.h>
#endif
#include <boost/array.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <comma/sync/synchronized.h>
#include <comma/visiting/traits.h>
#include "../../math/interval.h"
#include "../../point_cloud/partition.h"
#include "../../tbb/bursty_reader.h"
#include "../../tbb/types.h"
#include "../../visiting/eigen.h"

#ifdef PROFILE
#include <google/profiler.h>
#endif

static void usage( bool )
{
    std::cerr << std::endl;
    std::cerr << "partition 3d point cloud" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat points.csv | points-to-partitions [<options>] > partitions.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "<options>" << std::endl;
    std::cerr << "    partitioning options:" << std::endl;
    std::cerr << "        --min-id: minimum partition id; default 0" << std::endl;
    std::cerr << "        --min-density: min partition density, i.e: number of points in partition / number of voxels in partition; default: 0" << std::endl;
    std::cerr << "        --min-points-per-voxel <n>: min number of points in a non-empty voxel; default: 1" << std::endl;
    std::cerr << "        --min-voxels-per-partition <n>: min number of voxels in a partition; default: 1" << std::endl;
    std::cerr << "        --min-points-per-partition <n>: min number of points in a partition; default: 1" << std::endl;
    std::cerr << "        --resolution <resolution>: <value> or <x>,<y>,<z>; e.g: --resolution=0.3 or resolution=0.2,0.2,1; default: 0.2 metres" << std::endl;
    std::cerr << "    data flow options:" << std::endl;
    std::cerr << "        --discard,-d: if present, partition as many points as possible, discard the rest" << std::endl;
    std::cerr << "        --output-all: output all points, even non-partitioned; the latter with id: max uint32" << std::endl;
    std::cerr << "        --output-largest=[<n>]; output up to <n> largest partitions, discard the rest" << std::endl;
    std::cerr << "        --verbose, -v: output progress info" << std::endl;
    std::cerr << std::endl;
    std::cerr << "<fields>" << std::endl;
    std::cerr << "    required fields: x,y,z" << std::endl;
    std::cerr << "    default: \"x,y,z\"" << std::endl;
    std::cerr << "    block: data block id, if present, accumulate and partition each data block separately" << std::endl;
    std::cerr << "           if absent, read until the end of file/stream and then partition" << std::endl;
    std::cerr << "    flag: if present, partition only the points with this field equal 1" << std::endl;
    std::cerr << "          if --output-all specified; skipped points will have partition id as max uint32" << std::endl;
    std::cerr << std::endl;
    std::cerr << comma::csv::options::usage() << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    partition all points in points.csv" << std::endl;
    std::cerr << "    cat points.csv | points-to-partitions > partitions.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    partition points.csv individually each data block (e.g. a scan) :" << std::endl;
    std::cerr << "    cat points.csv | points-to-partitions --fields=\",x,y,z,,,block\" > partitions.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    partition a point stream that gives delta only from a socket, if too slow, drop points:" << std::endl;
    std::cerr << "    netcat localhost 1234 | points-to-partitions --fields=\",x,y,z,,,block\" --discard > partitions.csv" << std::endl;
    std::cerr << std::endl;
    exit( -1 );
}

static bool verbose;
static std::size_t min_points_per_voxel = 1;
static std::size_t min_voxels_per_partition = 1;
static std::size_t min_points_per_partition = 1;
static std::size_t output_largest = 0;
static double min_density;
static Eigen::Vector3d resolution;
static comma::csv::options csv;
static comma::uint32 min_id;
static bool discard;
static bool output_all;
static boost::scoped_ptr< snark::partition > partition;

struct input_t
{
    Eigen::Vector3d point;
    bool flag;
    comma::uint32 block;
    const boost::optional< comma::uint32 >* id;

    input_t() : point( 0, 0, 0 ), flag( true ), id( nullptr ) {}
};

namespace comma { namespace visiting {

template <> struct traits< input_t >
{
    template < typename K, typename V > static void visit( const K&, input_t& p, V& v )
    {
        v.apply( "point", p.point );
        v.apply( "block", p.block );
        v.apply( "flag", p.flag );
    }

    template < typename K, typename V > static void visit( const K&, const input_t& p, V& v )
    {
        v.apply( "point", p.point );
        v.apply( "block", p.block );
        v.apply( "flag", p.flag );
    }
};

} } // namespace ark { namespace visiting {

struct block_t // quick and dirty, no optimization for now
{
    typedef std::pair< input_t, std::string > pair_t;
    typedef std::deque< pair_t > pairs_t;
    boost::scoped_ptr< pairs_t > points;
    comma::uint32 id;
    volatile bool empty;
    boost::scoped_ptr< snark::partition > partition;
    block_t() : id( 0 ), empty( true ) {}
    void clear() { partition.reset(); points.reset(); empty = true; }
};

static comma::signal_flag is_shutdown;
static boost::scoped_ptr< snark::tbb::bursty_reader< block_t* > > bursty_reader;

static block_t* read_block_impl_( ::tbb::flow_control* flow = nullptr )
{
    static boost::array< block_t, 3 > blocks;
    static boost::optional< block_t::pair_t > last;
    static comma::uint32 block_id = 0;
    block_t::pairs_t* points = new block_t::pairs_t;
    while( true ) // quick and dirty, only if --discard
    {
        static comma::csv::input_stream< input_t > istream( std::cin, csv );
        points->clear();
        while( true )
        {
            if( last )
            {
                block_id = last->first.block;
                points->push_back( *last );
                last.reset();
            }
            if( is_shutdown || std::cout.bad() || std::cin.bad() || std::cin.eof() )
            {
                if( bursty_reader ) { bursty_reader->stop(); } // quick and dirty, it sucks...
                if( flow ) { flow->stop(); }
                break;
            }
            const input_t* p = istream.read();
            if( !p ) { break; }
            std::string line;
            if( csv.binary() )
            {
                line.resize( csv.format().size() );
                ::memcpy( &line[0], istream.binary().last(), csv.format().size() );
            }
            else
            {
                line = comma::join( istream.ascii().last(), csv.delimiter );
            }
            last = std::make_pair( *p, line );
            if( p->block != block_id ) { break; }
        }
        for( unsigned int i = 0; i < blocks.size(); ++i )
        {
            if( !blocks[i].empty ) { continue; }
            blocks[i].clear();
            blocks[i].id = block_id;
            blocks[i].points.reset( points );
            blocks[i].empty = false;
            return &blocks[i];
        }
    }
}

static block_t* read_block_( ::tbb::flow_control& flow ) { return read_block_impl_( &flow ); }
static block_t* read_block_bursty_() { return read_block_impl_(); }

static void write_block_( block_t* block )
{
    if( !block ) { return; } // quick and dirty for now, only if --discard
        for( std::size_t i = 0; i < block->points->size(); ++i )
    {
        const block_t::pair_t& p = block->points->operator[]( i );
        if( !( p.first.id && *p.first.id ) && !output_all ) { continue; }
        comma::uint32 id = p.first.id && *p.first.id ? **p.first.id : std::numeric_limits< comma::uint32 >::max();
        std::cout.write( &p.second[0], p.second.size() );
        if( csv.binary() ) { std::cout.write( reinterpret_cast< const char* >( &id ), sizeof( comma::uint32 ) ); }
        else { std::cout << csv.delimiter << id << std::endl; }
    }
    std::cout.flush();
    block->clear();
}

static block_t* partition_( block_t* block )
{
    if( !block ) { return nullptr; } // quick and dirty for now, only if --discard
    if( block->points->empty() ) { return block; }
    snark::math::closed_interval< double, 3 > extents;
    for( std::size_t i = 0; i < block->points->size(); ++i ) { extents.set_hull( block->points->operator[](i).first.point ); }
    block->partition.reset( new snark::partition( extents, resolution, min_points_per_voxel ) );
    for( std::size_t i = 0; i < block->points->size(); ++i )
    {
        block_t::pair_t& p = block->points->operator[]( i );
        if( p.first.flag ) { p.first.id = &block->partition->insert( p.first.point ); }
    }
    block->partition->commit( min_voxels_per_partition, min_points_per_partition, min_id, min_density, output_largest );
    return block;
}

int main( int ac, char** av )
{
    try
    {
        #ifdef WIN32
        _setmode( _fileno( stdout ), _O_BINARY );
        #endif
        comma::command_line_options options( ac, av, usage );
        options.assert_mutually_exclusive( "--output-all,--output-largest" );
        csv = comma::csv::options( options, "x,y,z" );
        csv.full_xpath = false;
        min_points_per_voxel = options.value( "--min-points-per-voxel", 1u );
        min_voxels_per_partition = options.value( "--min-voxels-per-partition", 1u );
        min_points_per_partition = options.value( "--min-points-per-partition", 1u );
        min_density = options.value( "--min-density", 0.0 );
        output_largest = options.value( "--output-largest", 0 );
        if( min_points_per_voxel == 0 ) { std::cerr << "points-to-partitions: expected minimum number of points in a non-empty voxel, got zero" << std::endl; }
        verbose = options.exists( "--verbose,-v" );
        std::string resolution_string = options.value< std::string >( "--resolution", "0.2" );
        const std::vector< std::string >& v = comma::split( resolution_string, ',' );
        switch( v.size() )
        {
            case 1: { double r = boost::lexical_cast< double >( v[0] ); resolution = Eigen::Vector3d( r, r, r ); break; }
            case 3: resolution = comma::csv::ascii< Eigen::Vector3d >().get( v ); break;
            default: std::cerr << "points-to-partitions: expected resolution, got: \"" << resolution_string << "\"" << std::endl; return 1;
        }
        discard = options.exists( "--discard,-d" );
        min_id = options.value( "--min-id", 0 );
        output_all = options.exists( "--output-all" );
        snark::tbb::filter< block_t*, block_t* >::type partition_filter( snark::tbb::filter_mode::serial_in_order, &partition_ );
        snark::tbb::filter< block_t*, void >::type write_filter( snark::tbb::filter_mode::serial_in_order, &write_block_ );
        #ifdef PROFILE
        ProfilerStart( "points-to-partitions.prof" ); {
        #endif
        if( discard )
        {
            bursty_reader.reset( new snark::tbb::bursty_reader< block_t* >( &read_block_bursty_ ) );
            snark::tbb::filter< void, void >::type filters = bursty_reader->filter() & partition_filter & write_filter;
            ::tbb::parallel_pipeline( 3, filters ); // while( bursty_reader->wait() ) { ::tbb::parallel_pipeline( 3, filters ); }
            bursty_reader->join();
        }
        else
        {
            snark::tbb::filter< void, block_t* >::type read_filter( snark::tbb::filter_mode::serial_in_order, &read_block_ );
            snark::tbb::filter< void, void >::type filters = read_filter & partition_filter & write_filter;
            ::tbb::parallel_pipeline( 3, filters );
        }
        #ifdef PROFILE
        ProfilerStop(); }
        #endif
        if( verbose ) { std::cerr << "points-to-partitions: " << ( is_shutdown ? "caught signal" : "end of stream" ) << std::endl; }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "points-to-partitions: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-to-partitions: unknown exception" << std::endl; }
    return 1;
}
