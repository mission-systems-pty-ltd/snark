// Copyright (c) 2011 The University of Sydney

#include <array>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <comma/base/exception.h>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/types.h>
#include <comma/csv/ascii.h>
#include <comma/csv/stream.h>
#include <comma/csv/impl/program_options.h>
#include <comma/visiting/traits.h>
#include "../../visiting/eigen.h"
#include "../voxel_map.h"

struct input_point
{
    Eigen::Vector3d point;
    comma::uint32 block;

    input_point() : point( 0, 0, 0 ), block( 0 ) {}
};

struct centroid
{
    std::array< comma::int32, 3 > index;
    Eigen::Vector3d mean;
    comma::uint32 size;
    comma::uint32 block;

    centroid() : mean( 0, 0, 0 ), size( 0 ), block( 0 ) {}

    void operator+=( const Eigen::Vector3d& point )
    {
        ++size;
        mean = ( mean * ( size - 1 ) + point ) / size;
    }
};

namespace comma { namespace visiting {

template <> struct traits< input_point >
{
    template < typename K, typename V > static void visit( const K&, input_point& p, V& v )
    {
        v.apply( "point", p.point );
        v.apply( "block", p.block );
    }

    template < typename K, typename V > static void visit( const K&, const input_point& p, V& v )
    {
        v.apply( "point", p.point );
        v.apply( "block", p.block );
    }
};

template <> struct traits< centroid >
{
    template < typename K, typename V > static void visit( const K&, centroid& p, V& v )
    {
        v.apply( "index", p.index );
        v.apply( "mean", p.mean );
        v.apply( "size", p.size );
        v.apply( "block", p.block );
    }

    template < typename K, typename V > static void visit( const K&, const centroid& p, V& v )
    {
        v.apply( "index", p.index );
        v.apply( "mean", p.mean );
        v.apply( "size", p.size );
        v.apply( "block", p.block );
    }
};

} } // namespace comma { namespace visiting {

template < typename T, std::size_t Size >
std::ostream& operator<<( std::ostream& os, const std::array< T, Size >& a )
{
    for( std::size_t i = 0; i < Size; ++i ) { os << a[i] << " "; }
    return os;
}

int main( int argc, char** argv )
{
    try
    {
        std::string origin_string;
        std::string resolution_string;
        boost::program_options::options_description description( "options" );
        comma::uint32 min_size;
        comma::int32 neighbourhood_radius;
        description.add_options()
            ( "help,h", "display help message" )
            ( "centers", "output voxel centers instead of means" )
            ( "min-size", boost::program_options::value< comma::uint32 >( &min_size )->default_value( 0 ), "if non-zero, output only voxels having more than a given number of points" )
            ( "resolution", boost::program_options::value< std::string >( &resolution_string ), "voxel map resolution, e.g. \"0.2\" or \"0.2,0.2,0.5\"" )
            ( "origin", boost::program_options::value< std::string >( &origin_string )->default_value( "0,0,0" ), "voxel map origin" )
            ( "output-fields", "print output fields to stdout and exit" )
            ( "output-format", "print binary output format to stdout and exit" )
            ( "neighbourhood-radius,r", boost::program_options::value< comma::int32 >( &neighbourhood_radius )->default_value( 0 ), "calculate count of neighbours at given radius" );
        description.add( comma::csv::program_options::description( "x,y,z" ) );
        boost::program_options::variables_map vm;
        boost::program_options::store( boost::program_options::parse_command_line( argc, argv, description), vm );
        boost::program_options::notify( vm );
        if ( vm.count( "help" ) )
        {
            std::cerr << "downsample a point cloud using a voxel map" << std::endl;
            std::cerr << std::endl;
            std::cerr << "usage: cat points.csv | points-to-voxels [options] > voxels.csv" << std::endl;
            std::cerr << std::endl;
            std::cerr << "input: points: x,y,z[,block]; default: x,y,z" << std::endl;
            std::cerr << "output: voxels with indices, centroids, and weights (number of points): i,j,k,x,y,z,weight[,neighbour count][,block]" << std::endl;
            std::cerr << "binary output format: 3ui,3d,ui[,ui][,ui]" << std::endl;
            std::cerr << std::endl;
            std::cerr << description << std::endl;
            std::cerr << std::endl;
            return 0;
        }
        comma::csv::options csv = comma::csv::program_options::get( vm );
        csv.full_xpath = false;
        comma::csv::input_stream< input_point > istream( std::cin, csv );
        comma::csv::options output_csv = csv;
        if( csv.has_field( "block" ) ) // todo: quick and dirty, make output fields configurable?
        {
            if( vm.count( "output-fields" ) > 0 ) { std::cout << "index/x,index/y,index/z,mean/x,mean/y,mean/z,size,block" << std::endl; return 0; }
            if( vm.count( "output-format" ) > 0 ) { std::cout << "3ui,3d,ui,ui" << std::endl; return 0; }
            output_csv.fields = "index,mean,size,block";
            if( csv.binary() ) { output_csv.format( "3ui,3d,ui,ui" ); }
        }
        else
        {
            if( vm.count( "output-fields" ) > 0 ) { std::cout << "index/x,index/y,index/z,mean/x,mean/y,mean/z,size" << std::endl; return 0; }
            if( vm.count( "output-format" ) > 0 ) { std::cout << "3ui,3d,ui" << std::endl; return 0; }
            output_csv.fields = "index,mean,size";
            if( csv.binary() ) { output_csv.format( "3ui,3d,ui" ); }
        }
        if( vm.count( "resolution" ) == 0 ) { COMMA_THROW( comma::exception, "please specify --resolution" ); }
        Eigen::Vector3d origin;
        Eigen::Vector3d resolution;
        comma::csv::ascii< Eigen::Vector3d >().get( origin, origin_string );
        if( resolution_string.find_first_of( ',' ) == std::string::npos ) { resolution_string = resolution_string + ',' + resolution_string + ',' + resolution_string; }
        comma::csv::ascii< Eigen::Vector3d >().get( resolution, resolution_string );
        comma::csv::output_stream< centroid > ostream( std::cout, output_csv );
        comma::signal_flag is_shutdown;
        unsigned int block = 0;
        const input_point* last = nullptr;
        bool output_centers = vm.count( "centers" );
        while( !is_shutdown && !std::cin.eof() && std::cin.good() )
        {
            snark::voxel_map< centroid, 3 > voxels( origin, resolution );
            if( last ) { voxels.touch_at( last->point )->second += last->point; }
            while( !is_shutdown && !std::cin.eof() && std::cin.good() )
            {
                last = istream.read();
                if( !last || last->block != block ) { break; }
                voxels.touch_at( last->point )->second += last->point;
            }
            if( is_shutdown ) { break; }
//             for( snark::voxel_map< centroid, 3 >::iterator it = voxels.begin(); it != voxels.end(); ++it )
//             {
//                 it->second.block = block;
//                 it->second.index = snark::voxel_map< centroid, 3 >::index_of( it->second.mean, origin, resolution );
//                 if( neighbourhood_radius > 0 ) // quick and dirty
//                 {
//                     snark::voxel_map< centroid, 3 >::index_type index;
//                     snark::voxel_map< centroid, 3 >::index_type begin = {{ it->first[0] - neighbourhood_radius, it->first[1] - neighbourhood_radius, it->first[2] - neighbourhood_radius }};
//                     snark::voxel_map< centroid, 3 >::index_type end = {{ it->first[0] + neighbourhood_radius + 1, it->first[1] + neighbourhood_radius + 1, it->first[2] + neighbourhood_radius + 1 }};
//                     std::size_t size = 0;
//                     Eigen::Vector3d mean( 0, 0, 0 );
//                     for( index[0] = begin[0]; index[0] < end[0]; ++index[0] )
//                     {
//                         for( index[1] = begin[1]; index[1] < end[1]; ++index[1] )
//                         {
//                             for( index[2] = begin[2]; index[2] < end[2]; ++index[2] )
//                             {
//                                 snark::voxel_map< centroid, 3 >::const_iterator nit = voxels.find( index );
//                                 if( nit == voxels.end() ) { continue; }
//                                 ostream.write( it->second ); // todo: remove!!
//                                 size += nit->second.size;
//                                 //mean += ( nit->second.mean * nit->second.size );
//                             }
//                         }
//                     }
//                     //it->second.size = size;
//
//                     //mean /= size;
//                     //it->second.mean = mean;
//                 }
//                 ostream.write( it->second );
//             }

            auto center = [&]( const std::array< comma::int32, 3 >& i ) -> Eigen::Vector3d { return origin + Eigen::Vector3d( ( i[0] + 0.5 ) * resolution[0], ( i[1] + 0.5 ) * resolution[1], ( i[2] + 0.5 ) * resolution[2] ); };
            for( snark::voxel_map< centroid, 3 >::iterator it = voxels.begin(); it != voxels.end(); ++it )
            {
                if( min_size > 0 && it->second.size < min_size ) { continue; }
                it->second.block = block;
                it->second.index = snark::voxel_map< centroid, 3 >::index_of( it->second.mean, origin, resolution );
                if( output_centers ) { it->second.mean = center( it->second.index ); }
                if( neighbourhood_radius == 0 )
                {
                    ostream.write( it->second );
                }
                else
                {
                    centroid c = it->second;
                    snark::voxel_map< centroid, 3 >::index_type index;
                    snark::voxel_map< centroid, 3 >::index_type begin = {{ static_cast<int>(it->first[0] - neighbourhood_radius), static_cast<int>(it->first[1] - neighbourhood_radius), static_cast<int>(it->first[2] - neighbourhood_radius) }};
                    snark::voxel_map< centroid, 3 >::index_type end = {{ static_cast<int>(it->first[0] + neighbourhood_radius + 1), static_cast<int>(it->first[1] + neighbourhood_radius + 1), static_cast<int>(it->first[2] + neighbourhood_radius + 1) }};
                    for( index[0] = begin[0]; index[0] < end[0]; ++index[0] )
                    {
                        for( index[1] = begin[1]; index[1] < end[1]; ++index[1] )
                        {
                            for( index[2] = begin[2]; index[2] < end[2]; ++index[2] )
                            {
                                snark::voxel_map< centroid, 3 >::const_iterator nit = voxels.find( index );
                                if( nit == voxels.end() ) { continue; }
                                c.size += nit->second.size;
                                c.mean += ( nit->second.mean * nit->second.size );
                            }
                        }
                    }
                    if( min_size > 0 && c.size < min_size ) { continue; }
                    if( output_centers ) { c.mean = origin + Eigen::Vector3d( ( c.index[0] + 0.5 ) * resolution[0], ( c.index[1] + 0.5 ) * resolution[1], ( c.index[2] + 0.5 ) * resolution[2] ); }
                    else { c.mean /= c.size; }
                    ostream.write( c );
                }
            }
            if( !last ) { break; }
            block = last->block;
        }
        if( is_shutdown ) { std::cerr << "points-to-voxels: caught signal" << std::endl; return 1; }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "points-to-voxels: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-to-voxels: unknown exception" << std::endl; }
    return 1;
}
