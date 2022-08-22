// Copyright (c) 2011 The University of Sydney

/// @author vsevolod vlaskine

#include <memory>
#include <boost/optional.hpp>
#include <boost/scoped_ptr.hpp>
#include <Eigen/Core>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include "../../visiting/eigen.h"
#include "../voxel_map.h"

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "mesh operations" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: " << "cat points.csv | points-mesh <operation> [<options>] > mesh.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations" << std::endl;
    std::cerr << "    grid: each point has pixel index, e.g. as in a point cloud coming from realsense; triangulate neighbour pixels only" << std::endl;
    std::cerr << "          triangle points output order is such that the triangle normal would point towards the viewer" << std::endl;
    std::cerr << "          according the right-hand screw rule" << std::endl;
    std::cerr << "          by default, pixel indices are counted from the top-left corner of an image" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        fields" << std::endl;
    std::cerr << "            index/x, index/y: pixel coordinates" << std::endl;
    std::cerr << "            block: block number" << std::endl;
    std::cerr << "            default: index/x,index/y" << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --input-fields: output input fields and exit" << std::endl;
    std::cerr << "            --reverse: invert order of triangle corners in the output, i.e. 'pixels' will be counted from bottom-left corner" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h:       show this help; --help --verbose for more help" << std::endl;
    std::cerr << "    --verbose,-v:    more output" << std::endl;
    std::cerr << std::endl;
    std::cerr << "csv options" << std::endl;
    std::cerr << comma::csv::options::usage( "x,y", verbose ) << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

class grid
{
    public:
        struct input
        {
            Eigen::Vector2i index;
            comma::uint32 block;

            input() : index( Eigen::Vector2i::Zero() ), block( 0 ) {}
        };
        
        static int run( const comma::command_line_options& options )
        {
            if( options.exists( "--input-fields" ) ) { std::cout << comma::join( comma::csv::names< grid::input >( true ), ',' ) << std::endl; return 0; }
            grid g( options );
            return g.read_();
        }
                
    private:
        comma::csv::options csv_;
        comma::csv::input_stream< input > istream_;
        bool permissive_;
        bool reverse_;
        typedef std::string voxel_;
        typedef snark::voxel_map< voxel_, 2 > voxel_map_t_;
        typedef voxel_map_t_::const_iterator iterator_;
        voxel_map_t_ voxel_map_;
        boost::optional< comma::uint32 > block_;
        
        static comma::csv::options make_csv_options_( const comma::command_line_options& options )
        {
            comma::csv::options csv( options );
            csv.full_xpath = false;
            if( csv.fields.empty() ) { csv.fields = "index"; }
            return csv;
        }
        
        static voxel_map_t_::point_type resolution_( double d ) { return voxel_map_t_::point_type( d, d ); }
        
        grid( const comma::command_line_options& options )
            : csv_( make_csv_options_( options ) )
            , istream_( std::cin, csv_ )
            , permissive_( options.exists( "--permissive" ) )
            , reverse_( options.exists( "--reverse" ) )
            , voxel_map_( resolution_( options.value( "--resolution", 1. ) ) ) // quick and dirty: does not matter for now, but may be used in future
        {
        }
        
        int read_()
        {
            while( istream_.ready() || std::cin.good() )
            {
                const grid::input* p = istream_.read();
                if( !p || ( block_ && *block_ != p->block ) ) { handle_block_(); if( !p ) { break; } }
                block_ = p->block;
                voxel_map_t_::index_type i = {{ p->index.x(), p->index.y() }};
                std::pair< voxel_map_t_::iterator, bool > r = voxel_map_.base_type::insert( std::make_pair( i, voxel_() ) );
                if( !r.second )
                {
                    if( permissive_ ) { continue; }
                    std::cerr << "points-mesh: grid: got duplicated index: " << p->index.x() << "," << p->index.y() << "; if it is intended, use --permissive" << std::endl;
                    return 1;
                }
                if( istream_.is_binary() )
                {
                    r.first->second.resize( csv_.format().size() );
                    std::memcpy( &r.first->second[0], istream_.binary().last(), csv_.format().size() );
                }
                else
                {
                    r.first->second = comma::join( istream_.ascii().last(), csv_.delimiter );
                }
            }
            return 0;
        }
        
        void output_( const iterator_& c1, const iterator_& b, const iterator_& c3 )
        {
            const iterator_& a = reverse_ ? c3 : c1;
            const iterator_& c = reverse_ ? c1 : c3;
            if( csv_.binary() )
            {
                std::cout.write( &a->second[0], csv_.format().size() );
                std::cout.write( &b->second[0], csv_.format().size() );
                std::cout.write( &c->second[0], csv_.format().size() );
            }
            else
            {
                std::cout << a->second << csv_.delimiter << b->second << csv_.delimiter << c->second << std::endl;
            }
        }
        
        void handle_block_()
        {
            for( iterator_ it = voxel_map_.begin(); it != voxel_map_.end(); ++it )
            {
                voxel_map_t_::index_type i;
                i[0] = it->first[0] + 1;
                i[1] = it->first[1];
                iterator_ right = voxel_map_.find( i );
                i[0] = it->first[0];
                i[1] = it->first[1] + 1;
                iterator_ up = voxel_map_.find( i );
                i[0] = it->first[0] + 1;
                i[1] = it->first[1] + 1;
                iterator_ right_up = voxel_map_.find( i );
                if( right_up == voxel_map_.end() )
                {
                    if( right != voxel_map_.end() && up != voxel_map_.end() ) { output_( it, up, right ); }
                }
                else
                {
                    if( right != voxel_map_.end() ) { output_( it, right_up, right ); }
                    if( up != voxel_map_.end() ) { output_( it, up, right_up ); }
                }
                if( right != voxel_map_.end() )
                {
                    i[0] = it->first[0];
                    i[1] = it->first[1] - 1;
                    iterator_ down = voxel_map_.find( i );
                    if( down == voxel_map_.end() ) // if there is no down
                    {
                        i[0] = it->first[0] + 1;
                        i[1] = it->first[1] - 1;
                        iterator_ right_down = voxel_map_.find( i ); // but there is right_down
                        if( right_down != voxel_map_.end() ) { output_( it, right, right_down ); }
                    }
                }
            }
            voxel_map_.clear();
        }
};

namespace comma { namespace visiting {

template <> struct traits< ::grid::input >
{
    template < typename K, typename V > static void visit( const K&, ::grid::input& p, V& v )
    {
        v.apply( "index", p.index );
        v.apply( "block", p.block );
    }

    template < typename K, typename V > static void visit( const K&, const ::grid::input& p, V& v )
    {
        v.apply( "index", p.index );
        v.apply( "block", p.block );
    }
};

} } // namespace comma { namespace visiting {

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        const std::vector< std::string >& unnamed = options.unnamed( "--verbose,-v,--flush,--strict", "-.*" );
        if( unnamed.empty() ) { std::cerr << "points-mesh: please specify operation" << std::endl; return 1; }
        if( unnamed.size() > 1 ) { std::cerr << "points-mesh: expected one operation; got: " << comma::join( unnamed, ';' ) << std::endl; return 1; }
        std::string operation = unnamed[0];
        if( operation == "grid" ) { return ::grid::run( options ); }
        std::cerr << "points-mesh: expected operation; got: \"" << operation << "\"" << std::endl;
    }
    catch( std::exception& ex ) { std::cerr << "points-mesh: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-mesh: unknown exception" << std::endl; }
    return 1;
}
