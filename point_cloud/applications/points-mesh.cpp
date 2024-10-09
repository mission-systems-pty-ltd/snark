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
    std::cerr << R"(
mesh operations

usage: " << "cat points.csv | points-mesh <operation> [<options>] > mesh.csv

operations
    grid:   each point has 'pixel' index, e.g. as in a point cloud coming
            from realsense; triangulate neighbour pixels only triangle points
            output order is such that the triangle normal would point towards
            the viewer according the right-hand screw rule by default, pixel
            indices are counted from the top-left corner of an image
        fields
            index/x, index/y: pixel coordinates
            block: block number
            default: index/x,index/y
        options
            --input-fields: output input fields and exit
            --reverse: invert order of triangle corners in the output, i.e.
                       'pixels' will be counted from bottom-left corner

    scan:   each point has 'pixel' index; connect points in a scan pattern
            todo: more documentation, example
        fields
            index/x, index/y: pixel coordinates
            block: block number
            default: index/x,index/y
        options
            --axis=<name>; default=x; choices: x, y
            --input-fields: output input fields and exit

options
    --help,-h:       show this help; --help --verbose for more help
    --verbose,-v:    more output

csv options)" << std::endl;
    std::cerr << comma::csv::options::usage( "x,y", verbose ) << std::endl;
    std::cerr << std::endl;
    std::cerr << R"(examples
    grid
        csv-paste 'line-number;shape=10,15' value=0 --head 150 \
            | points-mesh grid \
            | view-points '-;shape=triangle'
    scan
        csv-paste 'line-number;shape=10,15' value=0 --head 150 \
            | points-mesh scan \
            | view-points '-;shape=line'
        csv-paste 'line-number;shape=10,15' value=0 --head 150 \
            | points-mesh scan --axis=y \
            | view-points '-;shape=line'
)" << std::endl;
    exit( 0 );
}

class gridlike
{
    public:
        virtual ~gridlike() = default;

        struct input
        {
            Eigen::Vector2i index{0, 0};
            comma::uint32 block{0};
        };
        
        template < typename G > static int run( const comma::command_line_options& options )
        {
            if( options.exists( "--input-fields" ) ) { std::cout << comma::join( comma::csv::names< gridlike::input >( true ), ',' ) << std::endl; return 0; }
            G g( options );
            return g.read_();
        }
                
    protected:
        comma::csv::options csv_;
        comma::csv::input_stream< input > istream_;
        bool permissive_;
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
        
        gridlike( const comma::command_line_options& options )
            : csv_( make_csv_options_( options ) )
            , istream_( std::cin, csv_ )
            , permissive_( options.exists( "--permissive" ) )
            , voxel_map_( resolution_( options.value( "--resolution", 1. ) ) ) // quick and dirty: does not matter for now, but may be used in future
        {
        }

        virtual void handle_block_() = 0;
        
        int read_()
        {
            while( istream_.ready() || std::cin.good() )
            {
                const gridlike::input* p = istream_.read();
                if( !p || ( block_ && *block_ != p->block ) )
                {
                    handle_block_();
                    voxel_map_.clear();
                    if( !p ) { break; }
                }
                block_ = p->block;
                voxel_map_t_::index_type i = {{ p->index.x(), p->index.y() }};
                std::pair< voxel_map_t_::iterator, bool > r = voxel_map_.as_map::insert( std::make_pair( i, voxel_() ) );
                if( !r.second )
                {
                    if( permissive_ ) { continue; }
                    comma::say() << "grid: got duplicated index: " << p->index.x() << "," << p->index.y() << "; if it is intended, use --permissive" << std::endl;
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
};

class grid : public gridlike
{
    public:
        grid( const comma::command_line_options& options ): gridlike( options ), reverse_( options.exists( "--reverse" ) ) {}
        
    protected:
        bool reverse_{false};

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
};

class scan : public gridlike
{
    public:
        scan( const comma::command_line_options& options ): gridlike( options ), _x_is_axis( options.value< std::string >( "--axis", "x" ) == "x" ) {}
        
    protected:
        bool _x_is_axis{true};

        void handle_block_()
        {
            for( iterator_ it = voxel_map_.begin(); it != voxel_map_.end(); ++it )
            {
                // todo!
            }
        }

        void output_( const iterator_& a, const iterator_& b )
        {
            if( csv_.binary() )
            {
                std::cout.write( &a->second[0], csv_.format().size() );
                std::cout.write( &b->second[0], csv_.format().size() );
            }
            else
            {
                std::cout << a->second << csv_.delimiter << b->second << std::endl;
            }
        }
};

namespace comma { namespace visiting {

template <> struct traits< ::gridlike::input >
{
    template < typename K, typename V > static void visit( const K&, ::gridlike::input& p, V& v )
    {
        v.apply( "index", p.index );
        v.apply( "block", p.block );
    }

    template < typename K, typename V > static void visit( const K&, const ::gridlike::input& p, V& v )
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
        COMMA_ASSERT_BRIEF( !unnamed.empty(), "please specify operation" );
        COMMA_ASSERT_BRIEF( unnamed.size() == 1, "expected one operation; got: " << comma::join( unnamed, ' ' ) );
        std::string operation = unnamed[0];
        if( operation == "grid" ) { return ::gridlike::run< grid >( options ); }
        if( operation == "scan" ) { return ::gridlike::run< scan >( options ); }
        comma::say() << "expected operation; got: '" << operation << "'" << std::endl;
    }
    catch( std::exception& ex ) { comma::say() << ex.what() << std::endl; }
    catch( ... ) { comma::say() << "unknown exception" << std::endl; }
    return 1;
}
