// Copyright (c) 2017 The University of Sydney
// Copyright (c) 2021 Mission Systems Pty Ltd

#include "../../file-util.h"
#include "std_msgs/traits.h"
#include <comma/io/stream.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <vector>
#include <fstream>

namespace {

void bash_completion( unsigned const ac, char const * const * av )
{
    static char const * const arguments[] = {
        " --binary -b --delimiter -d --fields -f --flush --format --help -h --precision --verbose -v",
        " --dimension --dim --from node-name --to --bags"
    };
    std::cout << arguments << std::endl;
    exit( 0 );
}

void usage( bool const verbose )
{
    static const char* const indent="    ";

    std::cerr << std::endl;
    std::cerr << "Convert mult dimensional array from ros to csv and vice versa. Binary i/o only." << std::endl;
    std::cerr << std::endl;
    std::cerr << "Usage:" << std::endl;
    std::cerr << indent << comma::verbose.app_name() << " <operation> [<options>...]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << "    --help,-h; print help and exit." << std::endl;
    std::cerr << "    --node-name=<name>; node name for this process, when not specified uses ros::init_options::AnonymousName flag." << std::endl;
    std::cerr << "    --queue-size=[<n>]; default=1; ROS Subscriber queue size." << std::endl;
    std::cerr << "    --type=<data_type>; csv format types. One of b,ub,w,uw,i,ui,l,ul,f or d" << std::endl;
    std::cerr << "    --from=<topic>; ros topic, mutually exclusive with --to." << std::endl;
    std::cerr << "        --bags=[<bagfiles>]; read from bag files." << std::endl;
    std::cerr << "        --flush; flush stream after each stride." << std::endl;
    std::cerr << "        --output-format; print output format and exit." << std::endl;
    std::cerr << "    --to=<topic>; ros topic, mutually exclusive with --from." << std::endl;
    std::cerr << "        --dimension,--dim=<attr>; dimension attributes label,size,stride. For example --dim=rows,3,9." << std::endl;
    std::cerr << "        --input-format; print format according to given dimension attributes and exit." << std::endl;
    std::cerr << "        --max-datagram-size=[<size>]; If a UDP transport is used, specifies the maximum datagram size (see ros::TransportHints)." << std::endl;
    std::cerr << "        --latch;  ROS publisher option; If true, the last message published on this topic will be saved and sent to new subscribers when they connect" << std::endl;
    std::cerr << std::endl;
}

void ros_init( char **av, boost::optional< std::string > node_name, std::string const& suffix )
{
    uint32_t node_options = 0;
    int rac = 1;
    if( !node_name )
    {
        node_name = "ros_multi_array" + suffix;
        node_options = ::ros::InitOption::AnonymousName;
    }
    ros::init( rac, av, *node_name, node_options );
}

template< typename T >
static std::string ros_layout_format( std_msgs::MultiArrayLayout const& layout )
{
    return std::to_string( layout.dim[0].stride ) + snark::ros::type_to_std_msgs< T >::as_string();
}

template< typename T >
class ros_subscriber
{
public:
    using array_type = typename snark::ros::type_to_std_msgs< T >::array_type;
    using message_type = typename array_type::ConstPtr;

    ros_subscriber( comma::command_line_options const& options )
        : flush( options.exists( "--flush" ) )
        , output_format( options.exists( "--output-format" ) )
        , from_bag( options.exists( "--bags" ) )
    {
        if( from_bag )
        {
            for( auto name: comma::split( options.value< std::string >( "--bags", "" ), ',' ))
            {
                std::vector< std::string > expansion = snark::ros::glob( name );
                bag_names.insert( bag_names.end(), expansion.begin(), expansion.end() );
            }
        }
        else
        {
            auto datagram = options.optional< int >( "--max-datagram-size" );

            ros::TransportHints hints;
            if( datagram ) { hints = ros::TransportHints().maxDatagramSize( *datagram ); }
            subscriber_ = node_.subscribe( options.value< std::string >( "--from" )
                    , options.value< unsigned >( "--queue-size", 1U )
                    , &ros_subscriber::process, this
                    , hints );
        }
    }

    void write( message_type const msg )
    {
        std::cout.write( reinterpret_cast< char const* >( msg->data.data() + msg->layout.data_offset ), sizeof( T ) * msg->layout.dim[ 0 ].stride );
        if( flush ) { std::cout.flush(); }
    }

    void process( message_type const msg )
    {
        if( output_format ) { std::cout << ros_layout_format< T >( msg->layout ) << std::endl; ros::shutdown(); return; }
        write( msg );
        if( !std::cout.good() ) { ros::shutdown(); }
    }

    void subscribe( void )
    {
        if( from_bag )
        {
            rosbag::Bag bag;
            for( auto bag_name: bag_names )
            {
                comma::verbose << "opening " << bag_name << std::endl;
                rosbag::View view_;
                bag.open( bag_name );
                for( rosbag::MessageInstance const mi : rosbag::View( bag, rosbag::TopicQuery( topic )))
                {
                    message_type const msg = mi.instantiate< array_type >();
                    write( msg );
                }
                bag.close();
            }
        }
        else
        {
            ros::spin();
        }
    }

private:
    bool const flush;
    bool const output_format;
    bool const from_bag;
    ros::NodeHandle node_;
    ros::Subscriber subscriber_;
    std::vector< std::string > bag_names;
    std::string topic;
};


template< typename T >
class ros_publisher
{
public:
    static std_msgs::MultiArrayLayout get_layout( std::vector< std::string > const& dim_strings )
    {
        std_msgs::MultiArrayLayout layout;
        layout.data_offset = 0;

        comma::csv::ascii< std_msgs::MultiArrayDimension > dim_parse( "label,size,stride" );
        for( auto const& si : dim_strings )
        {
            layout.dim.push_back( dim_parse.get( si ) );
        }
        return layout;
    }

    ros_publisher( comma::command_line_options const& options )
    {
        message_.layout = get_layout(options.values< std::string >( "--dimensions,--dim" ) );
        message_.data.resize( message_.layout.dim[ 0 ].stride );
        publisher_ = node_.advertise< typename snark::ros::type_to_std_msgs< T >::array_type >( options.value< std::string >( "--to" )
                , options.value< unsigned >( "--queue-size", 1U )
                , options.exists( "--latch" ) );
        ros::spinOnce();
    }

    std::string input_format() { return ros_layout_format< T >( message_.layout ); }

    void publish()
    {
        while( std::cin.good() )
        {
            std::cin.read( reinterpret_cast< char* >( message_.data.data() ), sizeof( T ) * message_.layout.dim[ 0 ].stride );
            publisher_.publish( message_ );
        }
    }

private:
    ros::NodeHandle node_;
    ros::Publisher publisher_;
    typename snark::ros::type_to_std_msgs< T >::array_type message_;
};

template< typename T >
void ros_execute( char **av, comma::command_line_options const& options )
{
    if( options.exists( "--from" ) )
    {
        ros_init( av, options.optional< std::string >( "--node-name" ), "_subscriber" );
        ros_subscriber< T > subscriber( options );
        subscriber.subscribe();
    }
    else
    {
        if( options.exists( "--input-format" ) )
        {
            std::cout << ros_layout_format< T >( ros_publisher< T >::get_layout( options.values< std::string >( "--dimensions,--dim" ) ) ) << std::endl; return;
        }
        ros_init( av, options.optional< std::string >( "--node-name" ), "_publisher" );
        ros_publisher< T > publisher( options );
        publisher.publish();
    }
}

}

int main( int ac, char* av[] )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--bash-completion" ) ) bash_completion( ac, av );
        options.assert_mutually_exclusive( "--from,--flush,--output-fields", "--to,--dimensions,--dim,--input-fields" );
        comma::csv::options csv( options );

        auto const type = options.value< std::string >( "--type" );
        if( 1 == type.size() )
        {
            switch( type[0] )
            {
                case 'b': ros_execute< char >( av, options ); return 0;
                case 'w': ros_execute< comma::int16 >( av, options ); return 0;
                case 'i': ros_execute< comma::int32 >( av, options ); return 0;
                case 'l': ros_execute< comma::int64 >( av, options ); return 0;
                case 'f': ros_execute< float >( av, options ); return 0;
                case 'd': ros_execute< double >( av, options ); return 0;
            }
        } else if( 2 == type.size() && 'u' == type[0] )
        {
            switch( type[0] )
            {
                case 'b': ros_execute< unsigned char >( av, options ); return 0;
                case 'w': ros_execute< comma::uint16 >( av, options ); return 0;
                case 'i': ros_execute< comma::uint32 >( av, options ); return 0;
                case 'l': ros_execute< comma::uint64 >( av, options ); return 0;
            }
        }
        std::cerr << comma::verbose.app_name() << ": unknown --type: " << type << std::endl;
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": unknown exception" << std::endl; }
    return 1;
}

