// Copyright (c) 2019 The University of Sydney
// Copyright (c) 2021 Mission Systems Pty Ltd

#include "../../file-util.h"
#include "../../ros-cv-format.h"
#include "../../../imaging/cv_mat/serialization.h"
#include "../../../imaging/cv_mat/traits.h"
#include <comma/application/signal_flag.h>
#include <comma/io/stream.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>

void bash_completion( unsigned const ac, char const * const * av )
{
    static char const * const arguments =
        " --help -h --node-name --queue-size"
        " --from --bags --flush"
        " --to --max-datagram-size --latch --frame"
        ;
    std::cout << arguments << std::endl;
    exit( 0 );
}

void usage( bool )
{
    std::cerr << "\nconvert image from ros to cv image and vice-versa";
    std::cerr << "\n";
    std::cerr << "\nusage: " << comma::verbose.app_name() << " --from=<topic> [<options>]";
    std::cerr << "\n       " << comma::verbose.app_name() << " --to=<topic> [<options>]";
    std::cerr << "\n";
    std::cerr << "\ngeneral options:";
    std::cerr << "\n    --help,-h:                 print help and exit";
    std::cerr << "\n    --node,--node-name=<name>: default=ros_image_<publisher|subscriber>";
    std::cerr << "\n    --queue-size=[<n>]:        default=1; ROS queue size";
    std::cerr << "\n";
    std::cerr << "\n    when --node is not specified also sets ros::init_options::AnonymousName flag";
    std::cerr << "\n";
    std::cerr << "\nfrom options:";
    std::cerr << "\n    --from=<topic>:   topic to read";
    std::cerr << "\n    --bags=[<files>]: comma-separated list of bag files, wildcards accepted";
    std::cerr << "\n    --flush:          flush stream after each image";
    std::cerr << "\n";
    std::cerr << "\nto options:";
    std::cerr << "\n    --to=<topic>:                 topic to publish to";
    std::cerr << "\n    --frame=[<frame_id>]:         ros header frame id";
    std::cerr << "\n    --latch:                      latch last message for new subscribers";
    std::cerr << "\n    --max-datagram-size=[<size>]: maximum datagram size for UDP";
    std::cerr << "\n";
    std::cerr << "\nexamples:";
    std::cerr << "\n    read from and write to live topics:";
    std::cerr << "\n    $ ros-image --from camera/image_raw | cv-cat \"view;null\"";
    std::cerr << "\n    $ cat image.bin | ros-image --to camera/image_raw";
    std::cerr << "\n";
    std::cerr << "\n    read from bag files:";
    std::cerr << "\n    $ ros-image --from camera/image_raw --bags a.bag,b.bag | cv-cat \"view;null\"";
    std::cerr << "\n";
    std::cerr << "\n    wildcard expansion need to be protected from expanding by bash";
    std::cerr << "\n    $ ros-image --from camera/image_raw --bags \"*.bag\" | cv-cat \"view;null\"";
    std::cerr << "\n" << std::endl;
}

void unblock_signal( int signum )
{
    #ifndef WIN32
    // Undo ROS's ignoring of SIGPIPE (see roscpp/init.cpp:469)
    sigset_t sigmask;
    sigemptyset( &sigmask );
    sigaddset( &sigmask, signum );
    sigprocmask( SIG_UNBLOCK, &sigmask, NULL );
    #endif
}

void ros_init( char **av, std::string node_name, std::string const& suffix )
{
    uint32_t node_options = ::ros::InitOption::NoSigintHandler;
    int rac = 1;
    if( node_name.empty() )
    {
        node_name = "ros_image" + suffix;
        node_options |= ::ros::InitOption::AnonymousName;
    }
    ros::init( rac, av, node_name, node_options );
}

class ros_subscriber
{
public:
    using message_type = typename sensor_msgs::Image::ConstPtr;

    ros_subscriber( comma::command_line_options const& options )
        : flush( options.exists( "--flush" ))
        , from_bag( options.exists( "--bags" ))
        , topic( options.value< std::string >( "--from" ))
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
            node.reset( new ros::NodeHandle() );
            subscriber = node->subscribe( topic
                                        , options.value< unsigned >( "--queue-size", 1U )
                                        , &ros_subscriber::process, this
                                        , hints );
        }
    }

    void write( message_type const msg )
    {
        cv_stream.write( std::cout
                       , std::make_pair( msg->header.stamp.toBoost()
                                       , cv::Mat( msg->height, msg->width, snark::ros::ros_to_cv_format( msg->encoding )
                                                , ( void* )msg->data.data(), cv::Mat::AUTO_STEP ))
                       , flush );
    }

    void process( message_type const msg )
    {
        if( std::cout.good() && !is_shutdown ) { write( msg ); }
        else { ros::shutdown(); }
    }

    void subscribe( void )
    {
        if( from_bag )
        {
            unblock_signal( SIGPIPE );  // so we can catch it in is_shutdown
            for( auto bag_name: bag_names )
            {
                comma::verbose << "opening " << bag_name << std::endl;
                rosbag::Bag bag( bag_name );
                for( rosbag::MessageInstance const mi : rosbag::View( bag, rosbag::TopicQuery( topic )))
                {
                    if( is_shutdown ) { return; }
                    message_type const msg = mi.instantiate< sensor_msgs::Image >();
                    write( msg );
                }
            }
        }
        else
        {
            ros::spin();
        }
    }

private:
    snark::cv_mat::serialization cv_stream;
    bool const flush;
    bool const from_bag;
    std::unique_ptr< ros::NodeHandle > node;
    ros::Subscriber subscriber;
    std::vector< std::string > bag_names;
    std::string topic;
    comma::signal_flag is_shutdown;
};

class ros_publisher
{
public:
    ros_publisher( comma::command_line_options const& options )
    {
        message.header.frame_id = options.value< std::string >( "--frame", std::string() );

        publisher = node.advertise< sensor_msgs::Image >( options.value< std::string >( "--to" )
                                                        , options.value< unsigned >( "--queue-size", 1U )
                                                        , options.exists( "--latch" ));
        ros::spinOnce();
    }

    void publish()
    {
        while( std::cin.good() && !is_shutdown )
        {
            auto record = cv_stream.read< boost::posix_time::ptime >( std::cin );
            message.header.seq++;
            message.header.stamp = ros::Time::fromBoost( record.first );
            sensor_msgs::fillImage( message
                                  , snark::ros::cv_to_ros_format( record.second.type() )
                                  , record.second.rows
                                  , record.second.cols
                                  , record.second.step
                                  , record.second.data );
            publisher.publish( message );
        }
    }

private:
    snark::cv_mat::serialization cv_stream;
    ros::NodeHandle node;
    ros::Publisher publisher;
    typename sensor_msgs::Image message;
    comma::signal_flag is_shutdown;
};

void ros_execute( char** av, comma::command_line_options const& options )
{
    std::string node_name = options.value< std::string >( "--node-name,--node", "" );
    if( options.exists( "--from" ))
    {
        if( !options.exists( "--bags" )) { ros_init( av, node_name, "_subscriber" ); }
        ros_subscriber subscriber( options );
        subscriber.subscribe();
    }
    else
    {
        ros_init( av, node_name, "_publisher" );
        ros_publisher publisher( options );
        publisher.publish();
        ros::shutdown();
    }
}

int main( int ac, char* av[] )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--bash-completion" )) { bash_completion( ac, av ); }
        options.assert_mutually_exclusive( "--from,--flush,--output-fields", "--to,--dimensions,--dim,--input-fields" );
        options.assert_exists( "--from,--to" );
        ros_execute( av, options );
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": unknown exception" << std::endl; }
    return 1;
}
