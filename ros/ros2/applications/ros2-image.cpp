// Copyright (c) 2019 The University of Sydney
// Copyright (c) 2026 Mission Systems Pty Ltd

#include "../../file-util.h"
#include "../../ros-cv-format.h"
#include "../../../imaging/cv_mat/serialization.h"
#include "../../../imaging/cv_mat/traits.h"
#include <comma/application/signal_flag.h>
#include <comma/io/stream.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <unistd.h>

void bash_completion( unsigned const ac, char const * const * av )
{
    static char const * const arguments =
        " --help -h --node-name --queue-size"
        " --from --bags --flush"
        " --to --latch --frame"
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
    std::cerr << "\n    --help,-h:                  print help and exit";
    std::cerr << "\n    --node,--node-name=<name>:  default=ros_image_<publisher|subscriber>";
    std::cerr << "\n    --queue-size=[<n>]:         default=1; ROS queue size";
    std::cerr << "\n    --latch:                    latch last message";
    std::cerr << "\n";
    std::cerr << "\nfrom options:";
    std::cerr << "\n    --from=<topic>:             topic to read";
    std::cerr << "\n    --bags=[<files>]:           comma-separated list of bag files, see examples";
    std::cerr << "\n    --flush:                    flush stream after each image";
    std::cerr << "\n";
    std::cerr << "\nto options:";
    std::cerr << "\n    --to=<topic>:               topic to publish to";
    std::cerr << "\n    --frame=[<frame_id>]:       ros header frame id";
    std::cerr << "\n";
    std::cerr << "\n    note that the --latch option must match between publishers and subscribers";
    std::cerr << "\n    in code it sets qos.transient_local()";
    std::cerr << "\n";
    std::cerr << "\nexamples:";
    std::cerr << "\n    read from and write to live topics:";
    std::cerr << "\n    $ ros2-image --from camera/image_raw | cv-cat \"view;null\"";
    std::cerr << "\n    $ cat image.bin | ros2-image --to camera/image_raw";
    std::cerr << "\n";
    std::cerr << "\n    read from bag files:";
    std::cerr << "\n    $ ros2-image --from camera/image_raw --bags a.bag,b.bag | cv-cat \"view;null\"";
    std::cerr << "\n";
    std::cerr << "\n    wildcard expansion need to be protected from expanding by bash";
    std::cerr << "\n    $ ros2-image --from camera/image_raw --bags \"*.bag\" | cv-cat \"view;null\"";
    std::cerr << "\n" << std::endl;
}

std::shared_ptr<rclcpp::Node> ros_init( char **av, std::string node_name, std::string const& suffix )
{
    if( node_name.empty() ) { node_name = "ros_image" + suffix + std::to_string( getpid() ); }

    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters( true );
    node_options.automatically_declare_parameters_from_overrides( true );

    auto node = rclcpp::Node::make_shared( node_name, node_options );
    return node;
}
/*
static unsigned ros_to_cv_format( std::string const& ros_encoding )
{
    static const std::unordered_map< std::string, unsigned > map = {
        { "rgb8", CV_8UC3 },
        { "rgba8", CV_8UC4 },
        { "rgb16", CV_16UC3 },
        { "rgba16", CV_16UC4 },
        { "bgr8", CV_8UC3 },
        { "bgra8", CV_8UC4 },
        { "bgr16", CV_16UC3 },
        { "bgra16", CV_16UC4 },
        { "mono8", CV_8UC1 },
        { "mono16", CV_16UC1 },
        { "8UC1", CV_8UC1 },
        { "8UC2", CV_8UC2 },
        { "8UC3", CV_8UC3 },
        { "8UC4", CV_8UC4 },
        { "8SC1", CV_8SC1 },
        { "8SC2", CV_8SC2 },
        { "8SC3", CV_8SC3 },
        { "8SC4", CV_8SC4 },
        { "16UC1", CV_16UC1 },
        { "16UC2", CV_16UC2 },
        { "16UC3", CV_16UC3 },
        { "16UC4", CV_16UC4 },
        { "16SC1", CV_16SC1 },
        { "16SC2", CV_16SC2 },
        { "16SC3", CV_16SC3 },
        { "16SC4", CV_16SC4 },
        { "32SC1", CV_32SC1 },
        { "32SC2", CV_32SC2 },
        { "32SC3", CV_32SC3 },
        { "32SC4", CV_32SC4 },
        { "32FC1", CV_32FC1 },
        { "32FC2", CV_32FC2 },
        { "32FC3", CV_32FC3 },
        { "32FC4", CV_32FC4 },
        { "64FC1", CV_64FC1 },
        { "64FC2", CV_64FC2 },
        { "64FC3", CV_64FC3 },
        { "64FC4", CV_64FC4 },
        { "bayer_rggb8", CV_8UC4 },
        { "bayer_bggr8", CV_8UC4 },
        { "bayer_gbrg8", CV_8UC4 },
        { "bayer_grbg8", CV_8UC4 },
        { "bayer_rggb16", CV_16UC4 },
        { "bayer_bggr16", CV_16UC4 },
        { "bayer_gbrg16", CV_16UC4 },
        { "bayer_grbg16", CV_16UC4 },
    };
    return map.at( ros_encoding );
}

static std::string cv_to_ros_format( unsigned const cv_encoding )
{
    static const std::unordered_map< unsigned, std::string > map = {
        { CV_8UC3, "rgb8" },
        { CV_8UC4, "rgba8" },
        { CV_16UC3, "rgb16" },
        { CV_16UC4, "rgba16" },
        { CV_8UC3, "bgr8" },
        { CV_8UC4, "bgra8" },
        { CV_16UC3, "bgr16" },
        { CV_16UC4, "bgra16" },
        { CV_8UC1, "mono8" },
        { CV_16UC1, "mono16" },
        { CV_8UC1, "8UC1" },
        { CV_8UC2, "8UC2" },
        { CV_8UC3, "8UC3" },
        { CV_8UC4, "8UC4" },
        { CV_8SC1, "8SC1" },
        { CV_8SC2, "8SC2" },
        { CV_8SC3, "8SC3" },
        { CV_8SC4, "8SC4" },
        { CV_16UC1, "16UC1" },
        { CV_16UC2, "16UC2" },
        { CV_16UC3, "16UC3" },
        { CV_16UC4, "16UC4" },
        { CV_16SC1, "16SC1" },
        { CV_16SC2, "16SC2" },
        { CV_16SC3, "16SC3" },
        { CV_16SC4, "16SC4" },
        { CV_32SC1, "32SC1" },
        { CV_32SC2, "32SC2" },
        { CV_32SC3, "32SC3" },
        { CV_32SC4, "32SC4" },
        { CV_32FC1, "32FC1" },
        { CV_32FC2, "32FC2" },
        { CV_32FC3, "32FC3" },
        { CV_32FC4, "32FC4" },
        { CV_64FC1, "64FC1" },
        { CV_64FC2, "64FC2" },
        { CV_64FC3, "64FC3" },
        { CV_64FC4, "64FC4" },
        { CV_8UC4, "bayer_rggb8" },
        { CV_8UC4, "bayer_bggr8" },
        { CV_8UC4, "bayer_gbrg8" },
        { CV_8UC4, "bayer_grbg8" },
        { CV_16UC4, "bayer_rggb16" },
        { CV_16UC4, "bayer_bggr16" },
        { CV_16UC4, "bayer_gbrg16" },
        { CV_16UC4, "bayer_grbg16" },
    };
    return map.at( cv_encoding );
}
*/
// Helper to convert rclcpp::Time to boost::posix_time::ptime
boost::posix_time::ptime to_boost_time( const rclcpp::Time& ros_time )
{
    int64_t total_nanoseconds = ros_time.nanoseconds();
    int64_t seconds = total_nanoseconds / 1000000000LL;
    int64_t nanoseconds = total_nanoseconds % 1000000000LL;

    boost::posix_time::ptime epoch(boost::gregorian::date( 1970, 1, 1 ));
    return epoch + boost::posix_time::seconds( seconds ) +
           boost::posix_time::microseconds( nanoseconds / 1000 );
}

// Helper to convert boost::posix_time::ptime to rclcpp::Time
rclcpp::Time from_boost_time( const boost::posix_time::ptime& ptime )
{
    boost::posix_time::ptime epoch(boost::gregorian::date( 1970, 1, 1 ));
    boost::posix_time::time_duration diff = ptime - epoch;

    int64_t seconds = diff.total_seconds();
    int64_t nanoseconds = ( diff.total_microseconds() % 1000000LL ) * 1000LL;

    return rclcpp::Time( seconds, nanoseconds );
}

class ros_subscriber
{
public:
    using message_type = typename sensor_msgs::msg::Image::SharedPtr;

    ros_subscriber( comma::command_line_options const& options, std::shared_ptr<rclcpp::Node> node_ )
        : flush( options.exists( "--flush" ))
        , from_bag( options.exists( "--bags" ))
        , node( node_ )
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
            // Set up QoS profile
            auto qos = rclcpp::QoS( rclcpp::KeepLast( options.value< unsigned >( "--queue-size", 1U )));
            if( options.exists( "--latch" )) { qos.transient_local(); }

            subscription = node->create_subscription< sensor_msgs::msg::Image >(
                topic,
                qos,
                [this]( message_type msg ) { this->process( msg ); }
            );
        }
    }

    void write( message_type const msg )
    {
        cv_stream.write( std::cout
                       , std::make_pair( to_boost_time( msg->header.stamp )
                                       , cv::Mat( msg->height, msg->width, snark::ros::ros_to_cv_format( msg->encoding )
                                                , ( void* )msg->data.data(), cv::Mat::AUTO_STEP ))
                       , flush );
    }

    void process( message_type const msg )
    {
        write( msg );
    }

    void subscribe( void )
    {
        if( from_bag )
        {
            for( auto bag_name: bag_names )
            {
                comma::verbose << "opening " << bag_name << std::endl;

                rosbag2_cpp::Reader reader;
                rosbag2_storage::StorageOptions storage_options;
                storage_options.uri = bag_name;
                storage_options.storage_id = "sqlite3";  // default storage plugin

                rosbag2_cpp::ConverterOptions converter_options;
                converter_options.input_serialization_format = "cdr";
                converter_options.output_serialization_format = "cdr";

                reader.open( storage_options, converter_options );

                // Filter for specific topic
                rosbag2_storage::StorageFilter filter;
                filter.topics.push_back( topic );
                reader.set_filter( filter );

                while( reader.has_next() )
                {
                    if( !std::cout.good() || is_shutdown ) { return; }

                    auto serialized_message = reader.read_next();

                    // Deserialize the message
                    rclcpp::SerializedMessage serialized_msg( *serialized_message->serialized_data );
                    auto msg = std::make_shared<sensor_msgs::msg::Image>();

                    rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
                    serialization.deserialize_message( &serialized_msg, msg.get() );

                    write( msg );
                }
            }
        }
        else
        {
            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node( node );
            while( std::cout.good() && !is_shutdown )
            {
                // we do it this way so we can periodically check the output and signal status,
                // in the case that we are listening to a topic that isn't currently publishing
                executor.spin_some( std::chrono::milliseconds( 100 ));
            }
        }
    }

private:
    snark::cv_mat::serialization cv_stream;
    bool const flush;
    bool const from_bag;
    std::shared_ptr< rclcpp::Node > node;
    rclcpp::Subscription< sensor_msgs::msg::Image >::SharedPtr subscription;
    std::vector< std::string > bag_names;
    std::string topic;
    comma::signal_flag is_shutdown;
};

class ros_publisher
{
public:
    ros_publisher( comma::command_line_options const& options, std::shared_ptr<rclcpp::Node> node_ )
        : node( node_ )
    {
        message.header.frame_id = options.value< std::string >( "--frame", std::string() );

        // Set up QoS profile
        auto qos = rclcpp::QoS(rclcpp::KeepLast(options.value< unsigned >( "--queue-size", 1U )));
        if( options.exists( "--latch" )) { qos.transient_local(); }

        publisher = node_->create_publisher< sensor_msgs::msg::Image >(
            options.value< std::string >( "--to" ),
            qos
        );

        // Give time for discovery
        rclcpp::sleep_for( std::chrono::milliseconds( 100 ));
    }

    void publish()
    {
        while( std::cin.good() && !is_shutdown )
        {
            comma::verbose << "publishing message" << std::endl;
            auto record = cv_stream.read< boost::posix_time::ptime >( std::cin );
            message.header.stamp = from_boost_time( record.first );

            // Manually fill image message (fillImage doesn't exist in ROS2)
            message.height = record.second.rows;
            message.width = record.second.cols;
            message.encoding = snark::ros::cv_to_ros_format( record.second.type() );
            message.is_bigendian = 0;
            message.step = record.second.step;

            size_t size = record.second.step * record.second.rows;
            message.data.resize( size );
            std::memcpy( message.data.data(), record.second.data, size );

            publisher->publish( message );
        }
    }

private:
    snark::cv_mat::serialization cv_stream;
    std::shared_ptr< rclcpp::Node > node;
    rclcpp::Publisher< sensor_msgs::msg::Image >::SharedPtr publisher;
    sensor_msgs::msg::Image message;
    comma::signal_flag is_shutdown;
};

void ros_execute( char** av, comma::command_line_options const& options )
{
    std::string node_name = options.value< std::string >( "--node-name,--node", "" );
    if( options.exists( "--from" ))
    {
        if( !options.exists( "--bags" ))
        {
            auto node = ros_init( av, node_name, "_subscriber" );
            ros_subscriber subscriber( options, node );
            subscriber.subscribe();
        }
        else
        {
            // For bag reading, we don't need to initialize ROS2
            ros_subscriber subscriber( options, nullptr );
            subscriber.subscribe();
        }
    }
    else
    {
        auto node = ros_init( av, node_name, "_publisher" );
        ros_publisher publisher( options, node );
        publisher.publish();
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

        rclcpp::init( ac, av );
        ros_execute( av, options );
        rclcpp::shutdown();
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": unknown exception" << std::endl; }
    return 1;
}
