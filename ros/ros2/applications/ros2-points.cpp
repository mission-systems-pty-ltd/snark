// Copyright (c) 2017 The University of Sydney
// Copyright (c) 2021,2026 Mission Systems Pty Ltd

#include "../../detail/ros-points-detail.h"
#include <rcutils/logging.h>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>

void usage( bool verbose = false )
{
    std::cerr << "\nconvert ROS PointCloud2 to csv and vice-versa";
    std::cerr << "\n";
    std::cerr << "\nusage: " << comma::verbose.app_name() << " --from=<topic> [<options>]";
    std::cerr << "\n       " << comma::verbose.app_name() << " --to=<topic> [<options>]";
    std::cerr << "\n";
    std::cerr << "\ngeneral options";
    std::cerr << "\n    --help,-h:       show help; --help --verbose: show more help";
    std::cerr << "\n    --verbose,-v:    show detailed messages";
    std::cerr << "\n    --header-fields: write csv field names of header to stdout and exit";
    std::cerr << "\n    --header-format: write csv format of header to stdout and exit";
    std::cerr << "\n    --node-name:     node name for this process, when not specified uses";
    std::cerr << "\n                     ros::init_options::AnonymousName flag";
    std::cerr << "\n    --output-fields: print field names and exit";
    std::cerr << "\n    --output-format: print format and exit";
    std::cerr << "\n";
    std::cerr << "\n    field names and format are extracted from the first message of the";
    std::cerr << "\n    subscribed topic";
    std::cerr << "\n";
    std::cerr << "\nfrom options";
    std::cerr << "\n    --from=<topic>:           topic to read";
    std::cerr << "\n    --bags=[<bags>]:          load from rosbags rather than subscribe";
    std::cerr << "\n    --fields=[<names>]:       only output listed fields";
    std::cerr << "\n    --flush:                  call flush on stdout after each write";
    std::cerr << "\n    --header,--output-header: prepend t,block header to output with t,ui format";
    std::cerr << "\n    --ignore-time-format:     don't do any interpretation of time format";
    std::cerr << "\n    --no-discard:             don't discard points with nan or inf";
    std::cerr << "\n";
    std::cerr << "\nto options";
    std::cerr << "\n    --to=<topic>:             topic to publish to";
    std::cerr << "\n    --all:                    send all records as one ros message";
    std::cerr << "\n    --fields,-f=<fields>:     fields names; default=x,y,z";
    std::cerr << "\n    --field-name-map=[<map>]: rename fields; format: old:new,...";
    std::cerr << "\n    --frame=[<frame>]:        ros message frame as string";
    std::cerr << "\n    --stay:                   wait before exiting so that subscribers can";
    std::cerr << "\n                              receive the last message";
    std::cerr << "\n    --latch:                  last message will be saved for future subscribers";
    std::cerr << "\n    --output,-o=[<bag>]:      write to bag rather than publish";
    std::cerr << "\n    --output-fields=[<fields>]: fields to output; default: all input fields";
    std::cerr << "\n    --pass-through,--pass:    pass input data to stdout";
    std::cerr << "\n    --time-format=<fmt>:      time format in ROS pointfield; default: none";
    std::cerr << "\n";
    std::cerr << "\n    note that the --latch option must match between publishers and subscribers";
    std::cerr << "\n    in code it sets qos.transient_local()";
    std::cerr << "\n";
    if( verbose )
    {
        std::cerr << "\nfield names";
        std::cerr << "\n    Field names are generally duplicated in the ROS PointCloud2 message.";
        std::cerr << "\n    They can be mapped to another name with the --field-name-map option.";
        std::cerr << "\n";
        std::cerr << "\ntime formats";
        std::cerr << "\n    The ROS PointCloud2 message contains a ROS-format timestamp in the message";
        std::cerr << "\n    header and also contains a time field in each point field if that has been";
        std::cerr << "\n    included in the fields (usually it is).";
        std::cerr << "\n";
        std::cerr << "\n    For the individual point timestamp there are several options:";
        std::cerr << "\n        none:               straight copy of the incoming timestamp";
        std::cerr << "\n        offset-seconds:     offset in seconds from header timestamp (float)";
        std::cerr << "\n        offset-nanoseconds: offset in nanoseconds from header timestamp (uint32)";
        std::cerr << "\n";
        std::cerr << "\n    The --time-format option applies to the --to operation. The --from operation";
        std::cerr << "\n    deduces the interpretation from the data type (float or uint32)";
        std::cerr << "\n";
        std::cerr << "\nexamples";
        std::cerr << "\n    --- view points from a published topic ---";
        std::cerr << "\n    " << comma::verbose.app_name() << " --from <topic> --fields x,y,z --binary 3f --header \\";
        std::cerr << "\n        | view-points --fields t,block,x,y,z --binary t,ui,3f";
        std::cerr << "\n";
        std::cerr << "\n    --- view points from a set of bags ---";
        std::cerr << "\n    " << comma::verbose.app_name() << " --from <topic> --bags \"*.bag\" --fields x,y,z --binary 3f \\";
        std::cerr << "\n        | view-points --fields t,block,x,y,z --binary t,ui,3f";
        std::cerr << "\n";
        std::cerr << "\n    --- publish on /points topic ---";
        std::cerr << "\n    cat data.bin | " << comma::verbose.app_name() << " --to /points -f t,block,x,y,z -b t,ui,3f";
        std::cerr << "\n";
        std::cerr << "\n    --- write /points topic to a bag file ---";
        std::cerr << "\n    cat data.bin | " << comma::verbose.app_name() << " --to /points -o my.bag -f t,block,x,y,z -b t,ui,3f";
        std::cerr << "\n";
        std::cerr << "\n    --- change a field name ---";
        std::cerr << "\n    " << comma::verbose.app_name() << " --to /points --fields t,channel,x,y,z --format t,ui,3d \\";
        std::cerr << "\n               --field-name-map channel:ring";
        std::cerr << "\n";
        std::cerr << "\n    --- read or write timestamps as offsets ---";
        std::cerr << "\n    " << comma::verbose.app_name() << " --to /points --fields t,x,y,z --format t,3d \\";
        std::cerr << "\n               --time-format offset-seconds";
        std::cerr << "\n";
        std::cerr << "\n    -- see underlying data in bag file written with offset-seconds ---";
        std::cerr << "\n    " << comma::verbose.app_name() << " --bags <file> --from /points --fields t,x,y,z --format f,3d \\";
        std::cerr << "\n               --ignore-time-format";
        std::cerr << "\n";
        std::cerr << "\n    --- create random test data ---";
        std::cerr << "\n    csv-random make --type 3f | csv-paste line-number - \\";
        std::cerr << "\n        | csv-blocks group --fields scalar --span 1000 | csv-time-stamp \\";
        std::cerr << "\n        | " << comma::verbose.app_name() << " --to /points -f t,id,x,y,z,block --format t,ui,3f,ui";
    }
    else
    {
        std::cerr << "\nrun \"" << comma::verbose.app_name() << " --help --verbose\" for more detail and examples of use";
    }
    std::cerr << "\n" << std::endl;
}


void bash_completion( unsigned int const ac, char const * const * av )
{
    static const char* completion_options =
        " --help -h --verbose -v"
        " --header-fields --header-format --node-name --output-fields --output-format"
        " --from --bags --fields --flush --header --output-header --ignore-time-format --no-discard "
        " --to --all --field-name-map --stay --frame --latch --node-name --pass-through --pass --time-format"
        ;
    std::cout << completion_options << std::endl;
    exit( 0 );
}


int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" )) bash_completion( argc, argv );

        if( !comma::verbose ) { [[maybe_unused]] auto rc = rcutils_logging_set_logger_level( "rosbag2_storage", RCUTILS_LOG_SEVERITY_WARN ); }

        if( options.exists( "--from" ))
        {
            if( options.exists( "--header-fields" )) { std::cout << comma::join( comma::csv::names< snark::ros::detail::pointcloud< 2 >::header >(), ',' ) << std::endl; return 0; }
            if( options.exists( "--header-format" )) { std::cout << comma::csv::format::value< snark::ros::detail::pointcloud< 2 >::header >() << std::endl; return 0; }
            std::string bags_option = options.value< std::string >( "--bags", "" );
            std::string topic = options.value< std::string >( "--from" );
            boost::optional< std::string > node_name = options.optional< std::string >( "--node-name" );

            rclcpp::init( argc, argv );
            snark::ros::detail::pointcloud< 2 >::points points( options );

            if( !bags_option.empty() )
            {
                std::vector< std::string > bag_names;
                for( auto name: comma::split( bags_option, ',' ))
                {
                    std::vector< std::string > expansion = snark::ros::glob( name );
                    bag_names.insert( bag_names.end(), expansion.begin(), expansion.end() );
                }
                for( auto bag_name: bag_names )
                {
                    comma::verbose << "opening " << bag_name << std::endl;
                    rosbag2_cpp::Reader reader;
                    reader.open( bag_name );
                    while( reader.has_next() )
                    {
                        auto bag_message = reader.read_next();
                        if( bag_message->topic_name != topic ) { continue; }
                        auto msg = std::make_shared< sensor_msgs::msg::PointCloud2 >();
                        rclcpp::SerializedMessage serialized_msg( *bag_message->serialized_data );
                        rclcpp::Serialization< sensor_msgs::msg::PointCloud2 > serialization;
                        serialization.deserialize_message( &serialized_msg, msg.get() );
                        points.process( msg );
                        if( !rclcpp::ok() ) { break; }
                    }
                    if( !rclcpp::ok() ) { break; }
                }
            }
            else
            {
                auto node = rclcpp::Node::make_shared( node_name ? *node_name : "ros_points" );
                auto qos = rclcpp::QoS( rclcpp::KeepLast( options.value< unsigned >( "--queue-size", 1U )));
                if( options.exists( "--latch" )) { qos.transient_local(); }

                auto subscriber = node->create_subscription< sensor_msgs::msg::PointCloud2 >(
                    topic,
                    qos,
                    [&points]( sensor_msgs::msg::PointCloud2::ConstSharedPtr msg )
                    {
                        points.process( std::const_pointer_cast< sensor_msgs::msg::PointCloud2 >( msg ));
                    }
                );
                rclcpp::spin( node );
            }
            rclcpp::shutdown();
            return status;
        }
        else if( options.exists( "--to" ))
        {
            comma::csv::options csv(options);
            if( !csv.binary() && !options.exists( "--format" )) { COMMA_THROW( comma::exception, "please specify --binary=<format>, or --format=<format> for ascii"); }
            csv.full_xpath = true;
            std::string topic = options.value< std::string >( "--to" );
            unsigned int queue_size = options.value< unsigned int >( "--queue-size", 1 );
            if( csv.fields.empty() ) { csv.fields = "x,y,z"; }
            bool has_block = csv.has_field( "block" );
            bool all = options.exists( "--all" );
            boost::optional< std::string > node_name = options.optional< std::string >( "--node-name" );
            bool pass_through = options.exists( "--pass-through,--pass" );
            std::string output_option = options.value< std::string >( "--output,-o", "" );
            bool publishing = output_option.empty();

            std::shared_ptr< rclcpp::Node > node;
            std::shared_ptr< rclcpp::Publisher< sensor_msgs::msg::PointCloud2 > > publisher;
            std::unique_ptr< rosbag2_cpp::Writer > bag;
            std::unique_ptr< std::function< void( sensor_msgs::msg::PointCloud2 ) > > publish_fn;

            if( publishing )
            {
                rclcpp::init( argc, argv );
                node = rclcpp::Node::make_shared( node_name ? *node_name : "ros_points" );
                publisher = node->create_publisher< sensor_msgs::msg::PointCloud2 >( topic, queue_size );
                rclcpp::spin_some( node );
                publish_fn.reset( new std::function< void( sensor_msgs::msg::PointCloud2 ) >(
                    [&]( sensor_msgs::msg::PointCloud2 msg )
                    {
                        publisher->publish( msg );
                        rclcpp::spin_some( node );
                    } ));
            }
            else
            {
                bag = std::make_unique< rosbag2_cpp::Writer >();
                bag->open( output_option );
                publish_fn.reset( new std::function< void( sensor_msgs::msg::PointCloud2 ) >(
                    [&]( sensor_msgs::msg::PointCloud2 msg )
                    {
                        rclcpp::Serialization< sensor_msgs::msg::PointCloud2 > serialization;
                        rclcpp::SerializedMessage serialized_msg;
                        serialization.serialize_message( &msg, &serialized_msg );
                        auto bag_message = std::make_shared< rosbag2_storage::SerializedBagMessage >();
                        bag_message->serialized_data = std::make_shared< rcutils_uint8_array_t >();
                        *bag_message->serialized_data = serialized_msg.release_rcl_serialized_message();
                        bag_message->topic_name = topic;
                        rclcpp::Time stamp( msg.header.stamp.sec, msg.header.stamp.nanosec );
                        bag_message->time_stamp = stamp.nanoseconds();
                        bag->write( bag_message );
                    } ));

                // register the topic with the writer before writing any messages
                rosbag2_storage::TopicMetadata topic_metadata;
                topic_metadata.name = topic;
                topic_metadata.type = "sensor_msgs/msg/PointCloud2";
                topic_metadata.serialization_format = "cdr";
                bag->create_topic( topic_metadata );
            }

            comma::csv::input_stream< snark::ros::pointcloud::record > is( std::cin, csv );
            comma::csv::passed< snark::ros::pointcloud::record > passed( is, std::cout, csv.flush );
            unsigned int block = 0;
            snark::ros::detail::pointcloud< 2 >::to_points points( options, csv );

            while( std::cin.good() )
            {
                const snark::ros::pointcloud::record* p = is.read();
                if (( !p || block != p->block ) && !points.empty() )
                {
                    points.send( *publish_fn.get() );
                }
                if( !p ) { break; }
                if( pass_through ) { passed.write(); }
                block = p->block;
                points.add_record( *p, is );
                if( !has_block && !all ) { points.send( *publish_fn.get() ); }
            }

            if( publishing && options.exists( "--stay" ))
            {
                for( int i = 0; i < 3; i++ )
                {
                    rclcpp::spin_some( node );
                    std::this_thread::sleep_for( std::chrono::milliseconds( 1000 ));
                }
            }

            if( publishing ) { rclcpp::shutdown(); }
            return 0;
        }
        else
        {
            std::cerr << comma::verbose.app_name() << ": requires --from or --to option" << std::endl;
            return 1;
        }
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": exception: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; }
    return 1;
}
