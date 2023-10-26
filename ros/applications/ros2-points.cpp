// Copyright (c) 2017 The University of Sydney
// Copyright (c) 2021 Mission Systems Pty Ltd

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rclcpp/serialization.hpp>
#include "../rclcpp/time.h"
#include "detail/pointcloud.h"

void bash_completion( unsigned int const ac, char const * const * av )
{
    static const char* completion_options =
        " --help -h --verbose -v"
        " --header-fields --header-format --node-name --output-fields --output-format"
        " --from --bags --fields --flush --header --output-header --ignore-time-format --max-datagram-size --no-discard --queue-size"
        " --to --all --field-name-map --hang-on --stay --frame --latch --node-name --pass-through --pass --queue-size --time-format"
        ;
    std::cout << completion_options << std::endl;
    exit( 0 );
}

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
    std::cerr << "\n    --max-datagram-size:      for UDP transport. See ros::TransportHints";
    std::cerr << "\n    --no-discard:             don't discard points with nan or inf";
    std::cerr << "\n    --queue-size=[<n>]:       ROS Subscriber queue size, default 1";
    std::cerr << "\n";
    std::cerr << "\nto options";
    std::cerr << "\n    --to=<topic>:             topic to publish to";
    std::cerr << "\n    --all:                    send all records as one ros message";
    std::cerr << "\n    --fields,-f=<fields>:     fields names; default=x,y,z";
    std::cerr << "\n    --field-name-map=[<map>]: rename fields; format: old:new,...";
    std::cerr << "\n    --frame=[<frame>]:        ros message frame as string";
    std::cerr << "\n    --hang-on,--stay:         wait before exiting so that subscribers can";
    std::cerr << "\n                              receive the last message";
    std::cerr << "\n    --latch:                  last message will be saved for future subscribers";
    std::cerr << "\n    --output,-o=[<bag>]:      write to bag rather than publish";
    std::cerr << "\n    --output-fields=[<fields>]: fields to output; default: all input fields";
    std::cerr << "\n    --pass-through,--pass:    pass input data to stdout";
    std::cerr << "\n    --queue-size=[<n>]:       ROS publisher queue size, default=1";
    std::cerr << "\n    --time-format=<fmt>:      time format in ROS pointfield; default: none";
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
        std::cerr << "\n    csv-random make --type 3d | csv-paste line-number - \\";
        std::cerr << "\n        | csv-blocks group --fields scalar --span 1000 | csv-time-stamp \\";
        std::cerr << "\n        | " << comma::verbose.app_name() << " --to /points -f t,id,x,y,z,block --format t,ui,3d,ui";
    }
    else
    {
        std::cerr << "\nrun \"" << comma::verbose.app_name() << " --help --verbose\" for more detail and examples of use";
    }
    std::cerr << "\n" << std::endl;
}

// =========================
// --from topic


int main( int argc, char** argv )
{
    // print the version 
    std::cerr << " Version is " << ROS_VERSION_MAJOR << "." << ROS_VERSION_MINOR << "." << ROS_VERSION_PATCH << std::endl;

    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" )) bash_completion( argc, argv );

        int argc2 = 0;
        char** argv2 = nullptr;
        rclcpp::init(argc2, argv2);
        boost::optional< std::string > node_name = options.optional< std::string >( "--node-name" );
        if( !node_name ){ node_name = "ros_points"; }
        auto node = std::make_shared<rclcpp::Node>(*node_name);

        if( options.exists( "--from" ))
        {
            if( options.exists( "--header-fields" )) { std::cout << comma::join( comma::csv::names< header >(), ',' ) << std::endl; return 0; }
            if( options.exists( "--header-format" )) { std::cout << comma::csv::format::value< header >() << std::endl; return 0; }
            std::string bags_option = options.value< std::string >( "--bags", "" );
            std::string topic = options.value< std::string >( "--from" );
            unsigned int queue_size = options.value< unsigned int >( "--queue-size", 1 );
            boost::optional< int > max_datagram_size = options.optional< int >( "--max-datagram-size" );
            points points( options );
            if( !bags_option.empty() )
            {
                COMMA_ASSERT( false, "bags not implemented" );
                // rosbag::Bag bag;
                // bag.open( bag_name );
                // for( rosbag::MessageInstance const m: rosbag::View( bag, rosbag::TopicQuery( topic )))
                // {
                //     sensor_msgs::msg::PointCloud2ConstPtr msg = m.instantiate< sensor_msgs::msg::PointCloud2 >();
                //     points.process( msg );
                //     if( ros::isShuttingDown() ) { break; }
                // }
                // bag.close();

                // rosbag2_cpp::Writer bag;
                // std::vector< std::string > bag_names;
                // for( auto name: comma::split( bags_option, ',' ))
                // {
                //     std::vector< std::string > expansion = snark::ros::glob( name );
                //     bag_names.insert( bag_names.end(), expansion.begin(), expansion.end() );
                // }
                // for( auto bag_name: bag_names )
                // {
                //     comma::verbose << "opening " << bag_name << std::endl;
                //     bag.open( bag_name );


                //     for( rosbag2_cpp::SerializedBagMessage const m: rosbag::View( bag, rosbag::TopicQuery( topic )))
                //     {
                //         sensor_msgs::msg::PointCloud2ConstPtr msg = m.instantiate< sensor_msgs::msg::PointCloud2 >();
                //         points.process( msg );
                //         if( ros::isShuttingDown() ) { break; }
                //     }
                //     bag.close();
                //     if( ros::isShuttingDown() ) { break; }
                // }
            }
            else
            {
                COMMA_ASSERT( false, "from ros msg not implemented" );
                // Old ros1 code.
                // ros::NodeHandle ros_node;
                // ros::TransportHints transport_hints;
                // if( max_datagram_size ) { transport_hints = ros::TransportHints().maxDatagramSize( *max_datagram_size ); }
                // ros::Subscriber subscriber = ros_node.subscribe( topic, queue_size, &points::process, &points, transport_hints );
                // ros::spin();
            }
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
            auto bag_writer = std::make_unique<rosbag2_cpp::Writer>();

            auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic, queue_size);
            std::unique_ptr< std::function< void( sensor_msgs::msg::PointCloud2 ) > > publish_fn;

            if( publishing )
            {
                if( !node_name ){ node_name = "ros_points"; }
                publish_fn = std::make_unique<std::function<void(sensor_msgs::msg::PointCloud2)>>(
                    [&publisher](sensor_msgs::msg::PointCloud2 msg) { publisher->publish(msg); }
                );
            }
            else
            {
                COMMA_ASSERT( false, "Write to ros bag not implemented" );

                // If writing to a ros bag
                bag_writer->open( output_option );
                bag_writer->create_topic({topic,"sensor_msgs::msg::PointCloud2",rmw_get_serialization_format(),""});

                auto publish_fn = std::make_unique<std::function<void(sensor_msgs::msg::PointCloud2)>>(
                    [&bag_writer, topic](sensor_msgs::msg::PointCloud2 msg) {
                        // auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>(rclcpp::serialize(msg));
                        // bag_writer->write(serialized_msg, topic, msg.header.stamp);
                        bag_writer->write(msg, topic, msg.header.stamp);
                        // bag_writer->write(std::make_shared<rclcpp::SerializedMessage>(rclcpp::serialize(msg)), topic, "sensor_msgs::msg::PointCloud2", "");
                    }
                );
            }

            comma::csv::input_stream< record > is( std::cin, csv );
            comma::csv::passed< record > passed( is, std::cout, csv.flush );
            unsigned int block = 0;
            to_points points( options, csv );

            while( std::cin.good() )
            {
                //read binary from input
                const record* p = is.read();
                if (( !p || block != p->block ) && !points.empty() )
                {
                    points.send( *publish_fn.get() );
                }
                if( !p ) { break; }
                if( pass_through ) { passed.write(); }
                block = p->block;
                points.add_record( *p, is );
                if( !has_block && !all ) { 
                    points.send( *publish_fn.get() ); 
                }
            }

            if( publishing && options.exists( "--hang-on,--stay" ))
            {
                for( int i = 0; i < 3; i++ )
                {
                    rclcpp::spin_some(node);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
            }

            rclcpp::shutdown();
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
