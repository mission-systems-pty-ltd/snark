// Copyright (c) 2017 The University of Sydney
// Copyright (c) 2021 Mission Systems Pty Ltd

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rclcpp/serialization.hpp>
#include "../rclcpp/time.h"
#include "detail/ros-points-detail.h"





// void bash_completion( unsigned int const ac, char const * const * av )
// {
//     static const char* completion_options =
//         " --help -h --verbose -v"
//         " --header-fields --header-format --node-name --output-fields --output-format"
//         " --from --bags --fields --flush --header --output-header --ignore-time-format --max-datagram-size --no-discard --queue-size"
//         " --to --all --field-name-map --hang-on --stay --frame --latch --node-name --pass-through --pass --queue-size --time-format"
//         ;
//     std::cout << completion_options << std::endl;
//     exit( 0 );
// }


// // =========================
// // --from topic


int main( int argc, char** argv )
{
//     // print the version 
//     std::cerr << " Version is " << ROS_VERSION_MAJOR << "." << ROS_VERSION_MINOR << "." << ROS_VERSION_PATCH << std::endl;

//     try
//     {
//         comma::command_line_options options( argc, argv, usage );
//         if( options.exists( "--bash-completion" )) bash_completion( argc, argv );

//         int argc2 = 0;
//         char** argv2 = nullptr;
//         rclcpp::init(argc2, argv2);
//         boost::optional< std::string > node_name = options.optional< std::string >( "--node-name" );
//         if( !node_name ){ node_name = "ros_points"; }
//         auto node = std::make_shared<rclcpp::Node>(*node_name);

//         if( options.exists( "--from" ))
//         {
//             if( options.exists( "--header-fields" )) { std::cout << comma::join( comma::csv::names< header >(), ',' ) << std::endl; return 0; }
//             if( options.exists( "--header-format" )) { std::cout << comma::csv::format::value< header >() << std::endl; return 0; }
//             std::string bags_option = options.value< std::string >( "--bags", "" );
//             std::string topic = options.value< std::string >( "--from" );
//             unsigned int queue_size = options.value< unsigned int >( "--queue-size", 1 );
//             boost::optional< int > max_datagram_size = options.optional< int >( "--max-datagram-size" );
//             points points( options );
//             if( !bags_option.empty() )
//             {
//                 COMMA_ASSERT( false, "bags not implemented" );
//                 // rosbag::Bag bag;
//                 // bag.open( bag_name );
//                 // for( rosbag::MessageInstance const m: rosbag::View( bag, rosbag::TopicQuery( topic )))
//                 // {
//                 //     sensor_msgs::msg::PointCloud2ConstPtr msg = m.instantiate< sensor_msgs::msg::PointCloud2 >();
//                 //     points.process( msg );
//                 //     if( ros::isShuttingDown() ) { break; }
//                 // }
//                 // bag.close();

//                 // rosbag2_cpp::Writer bag;
//                 // std::vector< std::string > bag_names;
//                 // for( auto name: comma::split( bags_option, ',' ))
//                 // {
//                 //     std::vector< std::string > expansion = snark::ros::glob( name );
//                 //     bag_names.insert( bag_names.end(), expansion.begin(), expansion.end() );
//                 // }
//                 // for( auto bag_name: bag_names )
//                 // {
//                 //     comma::verbose << "opening " << bag_name << std::endl;
//                 //     bag.open( bag_name );


//                 //     for( rosbag2_cpp::SerializedBagMessage const m: rosbag::View( bag, rosbag::TopicQuery( topic )))
//                 //     {
//                 //         sensor_msgs::msg::PointCloud2ConstPtr msg = m.instantiate< sensor_msgs::msg::PointCloud2 >();
//                 //         points.process( msg );
//                 //         if( ros::isShuttingDown() ) { break; }
//                 //     }
//                 //     bag.close();
//                 //     if( ros::isShuttingDown() ) { break; }
//                 // }
//             }
//             else
//             {
//                 COMMA_ASSERT( false, "from ros msg not implemented" );
//                 // Old ros1 code.
//                 // ros::NodeHandle ros_node;
//                 // ros::TransportHints transport_hints;
//                 // if( max_datagram_size ) { transport_hints = ros::TransportHints().maxDatagramSize( *max_datagram_size ); }
//                 // ros::Subscriber subscriber = ros_node.subscribe( topic, queue_size, &points::process, &points, transport_hints );
//                 // ros::spin();
//             }
//             return status;
//         }
//         else if( options.exists( "--to" ))
//         {
//             comma::csv::options csv(options);
//             if( !csv.binary() && !options.exists( "--format" )) { COMMA_THROW( comma::exception, "please specify --binary=<format>, or --format=<format> for ascii"); }
//             csv.full_xpath = true;
//             std::string topic = options.value< std::string >( "--to" );
//             unsigned int queue_size = options.value< unsigned int >( "--queue-size", 1 );
//             if( csv.fields.empty() ) { csv.fields = "x,y,z"; }
//             bool has_block = csv.has_field( "block" );
//             bool all = options.exists( "--all" );
//             boost::optional< std::string > node_name = options.optional< std::string >( "--node-name" );
//             bool pass_through = options.exists( "--pass-through,--pass" );
//             std::string output_option = options.value< std::string >( "--output,-o", "" );
//             bool publishing = output_option.empty();
//             auto bag_writer = std::make_unique<rosbag2_cpp::Writer>();

//             auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic, queue_size);
//             std::unique_ptr< std::function< void( sensor_msgs::msg::PointCloud2 ) > > publish_fn;

//             if( publishing )
//             {
//                 if( !node_name ){ node_name = "ros_points"; }
//                 publish_fn = std::make_unique<std::function<void(sensor_msgs::msg::PointCloud2)>>(
//                     [&publisher](sensor_msgs::msg::PointCloud2 msg) { publisher->publish(msg); }
//                 );
//             }
//             else
//             {
//                 COMMA_ASSERT( false, "Write to ros bag not implemented" );

//                 // If writing to a ros bag
//                 bag_writer->open( output_option );
//                 bag_writer->create_topic({topic,"sensor_msgs::msg::PointCloud2",rmw_get_serialization_format(),""});

//                 auto publish_fn = std::make_unique<std::function<void(sensor_msgs::msg::PointCloud2)>>(
//                     [&bag_writer, topic](sensor_msgs::msg::PointCloud2 msg) {
//                         // auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>(rclcpp::serialize(msg));
//                         // bag_writer->write(serialized_msg, topic, msg.header.stamp);
//                         bag_writer->write(msg, topic, msg.header.stamp);
//                         // bag_writer->write(std::make_shared<rclcpp::SerializedMessage>(rclcpp::serialize(msg)), topic, "sensor_msgs::msg::PointCloud2", "");
//                     }
//                 );
//             }

//             comma::csv::input_stream< record > is( std::cin, csv );
//             comma::csv::passed< record > passed( is, std::cout, csv.flush );
//             unsigned int block = 0;
//             to_points points( options, csv );

//             while( std::cin.good() )
//             {
//                 //read binary from input
//                 const record* p = is.read();
//                 if (( !p || block != p->block ) && !points.empty() )
//                 {
//                     points.send( *publish_fn.get() );
//                 }
//                 if( !p ) { break; }
//                 if( pass_through ) { passed.write(); }
//                 block = p->block;
//                 points.add_record( *p, is );
//                 if( !has_block && !all ) { 
//                     points.send( *publish_fn.get() ); 
//                 }
//             }

//             if( publishing && options.exists( "--hang-on,--stay" ))
//             {
//                 for( int i = 0; i < 3; i++ )
//                 {
//                     rclcpp::spin_some(node);
//                     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//                 }
//             }

//             rclcpp::shutdown();
//             return 0;
//         }
//         else
//         {
//             std::cerr << comma::verbose.app_name() << ": requires --from or --to option" << std::endl;
//             return 1;
//         }
//     }
//     catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": exception: " << ex.what() << std::endl; }
//     catch( ... ) { std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; }
//     return 1;
}
