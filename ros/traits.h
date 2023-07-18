#pragma once
#include <comma/csv/format.h>

#include <ros/ros.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

/// TODO: Every ROS message that has a variable sized field [], has been commented out and ignored.
/// These are:
///      diagnostic_msgs::DiagnosticArray
///      diagnostic_msgs::DiagnosticStatus
///      geometry_msgs::Polygon
///      geometry_msgs::PoseArray
///      nav_msgs::GridCells
///      nav_msgs::OccupancyGrid
///      nav_msgs::Path
///      sensor_msgs::BatteryState
///      sensor_msgs::CameraInfo
///      sensor_msgs::ChannelFloat32
///      sensor_msgs::CompressedImage
///      sensor_msgs::Image
///      sensor_msgs::JointState
///      sensor_msgs::Joy
///      sensor_msgs::JoyFeedbackArray
///      sensor_msgs::LaserEcho
///      sensor_msgs::LaserScan
///      sensor_msgs::MultiDOFJointState
///      sensor_msgs::MultiEchoLaserScan
///      sensor_msgs::PointCloud
///      sensor_msgs::PointCloud2

/// TODO: Maybe have this in a c++ file to reduce compilation time, will require having explicate specialisation in .h still.
/// just a thought...

namespace comma { namespace visiting {

// requires geometry_msgs first, also requires header & std_msgs 
template <> struct traits< ros::Time > {
    template < typename K, typename V > static void visit( const K&, const ros::Time& p, V& v ) {
        long sec = p.sec;
        long nsec = p.nsec;
        auto t = comma::csv::time::from_microseconds( sec * 1000000 + nsec / 1000);
        v.apply("t", t);
    }
    template < typename K, typename V > static void visit( const K&, ros::Time& p, V& v ) {
        boost::posix_time::ptime t;
        v.apply("t", t);
        long microseconds = comma::csv::time::to_microseconds(t);
        p.sec = microseconds / 1000000;
        p.nsec = (microseconds % 1000000) * 1000;
    }
};

} } // namespace comma { namespace visiting {

