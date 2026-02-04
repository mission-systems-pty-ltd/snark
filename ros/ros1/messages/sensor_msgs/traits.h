#pragma once
#include <comma/csv/format.h>
#include <snark/ros/messages/traits.h>

#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Illuminance.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>
#include <sensor_msgs/JoyFeedbackArray.h>
#include <sensor_msgs/LaserEcho.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/MultiDOFJointState.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/RelativeHumidity.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/TimeReference.h>

namespace comma { namespace visiting {

// sensor_msgs::BatteryState
// std_msgs/Header header
// float32 voltage
// float32 temperature
// float32 current
// float32 charge
// float32 capacity
// float32 design_capacity
// float32 percentage
// uint8 power_supply_status
// uint8 power_supply_health
// uint8 power_supply_technology
// bool present
// float32[] cell_voltage
// float32[] cell_temperature
// string location
// string serial_number
template <> struct traits< sensor_msgs::BatteryState > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::BatteryState& p, V& v ) {
        v.apply("header", p.header);
        v.apply("voltage", p.voltage);
        #if ROS_VERSION_MINIMUM(1,16,0)
        v.apply("temperature", p.temperature);
	#endif
        v.apply("current", p.current);
        v.apply("charge", p.charge);
        v.apply("capacity", p.capacity);
        v.apply("design_capacity", p.design_capacity);
        v.apply("percentage", p.percentage);
        v.apply("power_supply_status", p.power_supply_status);
        v.apply("power_supply_health", p.power_supply_health);
        v.apply("power_supply_technology", p.power_supply_technology);
        v.apply("present", p.present);
        v.apply("cell_voltage", p.cell_voltage);
        #if ROS_VERSION_MINIMUM(1,16,0)
        v.apply("cell_temperature", p.cell_temperature);
	#endif
        v.apply("location", p.location);
        v.apply("serial_number", p.serial_number);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::BatteryState& p, V& v ) {
        v.apply("header", p.header);
        v.apply("voltage", p.voltage);
        #if ROS_VERSION_MINIMUM(1,16,0)
        v.apply("temperature", p.temperature);
	#endif
        v.apply("current", p.current);
        v.apply("charge", p.charge);
        v.apply("capacity", p.capacity);
        v.apply("design_capacity", p.design_capacity);
        v.apply("percentage", p.percentage);
        v.apply("power_supply_status", p.power_supply_status);
        v.apply("power_supply_health", p.power_supply_health);
        v.apply("power_supply_technology", p.power_supply_technology);
        v.apply("present", p.present);
        v.apply("cell_voltage", p.cell_voltage);
        #if ROS_VERSION_MINIMUM(1,16,0)
        v.apply("cell_temperature", p.cell_temperature);
	#endif
        v.apply("location", p.location);
        v.apply("serial_number", p.serial_number);
    }
};

// sensor_msgs::CameraInfo
// std_msgs/Header header
// uint32 height
// uint32 width
// string distortion_model
// float64[] D
// float64[9] K
// float64[9] R
// float64[12] P
// uint32 binning_x
// uint32 binning_y
// sensor_msgs/RegionOfInterest roi
template <> struct traits< sensor_msgs::CameraInfo > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::CameraInfo& p, V& v ) {
        v.apply("header", p.header);
        v.apply("height", p.height);
        v.apply("width", p.width);
        v.apply("distortion_model", p.distortion_model);
        v.apply("D", p.D);
        v.apply("K", p.K);
        v.apply("R", p.R);
        v.apply("P", p.P);
        v.apply("binning_x", p.binning_x);
        v.apply("binning_y", p.binning_y);
        v.apply("roi", p.roi);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::CameraInfo& p, V& v ) {
        v.apply("header", p.header);
        v.apply("height", p.height);
        v.apply("width", p.width);
        v.apply("distortion_model", p.distortion_model);
        v.apply("D", p.D);
        v.apply("K", p.K);
        v.apply("R", p.R);
        v.apply("P", p.P);
        v.apply("binning_x", p.binning_x);
        v.apply("binning_y", p.binning_y);
        v.apply("roi", p.roi);
    }
};

// sensor_msgs::ChannelFloat32
// string name
// float32[] values
template <> struct traits< sensor_msgs::ChannelFloat32 > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::ChannelFloat32& p, V& v ) {
        v.apply("name", p.name);
        v.apply("values", p.values);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::ChannelFloat32& p, V& v ) {
        v.apply("name", p.name);
        v.apply("values", p.values);
    }
};

// sensor_msgs::CompressedImage
// std_msgs/Header header
// string format
// uint8[] data
template <> struct traits< sensor_msgs::CompressedImage > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::CompressedImage& p, V& v ) {
        v.apply("header", p.header);
        v.apply("format", p.format);
        v.apply("data", p.data);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::CompressedImage& p, V& v ) {
        v.apply("header", p.header);
        v.apply("format", p.format);
        v.apply("data", p.data);
    }
};

// sensor_msgs::FluidPressure
// std_msgs/Header header
// float64 fluid_pressure
// float64 variance
template <> struct traits< sensor_msgs::FluidPressure > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::FluidPressure& p, V& v ) {
        v.apply("header", p.header);
        v.apply("fluid_pressure", p.fluid_pressure);
        v.apply("variance", p.variance);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::FluidPressure& p, V& v ) {
        v.apply("header", p.header);
        v.apply("fluid_pressure", p.fluid_pressure);
        v.apply("variance", p.variance);
    }
};

// sensor_msgs::Illuminance
// std_msgs/Header header
// float64 illuminance
// float64 variance
template <> struct traits< sensor_msgs::Illuminance > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::Illuminance& p, V& v ) {
        v.apply("header", p.header);
        v.apply("illuminance", p.illuminance);
        v.apply("variance", p.variance);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::Illuminance& p, V& v ) {
        v.apply("header", p.header);
        v.apply("illuminance", p.illuminance);
        v.apply("variance", p.variance);
    }
};

// sensor_msgs::Image
// std_msgs/Header header
// uint32 height
// uint32 width
// string encoding
// uint8 is_bigendian
// uint32 step
// uint8[] data
template <> struct traits< sensor_msgs::Image > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::Image& p, V& v ) {
        v.apply("header", p.header);
        v.apply("height", p.height);
        v.apply("width", p.width);
        v.apply("encoding", p.encoding);
        v.apply("is_bigendian", p.is_bigendian);
        v.apply("step", p.step);
        v.apply("data", p.data);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::Image& p, V& v ) {
        v.apply("header", p.header);
        v.apply("height", p.height);
        v.apply("width", p.width);
        v.apply("encoding", p.encoding);
        v.apply("is_bigendian", p.is_bigendian);
        v.apply("step", p.step);
        v.apply("data", p.data);
    }
};

// sensor_msgs::Imu
// std_msgs/Header header
// geometry_msgs/Quaternion orientation
// float64[9] orientation_covariance
// geometry_msgs/Vector3 angular_velocity
// float64[9] angular_velocity_covariance
// geometry_msgs/Vector3 linear_acceleration
// float64[9] linear_acceleration_covariance
template <> struct traits< sensor_msgs::Imu > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::Imu& p, V& v ) {
        v.apply("header", p.header) ;
        v.apply("orientation", p.orientation) ;
        v.apply("orientation_covariance", p.orientation_covariance) ;
        v.apply("angular_velocity", p.angular_velocity) ;
        v.apply("angular_velocity_covariance", p.angular_velocity_covariance) ;
        v.apply("linear_acceleration", p.linear_acceleration) ;
        v.apply("linear_acceleration_covariance", p.linear_acceleration_covariance) ;
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::Imu& p, V& v ) {
        v.apply("header", p.header) ;
        v.apply("orientation", p.orientation) ;
        v.apply("orientation_covariance", p.orientation_covariance) ;
        v.apply("angular_velocity", p.angular_velocity) ;
        v.apply("angular_velocity_covariance", p.angular_velocity_covariance) ;
        v.apply("linear_acceleration", p.linear_acceleration) ;
        v.apply("linear_acceleration_covariance", p.linear_acceleration_covariance) ;
    }
};

// sensor_msgs::JointState
// std_msgs/Header header
// string[] name
// float64[] position
// float64[] velocity
// float64[] effort
template <> struct traits< sensor_msgs::JointState > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::JointState& p, V& v ) {
        v.apply("header", p.header);
        v.apply("name", p.name);
        v.apply("position", p.position);
        v.apply("velocity", p.velocity);
        v.apply("effort", p.effort);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::JointState& p, V& v ) {
        v.apply("header", p.header);
        v.apply("name", p.name);
        v.apply("position", p.position);
        v.apply("velocity", p.velocity);
        v.apply("effort", p.effort);
    }
};

// sensor_msgs::Joy
// std_msgs/Header header
// float32[] axes
// int32[] buttons
template <> struct traits < sensor_msgs::Joy > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::Joy& p, V& v ) {
        v.apply("header", p.header);
        v.apply("axes", p.axes);
        v.apply("buttons", p.buttons);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::Joy& p, V& v ) {
        v.apply("header", p.header);
        v.apply("axes", p.axes);
        v.apply("buttons", p.buttons);
    }
};

// sensor_msgs::JoyFeedback
// uint8 type
// uint8 id
// float32 intensity
template <> struct traits < sensor_msgs::JoyFeedback > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::JoyFeedback& p, V& v ) {
        v.apply("type", p.type);
        v.apply("id", p.id);
        v.apply("intensity", p.intensity);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::JoyFeedback& p, V& v ) {
        v.apply("type", p.type);
        v.apply("id", p.id);
        v.apply("intensity", p.intensity);
    }
};

// sensor_msgs::JoyFeedbackArray
// sensor_msgs/JoyFeedback[] array
template <> struct traits < sensor_msgs::JoyFeedbackArray > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::JoyFeedbackArray& p, V& v ) {
        v.apply("array", p.array);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::JoyFeedbackArray& p, V& v ) {
        v.apply("array", p.array);
    }
};

// sensor_msgs::LaserEcho
// float32[] echoes
template <> struct traits < sensor_msgs::LaserEcho > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::LaserEcho& p, V& v ) {
        v.apply("echoes", p.echoes);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::LaserEcho& p, V& v ) {
        v.apply("echoes", p.echoes);
    }
};

// sensor_msgs::LaserScan
// std_msgs/Header header
// float32 angle_min
// float32 angle_max
// float32 angle_increment
// float32 time_increment
// float32 scan_time
// float32 range_min
// float32 range_max
// float32[] ranges
// float32[] intensities
template <> struct traits < sensor_msgs::LaserScan > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::LaserScan& p, V& v ) {
        v.apply("header", p.header);
        v.apply("angle_min", p.angle_min);
        v.apply("angle_max", p.angle_max);
        v.apply("angle_increment", p.angle_increment);
        v.apply("time_increment", p.time_increment);
        v.apply("scan_time", p.scan_time);
        v.apply("range_min", p.range_min);
        v.apply("range_max", p.range_max);
        v.apply("ranges", p.ranges);
        v.apply("intensities", p.intensities);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::LaserScan& p, V& v ) {
        v.apply("header", p.header);
        v.apply("angle_min", p.angle_min);
        v.apply("angle_max", p.angle_max);
        v.apply("angle_increment", p.angle_increment);
        v.apply("time_increment", p.time_increment);
        v.apply("scan_time", p.scan_time);
        v.apply("range_min", p.range_min);
        v.apply("range_max", p.range_max);
        v.apply("ranges", p.ranges);
        v.apply("intensities", p.intensities);
    }
};

// sensor_msgs::MagneticField
// std_msgs/Header header
// geometry_msgs/Vector3 magnetic_field
// float64[9] magnetic_field_covariance
template <> struct traits < sensor_msgs::MagneticField > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::MagneticField& p, V& v ) {
        v.apply("header", p.header);
        v.apply("magnetic_field", p.magnetic_field);
        v.apply("magnetic_field_covariance", p.magnetic_field_covariance);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::MagneticField& p, V& v ) {
        v.apply("header", p.header);
        v.apply("magnetic_field", p.magnetic_field);
        v.apply("magnetic_field_covariance", p.magnetic_field_covariance);
    }
};

// sensor_msgs::MultiDOFJointState
// std_msgs/Header header
// string[] joint_names
// geometry_msgs/Transform[] transforms
// geometry_msgs/Twist[] twist
// geometry_msgs/Wrench[] wrench
template <> struct traits < sensor_msgs::MultiDOFJointState > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::MultiDOFJointState& p, V& v ) {
        v.apply("header", p.header);
        v.apply("joint_names", p.joint_names);
        v.apply("transforms", p.transforms);
        v.apply("twist", p.twist);
        v.apply("wrench", p.wrench);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::MultiDOFJointState& p, V& v ) {
        v.apply("header", p.header);
        v.apply("joint_names", p.joint_names);
        v.apply("transforms", p.transforms);
        v.apply("twist", p.twist);
        v.apply("wrench", p.wrench);
    }
};

// sensor_msgs::MultiEchoLaserScan
// std_msgs/Header header
// float32 angle_min
// float32 angle_max
// float32 angle_increment
// float32 time_increment
// float32 scan_time
// float32 range_min
// float32 range_max
// sensor_msgs/LaserEcho[] ranges
// sensor_msgs/LaserEcho[] intensities
template <> struct traits < sensor_msgs::MultiEchoLaserScan > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::MultiEchoLaserScan& p, V& v ) {
        v.apply("header", p.header);
        v.apply("angle_min", p.angle_min);
        v.apply("angle_max", p.angle_max);
        v.apply("angle_increment", p.angle_increment);
        v.apply("time_increment", p.time_increment);
        v.apply("scan_time", p.scan_time);
        v.apply("range_min", p.range_min);
        v.apply("range_max", p.range_max);
        v.apply("ranges", p.ranges);
        v.apply("intensities", p.intensities);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::MultiEchoLaserScan& p, V& v ) {
        v.apply("header", p.header);
        v.apply("angle_min", p.angle_min);
        v.apply("angle_max", p.angle_max);
        v.apply("angle_increment", p.angle_increment);
        v.apply("time_increment", p.time_increment);
        v.apply("scan_time", p.scan_time);
        v.apply("range_min", p.range_min);
        v.apply("range_max", p.range_max);
        v.apply("ranges", p.ranges);
        v.apply("intensities", p.intensities);
    }
};

// sensor_msgs::NavSatFix
// std_msgs/Header header
// sensor_msgs/NavSatStatus status
// float64 latitude
// float64 longitude
// float64 altitude
// float64[9] position_covariance
// uint8 position_covariance_type
template <> struct traits < sensor_msgs::NavSatFix > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::NavSatFix& p, V& v ) {
        v.apply("header", p.header);
        v.apply("status", p.status);
        v.apply("latitude", p.latitude);
        v.apply("longitude", p.longitude);
        v.apply("altitude", p.altitude);
        v.apply("position_covariance", p.position_covariance);
        v.apply("position_covariance_type", p.position_covariance_type);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::NavSatFix& p, V& v ) {
        v.apply("header", p.header);
        v.apply("status", p.status);
        v.apply("latitude", p.latitude);
        v.apply("longitude", p.longitude);
        v.apply("altitude", p.altitude);
        v.apply("position_covariance", p.position_covariance);
        v.apply("position_covariance_type", p.position_covariance_type);
    }
};

// sensor_msgs::NavSatStatus
// int8 status
// uint16 service
template <> struct traits < sensor_msgs::NavSatStatus > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::NavSatStatus& p, V& v ) {
        v.apply("status", p.status);
        v.apply("service", p.service);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::NavSatStatus& p, V& v ) {
        v.apply("status", p.status);
        v.apply("service", p.service);
    }
};

// sensor_msgs::PointCloud
// std_msgs/Header header
// geometry_msgs/Point32[] points
// sensor_msgs/ChannelFloat32[] channels
template <> struct traits < sensor_msgs::PointCloud > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::PointCloud& p, V& v ) {
        v.apply("header", p.header);
        v.apply("points", p.points);
        v.apply("channels", p.channels);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::PointCloud& p, V& v ) {
        v.apply("header", p.header);
        v.apply("points", p.points);
        v.apply("channels", p.channels);
    }
};

// sensor_msgs::PointCloud2
// std_msgs/Header header
// uint32 height
// uint32 width
// sensor_msgs/PointField[] fields
// bool is_bigendian
// uint32 point_step
// uint32 row_step
// uint8[] data
// bool is_dense
template <> struct traits < sensor_msgs::PointCloud2 > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::PointCloud2& p, V& v ) {
        v.apply("header", p.header);
        v.apply("height", p.height);
        v.apply("width", p.width);
        v.apply("fields", p.fields);
        v.apply("is_bigendian", p.is_bigendian);
        v.apply("point_step", p.point_step);
        v.apply("row_step", p.row_step);
        v.apply("data", p.data);
        v.apply("is_dense", p.is_dense);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::PointCloud2& p, V& v ) {
        v.apply("header", p.header);
        v.apply("height", p.height);
        v.apply("width", p.width);
        v.apply("fields", p.fields);
        v.apply("is_bigendian", p.is_bigendian);
        v.apply("point_step", p.point_step);
        v.apply("row_step", p.row_step);
        v.apply("data", p.data);
        v.apply("is_dense", p.is_dense);
    }
};

// sensor_msgs::PointField
// string name
// uint32 offset
// uint8 datatype
// uint32 count
template <> struct traits < sensor_msgs::PointField > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::PointField& p, V& v ) {
        v.apply("name", p.name);
        v.apply("offset", p.offset);
        v.apply("datatype", p.datatype);
        v.apply("count", p.count);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::PointField& p, V& v ) {
        v.apply("name", p.name);
        v.apply("offset", p.offset);
        v.apply("datatype", p.datatype);
        v.apply("count", p.count);
    }
};

// sensor_msgs::Range
// std_msgs/Header header
// uint8 radiation_type
// float32 field_of_view
// float32 min_range
// float32 max_range
// float32 range
template <> struct traits < sensor_msgs::Range > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::Range& p, V& v ) {
        v.apply("header", p.header);
        v.apply("radiation_type", p.radiation_type);
        v.apply("field_of_view", p.field_of_view);
        v.apply("min_range", p.min_range);
        v.apply("max_range", p.max_range);
        v.apply("range", p.range);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::Range& p, V& v ) {
        v.apply("header", p.header);
        v.apply("radiation_type", p.radiation_type);
        v.apply("field_of_view", p.field_of_view);
        v.apply("min_range", p.min_range);
        v.apply("max_range", p.max_range);
        v.apply("range", p.range);
    }
};

// sensor_msgs::RegionOfInterest
// uint32 x_offset
// uint32 y_offset
// uint32 height
// uint32 width
// bool do_rectify
template <> struct traits < sensor_msgs::RegionOfInterest > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::RegionOfInterest& p, V& v ) {
        v.apply("x_offset", p.x_offset);
        v.apply("y_offset", p.y_offset);
        v.apply("height", p.height);
        v.apply("width", p.width);
        v.apply("do_rectify", p.do_rectify);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::RegionOfInterest& p, V& v ) {
        v.apply("x_offset", p.x_offset);
        v.apply("y_offset", p.y_offset);
        v.apply("height", p.height);
        v.apply("width", p.width);
        v.apply("do_rectify", p.do_rectify);
    }
};

// sensor_msgs::RelativeHumidity
// std_msgs/Header header
// float64 relative_humidity
// float64 variance
template <> struct traits < sensor_msgs::RelativeHumidity > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::RelativeHumidity& p, V& v ) {
        v.apply("header", p.header);
        v.apply("relative_humidity", p.relative_humidity);
        v.apply("variance", p.variance);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::RelativeHumidity& p, V& v ) {
        v.apply("header", p.header);
        v.apply("relative_humidity", p.relative_humidity);
        v.apply("variance", p.variance);
    }
};

// sensor_msgs::Temperature
// std_msgs/Header header
// float64 temperature
// float64 variance
template <> struct traits < sensor_msgs::Temperature > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::Temperature& p, V& v ) {
        v.apply("header", p.header);
        v.apply("temperature", p.temperature);
        v.apply("variance", p.variance);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::Temperature& p, V& v ) {
        v.apply("header", p.header);
        v.apply("temperature", p.temperature);
        v.apply("variance", p.variance);
    }
};

// sensor_msgs::TimeReference
// std_msgs/Header header
// time time_ref
// string source
template <> struct traits < sensor_msgs::TimeReference > {
    template < typename K, typename V > static void visit( const K&, const sensor_msgs::TimeReference& p, V& v ) {
        v.apply("header", p.header);
        v.apply("time_ref", p.time_ref);
        v.apply("source", p.source);
    }
    template < typename K, typename V > static void visit( const K&, sensor_msgs::TimeReference& p, V& v ) {
        v.apply("header", p.header);
        v.apply("time_ref", p.time_ref);
        v.apply("source", p.source);
    }
};

}} // namespace comma { namespace visiting { 
