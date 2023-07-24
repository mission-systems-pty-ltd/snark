#pragma once
#include <comma/csv/format.h>
#include <snark/ros/messages/traits.h>

#include <geometry_msgs/Accel.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/AccelWithCovariance.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/InertiaStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

namespace comma { namespace visiting {

// geometry_msgs::Accel
// geometry_msgs/Vector3 linear
// geometry_msgs/Vector3 angular
template <> struct traits< geometry_msgs::Accel > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::Accel& p, V& v ) {
        v.apply( "linear", p.linear );
        v.apply( "angular", p.angular );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::Accel& p, V& v ) {
        v.apply( "linear", p.linear );
        v.apply( "angular", p.angular );
    }
};

// geometry_msgs::AccelStamped
// std_msgs/Header header
// geometry_msgs/Accel accel
template <> struct traits< geometry_msgs::AccelStamped > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::AccelStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "accel", p.accel );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::AccelStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "accel", p.accel );
    }
};

// geometry_msgs::AccelWithCovariance
// geometry_msgs/Accel accel
// float64[36] covariance
template <> struct traits< geometry_msgs::AccelWithCovariance > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::AccelWithCovariance& p, V& v ) {
        v.apply( "accel", p.accel );
        v.apply( "covariance", p.covariance );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::AccelWithCovariance& p, V& v ) {
        v.apply( "accel", p.accel );
        v.apply( "covariance", p.covariance );
    }
};

// geometry_msgs::AccelWithCovarianceStamped
// std_msgs/Header header
// geometry_msgs/AccelWithCovariance accel
template <> struct traits< geometry_msgs::AccelWithCovarianceStamped > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::AccelWithCovarianceStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "accel", p.accel );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::AccelWithCovarianceStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "accel", p.accel );
    }
};

// geometry_msgs::Inertia
// float64 m
// geometry_msgs/Vector3 com
// float64 ixx
// float64 ixy
// float64 ixz
// float64 iyy
// float64 iyz
// float64 izz
template <> struct traits< geometry_msgs::Inertia > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::Inertia& p, V& v ) {
        v.apply( "m", p.m );
        v.apply( "com", p.com );
        v.apply( "ixx", p.ixx );
        v.apply( "ixy", p.ixy );
        v.apply( "ixz", p.ixz );
        v.apply( "iyy", p.iyy );
        v.apply( "iyz", p.iyz );
        v.apply( "izz", p.izz );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::Inertia& p, V& v ) {
        v.apply( "m", p.m );
        v.apply( "com", p.com );
        v.apply( "ixx", p.ixx );
        v.apply( "ixy", p.ixy );
        v.apply( "ixz", p.ixz );
        v.apply( "iyy", p.iyy );
        v.apply( "iyz", p.iyz );
        v.apply( "izz", p.izz );
    }
};

// geometry_msgs::InertiaStamped
// std_msgs/Header header
// geometry_msgs/Inertia inertia
template <> struct traits< geometry_msgs::InertiaStamped > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::InertiaStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "inertia", p.inertia );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::InertiaStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "inertia", p.inertia );
    }
};

// geometry_msgs::Point
// float64 x
// float64 y
// float64 z
template <> struct traits< geometry_msgs::Point > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::Point& p, V& v ) {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
        v.apply( "z", p.z );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::Point& p, V& v ) {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
        v.apply( "z", p.z );
    }
};

// geometry_msgs::Point32
// float32 x
// float32 y
// float32 z
template <> struct traits< geometry_msgs::Point32 > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::Point32& p, V& v ) {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
        v.apply( "z", p.z );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::Point32& p, V& v ) {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
        v.apply( "z", p.z );
    }
};

// geometry_msgs::PointStamped
// std_msgs/Header header
// geometry_msgs/Point point
template <> struct traits< geometry_msgs::PointStamped > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::PointStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "point", p.point );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::PointStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "point", p.point );
    }
}; 

// geometry_msgs::Polygon
// geometry_msgs/Point32[] points
template <> struct traits< geometry_msgs::Polygon > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::Polygon& p, V& v ) {
        v.apply( "points", p.points );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::Polygon& p, V& v ) {
        v.apply( "points", p.points );
    }
};

// geometry_msgs::PolygonStamped
// std_msgs/Header header
// geometry_msgs/Polygon polygon
template <> struct traits< geometry_msgs::PolygonStamped > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::PolygonStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "polygon", p.polygon );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::PolygonStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "polygon", p.polygon );
    }
};

// geometry_msgs::Pose
// geometry_msgs/Point position
// geometry_msgs/Quaternion orientation
template <> struct traits< geometry_msgs::Pose > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::Pose& p, V& v ) {
        v.apply( "position", p.position );
        v.apply( "orientation", p.orientation );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::Pose& p, V& v ) {
        v.apply( "position", p.position );
        v.apply( "orientation", p.orientation );
    }
};

// geometry_msgs::Pose2D
// float64 x
// float64 y
// float64 theta
template <> struct traits< geometry_msgs::Pose2D > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::Pose2D& p, V& v ) {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
        v.apply( "theta", p.theta );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::Pose2D& p, V& v ) {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
        v.apply( "theta", p.theta );
    }
};

// geometry_msgs::PoseArray
// std_msgs/Header header
// geometry_msgs/Pose[] poses
template <> struct traits< geometry_msgs::PoseArray > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::PoseArray& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "poses", p.poses );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::PoseArray& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "poses", p.poses );
    }
};

// geometry_msgs::PoseStamped
// std_msgs/Header header
// geometry_msgs/Pose pose
template <> struct traits< geometry_msgs::PoseStamped > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::PoseStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "pose", p.pose );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::PoseStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "pose", p.pose );
    }
};

// geometry_msgs::PoseWithCovariance
// geometry_msgs/Pose pose
// float64[36] covariance
template <> struct traits< geometry_msgs::PoseWithCovariance > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::PoseWithCovariance& p, V& v ) {
        v.apply( "pose", p.pose );
        v.apply( "covariance", p.covariance );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::PoseWithCovariance& p, V& v ) {
        v.apply( "pose", p.pose );
        v.apply( "covariance", p.covariance );
    }
};

// geometry_msgs::PoseWithCovarianceStamped
// std_msgs/Header header
// geometry_msgs/PoseWithCovariance pose
template <> struct traits< geometry_msgs::PoseWithCovarianceStamped > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::PoseWithCovarianceStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "pose", p.pose );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::PoseWithCovarianceStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "pose", p.pose );
    }
};

// geometry_msgs::Quaternion
// float64 x
// float64 y
// float64 z
// float64 w
template <> struct traits< geometry_msgs::Quaternion > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::Quaternion& p, V& v ) {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
        v.apply( "z", p.z );
        v.apply( "w", p.w );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::Quaternion& p, V& v ) {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
        v.apply( "z", p.z );
        v.apply( "w", p.w );
    }
};

// geometry_msgs::QuaternionStamped
// std_msgs/Header header
// geometry_msgs/Quaternion quaternion
template <> struct traits< geometry_msgs::QuaternionStamped > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::QuaternionStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "quaternion", p.quaternion );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::QuaternionStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "quaternion", p.quaternion );
    }
};

// geometry_msgs::Transform
// geometry_msgs/Vector3 translation
// geometry_msgs/Quaternion rotation
template <> struct traits< geometry_msgs::Transform > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::Transform& p, V& v ) {
        v.apply( "translation", p.translation );
        v.apply( "rotation", p.rotation );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::Transform& p, V& v ) {
        v.apply( "translation", p.translation );
        v.apply( "rotation", p.rotation );
    }
};

// geometry_msgs::TransformStamped
// std_msgs/Header header
// string child_frame_id
// geometry_msgs/Transform transform
template <> struct traits< geometry_msgs::TransformStamped > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::TransformStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "child_frame_id", p.child_frame_id );
        v.apply( "transform", p.transform );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::TransformStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "child_frame_id", p.child_frame_id );
        v.apply( "transform", p.transform );
    }
};

// geometry_msgs::Twist
// geometry_msgs/Vector3 linear
// geometry_msgs/Vector3 angular
template <> struct traits< geometry_msgs::Twist > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::Twist& p, V& v ) {
        v.apply( "linear", p.linear );
        v.apply( "angular", p.angular );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::Twist& p, V& v ) {
        v.apply( "linear", p.linear );
        v.apply( "angular", p.angular );
    }
};

// geometry_msgs::TwistStamped
// std_msgs/Header header
// geometry_msgs/Twist twist
template <> struct traits< geometry_msgs::TwistStamped > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::TwistStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "twist", p.twist );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::TwistStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "twist", p.twist );
    }
};

// geometry_msgs::TwistWithCovariance
// geometry_msgs/Twist twist
// float64[36] covariance
template <> struct traits< geometry_msgs::TwistWithCovariance > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::TwistWithCovariance& p, V& v ) {
        v.apply( "twist", p.twist );
        v.apply( "covariance", p.covariance );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::TwistWithCovariance& p, V& v ) {
        v.apply( "twist", p.twist );
        v.apply( "covariance", p.covariance );
    }
};

// geometry_msgs::TwistWithCovarianceStamped
// std_msgs/Header header
// geometry_msgs/TwistWithCovariance twist
template <> struct traits< geometry_msgs::TwistWithCovarianceStamped > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::TwistWithCovarianceStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "twist", p.twist );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::TwistWithCovarianceStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "twist", p.twist );
    }
};

// geometry_msgs::Vector3
// float64 x
// float64 y
// float64 z
template <> struct traits< geometry_msgs::Vector3 > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::Vector3& p, V& v ) {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
        v.apply( "z", p.z );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::Vector3& p, V& v ) {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
        v.apply( "z", p.z );
    }
};

// geometry_msgs::Vector3Stamped
// std_msgs/Header header
// geometry_msgs/Vector3 vector
template <> struct traits< geometry_msgs::Vector3Stamped > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::Vector3Stamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "vector", p.vector );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::Vector3Stamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "vector", p.vector );
    }
};

// geometry_msgs::Wrench
// geometry_msgs/Vector3 force
// geometry_msgs/Vector3 torque
template <> struct traits< geometry_msgs::Wrench > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::Wrench& p, V& v ) {
        v.apply( "force", p.force );
        v.apply( "torque", p.torque );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::Wrench& p, V& v ) {
        v.apply( "force", p.force );
        v.apply( "torque", p.torque );
    }
};

// geometry_msgs::WrenchStamped
// std_msgs/Header header
// geometry_msgs/Wrench wrench
template <> struct traits< geometry_msgs::WrenchStamped > {
    template < typename K, typename V > static void visit( const K&, const geometry_msgs::WrenchStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "wrench", p.wrench );
    }
    template < typename K, typename V > static void visit( const K&, geometry_msgs::WrenchStamped& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "wrench", p.wrench );
    }
};

}} // namespace comma { namespace visiting { 