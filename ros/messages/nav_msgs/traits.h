#pragma once
#include <comma/csv/format.h>
#include <snark/ros/messages/traits.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

namespace comma { namespace visiting {

// nav_msgs::GridCells
// std_msgs/Header header
// float32 cell_width
// float32 cell_height
// geometry_msgs/Point[] cells
template <> struct traits< nav_msgs::GridCells > {
    template < typename K, typename V > static void visit( const K&, const nav_msgs::GridCells& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "cell_width", p.cell_width );
        v.apply( "cell_height", p.cell_height );
        v.apply( "cells", p.cells );
    }
    template < typename K, typename V > static void visit( const K&, nav_msgs::GridCells& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "cell_width", p.cell_width );
        v.apply( "cell_height", p.cell_height );
        v.apply( "cells", p.cells );
    }
};

// nav_msgs::MapMetaData
// time map_load_time
// float32 resolution
// uint32 width
// uint32 height
// geometry_msgs/Pose origin
template <> struct traits< nav_msgs::MapMetaData > {
    template < typename K, typename V > static void visit( const K&, const nav_msgs::MapMetaData& p, V& v ) {
        v.apply( "map_load_time", p.map_load_time );
        v.apply( "resolution", p.resolution );
        v.apply( "width", p.width );
        v.apply( "height", p.height );
        v.apply( "origin", p.origin );
    }
    template < typename K, typename V > static void visit( const K&, nav_msgs::MapMetaData& p, V& v ) {
        v.apply( "map_load_time", p.map_load_time );
        v.apply( "resolution", p.resolution );
        v.apply( "width", p.width );
        v.apply( "height", p.height );
        v.apply( "origin", p.origin );
    }
};

// nav_msgs::OccupancyGrid
// std_msgs/Header header
// nav_msgs/MapMetaData info
// int8[] data
template <> struct traits< nav_msgs::OccupancyGrid > {
    template < typename K, typename V > static void visit( const K&, const nav_msgs::OccupancyGrid& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "info", p.info );
        v.apply( "data", p.data );
    }
    template < typename K, typename V > static void visit( const K&, nav_msgs::OccupancyGrid& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "info", p.info );
        v.apply( "data", p.data );
    }
};

// nav_msgs::Odometry
// std_msgs/Header header
// string child_frame_id
// geometry_msgs/PoseWithCovariance pose
// geometry_msgs/TwistWithCovariance twist
template <> struct traits< nav_msgs::Odometry > {
    template < typename K, typename V > static void visit( const K&, const nav_msgs::Odometry& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "child_frame_id", p.child_frame_id );
        v.apply( "pose", p.pose );
        v.apply( "twist", p.twist );
    }
    template < typename K, typename V > static void visit( const K&, nav_msgs::Odometry& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "child_frame_id", p.child_frame_id );
        v.apply( "pose", p.pose );
        v.apply( "twist", p.twist );
    }
};

// nav_msgs::Path
// std_msgs/Header header
// geometry_msgs/PoseStamped[] poses
template <> struct traits< nav_msgs::Path > {
    template < typename K, typename V > static void visit( const K&, const nav_msgs::Path& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "poses", p.poses );
    }
    template < typename K, typename V > static void visit( const K&, nav_msgs::Path& p, V& v ) {
        v.apply( "header", p.header );
        v.apply( "poses", p.poses );
    }
};

}} // namespace comma { namespace visiting {