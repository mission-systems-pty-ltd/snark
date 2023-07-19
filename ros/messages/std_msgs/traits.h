#pragma once
#include <comma/csv/format.h>
#include <snark/ros/traits.h>

#include <std_msgs/Header.h>

namespace comma { namespace visiting {

// std_msgs::Header
// uint32 seq
// time stamp
// string frame_id
template <> struct traits< std_msgs::Header > {
    template < typename K, typename V > static void visit( const K&, const std_msgs::Header& p, V& v ) {
        v.apply("seq", p.seq);
        v.apply("stamp", p.stamp);
        v.apply("frame_id", p.frame_id);
    }
    template < typename K, typename V > static void visit( const K&, std_msgs::Header& p, V& v ) {
        v.apply("seq", p.seq);
        v.apply("stamp", p.stamp);
        v.apply("frame_id", p.frame_id);
    }
};


}} // namespace comma { namespace visiting {