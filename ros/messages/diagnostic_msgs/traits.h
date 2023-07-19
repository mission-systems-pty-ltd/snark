#pragma once
#include <comma/csv/format.h>
#include <snark/ros/traits.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

namespace comma { namespace visiting {

// diagnostic_msgs::DiagnosticArray
// std_msgs/Header header
// diagnostic_msgs/DiagnosticStatus[] status
template <> struct traits < diagnostic_msgs::DiagnosticArray > {
    template < typename K, typename V > static void visit( const K&, const diagnostic_msgs::DiagnosticArray& p, V& v ) {
        v.apply("header", p.header);
        v.apply("status", p.status);
    }
    template < typename K, typename V > static void visit( const K&, diagnostic_msgs::DiagnosticArray& p, V& v ) {
        v.apply("header", p.header);
        v.apply("status", p.status);
    }
};

// diagnostic_msgs::DiagnosticStatus
// byte level
// string name
// string message
// string hardware_id
// diagnostic_msgs/KeyValue[] values
template <> struct traits < diagnostic_msgs::DiagnosticStatus > {
    template < typename K, typename V > static void visit( const K&, const diagnostic_msgs::DiagnosticStatus& p, V& v ) {
        v.apply("level", p.level);
        v.apply("name", p.name);
        v.apply("message", p.message);
        v.apply("hardware_id", p.hardware_id);
        v.apply("values", p.values);
    }
    template < typename K, typename V > static void visit( const K&, diagnostic_msgs::DiagnosticStatus& p, V& v ) {
        v.apply("level", p.level);
        v.apply("name", p.name);
        v.apply("message", p.message);
        v.apply("hardware_id", p.hardware_id);
        v.apply("values", p.values);
    }
};

// diagnostic_msgs::KeyValue
// string key
// string value
template <> struct traits < diagnostic_msgs::KeyValue > {
    template < typename K, typename V > static void visit( const K&, const diagnostic_msgs::KeyValue& p, V& v ) {
        v.apply("key", p.key);
        v.apply("value", p.value);
    }
    template < typename K, typename V > static void visit( const K&, diagnostic_msgs::KeyValue& p, V& v ) {
        v.apply("key", p.key);
        v.apply("value", p.value);
    }
};

}} // namespace comma { namespace visiting {
