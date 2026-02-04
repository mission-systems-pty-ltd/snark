#include <comma/csv/format.h>

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/UInt64MultiArray.h>


namespace snark { namespace ros {

template< typename T > struct type_to_std_msgs {};

template<> struct type_to_std_msgs< char > : public comma::csv::format::type_to_enum< char >
{
    using type = std_msgs::Int8;
    using array_type = std_msgs::Int8MultiArray;
};

template<> struct type_to_std_msgs< unsigned char > : public comma::csv::format::type_to_enum< unsigned char >
{
    using type = std_msgs::UInt8;
    using array_type = std_msgs::UInt8MultiArray;
};

template<> struct type_to_std_msgs< comma::int16 > : public comma::csv::format::type_to_enum< comma::int16 >
{
    using type = std_msgs::Int16;
    using array_type = std_msgs::Int16MultiArray;
};

template<> struct type_to_std_msgs< comma::uint16 > : public comma::csv::format::type_to_enum< comma::uint16 >
{
    using type = std_msgs::UInt16;
    using array_type = std_msgs::UInt16MultiArray;
};

template<> struct type_to_std_msgs< comma::int32 > : public comma::csv::format::type_to_enum< comma::int32 >
{
    using type = std_msgs::Int32;
    using array_type = std_msgs::Int32MultiArray;
};

template<> struct type_to_std_msgs< comma::uint32 > : public comma::csv::format::type_to_enum< comma::uint32 >
{
    using type = std_msgs::UInt32;
    using array_type = std_msgs::UInt32MultiArray;
};

template<> struct type_to_std_msgs< comma::int64 > : public comma::csv::format::type_to_enum< comma::int64 >
{
    using type = std_msgs::Int64;
    using array_type = std_msgs::Int64MultiArray;
};

template<> struct type_to_std_msgs< comma::uint64 > : public comma::csv::format::type_to_enum< comma::uint64 >
{
    using type = std_msgs::UInt64;
    using array_type = std_msgs::UInt64MultiArray;
};

template<> struct type_to_std_msgs< float > : public comma::csv::format::type_to_enum< float >
{
    using type = std_msgs::Float32;
    using array_type = std_msgs::Float32MultiArray;
};

template<> struct type_to_std_msgs< double > : public comma::csv::format::type_to_enum< double >
{
    using type = std_msgs::Float64;
    using array_type = std_msgs::Float64MultiArray;
};

}}

namespace comma { namespace visiting {

template <> struct traits< std_msgs::MultiArrayDimension >
{
    template < typename K, typename V > static void visit ( K const&, std_msgs::MultiArrayDimension& t, V& v )
    {
        v.apply( "label", t.label );
        v.apply( "size", t.size );
        v.apply( "stride", t.stride );
    }

    template < typename K, typename V > static void visit ( K const&, std_msgs::MultiArrayDimension const& t, V& v )
    {
        v.apply( "label", t.label );
        v.apply( "size", t.size );
        v.apply( "stride", t.stride );
    }
};

}}


