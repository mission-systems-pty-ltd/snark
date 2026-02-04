#pragma once
#include <comma/csv/format.h>
#include <rclcpp/rclcpp.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
//#include "../version.h"

namespace snark { namespace ros { namespace time {

boost::posix_time::ptime to_boost( const rclcpp::Time& time ){
    int64_t nanoseconds = time.nanoseconds();
    // Convert nanoseconds to seconds and remainder microseconds
    int64_t seconds = nanoseconds / 1000000000;
    int64_t microseconds = (nanoseconds % 1000000000) / 1000;
    // Convert to Boost ptime
    boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1));
    boost::posix_time::time_duration time_from_epoch = 
    boost::posix_time::seconds(seconds) + 
    boost::posix_time::microseconds(microseconds);
    return epoch + time_from_epoch;
}

rclcpp::Time from_boost(const boost::posix_time::ptime& boost_time)
{
    // Define the epoch
    boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1));

    // Calculate the time duration from the epoch to the given boost_time
    boost::posix_time::time_duration duration = boost_time - epoch;

    // Extract seconds and nanoseconds
    int64_t seconds = duration.total_seconds();
    int64_t nanoseconds = (duration.total_microseconds() % 1000000) * 1000;

    // Convert to rclcpp::Time
    return rclcpp::Time(seconds, nanoseconds);
}

}}} // namespace snark { namespace ros { namespace time {
