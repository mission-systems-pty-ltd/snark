// Copyright (c) 2024 Mission Systems Pty Ltd

#pragma once

#include "config.h"
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <optional>

namespace snark { namespace ouster { namespace lidar {

class timestamp_converter_t
{
public:
    timestamp_converter_t( config::time_standard_t time_standard_ )
        : time_standard( time_standard_ )
        , leap_seconds( 0 )
        , valid_until( boost::date_time::neg_infin )
    {}

    boost::posix_time::ptime to_utc( comma::uint64 timestamp );

private:
    boost::posix_time::seconds offset( boost::posix_time::ptime uncorrected_time );

    config::time_standard_t time_standard;
    boost::posix_time::seconds leap_seconds;
    boost::posix_time::ptime valid_until;
};

} } } // namespace snark { namespace ouster { namespace lidar {
