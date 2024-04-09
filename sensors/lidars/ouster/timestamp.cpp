// Copyright (c) 2024 Mission Systems Pty Ltd

#include "timestamp.h"
#include "../../../timing/time.h"
#include <comma/base/exception.h>
#include <comma/timing/tai.h>

namespace snark { namespace ouster { namespace lidar {

boost::posix_time::seconds timestamp_converter_t::offset( boost::posix_time::ptime uncorrected_time )
{
    if( uncorrected_time < valid_until ) { return leap_seconds; }

    switch( time_standard )
    {
        case snark::ouster::lidar::config::time_standard_t::tai:
            {
                auto [ leap_seconds_, valid_until_ ] = comma::timing::tai::leap_seconds_with_valid_time( uncorrected_time, false );
                leap_seconds = boost::posix_time::seconds( -leap_seconds_ );
                valid_until = valid_until_;
                break;
            }
        case snark::ouster::lidar::config::time_standard_t::utc:
            {
                leap_seconds = boost::posix_time::seconds( 0 );
                valid_until = boost::date_time::pos_infin;
                break;
            }
    }
    return leap_seconds;
}

boost::posix_time::ptime timestamp_converter_t::to_utc( comma::uint64 timestamp )
{
    static_assert( sizeof( long ) == 8, "expected long to be eight bytes" );
    boost::posix_time::ptime uncorrected_time( snark::timing::epoch
                                             , boost::posix_time::microseconds( static_cast< long >( timestamp / 1000 )));
    return uncorrected_time + offset( uncorrected_time );
}

} } } // namespace snark { namespace ouster { namespace lidar {
