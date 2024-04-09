// Copyright (c) 2024 Mission Systems Pty Ltd

#include "../timestamp.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <gtest/gtest.h>

using namespace snark::ouster::lidar;

// some useful unix-time (seconds from epoch) values
// you can get these with $ echo <date> | csv-time --to seconds
const unsigned long ut_20160101T120000 = 1451649600ul;
const unsigned long ut_20170101T000015 = 1483228815ul;
const unsigned long ut_20240101T120000 = 1704110400ul;

// take a base and offset in seconds and return nanoseconds
comma::uint64 make_timestamp( unsigned long base, long offset )
{
    return static_cast< comma::uint64 >( base + offset ) * 1000000000;
}

// In a couple of methods timestamp_converter casts comma::uint64 to long, to
// pass to boost::posix_time::microseconds. Make sure long is 64 bits wide.
TEST( timestamp_converter, sizeof_long )
{
    EXPECT_EQ( sizeof( long ), 8 );
}

TEST( timestamp_converter, utc_36 )
{
    timestamp_converter_t tc( config::time_standard_t::utc );
    EXPECT_EQ( tc.to_utc( make_timestamp( ut_20160101T120000, 0 )), boost::posix_time::from_iso_string( "20160101T120000" ) );
    EXPECT_EQ( tc.to_utc( make_timestamp( ut_20160101T120000, 1 )), boost::posix_time::from_iso_string( "20160101T120001" ) );
}

TEST( timestamp_converter, tai_36 )
{
    timestamp_converter_t tc( config::time_standard_t::tai );
    EXPECT_EQ( tc.to_utc( make_timestamp( ut_20160101T120000, 36 )), boost::posix_time::from_iso_string( "20160101T120000" ) );
    EXPECT_EQ( tc.to_utc( make_timestamp( ut_20160101T120000, 37 )), boost::posix_time::from_iso_string( "20160101T120001" ) );
}

TEST( timestamp_converter, utc_37 )
{
    timestamp_converter_t tc( config::time_standard_t::utc );
    EXPECT_EQ( tc.to_utc( make_timestamp( ut_20240101T120000, 0 )), boost::posix_time::from_iso_string( "20240101T120000" ) );
    EXPECT_EQ( tc.to_utc( make_timestamp( ut_20240101T120000, 1 )), boost::posix_time::from_iso_string( "20240101T120001" ) );
}

TEST( timestamp_converter, tai_37 )
{
    timestamp_converter_t tc( config::time_standard_t::tai );
    EXPECT_EQ( tc.to_utc( make_timestamp( ut_20240101T120000, 37 )), boost::posix_time::from_iso_string( "20240101T120000" ) );
    EXPECT_EQ( tc.to_utc( make_timestamp( ut_20240101T120000, 38 )), boost::posix_time::from_iso_string( "20240101T120001" ) );
}

TEST( timestamp_converter, utc_near_boundary )
{
    timestamp_converter_t tc( config::time_standard_t::utc );
    EXPECT_EQ( tc.to_utc( make_timestamp( ut_20170101T000015, 0 )), boost::posix_time::from_iso_string( "20170101T000015" ) );
}

TEST( timestamp_converter, tai_near_boundary )
{
    timestamp_converter_t tc( config::time_standard_t::tai );
    // The timestamp below appears to be after the leap-second change boundary
    // of 01-Jan-2017 but because it's TAI and boundaries are UTC it's actually
    // before so the leap-seconds are 36 rather than 37.
    EXPECT_EQ( tc.to_utc( make_timestamp( ut_20170101T000015, 0 )), boost::posix_time::from_iso_string( "20161231T235939" ) );
}

TEST( timestamp_converter, utc_cross_boundary )
{
    timestamp_converter_t tc( config::time_standard_t::utc );
    EXPECT_EQ( tc.to_utc( make_timestamp( ut_20160101T120000, 0 )), boost::posix_time::from_iso_string( "20160101T120000" ) );
    EXPECT_EQ( tc.to_utc( make_timestamp( ut_20160101T120000, 1 )), boost::posix_time::from_iso_string( "20160101T120001" ) );
    EXPECT_EQ( tc.to_utc( make_timestamp( ut_20240101T120000, 0 )), boost::posix_time::from_iso_string( "20240101T120000" ) );
    EXPECT_EQ( tc.to_utc( make_timestamp( ut_20240101T120000, 1 )), boost::posix_time::from_iso_string( "20240101T120001" ) );
}

TEST( timestamp_converter, tai_cross_boundary )
{
    timestamp_converter_t tc( config::time_standard_t::tai );
    EXPECT_EQ( tc.to_utc( make_timestamp( ut_20160101T120000, 36 )), boost::posix_time::from_iso_string( "20160101T120000" ) );
    EXPECT_EQ( tc.to_utc( make_timestamp( ut_20160101T120000, 37 )), boost::posix_time::from_iso_string( "20160101T120001" ) );
    EXPECT_EQ( tc.to_utc( make_timestamp( ut_20240101T120000, 37 )), boost::posix_time::from_iso_string( "20240101T120000" ) );
    EXPECT_EQ( tc.to_utc( make_timestamp( ut_20240101T120000, 38 )), boost::posix_time::from_iso_string( "20240101T120001" ) );
}

int main( int argc, char* argv[] )
{
    ::testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
    return 0;
}
