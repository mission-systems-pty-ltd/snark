#include <gtest/gtest.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "../traits.h"

static double deg2rad( double degrees ) { return degrees / 180.0 * M_PI; }

TEST( nmea, GPGGA )
{
    std::string line = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
    snark::nmea::string s( line );
    EXPECT_FALSE( s.is_proprietary() );
    EXPECT_EQ( "GP", s.talker_id() );
    EXPECT_EQ( "GGA", s.message_type() );
    EXPECT_EQ( snark::nmea::messages::gga::type, s.message_type() );
    snark::nmea::messages::gga gga = comma::csv::ascii< snark::nmea::messages::gga >().get( s.values() );
    EXPECT_EQ( "GPGGA", gga.id );
    EXPECT_EQ( boost::posix_time::ptime( boost::posix_time::microsec_clock::universal_time().date(), boost::posix_time::time_duration( 12, 35, 19 ) ), gga.time.value );
    EXPECT_NEAR( deg2rad( 48.1173 ), gga.coordinates.latitude.value, 1e-9 );
    EXPECT_NEAR( deg2rad( 11.5166667 ), gga.coordinates.longitude.value, 1e-9 );
    EXPECT_EQ( snark::nmea::messages::gga::quality_t::gps_fix, gga.quality );
    EXPECT_EQ( 8u, gga.satellites_in_use );
    EXPECT_NEAR( 0.9, gga.hdop, 1e-9 );
    EXPECT_NEAR( 545.4, gga.orthometric_height, 1e-9 );
    EXPECT_EQ( "M", gga.height_unit );
    EXPECT_NEAR( 46.9, gga.geoid_separation, 1e-9 );
    EXPECT_EQ( "M", gga.geoid_separation_unit );
    EXPECT_EQ( 0, gga.age_of_differential_gps_data_record );
    EXPECT_EQ( "", gga.reference_station_id );
}

TEST( nmea, GNGGA )
{
    std::string line = "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*59";
    snark::nmea::string s( line );
    EXPECT_FALSE( s.is_proprietary() );
    EXPECT_EQ( "GN", s.talker_id() );
    EXPECT_EQ( "GGA", s.message_type() );
    EXPECT_EQ( snark::nmea::messages::gga::type, s.message_type() );
    snark::nmea::messages::gga gga = comma::csv::ascii< snark::nmea::messages::gga >().get( s.values() );
    EXPECT_EQ( "GNGGA", gga.id );
    EXPECT_EQ( boost::posix_time::ptime( boost::posix_time::microsec_clock::universal_time().date(), boost::posix_time::time_duration( 12, 35, 19 ) ), gga.time.value );
    EXPECT_NEAR( deg2rad( 48.1173 ), gga.coordinates.latitude.value, 1e-9 );
    EXPECT_NEAR( deg2rad( 11.5166667 ), gga.coordinates.longitude.value, 1e-9 );
    EXPECT_EQ( snark::nmea::messages::gga::quality_t::gps_fix, gga.quality );
    EXPECT_EQ( 8u, gga.satellites_in_use );
    EXPECT_NEAR( 0.9, gga.hdop, 1e-9 );
    EXPECT_NEAR( 545.4, gga.orthometric_height, 1e-9 );
    EXPECT_EQ( "M", gga.height_unit );
    EXPECT_NEAR( 46.9, gga.geoid_separation, 1e-9 );
    EXPECT_EQ( "M", gga.geoid_separation_unit );
    EXPECT_EQ( 0, gga.age_of_differential_gps_data_record );
    EXPECT_EQ( "", gga.reference_station_id );
}

TEST( nmea, GLGGA )
{
    std::string line = "$GLGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*5B";
    snark::nmea::string s( line );
    EXPECT_FALSE( s.is_proprietary() );
    EXPECT_EQ( "GL", s.talker_id() );
    EXPECT_EQ( "GGA", s.message_type() );
    EXPECT_EQ( snark::nmea::messages::gga::type, s.message_type() );
    snark::nmea::messages::gga gga = comma::csv::ascii< snark::nmea::messages::gga >().get( s.values() );
    EXPECT_EQ( "GLGGA", gga.id );
    EXPECT_EQ( boost::posix_time::ptime( boost::posix_time::microsec_clock::universal_time().date(), boost::posix_time::time_duration( 12, 35, 19 ) ), gga.time.value );
    EXPECT_NEAR( deg2rad( 48.1173 ), gga.coordinates.latitude.value, 1e-9 );
    EXPECT_NEAR( deg2rad( 11.5166667 ), gga.coordinates.longitude.value, 1e-9 );
    EXPECT_EQ( snark::nmea::messages::gga::quality_t::gps_fix, gga.quality );
    EXPECT_EQ( 8u, gga.satellites_in_use );
    EXPECT_NEAR( 0.9, gga.hdop, 1e-9 );
    EXPECT_NEAR( 545.4, gga.orthometric_height, 1e-9 );
    EXPECT_EQ( "M", gga.height_unit );
    EXPECT_NEAR( 46.9, gga.geoid_separation, 1e-9 );
    EXPECT_EQ( "M", gga.geoid_separation_unit );
    EXPECT_EQ( 0, gga.age_of_differential_gps_data_record );
    EXPECT_EQ( "", gga.reference_station_id );
}

TEST( nmea, GNRMC ) {
    std::string line = "$GNRMC,232341.40,A,3648.2223524,N,12011.3811276,W,0.05,,061219,,,A*45";
    snark::nmea::string s( line );
    EXPECT_FALSE( s.is_proprietary() );
    EXPECT_EQ( "GN", s.talker_id() );
    EXPECT_EQ( "RMC", s.message_type() );
    EXPECT_EQ( snark::nmea::messages::rmc::type, s.message_type() );
    snark::nmea::messages::rmc rmc = comma::csv::ascii< snark::nmea::messages::rmc>().get( s.values() );
    EXPECT_EQ( "GNRMC", rmc.id );
    EXPECT_EQ( boost::posix_time::ptime( boost::posix_time::microsec_clock::universal_time().date(), boost::posix_time::time_duration( 23, 23, 41, 400000 ) ), rmc.time.value );
    EXPECT_EQ( "A", rmc.validity );
    EXPECT_NEAR( deg2rad( 36.8037058733 ), rmc.coordinates.latitude.value, 1e-9 );
    EXPECT_NEAR( deg2rad( -120.18968546 ), rmc.coordinates.longitude.value, 1e-9 );
    EXPECT_NEAR( 0.05, rmc.speed_in_knots, 1e-9 );
    EXPECT_NEAR( 0.0, rmc.true_course, 1e-9 );
    EXPECT_EQ( boost::gregorian::date(2019, 12, 6), rmc.date.value );
    EXPECT_NEAR( 0.0, rmc.variation, 1e-9 );
    EXPECT_TRUE(  rmc.east_west.empty() );
}

TEST( nmea, trimble_AVR )
{
    std::string line = "$PTNL,AVR,181059.6,+149.4688,Yaw,+0.0134,Tilt,,,60.191,3,2.5,6*00";
    snark::nmea::string s( line );
    EXPECT_TRUE( s.is_proprietary() );
    EXPECT_EQ( "TNL", s.manufacturer_code() );
    EXPECT_EQ( snark::nmea::messages::trimble::manufacturer_code, s.manufacturer_code() );
    const snark::nmea::messages::trimble::string& t = static_cast< const snark::nmea::messages::trimble::string& >( s );
    EXPECT_EQ( "AVR", t.message_type() );
    EXPECT_EQ( snark::nmea::messages::trimble::avr::type, t.message_type() );
    snark::nmea::messages::trimble::avr avr = comma::csv::ascii< snark::nmea::messages::trimble::avr >().get( s.values() );
    EXPECT_EQ( "PTNL", avr.id );
    EXPECT_EQ( "AVR", avr.message_type );
    EXPECT_EQ( boost::posix_time::ptime( boost::posix_time::microsec_clock::universal_time().date(), boost::posix_time::time_duration( 18, 10, 59, 600000 ) ), avr.time.value );
    EXPECT_NEAR( deg2rad( 149.4688 ), avr.yaw.value, 1e-9 );
    EXPECT_EQ( "Yaw", avr.yaw_string );
    EXPECT_NEAR( deg2rad( 0.0134 ), avr.tilt.value, 1e-9 );
    EXPECT_EQ( "Tilt", avr.tilt_string );
    EXPECT_EQ( 0, avr.roll.value );
    EXPECT_EQ( "", avr.roll_string );
    EXPECT_NEAR( 60.191, avr.range, 1e-9 );
    EXPECT_EQ( snark::nmea::messages::trimble::avr::quality_t::differential_carrier_phase_solution_rtk_int, avr.quality );
    EXPECT_NEAR( 2.5, avr.pdop, 1e-9 );
    EXPECT_EQ( 6u, avr.satellites_in_use );
}
