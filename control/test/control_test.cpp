// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2015 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>
#include <comma/base/exception.h>
#include <boost/date_time/posix_time/ptime.hpp>
#include <comma/csv/stream.h>
#include "../pid.h"
#include "../wayline.h"

namespace snark { namespace test {

TEST( pid, constructor )
{
    snark::control::pid p( 0.1, 0.01, 0.001 );
    snark::control::pid p1( 0.1, 0.01, 0.001 );
    snark::control::pid p2( 0.1, 0.01, 0.001, 0.25 );
    snark::control::pid p3( 0.1, 0.01, 0.001 );
    snark::control::pid p4( 0.1, 0.01, 0.001, 0.25 );
}

TEST( pid, threshold_throw )
{
    ASSERT_THROW( snark::control::pid( 0, 0, 0, -1.0 ), comma::exception );
    ASSERT_THROW( snark::control::pid( 0, 0, 0, 0.0 ), comma::exception );
}

static const boost::posix_time::ptime initial_time = boost::posix_time::from_iso_string( "20101010T121212.123456" );

TEST( pid, negative_time_increment_throw )
{
    snark::control::pid pid( 0, 0, 0 );
    boost::posix_time::ptime t = initial_time;
    pid( 0, t );
    boost::posix_time::time_duration dt = boost::posix_time::microseconds( 1 );
    ASSERT_THROW( pid( 0, t - dt ), comma::exception );
}

TEST( pid, zero_time_increment )
{
    snark::control::pid pid( 0, 0, 1 );
    boost::posix_time::ptime t = initial_time;
    pid( 1, t );
    EXPECT_NEAR( 0, pid( 1, t ), 1e-12 );
}

TEST( pid, proportional )
{
    const double p = 0.12;
    snark::control::pid pid( p, 0, 0 );
    boost::posix_time::ptime t = initial_time;
    double error = 0.1;
    EXPECT_EQ( p*error, pid( error, t ) );
    t += boost::posix_time::microseconds( 1 );
    error = 0.11;
    EXPECT_EQ( p*error, pid( error, t ) );
    t += boost::posix_time::microseconds( 2 );
    error = 0.12;
    EXPECT_EQ( p*error, pid( error, t ) );
}

double to_seconds( double microseconds ) { return microseconds/1e6; }

TEST( pid, integral )
{
    const double i = 0.04;
    snark::control::pid pid( 0, i, 0 );
    boost::posix_time::ptime t = initial_time;
    double error = 0.1;
    double integral = 0;
    EXPECT_EQ( i*integral, pid( error, t ) );
    int microseconds = 10;
    t += boost::posix_time::microseconds( microseconds );
    error = 0.11;
    integral += error * to_seconds( microseconds );
    EXPECT_EQ( i*integral, pid( error, t ) );
    microseconds = 20;
    t += boost::posix_time::microseconds( microseconds );
    error = -0.12;
    integral += error * to_seconds( microseconds );
    EXPECT_EQ( i*integral, pid( error, t ) );
}

TEST( pid, threshold )
{
    const double i = 0.1;
    const double threshold = 0.00015;
    snark::control::pid pid( 0, i, 0, threshold );
    boost::posix_time::ptime t = initial_time;
    double error = 0;
    EXPECT_EQ( 0, pid( error, t ) );
    int microseconds = 2000;
    t += boost::posix_time::microseconds( microseconds );
    error = -0.1;
    EXPECT_EQ( i*(-threshold), pid( error, t ) );
}

TEST( pid, derivative_internal )
{
    const double d = 0.1;
    snark::control::pid pid( 0, 0, d );
    boost::posix_time::ptime t = initial_time;
    double error = 0.1;
    EXPECT_EQ( 0, pid( error, t ) );
    int microseconds = 1;
    t += boost::posix_time::microseconds( microseconds );
    double previous_error = error;
    error = 0.11;
    double derivative = ( error - previous_error )/ to_seconds( microseconds );
    EXPECT_EQ( d*derivative, pid( error, t ) );
    microseconds = 2;
    t += boost::posix_time::microseconds( microseconds );
    previous_error = error;
    error = 0.12;
    derivative = ( error - previous_error )/ to_seconds( microseconds );
    EXPECT_EQ( d*derivative, pid( error, t ) );
}

TEST( pid, derivative_external )
{
    const double d = 0.1;
    snark::control::pid pid( 0, 0, d );
    boost::posix_time::ptime t = initial_time;
    double error = 0;
    EXPECT_EQ( 0, pid( error, t ) );
    t += boost::posix_time::microseconds( 1 );
    double derivative = 0.1;
    EXPECT_EQ( d*derivative, pid( error, derivative, t ) );
    t += boost::posix_time::microseconds( 2 );
    derivative = 0.2;
    EXPECT_EQ( d*derivative, pid( error, derivative, t ) );
}

TEST( pid, reset )
{
    double i = 0.1;
    snark::control::pid pid( 0, i, 0 );
    boost::posix_time::ptime t = initial_time;
    double error = 0.1;
    double integral = 0;
    EXPECT_EQ( i*integral, pid( error, t ) );
    int microseconds = 2;
    t += boost::posix_time::microseconds( microseconds );
    error = 0.2;
    integral += error * to_seconds( microseconds );
    EXPECT_EQ( i*integral, pid( error, t ) );
    pid.reset();
    integral = 0;
    EXPECT_EQ( i*integral, pid( error, t ) );
    microseconds = 3;
    t += boost::posix_time::microseconds( microseconds );
    error = 0.3;
    integral = error * to_seconds( microseconds );
    EXPECT_EQ( i*integral, pid( error, t ) );
}

TEST( pid, combination )
{
    double p = 0.1;
    double i = 0.01;
    double d = 0.001;
    snark::control::pid pid( p, i, d );
    boost::posix_time::ptime t = initial_time;
    double error = 0.1;
    double integral = 0;
    double derivative = 0;
    EXPECT_EQ( p*error + i*integral + d*derivative, pid( error, t ) );
    int microseconds = 2;
    t += boost::posix_time::microseconds( microseconds );
    double previous_error = error;
    error = -0.2;
    integral += error * to_seconds( microseconds );
    derivative = ( error - previous_error ) / to_seconds( microseconds );
    EXPECT_EQ( p*error + i*integral + d*derivative, pid( error, t ) );
    microseconds = 3;
    t += boost::posix_time::microseconds( microseconds );
    previous_error = error;
    error = 0.3;
    integral += error * to_seconds( microseconds );
    derivative = ( error - previous_error ) / to_seconds( microseconds );
    EXPECT_EQ( p*error + i*integral + d*derivative, pid( error, t ) );
}

static const double eps = 1e-12;

struct shift
{
    double dx;
    double dy;
    shift() : dx( 0 ), dy( 0 ) {}
    shift( double dx, double dy) : dx( dx ), dy( dy ) {}
    double heading( double from_x, double from_y, double to_x, double to_y )
    {
        snark::control::wayline::position_t from( from_x + dx, from_y + dy );
        snark::control::wayline::position_t to( to_x + dx, to_y + dy );
        snark::control::wayline wayline( from, to );
        return wayline.heading();
    }
};

TEST( wayline, heading )
{
    {
        shift s;
        EXPECT_NEAR( 0.0, s.heading(0,0,2,0), eps );
        EXPECT_NEAR( M_PI/4, s.heading(0,0,2,2), eps );
        EXPECT_NEAR( M_PI/2, s.heading(0,0,0,2), eps );
        EXPECT_NEAR( 3*M_PI/4, s.heading(0,0,-2,2), eps );
        EXPECT_NEAR( M_PI, s.heading(0,0,-2,eps/10), eps );

        EXPECT_NEAR( -M_PI/4, s.heading(0,0,2,-2), eps );
        EXPECT_NEAR( -M_PI/2, s.heading(0,0,0,-2), eps );
        EXPECT_NEAR( -3*M_PI/4, s.heading(0,0,-2,-2), eps );
        EXPECT_NEAR( -M_PI, s.heading(0,0,-2,-eps/10), eps );
    }

    {
        shift s( -0.12345, -5.4321 );
        EXPECT_NEAR( 0.0, s.heading(0,0,2,0), eps );
        EXPECT_NEAR( M_PI/4, s.heading(0,0,2,2), eps );
        EXPECT_NEAR( M_PI/2, s.heading(0,0,0,2), eps );
        EXPECT_NEAR( 3*M_PI/4, s.heading(0,0,-2,2), eps );
        EXPECT_NEAR( M_PI, s.heading(0,0,-2,eps/10), eps );

        EXPECT_NEAR( -M_PI/4, s.heading(0,0,2,-2), eps );
        EXPECT_NEAR( -M_PI/2, s.heading(0,0,0,-2), eps );
        EXPECT_NEAR( -3*M_PI/4, s.heading(0,0,-2,-2), eps );
        EXPECT_NEAR( -M_PI, s.heading(0,0,-2,-eps/10), eps );
    }
}

TEST( wayline, is_past_endpoint )
{
    snark::control::wayline::position_t from( 1, 2 );
    snark::control::wayline::position_t to( 3, 5 );
    snark::control::wayline wayline( from, to );
    EXPECT_TRUE( wayline.is_past_endpoint( snark::control::wayline::position_t( 4, 5 ) ) );
    EXPECT_TRUE( wayline.is_past_endpoint( snark::control::wayline::position_t( 2, 6 ) ) );
    EXPECT_FALSE( wayline.is_past_endpoint( snark::control::wayline::position_t( 4, 4 ) ) );
    EXPECT_FALSE( wayline.is_past_endpoint( snark::control::wayline::position_t( 1, 6 ) ) );
    EXPECT_FALSE( wayline.is_past_endpoint( snark::control::wayline::position_t( 1, 2 ) ) );
    EXPECT_FALSE( wayline.is_past_endpoint( snark::control::wayline::position_t( 0, 0 ) ) );
}

struct wayline
{
    snark::control::wayline wayline_;
    wayline( double from_x, double from_y, double to_x, double to_y )
        : wayline_( snark::control::wayline(
            snark::control::wayline::position_t( from_x, from_y ),
            snark::control::wayline::position_t( to_x, to_y )
                                                    ) ) {}
    double cross_track_error( double x, double y )
    {
        return wayline_.cross_track_error( snark::control::wayline::position_t( x, y ) );
    }
    double heading_error( double current_heading, double target_heading )
    {
        return wayline_.heading_error( current_heading, target_heading );
    }
};

TEST( wayline, heading_error )
{
    const double target_heading = 0;
    {
        wayline w( 0, 0, 1, 0 );
        EXPECT_NEAR( 0, w.heading_error( 0, target_heading ), eps );
        EXPECT_NEAR( -1, w.heading_error( 1, target_heading ), eps );
        EXPECT_NEAR( 1, w.heading_error( -1, target_heading ), eps );
        EXPECT_NEAR( -3, w.heading_error( 3, target_heading ), eps );
        EXPECT_NEAR( 3, w.heading_error( -3, target_heading ), eps );
        EXPECT_NEAR( -M_PI, w.heading_error( M_PI-eps/10, target_heading ), eps );
        EXPECT_NEAR( M_PI, w.heading_error( -M_PI+eps/10, target_heading ), eps );
    }
    {
        wayline w( 0, 0, 0, 1 );
        EXPECT_NEAR( M_PI/2, w.heading_error( 0, target_heading ), eps );
        EXPECT_NEAR( 1, w.heading_error( M_PI/2-1, target_heading ), eps );
        EXPECT_NEAR( -1, w.heading_error( M_PI/2+1, target_heading ), eps );
        EXPECT_NEAR( M_PI, w.heading_error( -M_PI/2+eps/10, target_heading ), eps );
        EXPECT_NEAR( -M_PI, w.heading_error( -M_PI/2-eps/10, target_heading ), eps );
        EXPECT_NEAR( -M_PI+1, w.heading_error( -M_PI/2-1, target_heading ), eps );
        EXPECT_NEAR( -M_PI/2, w.heading_error( M_PI, target_heading ), eps );
    }
    {
        wayline w( 0, 0, -1, 0 );
        EXPECT_NEAR( M_PI, w.heading_error( eps/10, target_heading ), eps );
        EXPECT_NEAR( M_PI/2, w.heading_error( M_PI/2, target_heading ), eps );
        EXPECT_NEAR( 0, w.heading_error( M_PI, target_heading ), eps );
        EXPECT_NEAR( -M_PI, w.heading_error( -eps/10, target_heading ), eps );
        EXPECT_NEAR( -M_PI/2, w.heading_error( -M_PI/2, target_heading ), eps );
    }
    {
        wayline w( 0, 0, 0, -1 );
        EXPECT_NEAR( -M_PI/2, w.heading_error( 0, target_heading ), eps );
        EXPECT_NEAR( 0, w.heading_error( -M_PI/2, target_heading ), eps );
        EXPECT_NEAR( -M_PI, w.heading_error( M_PI/2-eps/10, target_heading ), eps );
        EXPECT_NEAR( M_PI, w.heading_error( M_PI/2+eps/10, target_heading ), eps );
        EXPECT_NEAR( M_PI/2, w.heading_error( M_PI, target_heading ), eps );
    }
}

TEST( wayline, cross_track_error )
{
    {
        wayline w( 1, 2, 3, 4 );
        EXPECT_NEAR( 1/std::sqrt(2), w.cross_track_error( 2, 2 ), eps );
        EXPECT_NEAR( -1/std::sqrt(2), w.cross_track_error( 1, 3 ), eps );
        EXPECT_NEAR( 1/std::sqrt(2), w.cross_track_error( 0, 0 ), eps );
        EXPECT_NEAR( -1/std::sqrt(2), w.cross_track_error( -1, 1 ), eps );
        EXPECT_NEAR( 1/std::sqrt(2), w.cross_track_error( 4, 4 ), eps );
        EXPECT_NEAR( -1/std::sqrt(2), w.cross_track_error( 3, 5 ), eps );

        EXPECT_NEAR( 0, w.cross_track_error( 0, 1 ), eps );
        EXPECT_NEAR( 0, w.cross_track_error( 1, 2 ), eps );
        EXPECT_NEAR( 0, w.cross_track_error( 2, 3 ), eps );
        EXPECT_NEAR( 0, w.cross_track_error( 3, 4 ), eps );
        EXPECT_NEAR( 0, w.cross_track_error( 4, 5 ), eps );
    }
    {
        wayline w( -1, -2, -3, -4 );
        EXPECT_NEAR( 1/std::sqrt(2), w.cross_track_error( -2, -2 ), eps );
        EXPECT_NEAR( -1/std::sqrt(2), w.cross_track_error( -1, -3 ), eps );
        EXPECT_NEAR( 1/std::sqrt(2), w.cross_track_error( 0, 0 ), eps );
        EXPECT_NEAR( -1/std::sqrt(2), w.cross_track_error( 1, -1 ), eps );
        EXPECT_NEAR( 1/std::sqrt(2), w.cross_track_error( -4, -4 ), eps );
        EXPECT_NEAR( -1/std::sqrt(2), w.cross_track_error( -3, -5 ), eps );

        EXPECT_NEAR( 0, w.cross_track_error( 0, -1 ), eps );
        EXPECT_NEAR( 0, w.cross_track_error( -1, -2 ), eps );
        EXPECT_NEAR( 0, w.cross_track_error( -2, -3 ), eps );
        EXPECT_NEAR( 0, w.cross_track_error( -3, -4 ), eps );
        EXPECT_NEAR( 0, w.cross_track_error( -4, -5 ), eps );
    }
}

} } // namespace snark {  namespace test {

int main( int ac, char** av )
{
    ::testing::InitGoogleTest( &ac, av );
    return RUN_ALL_TESTS();
}
