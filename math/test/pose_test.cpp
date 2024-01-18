// Copyright (c) 2023 Vsevolod Vlaskine
// All rights reserved.

#include <gtest/gtest.h>
#include "../pose.h"

namespace snark {

static void expect_near( const Eigen::Vector3d& p, const Eigen::Vector3d& q )
{
    EXPECT_NEAR( p.x(), q.x(), 10e-6 );
    EXPECT_NEAR( p.y(), q.y(), 10e-6 );
    EXPECT_NEAR( p.z(), q.z(), 10e-6 );
}

static void expect( const Eigen::Vector3d& expected, const pose& p, const roll_pitch_yaw& v )
{
    expect_near( p.tangent_velocity( v ), expected );
}

static void expect( const pose& expected, const pose& p, const pose& f, const pose& v )
{
    const auto& r = p.velocity_from( f, v );
    expect_near( r.translation, expected.translation );
    EXPECT_NEAR( r.roll(), expected.roll(), 10e-6 );
    EXPECT_NEAR( r.pitch(), expected.pitch(), 10e-6 );
    EXPECT_NEAR( r.yaw(), expected.yaw(), 10e-6 );
}

TEST( math_pose, tangent_velocity )
{
    expect( {0, 0, 0},   {},                       roll_pitch_yaw() );
    expect( {0, 0, 0},   {},                       roll_pitch_yaw(0.1, 0.2, 0.3) );
    expect( {0, 0, 0},   {0, 0, 0, 0.1, 0.2, 0.3}, roll_pitch_yaw(0.1, 0.2, 0.3) );
    expect( {0, 0, 0},   {1, 2, 3, 0.1, 0.2, 0.3}, roll_pitch_yaw() );
    expect( {0, 0, 0},   {10, 0, 0, 0, 0, 0},      roll_pitch_yaw(0.1, 0, 0) );
    expect( {0, 0, 1},   {0, 10, 0, 0, 0, 0},      roll_pitch_yaw(0.1, 0, 0) );
    expect( {0, -1, 0},  {0, 0, 10, 0, 0, 0},      roll_pitch_yaw(0.1, 0, 0) );
    expect( {0, 0, 1},   {10, 10, 0, 0, 0, 0},     roll_pitch_yaw(0.1, 0, 0) );
    expect( {0, -1, 0},  {10, 0, 10, 0, 0, 0},     roll_pitch_yaw(0.1, 0, 0) );
    expect( {0, -2, 1},  {100, 10, 20, 0, 0, 0},   roll_pitch_yaw(0.1, 0, 0) );
    expect( {-2, 1, 0},  {10, 20, 0, 0, 0, 0},     roll_pitch_yaw(0, 0, 0.1) );
    expect( {2, 0, -1},  {10, 100, 20, 0, 0, 0},   roll_pitch_yaw(0, 0.1, 0) );
    // todo: more tests
}

TEST( math_pose, velocity_from_reference_frame )
{
    expect( pose(),                      pose(),                         pose(),                      pose() );
    expect( pose(),                      pose( 1, 2, 3, 0.1, 0.2, 0.3 ), pose(),                      pose() );
    expect( pose(),                      pose( 1, 2, 3, 0.1, 0.2, 0.3 ), pose( 1, 2, 3, 4, 5, 6 ),    pose() );
    expect( pose( 11, 22, 33, 0, 0, 0 ), pose(),                         pose(),                      pose( 11, 22, 33 ) );
    expect( pose( 11, 22, 33, 0, 0, 0 ), pose( 1, 2, 3, 0.1, 0.2, 0.3 ), pose(),                      pose( 11, 22, 33 ) );
    expect( pose( 0, 0, 0, 0.1, 0, 0 ),  pose( 10, 0, 0 ),               pose(),                      pose( 0, 0, 0, 0.1, 0, 0 ) );
    expect( pose( 0, 0, 1, 0.1, 0, 0 ),  pose( 0, 10, 0 ),               pose(),                      pose( 0, 0, 0, 0.1, 0, 0 ) );
    expect( pose( 0, -1, 0, 0.1, 0, 0 ), pose( 0, 0, 10 ),               pose(),                      pose( 0, 0, 0, 0.1, 0, 0 ) );
    expect( pose( 0, 1, 0, 0.1, 0, 0 ),  pose( 0, 0, 10 ),               pose( 0, 0, 0, M_PI, 0, 0 ), pose( 0, 0, 0, 0.1, 0, 0 ) );
    // todo: more tests
}

TEST( geometry_pose, tangent_velocity_high_value )
{
    expect( {0, M_PI * 2, 0},  {0, 0, 1, 0, 0, 0},   roll_pitch_yaw( M_PI * 2, 0, 0 ) );
    expect( {0, M_PI * 15.123, 0},  {0, 0, 1, 0, 0, 0},   roll_pitch_yaw( M_PI * 15.123, 0, 0 ) );
    // todo! WAY more tests!
}

} // namespace snark {
