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

static void expect_true( const pose& p, const roll_pitch_yaw& v, const Eigen::Vector3d& expected )
{
    expect_near( p.tangent_velocity( v ), expected );
}

static void expect_true( const pose& p, const pose& v, const pose& expected )
{
    const auto& r = p.velocity_from( v );
    expect_near( r.translation, expected.translation );
    EXPECT_NEAR( r.roll(), expected.roll(), 10e-6 );
    EXPECT_NEAR( r.pitch(), expected.pitch(), 10e-6 );
    EXPECT_NEAR( r.yaw(), expected.yaw(), 10e-6 );
}

TEST( math_pose, tangent_velocity )
{
    expect_true( {}, roll_pitch_yaw(), {0, 0, 0} );
    expect_true( {}, roll_pitch_yaw(1, 2, 3), {0, 0, 0} );
    expect_true( {0, 0, 0, 0.1, 0.2, 0.3}, roll_pitch_yaw(1, 2, 3), {0, 0, 0} );
    expect_true( {1, 2, 3, 0.1, 0.2, 0.3}, roll_pitch_yaw(), {0, 0, 0} );
    expect_true( {10, 0, 0, 0, 0, 0}, roll_pitch_yaw(), {0, 0, 0} );
    expect_true( {10, 0, 0, 0, 0, 0}, roll_pitch_yaw(0.1, 0, 0), {0, 0, 0} );
    expect_true( {0, 10, 0, 0, 0, 0}, roll_pitch_yaw(0.1, 0, 0), {0, 0, 1} );
    expect_true( {0, 0, 10, 0, 0, 0}, roll_pitch_yaw(0.1, 0, 0), {0, -1, 0} );
}

TEST( math_pose, velocity_from_reference_frame )
{
    expect_true( pose(), pose(), pose() );
    expect_true( pose( 1, 2, 3, 0.1, 0.2, 0.3 ), pose(), pose() );
    expect_true( pose(), pose( 11, 22, 33 ), pose( 11, 22, 33, 0, 0, 0 ) );
    expect_true( pose( 1, 2, 3, 0.1, 0.2, 0.3 ), pose( 11, 22, 33 ), pose( 11, 22, 33, 0, 0, 0 ) );
    // todo: confirm tangent direction is correct
    // expect_true( pose( 10, 0, 0 ), pose( 0, 0, 0, 0.1, 0, 0 ), pose( 0, 0, 0, 0.1, 0, 0 ) );
    expect_true( pose( 0, 10, 0 ), pose( 0, 0, 0, 0.1, 0, 0 ), pose( 0, 0, 1, 0.1, 0, 0 ) );
    // expect_true( pose( 0, 0, 10 ), pose( 0, 0, 0, 0.1, 0, 0 ), pose( 0, 0, 1, 0.1, 0, 0 ) );
    // todo: confirm chained frame velocities produce correct velocity
}

} // namespace snark {
