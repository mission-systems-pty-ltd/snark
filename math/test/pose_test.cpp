// Copyright (c) 2023 Vsevolod Vlaskine
// All rights reserved.

#include <gtest/gtest.h>
#include "../pose.h"

namespace snark {

static void expect_true( const pose& p, const pose& v, const pose& expected )
{
    const auto& r = p.velocity_from( v );
    EXPECT_NEAR( r.x(), expected.x(), 0.001 );
    EXPECT_NEAR( r.y(), expected.y(), 0.001 );
    EXPECT_NEAR( r.z(), expected.z(), 0.001 );
    EXPECT_NEAR( r.roll(), expected.roll(), 0.001 );
    EXPECT_NEAR( r.pitch(), expected.pitch(), 0.001 );
    EXPECT_NEAR( r.yaw(), expected.yaw(), 0.001 );
}

TEST( math_pose, velocity )
{
    expect_true( pose(), pose(), pose() );
    expect_true( pose( 1, 2, 3, 0.1, 0.2, 0.3 ), pose(), pose() );
    expect_true( pose(), pose( 11, 22, 33 ), pose( 11, 22, 33, 0, 0, 0 ) );
    expect_true( pose( 1, 2, 3, 0.1, 0.2, 0.3 ), pose( 11, 22, 33 ), pose( 11, 22, 33, 0, 0, 0 ) );
    // todo: confirm tangent direction is correct
    expect_true( pose( 0, 10, 0 ), pose( 0, 0, 0, 0.1, 0, 0 ), pose( 0, 0, 1, 0.1, 0, 0 ) );
    // todo: confirm chained frame velocities produce correct velocity
}

} // namespace snark {
