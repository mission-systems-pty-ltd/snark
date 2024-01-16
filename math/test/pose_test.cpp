// Copyright (c) 2023 Vsevolod Vlaskine
// All rights reserved.

#include <gtest/gtest.h>
#include "../pose.h"

namespace snark {

TEST( math_pose, velocity )
{
    EXPECT_EQ( pose().velocity_from( pose() ), pose() );
    // Eigen::Vector3d a( 0, 1, 2 );
    // Eigen::Vector3d b( 3, 4, 5 );

    // closed_interval< double, 3 > i( a, b );

    // EXPECT_TRUE( i.contains( a ) );
    // EXPECT_TRUE( i.contains( b ) );
    // EXPECT_TRUE( i.contains( 0.5*(a+b) ) );
    // EXPECT_FALSE( i.contains( -a ) );

    // i = i.hull( Eigen::Vector3d ( -1, 2, 3 ) );
    // EXPECT_EQ( i.min(), Eigen::Vector3d( -1, 1, 2 ) );
    // EXPECT_EQ( i.max(), Eigen::Vector3d( 3, 4, 5 ) );

    // i = i.hull( Eigen::Vector3d ( 0, 10, 3 ) );
    // EXPECT_EQ( i.min(), Eigen::Vector3d( -1, 1, 2 ) );
    // EXPECT_EQ( i.max(), Eigen::Vector3d( 3, 10, 5 ) );
}

} // namespace snark {
