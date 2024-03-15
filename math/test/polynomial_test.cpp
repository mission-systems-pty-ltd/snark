// Copyright (c) 2024 Vsevolod Vlaskine
// All rights reserved.

#include <gtest/gtest.h>
#include "../polynomial.h"

TEST( math_polynomial, basics )
{
    snark::polynomial< std::int64_t, 2, 2 > p{{ 10, 100, 1000, 10000, 100000, 1000000 }};
    EXPECT_EQ( p( 0, 0 ), 10 );
    EXPECT_EQ( p( 0, 1 ), 10 + 100 + 1000 );
    EXPECT_EQ( p( 0, 2 ), 10 + 100 * 2 + 1000 * 4 );
    EXPECT_EQ( p( 1, 0 ), 10 + 10000 + 1000000 );
    EXPECT_EQ( p( 2, 0 ), 10 + 10000 * 2 + 1000000 * 4 );
    EXPECT_EQ( p( 1, 1 ), 10 + 100 + 1000 + 10000 + 100000 + 1000000 );
    EXPECT_EQ( p( 2, 1 ), 10 + 100 + 1000 + 10000 * 2 + 100000 * 2 + 1000000 * 4 );
    EXPECT_EQ( p( 1, 2 ), 10 + 100 * 2 + 1000 * 4 + 10000 + 100000 * 2 + 1000000 );
    // todo! more tests
}