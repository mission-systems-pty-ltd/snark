#include <gtest/gtest.h>
#include <boost/optional.hpp>
#include "../wheel_command.h"

using namespace snark::wheels;

class wheels_test : public ::testing::Test
{
    protected:
        virtual void SetUp()
        {
            rear_left_pose
                << -1,  0,  0, -1,
                    0,  0, -1,  1,
                    0, -1,  0,  0,
                    0,  0,  0,  1;
            front_left_pose
                << -1,  0,  0,  1,
                    0,  0, -1,  1,
                    0, -1,  0,  0,
                    0,  0,  0,  1;
            front_right_pose
                <<  1,  0,  0,  1,
                    0,  0,  1, -1,
                    0, -1,  0,  0,
                    0,  0,  0,  1;
            rear_right_pose
                <<  1,  0,  0, -1,
                    0,  0,  1, -1,
                    0, -1,  0,  0,
                    0,  0,  0,  1;
            wheel_offset = 0;
            angle_limit = limit( M_PI*0.5 );
        }

        virtual void TearDown()
        {
        }

    public:
        Eigen::Matrix4d rear_left_pose;
        Eigen::Matrix4d front_left_pose;
        Eigen::Matrix4d rear_right_pose;
        Eigen::Matrix4d front_right_pose;
        double wheel_offset;
        boost::optional< limit > angle_limit;
};

class wheels_test_90_reverse : public wheels_test
{
    protected:
        virtual void SetUp()
        {
            wheels_test::SetUp();
            angle_limit = limit( -M_PI / 3 );
        }
};

class wheels_test_180 : public wheels_test
{
    protected:
        virtual void SetUp()
        {
            wheels_test::SetUp();
            angle_limit = limit( M_PI );
        }
};

class wheels_test_540 : public wheels_test
{
    protected:
        virtual void SetUp()
        {
            wheels_test::SetUp();
            angle_limit = limit( M_PI*3 );
        }
};

class wheels_test_720 : public wheels_test
{
    protected:
        virtual void SetUp()
        {
            wheels_test::SetUp();
            angle_limit = limit( M_PI*4 );
        }
};

class wheels_test_no_limit : public wheels_test
{
    protected:
        virtual void SetUp()
        {
            wheels_test::SetUp();
            angle_limit = boost::optional< limit >();
        }
};

TEST_F( wheels_test, crab_north_west )
{
    steer_command steer( 1, 1, 0 );
    wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit );
    wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit );
    wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit );
    wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit );
    EXPECT_NEAR(  M_PI*0.25, rear_left.turnrate, 1e-9 );
    EXPECT_NEAR( -std::sqrt(2), rear_left.velocity, 1e-9 );
    EXPECT_NEAR(  M_PI*0.25, front_left.turnrate, 1e-9 );
    EXPECT_NEAR( -std::sqrt(2), front_left.velocity, 1e-9 );
    EXPECT_NEAR(  M_PI*0.25, front_right.turnrate, 1e-9 );
    EXPECT_NEAR(  std::sqrt(2), front_right.velocity, 1e-9 );
    EXPECT_NEAR(  M_PI*0.25, rear_right.turnrate, 1e-9 );
    EXPECT_NEAR(  std::sqrt(2), rear_right.velocity, 1e-9 );
}

TEST_F( wheels_test, crab_north_wrap )
{
    steer_command steer( 1, 0, 0 );
    boost::optional< double > current_angle( M_PI );
    wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle, true );
    wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle, true );
    wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle, true );
    wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle,true );
    EXPECT_NEAR( 0, rear_left.turnrate, 1e-9 );
    EXPECT_NEAR( -1, rear_left.velocity, 1e-9 );
    EXPECT_NEAR( 0, front_left.turnrate, 1e-9 );
    EXPECT_NEAR( -1, front_left.velocity, 1e-9 );
    EXPECT_NEAR( 0, front_right.turnrate, 1e-9 );
    EXPECT_NEAR( 1, front_right.velocity, 1e-9 );
    EXPECT_NEAR( 0, rear_right.turnrate, 1e-9 );
    EXPECT_NEAR( 1, rear_right.velocity, 1e-9 );
}

TEST_F( wheels_test, crab_south_wrap )
{
    steer_command steer( -1, 0, 0 );
    boost::optional< double > current_angle( 0 );
    wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle, true );
    wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle, true );
    wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle, true );
    wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle,true );
    EXPECT_NEAR( 0, rear_left.turnrate, 1e-9 );
    EXPECT_NEAR( 1, rear_left.velocity, 1e-9 );
    EXPECT_NEAR( 0, front_left.turnrate, 1e-9 );
    EXPECT_NEAR( 1, front_left.velocity, 1e-9 );
    EXPECT_NEAR( 0, front_right.turnrate, 1e-9 );
    EXPECT_NEAR( -1, front_right.velocity, 1e-9 );
    EXPECT_NEAR( 0, rear_right.turnrate, 1e-9 );
    EXPECT_NEAR( -1, rear_right.velocity, 1e-9 );
}

TEST_F( wheels_test_90_reverse, crab_north_wrap )
{
    steer_command steer( 1, 0, 0 );
    boost::optional< double > current_angle( M_PI );
    wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle, true );
    wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle, true );
    wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle, true );
    wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle,true );
    EXPECT_NEAR( M_PI, rear_left.turnrate, 1e-9 );
    EXPECT_NEAR( 1, rear_left.velocity, 1e-9 );
    EXPECT_NEAR( M_PI, front_left.turnrate, 1e-9 );
    EXPECT_NEAR( 1, front_left.velocity, 1e-9 );
    EXPECT_NEAR( M_PI, front_right.turnrate, 1e-9 );
    EXPECT_NEAR( -1, front_right.velocity, 1e-9 );
    EXPECT_NEAR( M_PI, rear_right.turnrate, 1e-9 );
    EXPECT_NEAR( -1, rear_right.velocity, 1e-9 );
}

TEST_F( wheels_test_90_reverse, crab_south_wrap )
{
    steer_command steer( -1, 0, 0 );
    boost::optional< double > current_angle( 0 );
    wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle, true );
    wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle, true );
    wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle, true );
    wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle,true );
    EXPECT_NEAR( M_PI, rear_left.turnrate, 1e-9 );
    EXPECT_NEAR( -1, rear_left.velocity, 1e-9 );
    EXPECT_NEAR( M_PI, front_left.turnrate, 1e-9 );
    EXPECT_NEAR( -1, front_left.velocity, 1e-9 );
    EXPECT_NEAR( M_PI, front_right.turnrate, 1e-9 );
    EXPECT_NEAR( 1, front_right.velocity, 1e-9 );
    EXPECT_NEAR( M_PI, rear_right.turnrate, 1e-9 );
    EXPECT_NEAR( 1, rear_right.velocity, 1e-9 );
}

TEST_F( wheels_test, crab_north )
{
    steer_command steer( 1, 0, 0 );
    wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit );
    wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit );
    wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit );
    wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit );
    EXPECT_EQ(  0, rear_left.turnrate );
    EXPECT_EQ( -1, rear_left.velocity );
    EXPECT_EQ(  0, front_left.turnrate );
    EXPECT_EQ( -1, front_left.velocity );
    EXPECT_EQ(  0, front_right.turnrate );
    EXPECT_EQ(  1, front_right.velocity );
    EXPECT_EQ(  0, rear_right.turnrate );
    EXPECT_EQ(  1, rear_right.velocity );
}

TEST_F( wheels_test, crab_north_east )
{
    steer_command steer( 1, -1, 0 );
    wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit );
    wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit );
    wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit );
    wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit );
    EXPECT_NEAR( -M_PI*0.25, rear_left.turnrate, 1e-9 );
    EXPECT_NEAR( -std::sqrt(2), rear_left.velocity, 1e-9 );
    EXPECT_NEAR( -M_PI*0.25, front_left.turnrate, 1e-9 );
    EXPECT_NEAR( -std::sqrt(2), front_left.velocity, 1e-9 );
    EXPECT_NEAR( -M_PI*0.25, front_right.turnrate, 1e-9 );
    EXPECT_NEAR(  std::sqrt(2), front_right.velocity, 1e-9 );
    EXPECT_NEAR( -M_PI*0.25, rear_right.turnrate, 1e-9 );
    EXPECT_NEAR(  std::sqrt(2), rear_right.velocity, 1e-9 );
}

TEST_F( wheels_test, crab_east )
{
    steer_command steer( 0, -1, 0 );
    wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit );
    wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit );
    wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit );
    wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit );
    EXPECT_NEAR(  M_PI*0.5, rear_left.turnrate, 1e-9 );
    EXPECT_NEAR(  1, rear_left.velocity, 1e-9 );
    EXPECT_NEAR(  M_PI*0.5, front_left.turnrate, 1e-9 );
    EXPECT_NEAR(  1, front_left.velocity, 1e-9 );
    EXPECT_NEAR( -M_PI*0.5, front_right.turnrate, 1e-9 );
    EXPECT_NEAR(  1, front_right.velocity, 1e-9 );
    EXPECT_NEAR( -M_PI*0.5, rear_right.turnrate, 1e-9 );
    EXPECT_NEAR(  1, rear_right.velocity, 1e-9 );
}

TEST_F( wheels_test, crab_south_east )
{
    steer_command steer( -1, -1, 0 );
    wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit );
    wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit );
    wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit );
    wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit );
    EXPECT_NEAR(  M_PI*0.25, rear_left.turnrate, 1e-9 );
    EXPECT_NEAR(  std::sqrt(2), rear_left.velocity, 1e-9 );
    EXPECT_NEAR(  M_PI*0.25, front_left.turnrate, 1e-9 );
    EXPECT_NEAR(  std::sqrt(2), front_left.velocity, 1e-9 );
    EXPECT_NEAR(  M_PI*0.25, front_right.turnrate, 1e-9 );
    EXPECT_NEAR( -std::sqrt(2), front_right.velocity, 1e-9 );
    EXPECT_NEAR(  M_PI*0.25, rear_right.turnrate, 1e-9 );
    EXPECT_NEAR( -std::sqrt(2), rear_right.velocity, 1e-9 );
}

TEST_F( wheels_test, crab_south )
{
    steer_command steer( -1, 0, 0 );
    wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit );
    wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit );
    wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit );
    wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit );
    EXPECT_EQ(  0, rear_left.turnrate );
    EXPECT_EQ(  1, rear_left.velocity );
    EXPECT_EQ(  0, front_left.turnrate );
    EXPECT_EQ(  1, front_left.velocity );
    EXPECT_EQ(  0, front_right.turnrate );
    EXPECT_EQ( -1, front_right.velocity );
    EXPECT_EQ(  0, rear_right.turnrate );
    EXPECT_EQ( -1, rear_right.velocity );
}

TEST_F( wheels_test, crab_south_west )
{
    steer_command steer( -1,  1, 0 );
    wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit );
    wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit );
    wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit );
    wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit );
    EXPECT_NEAR( -M_PI*0.25, rear_left.turnrate, 1e-9 );
    EXPECT_NEAR(  std::sqrt(2), rear_left.velocity, 1e-9 );
    EXPECT_NEAR( -M_PI*0.25, front_left.turnrate, 1e-9 );
    EXPECT_NEAR(  std::sqrt(2), front_left.velocity, 1e-9 );
    EXPECT_NEAR( -M_PI*0.25, front_right.turnrate, 1e-9 );
    EXPECT_NEAR( -std::sqrt(2), front_right.velocity, 1e-9 );
    EXPECT_NEAR( -M_PI*0.25, rear_right.turnrate, 1e-9 );
    EXPECT_NEAR( -std::sqrt(2), rear_right.velocity, 1e-9 );
}

TEST_F( wheels_test, crab_west )
{
    steer_command steer( 0, 1, 0 );
    wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit );
    wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit );
    wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit );
    wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit );
    EXPECT_NEAR( -M_PI*0.5, rear_left.turnrate, 1e-9 );
    EXPECT_NEAR(  1, rear_left.velocity, 1e-9 );
    EXPECT_NEAR( -M_PI*0.5, front_left.turnrate, 1e-9 );
    EXPECT_NEAR(  1, front_left.velocity, 1e-9 );
    EXPECT_NEAR(  M_PI*0.5, front_right.turnrate, 1e-9 );
    EXPECT_NEAR(  1, front_right.velocity, 1e-9 );
    EXPECT_NEAR(  M_PI*0.5, rear_right.turnrate, 1e-9 );
    EXPECT_NEAR(  1, rear_right.velocity, 1e-9 );
}

TEST_F( wheels_test, spot_turn )
{
    steer_command steer( 0, 0, 1 );
    wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit );
    wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit );
    wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit );
    wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit );
    EXPECT_NEAR(  M_PI*0.25, rear_left.turnrate, 1e-9 );
    EXPECT_NEAR(  std::sqrt(2), rear_left.velocity, 1e-9 );
    EXPECT_NEAR( -M_PI*0.25, front_left.turnrate, 1e-9 );
    EXPECT_NEAR(  std::sqrt(2), front_left.velocity, 1e-9 );
    EXPECT_NEAR(  M_PI*0.25, front_right.turnrate, 1e-9 );
    EXPECT_NEAR(  std::sqrt(2), front_right.velocity, 1e-9 );
    EXPECT_NEAR( -M_PI*0.25, rear_right.turnrate, 1e-9 );
    EXPECT_NEAR(  std::sqrt(2), rear_right.velocity, 1e-9 );
}

TEST_F( wheels_test_180, crab_nearest_north )
{
    steer_command steer( 1, 0, 0 );
    {
        boost::optional< double > current_angle( 0 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  0, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  0, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  0, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_right.velocity, 1e-9 );
        EXPECT_NEAR(  0, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_right.velocity, 1e-9 );
    }
}

TEST_F( wheels_test_180, crab_nearest_south )
{
    steer_command steer( -1, 0, 0 );
    {
        boost::optional< double > current_angle( 0 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  0, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  0, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  0, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_right.velocity, 1e-9 );
        EXPECT_NEAR(  0, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_right.velocity, 1e-9 );
    }
}

TEST_F( wheels_test_180, crab_nearest_east )
{
    steer_command steer( 0, -1, 0 );
    {
        boost::optional< double > current_angle( 0 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*0.5, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.5, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.5, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.5, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI*0.25 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*0.5, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.5, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.5, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.5, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_right.velocity, 1e-9 );
    }
}

TEST_F( wheels_test_180, crab_nearest_west )
{
    steer_command steer( 0, 1, 0 );
    {
        boost::optional< double > current_angle( 0 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*0.5, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.5, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.5, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.5, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( -M_PI*0.25 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR( -M_PI*0.5, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.5, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.5, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.5, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_right.velocity, 1e-9 );
    }
}

TEST_F( wheels_test_180, crab_nearest_south_west )
{
    steer_command steer( -1, 1, 0 );
    {
        boost::optional< double > current_angle( 0 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR( -M_PI*0.25, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*0.75, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.75, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.75, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.75, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( -M_PI );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR( -M_PI*0.25, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_right.velocity, 1e-9 );
    }
}

TEST_F( wheels_test_180, spot_turn )
{
    steer_command steer( 0, 0, 1 );
    {
        boost::optional< double > current_angle( 0 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*0.25, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.25, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI*0.5 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*0.25, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.75, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.25, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.75, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( -M_PI*0.5 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR( -M_PI*0.75, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.75, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_right.velocity, 1e-9 );
    }
}

TEST_F( wheels_test_720, crab_nearest_north )
{
    steer_command steer( 1, 0, 0 );
    {
        boost::optional< double > current_angle( 350/180.0*M_PI );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*2, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_right.velocity, 1e-9 );
    }
}

TEST_F( wheels_test_540, crab_nearest_north )
{
    steer_command steer( 1, 0, 0 );
    {
        boost::optional< double > current_angle( 0 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  0, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  0, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  0, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_right.velocity, 1e-9 );
        EXPECT_NEAR(  0, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI*2 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*2, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI*3 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*3, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*3, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*3, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*3, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_right.velocity, 1e-9 );
    }
}

TEST_F( wheels_test_540, crab_nearest_south )
{
    steer_command steer( -1, 0, 0 );
    {
        boost::optional< double > current_angle( 0 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  0, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  0, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  0, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_right.velocity, 1e-9 );
        EXPECT_NEAR(  0, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI*2 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*2, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI*3 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*3, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*3, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*3, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*3, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_right.velocity, 1e-9 );
    }
}

TEST_F( wheels_test_540, crab_nearest_east )
{
    steer_command steer( 0, -1, 0 );
    {
        boost::optional< double > current_angle( 0 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*0.5, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.5, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.5, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.5, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI*0.25 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*0.5, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.5, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.5, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.5, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI*2 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*2.5, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*1.5, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2.5, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*1.5, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI*3 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*2.5, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2.5, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2.5, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2.5, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_right.velocity, 1e-9 );
    }
}

TEST_F( wheels_test_540, crab_nearest_west )
{
    steer_command steer( 0, 1, 0 );
    {
        boost::optional< double > current_angle( 0 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*0.5, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.5, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.5, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.5, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( -M_PI*0.25 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR( -M_PI*0.5, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.5, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.5, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.5, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI*2 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*2.5, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*1.5, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2.5, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*1.5, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI*3 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*2.5, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2.5, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -1, front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2.5, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2.5, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  1, rear_right.velocity, 1e-9 );
    }
}

TEST_F( wheels_test_540, crab_nearest_south_west )
{
    steer_command steer( -1, 1, 0 );
    {
        boost::optional< double > current_angle( 0 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR( -M_PI*0.25, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*0.75, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.75, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.75, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.75, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( -M_PI );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR( -M_PI*1.25, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*1.25, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*1.25, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*1.25, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI*2 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*1.75, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*1.75, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*1.75, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*1.75, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( -M_PI*2 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR( -M_PI*2.25, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*2.25, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*2.25, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*2.25, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI*3 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*2.75, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2.75, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2.75, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2.75, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( -M_PI*3 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR( -M_PI*2.25, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*2.25, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*2.25, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*2.25, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_right.velocity, 1e-9 );
    }
}

TEST_F( wheels_test_540, spot_turn )
{
    steer_command steer( 0, 0, 1 );
    {
        boost::optional< double > current_angle( 0 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*0.25, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.25, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI*0.5 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*0.25, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.75, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.25, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.75, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( -M_PI*0.5 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR( -M_PI*0.75, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.75, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI*2 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*2.25, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*1.75, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2.25, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*1.75, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( -M_PI*2 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR( -M_PI*1.75, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*2.25, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*1.75, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*2.25, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI*3 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*2.25, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2.75, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2.25, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*2.75, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( -M_PI*3 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR( -M_PI*2.75, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*2.25, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*2.75, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*2.25, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_right.velocity, 1e-9 );
    }
}

TEST_F( wheels_test_no_limit, crab_north )
{
    steer_command steer( 1, 0, 0 );
    boost::optional< double > current_angle;
    wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
    wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
    wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
    wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
    EXPECT_NEAR(  0, rear_left.turnrate, 1e-9 );
    EXPECT_NEAR( -1, rear_left.velocity, 1e-9 );
    EXPECT_NEAR(  0, front_left.turnrate, 1e-9 );
    EXPECT_NEAR( -1, front_left.velocity, 1e-9 );
    EXPECT_NEAR(  0, front_right.turnrate, 1e-9 );
    EXPECT_NEAR(  1, front_right.velocity, 1e-9 );
    EXPECT_NEAR(  0, rear_right.turnrate, 1e-9 );
    EXPECT_NEAR(  1, rear_right.velocity, 1e-9 );
}

TEST_F( wheels_test_no_limit, crab_nearest_south_west )
{
    steer_command steer( -1, 1, 0 );
    {
        boost::optional< double > current_angle( 0 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR( -M_PI*0.25, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*0.75, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.75, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.75, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.75, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( -M_PI );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR( -M_PI*1.25, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*1.25, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*1.25, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*1.25, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_right.velocity, 1e-9 );
    }
}

TEST_F( wheels_test_no_limit, spot_turn_nearest )
{
    steer_command steer( 0, 0, 1 );
    {
        boost::optional< double > current_angle( 0 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*0.25, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.25, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR(  M_PI*1.25, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.75, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*1.25, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.75, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( -M_PI );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle );
        EXPECT_NEAR( -M_PI*0.75, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*1.25, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.75, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*1.25, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_right.velocity, 1e-9 );
    }
}

TEST_F( wheels_test_no_limit, wrap_crab_nearest_north )
{
    steer_command steer( 1, 0, 0 );
    bool wrap = true;
    boost::optional< double > current_angle;
    wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, M_PI * -0.25, wrap );
    wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, M_PI * -0.75, wrap );
    wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, M_PI * 0.25, wrap );
    wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, M_PI * 0.75, wrap );
    EXPECT_NEAR(  0, rear_left.turnrate, 1e-9 );
    EXPECT_NEAR( -1, rear_left.velocity, 1e-9 );
    EXPECT_NEAR(  M_PI*-1, front_left.turnrate, 1e-9 );
    EXPECT_NEAR(  1, front_left.velocity, 1e-9 );
    EXPECT_NEAR(  0, front_right.turnrate, 1e-9 );
    EXPECT_NEAR(  1, front_right.velocity, 1e-9 );
    EXPECT_NEAR(  M_PI, rear_right.turnrate, 1e-9 );
    EXPECT_NEAR( -1, rear_right.velocity, 1e-9 );
}

TEST_F( wheels_test_no_limit, wrap_spot_turn_nearest )
{
    steer_command steer( 0, 0, 1 );
    bool wrap = true;
    {
        boost::optional< double > current_angle( 0 );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle, wrap );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle, wrap );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle, wrap );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle, wrap );
        EXPECT_NEAR(  M_PI*0.25, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, front_left.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.25, front_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.25, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR(  std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( M_PI );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle, wrap );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle, wrap );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle, wrap );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle, wrap );
        EXPECT_NEAR( -M_PI*0.75, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.75, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.75, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.75, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_right.velocity, 1e-9 );
    }
    {
        boost::optional< double > current_angle( -M_PI );
        wheel_command rear_left = compute_wheel_command( steer, rear_left_pose, wheel_offset, angle_limit, current_angle, wrap );
        wheel_command front_left = compute_wheel_command( steer, front_left_pose, wheel_offset, angle_limit, current_angle, wrap );
        wheel_command front_right = compute_wheel_command( steer, front_right_pose, wheel_offset, angle_limit, current_angle, wrap );
        wheel_command rear_right = compute_wheel_command( steer, rear_right_pose, wheel_offset, angle_limit, current_angle, wrap );
        EXPECT_NEAR( -M_PI*0.75, rear_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_left.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.75, front_left.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_left.velocity, 1e-9 );
        EXPECT_NEAR( -M_PI*0.75, front_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), front_right.velocity, 1e-9 );
        EXPECT_NEAR(  M_PI*0.75, rear_right.turnrate, 1e-9 );
        EXPECT_NEAR( -std::sqrt(2), rear_right.velocity, 1e-9 );
    }
}
