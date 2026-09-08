// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "manipulation_position_controllers/joint_space/kinematics/joint_reference.hpp"
#include "manipulation_position_controllers/joint_space/kinematics/reference_message_reader.hpp"

namespace mpc = manipulation_position_controllers::joint_space;

namespace
{

trajectory_msgs::msg::JointTrajectoryPoint make_point(
  const std::vector<double> & positions,
  double time_from_start_s,
  const std::vector<double> & velocities = {})
{
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions;
  point.velocities = velocities;
  point.time_from_start = rclcpp::Duration::from_seconds(time_from_start_s);
  return point;
}

// Fixture provides a clock and reusable joint-name / limit vectors so each
// test only declares the message and the config flags it cares about.
class ReferenceReaderTest : public ::testing::Test
{
protected:
  rclcpp::Clock clock_{RCL_SYSTEM_TIME};
  std::vector<std::string> joints_{"j1", "j2", "j3"};
  std::vector<double> lower_{-10.0, -10.0, -10.0};
  std::vector<double> upper_{10.0, 10.0, 10.0};

  mpc::ReferenceMessageReaderConfig make_config(
    bool reject_out_of_bounds = false,
    bool reject_zero_stamped = true,
    bool allow_partial = false)
  {
    return mpc::ReferenceMessageReaderConfig{
      joints_, lower_, upper_, 32,
      reject_out_of_bounds, reject_zero_stamped, allow_partial, clock_};
  }
};

}  // namespace

TEST_F(ReferenceReaderTest, EmptyJointNamesRejected)
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.points.push_back(make_point({0.0, 0.0, 0.0}, 0.0));
  auto config = make_config();
  mpc::JointReference reference;
  EXPECT_FALSE(mpc::read_reference_message(msg, config, reference));
}

TEST_F(ReferenceReaderTest, EmptyPointsRejected)
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.joint_names = joints_;
  auto config = make_config();
  mpc::JointReference reference;
  EXPECT_FALSE(mpc::read_reference_message(msg, config, reference));
}

TEST_F(ReferenceReaderTest, ValidMessageParsed)
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp.sec = 100;
  msg.joint_names = joints_;
  msg.points.push_back(make_point({0.1, 0.2, 0.3}, 0.0));
  auto config = make_config();

  mpc::JointReference reference;
  ASSERT_TRUE(mpc::read_reference_message(msg, config, reference));
  EXPECT_EQ(reference.size, 1u);
  EXPECT_FALSE(reference.used_zero_stamp_fallback);
  EXPECT_DOUBLE_EQ(reference.positions[0], 0.1);
  EXPECT_DOUBLE_EQ(reference.positions[1], 0.2);
  EXPECT_DOUBLE_EQ(reference.positions[2], 0.3);
}

TEST_F(ReferenceReaderTest, JointOrderRemappedToExpected)
{
  // Message lists joints in a different order than the controller expects.
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp.sec = 100;
  msg.joint_names = {"j3", "j1", "j2"};
  msg.points.push_back(make_point({0.3, 0.1, 0.2}, 0.0));  // j3, j1, j2
  auto config = make_config();

  mpc::JointReference reference;
  ASSERT_TRUE(mpc::read_reference_message(msg, config, reference));
  // Reordered into expected order j1, j2, j3.
  EXPECT_DOUBLE_EQ(reference.positions[0], 0.1);
  EXPECT_DOUBLE_EQ(reference.positions[1], 0.2);
  EXPECT_DOUBLE_EQ(reference.positions[2], 0.3);
}

TEST_F(ReferenceReaderTest, PartialReferenceFillsMissingWithNaN)
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp.sec = 100;
  msg.joint_names = {"j1", "j3"};  // j2 absent
  msg.points.push_back(make_point({0.1, 0.3}, 0.0));
  auto config = make_config(false, false, /*allow_partial=*/true);

  mpc::JointReference reference;
  ASSERT_TRUE(mpc::read_reference_message(msg, config, reference));
  EXPECT_DOUBLE_EQ(reference.positions[0], 0.1);
  EXPECT_TRUE(std::isnan(reference.positions[1]));  // j2 missing → NaN sentinel
  EXPECT_DOUBLE_EQ(reference.positions[2], 0.3);
}

TEST_F(ReferenceReaderTest, PartialReferenceRejectedWhenStrict)
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp.sec = 100;
  msg.joint_names = {"j1", "j3"};  // j2 absent
  msg.points.push_back(make_point({0.1, 0.3}, 0.0));
  auto config = make_config(false, false, /*allow_partial=*/false);

  mpc::JointReference reference;
  EXPECT_FALSE(mpc::read_reference_message(msg, config, reference));
}

TEST_F(ReferenceReaderTest, ZeroStampRejectedByDefault)
{
  trajectory_msgs::msg::JointTrajectory msg;
  // header.stamp left at zero
  msg.joint_names = joints_;
  msg.points.push_back(make_point({0.0, 0.0, 0.0}, 0.0));
  auto config = make_config();

  mpc::JointReference reference;
  EXPECT_FALSE(mpc::read_reference_message(msg, config, reference));
  EXPECT_EQ(
    mpc::read_reference_message_result(msg, config, reference),
    mpc::ReferenceReadResult::kZeroStamp);
}

TEST_F(ReferenceReaderTest, ZeroStampFallsBackToClock)
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.joint_names = joints_;
  msg.points.push_back(make_point({0.0, 0.0, 0.0}, 0.0));
  auto config = make_config(false, /*reject_zero_stamped=*/false, false);

  mpc::JointReference reference;
  ASSERT_TRUE(mpc::read_reference_message(msg, config, reference));
  EXPECT_TRUE(reference.used_zero_stamp_fallback);
}

TEST_F(ReferenceReaderTest, OutOfBoundsClampedByDefault)
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp.sec = 100;
  msg.joint_names = joints_;
  msg.points.push_back(make_point({99.0, -99.0, 0.0}, 0.0));
  auto config = make_config(/*reject_out_of_bounds=*/false, false, false);

  mpc::JointReference reference;
  ASSERT_TRUE(mpc::read_reference_message(msg, config, reference));
  EXPECT_DOUBLE_EQ(reference.positions[0], upper_[0]);   // clamped to +10
  EXPECT_DOUBLE_EQ(reference.positions[1], lower_[1]);   // clamped to -10
  EXPECT_DOUBLE_EQ(reference.positions[2], 0.0);
}

TEST_F(ReferenceReaderTest, OutOfBoundsRejectedWhenConfigured)
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp.sec = 100;
  msg.joint_names = joints_;
  msg.points.push_back(make_point({99.0, 0.0, 0.0}, 0.0));
  auto config = make_config(/*reject_out_of_bounds=*/true, false, false);

  mpc::JointReference reference;
  EXPECT_EQ(
    mpc::read_reference_message_result(msg, config, reference),
    mpc::ReferenceReadResult::kOutOfBounds);
}

TEST_F(ReferenceReaderTest, NonfiniteReferenceReportsSpecificCause)
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp.sec = 100;
  msg.joint_names = joints_;
  msg.points.push_back(
    make_point({0.0, std::numeric_limits<double>::quiet_NaN(), 0.0}, 0.0));
  auto config = make_config();

  mpc::JointReference reference;
  EXPECT_EQ(
    mpc::read_reference_message_result(msg, config, reference),
    mpc::ReferenceReadResult::kNonfinite);
}

TEST_F(ReferenceReaderTest, NonMonotonicTimeRejected)
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp.sec = 100;
  msg.joint_names = joints_;
  msg.points.push_back(make_point({0.0, 0.0, 0.0}, 1.0));
  msg.points.push_back(make_point({1.0, 1.0, 1.0}, 0.5));  // goes backwards
  auto config = make_config();

  mpc::JointReference reference;
  EXPECT_FALSE(mpc::read_reference_message(msg, config, reference));
}

TEST_F(ReferenceReaderTest, PointWithTooFewPositionsRejected)
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp.sec = 100;
  msg.joint_names = joints_;
  msg.points.push_back(make_point({0.1, 0.2}, 0.0));  // only 2 of 3
  auto config = make_config();

  mpc::JointReference reference;
  EXPECT_FALSE(mpc::read_reference_message(msg, config, reference));
}

TEST_F(ReferenceReaderTest, MissingVelocitiesFilledByFiniteDifference)
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp.sec = 100;
  msg.joint_names = joints_;
  msg.points.push_back(make_point({0.0, 0.0, 0.0}, 0.0));
  msg.points.push_back(make_point({2.0, 4.0, 6.0}, 1.0));
  auto config = make_config();

  mpc::JointReference reference;
  ASSERT_TRUE(mpc::read_reference_message(msg, config, reference));
  EXPECT_TRUE(reference.has_velocities);
  // Forward difference over dt = 1 s for the first point.
  EXPECT_DOUBLE_EQ(reference.velocities[0], 2.0);
  EXPECT_DOUBLE_EQ(reference.velocities[1], 4.0);
  EXPECT_DOUBLE_EQ(reference.velocities[2], 6.0);
}

TEST_F(ReferenceReaderTest, SinglePointMissingVelocitiesAreZero)
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp.sec = 100;
  msg.joint_names = joints_;
  msg.points.push_back(make_point({0.1, 0.2, 0.3}, 0.0));
  auto config = make_config();

  mpc::JointReference reference;
  ASSERT_TRUE(mpc::read_reference_message(msg, config, reference));
  EXPECT_TRUE(reference.has_velocities);
  EXPECT_DOUBLE_EQ(reference.velocities[0], 0.0);
  EXPECT_DOUBLE_EQ(reference.velocities[1], 0.0);
  EXPECT_DOUBLE_EQ(reference.velocities[2], 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
