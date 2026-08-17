// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <vector>

#include "manipulation_position_controllers/joint_space/kinematics/joint_reference.hpp"
#include "manipulation_position_controllers/joint_space/kinematics/trajectory_sampler.hpp"

namespace mpc = manipulation_position_controllers::joint_space;

namespace
{

// Write one point's joint positions (and optionally velocities) into the
// fixed-capacity JointReference storage.
void set_point(
  mpc::JointReference & reference,
  size_t point_index,
  double time_s,
  const std::vector<double> & positions,
  const std::vector<double> & velocities = {})
{
  reference.times[point_index] = time_s;
  for (size_t joint = 0; joint < positions.size(); ++joint) {
    reference.positions[point_index * mpc::kMaxJoints + joint] = positions[joint];
  }
  if (!velocities.empty()) {
    for (size_t joint = 0; joint < velocities.size(); ++joint) {
      reference.velocities[point_index * mpc::kMaxJoints + joint] = velocities[joint];
    }
  }
}

}  // namespace

TEST(TrajectorySampler, ZeroSizeReturnsFalse)
{
  mpc::JointReference reference;
  std::vector<double> target(2, 0.0);
  EXPECT_FALSE(mpc::sample_reference(reference, 0, 2, 1.0, target));
}

TEST(TrajectorySampler, TargetTooSmallReturnsFalse)
{
  mpc::JointReference reference;
  set_point(reference, 0, 0.0, {0.1, 0.2});
  std::vector<double> target(1, 0.0);  // smaller than joint_count
  EXPECT_FALSE(mpc::sample_reference(reference, 1, 2, 1.0, target));
}

TEST(TrajectorySampler, SinglePointHeldRegardlessOfTime)
{
  mpc::JointReference reference;
  set_point(reference, 0, 5.0, {0.1, 0.2, 0.3});
  std::vector<double> target(3, 0.0);

  // Before, at, and after the single point's timestamp: always the point.
  for (double t : {0.0, 5.0, 100.0}) {
    ASSERT_TRUE(mpc::sample_reference(reference, 1, 3, t, target));
    EXPECT_DOUBLE_EQ(target[0], 0.1);
    EXPECT_DOUBLE_EQ(target[1], 0.2);
    EXPECT_DOUBLE_EQ(target[2], 0.3);
  }
}

TEST(TrajectorySampler, TimeBeforeFirstPointHoldsFirst)
{
  mpc::JointReference reference;
  set_point(reference, 0, 10.0, {1.0});
  set_point(reference, 1, 11.0, {2.0});
  std::vector<double> target(1, 0.0);

  ASSERT_TRUE(mpc::sample_reference(reference, 2, 1, 9.5, target));
  EXPECT_DOUBLE_EQ(target[0], 1.0);
}

TEST(TrajectorySampler, TimeAfterLastPointHoldsLast)
{
  mpc::JointReference reference;
  set_point(reference, 0, 10.0, {1.0});
  set_point(reference, 1, 11.0, {2.0});
  std::vector<double> target(1, 0.0);

  ASSERT_TRUE(mpc::sample_reference(reference, 2, 1, 50.0, target));
  EXPECT_DOUBLE_EQ(target[0], 2.0);
}

TEST(TrajectorySampler, LinearInterpolationMidpoint)
{
  mpc::JointReference reference;
  reference.has_velocities = false;
  set_point(reference, 0, 0.0, {0.0, 10.0});
  set_point(reference, 1, 2.0, {1.0, 20.0});
  std::vector<double> target(2, 0.0);

  ASSERT_TRUE(mpc::sample_reference(reference, 2, 2, 1.0, target));
  EXPECT_DOUBLE_EQ(target[0], 0.5);    // midpoint of 0 → 1
  EXPECT_DOUBLE_EQ(target[1], 15.0);   // midpoint of 10 → 20
}

TEST(TrajectorySampler, LinearInterpolationQuarter)
{
  mpc::JointReference reference;
  reference.has_velocities = false;
  set_point(reference, 0, 0.0, {0.0});
  set_point(reference, 1, 4.0, {8.0});
  std::vector<double> target(1, 0.0);

  ASSERT_TRUE(mpc::sample_reference(reference, 2, 1, 1.0, target));  // ratio 0.25
  EXPECT_DOUBLE_EQ(target[0], 2.0);
}

TEST(TrajectorySampler, HermiteInterpolationWithVelocities)
{
  // dt = 2, q0 = 0, q1 = 1, v0 = 1.0, v1 = 0.0, sampled at midpoint (ratio 0.5).
  // h00=0.5, h10=0.125, h01=0.5, h11=-0.125
  // target = 0.5*0 + 0.125*2*1.0 + 0.5*1 + (-0.125)*2*0.0 = 0.75
  mpc::JointReference reference;
  reference.has_velocities = true;
  set_point(reference, 0, 0.0, {0.0}, {1.0});
  set_point(reference, 1, 2.0, {1.0}, {0.0});
  std::vector<double> target(1, 0.0);

  ASSERT_TRUE(mpc::sample_reference(reference, 2, 1, 1.0, target));
  EXPECT_DOUBLE_EQ(target[0], 0.75);
}

TEST(TrajectorySampler, HermiteWithZeroVelocityMatchesLinearAtMidpoint)
{
  mpc::JointReference reference;
  reference.has_velocities = true;
  set_point(reference, 0, 0.0, {0.0}, {0.0});
  set_point(reference, 1, 2.0, {1.0}, {0.0});
  std::vector<double> target(1, 0.0);

  ASSERT_TRUE(mpc::sample_reference(reference, 2, 1, 1.0, target));
  EXPECT_DOUBLE_EQ(target[0], 0.5);
}

TEST(TrajectorySampler, MultiSegmentSelectsCorrectInterval)
{
  mpc::JointReference reference;
  reference.has_velocities = false;
  set_point(reference, 0, 0.0, {0.0});
  set_point(reference, 1, 1.0, {10.0});
  set_point(reference, 2, 2.0, {20.0});
  std::vector<double> target(1, 0.0);

  // Sample inside the second segment [1.0, 2.0] at t = 1.5 → 15.0
  ASSERT_TRUE(mpc::sample_reference(reference, 3, 1, 1.5, target));
  EXPECT_DOUBLE_EQ(target[0], 15.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
