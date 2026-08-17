// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "manipulation_position_controllers/joint_space/kinematics/ruckig_adapter.hpp"

namespace mpc = manipulation_position_controllers::joint_space;

TEST(RuckigAdapter, InvalidBeforeConfigureReturnsFalse)
{
  mpc::RuckigAdapter adapter;
  std::vector<double> current{0.0};
  std::vector<double> target{1.0};
  std::vector<double> output{0.0};

  EXPECT_FALSE(adapter.update(current, target, output));
}

TEST(RuckigAdapter, RejectsMismatchedVectorSizes)
{
  mpc::RuckigAdapter adapter;
  mpc::RuckigConfig config;
  adapter.configure(2, 0.002, config);

  std::vector<double> current{0.0};
  std::vector<double> target{1.0, 1.0};
  std::vector<double> output{0.0, 0.0};

  EXPECT_FALSE(adapter.update(current, target, output));
}

TEST(RuckigAdapter, FirstStepRespectsVelocityLimit)
{
  mpc::RuckigAdapter adapter;
  mpc::RuckigConfig config;
  config.max_velocity_rad_s = 0.5;
  config.max_acceleration_rad_s2 = 1.0;
  config.max_jerk_rad_s3 = 2.0;
  adapter.configure(1, 0.01, config);

  std::vector<double> current{0.0};
  std::vector<double> target{1.0};
  std::vector<double> output{0.0};

  ASSERT_TRUE(adapter.update(current, target, output));
  EXPECT_GE(output[0], 0.0);
  EXPECT_LE(output[0], config.max_velocity_rad_s * 0.01);
}

TEST(RuckigAdapter, ConvergesToTarget)
{
  mpc::RuckigAdapter adapter;
  mpc::RuckigConfig config;
  config.max_velocity_rad_s = 1.0;
  config.max_acceleration_rad_s2 = 2.0;
  config.max_jerk_rad_s3 = 5.0;
  adapter.configure(1, 0.01, config);

  std::vector<double> command{0.0};
  const std::vector<double> target{0.5};

  for (size_t i = 0; i < 500; ++i) {
    ASSERT_TRUE(adapter.update(command, target, command));
  }

  EXPECT_NEAR(command[0], target[0], 1e-4);
  EXPECT_GT(adapter.trajectory_duration(), 0.0);
}

TEST(RuckigAdapter, TargetCanChangeDuringMotion)
{
  mpc::RuckigAdapter adapter;
  mpc::RuckigConfig config;
  config.max_velocity_rad_s = 1.0;
  config.max_acceleration_rad_s2 = 2.0;
  config.max_jerk_rad_s3 = 5.0;
  adapter.configure(1, 0.01, config);

  std::vector<double> command{0.0};
  std::vector<double> target{0.5};

  for (size_t i = 0; i < 50; ++i) {
    ASSERT_TRUE(adapter.update(command, target, command));
  }

  target[0] = -0.25;
  for (size_t i = 0; i < 500; ++i) {
    ASSERT_TRUE(adapter.update(command, target, command));
  }

  EXPECT_NEAR(command[0], target[0], 1e-4);
}

TEST(RuckigAdapter, ExtrapolateBeforeSeedIsNoOp)
{
  mpc::RuckigAdapter adapter;
  mpc::RuckigConfig config;
  adapter.configure(1, 0.001, config);
  // Must not crash or touch uninitialized OTG state.
  adapter.extrapolate_constant_accel(0.002);
}

TEST(RuckigAdapter, ExtrapolateAdvancesWithConstantAccel)
{
  mpc::RuckigAdapter adapter;
  mpc::RuckigConfig config;
  config.max_velocity_rad_s = 2.0;
  config.max_acceleration_rad_s2 = 5.0;
  config.max_jerk_rad_s3 = 50.0;
  adapter.configure(1, 0.001, config);

  std::vector<double> command{0.0};
  const std::vector<double> target{1.0};
  // Build non-zero velocity / acceleration on the internal input via OTG steps.
  for (size_t i = 0; i < 20; ++i) {
    ASSERT_TRUE(adapter.update(command, target, command));
  }
  const double before = command[0];

  // Constant-accel gap fill then one more OTG step should keep moving forward.
  adapter.extrapolate_constant_accel(0.002);
  ASSERT_TRUE(adapter.update(command, target, command));
  EXPECT_GT(command[0], before);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
