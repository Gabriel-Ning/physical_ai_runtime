// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <rclcpp/time.hpp>

#include "manipulation_position_controllers/common/stale_guard.hpp"

namespace common = manipulation_position_controllers::common;

TEST(StaleGuard, ReferenceFreshnessUsesSelectedClockDomain)
{
  EXPECT_TRUE(common::reference_is_fresh(10.1, 10.0, 0.2));
  EXPECT_FALSE(common::reference_is_fresh(10.3, 10.0, 0.2));
}

TEST(StaleGuard, BackwardClockJumpIsNeverFresh)
{
  EXPECT_FALSE(common::reference_is_fresh(1.0, 10.0, 0.2));

  bool hold = false;
  uint64_t hold_count = 0;
  EXPECT_TRUE(common::update_stale_hold_state(
    rclcpp::Time(1, 0, RCL_ROS_TIME),
    rclcpp::Time(10, 0, RCL_ROS_TIME), 0.2, hold, hold_count));
  EXPECT_TRUE(hold);
  EXPECT_EQ(hold_count, 1u);
}
