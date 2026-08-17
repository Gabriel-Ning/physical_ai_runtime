// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>

#include "joint_trajectory_controller_guard/guard_core.hpp"

using namespace std::chrono_literals;
using joint_trajectory_controller_guard::GuardCore;
using joint_trajectory_controller_guard::GuardFault;
using joint_trajectory_controller_guard::GuardState;

TEST(GuardCore, IdleNeverCancels)
{
  GuardCore core;
  const auto t0 = GuardCore::TimePoint{};
  EXPECT_FALSE(core.tick(t0 + 10s, 100ms));
  EXPECT_EQ(core.state(), GuardState::kIdle);
}

TEST(GuardCore, ActiveHeartbeatTimeoutRequestsOneCancel)
{
  GuardCore core;
  const auto t0 = GuardCore::TimePoint{};
  core.heartbeat(true, t0);
  EXPECT_FALSE(core.tick(t0 + 99ms, 100ms));
  EXPECT_TRUE(core.tick(t0 + 101ms, 100ms));
  EXPECT_FALSE(core.tick(t0 + 200ms, 100ms));
  EXPECT_EQ(core.state(), GuardState::kCanceling);
  EXPECT_EQ(core.fault(), GuardFault::kHeartbeatTimeout);
  EXPECT_EQ(core.sequence(), 1u);
}

TEST(GuardCore, CancelResultLatchesUntilExplicitDisarm)
{
  GuardCore core;
  const auto t0 = GuardCore::TimePoint{};
  core.heartbeat(true, t0);
  ASSERT_TRUE(core.tick(t0 + 101ms, 100ms));
  core.cancel_result(true, 1u);
  EXPECT_EQ(core.state(), GuardState::kFaulted);
  EXPECT_EQ(core.fault(), GuardFault::kCancelAccepted);

  core.heartbeat(true, t0 + 200ms);
  EXPECT_EQ(core.state(), GuardState::kFaulted);
  core.heartbeat(false, t0 + 201ms);
  EXPECT_EQ(core.state(), GuardState::kIdle);
  EXPECT_EQ(core.fault(), GuardFault::kNone);
}

TEST(GuardCore, CancelResponseTimeoutIsAnError)
{
  GuardCore core;
  const auto t0 = GuardCore::TimePoint{};
  core.heartbeat(true, t0);
  ASSERT_TRUE(core.tick(t0 + 101ms, 100ms));
  EXPECT_TRUE(core.cancel_response_expired(t0 + 202ms, 100ms));
  core.cancel_failed(GuardFault::kCancelResponseTimeout);
  EXPECT_EQ(core.state(), GuardState::kFaulted);
  EXPECT_EQ(core.fault(), GuardFault::kCancelResponseTimeout);
}

TEST(GuardCore, LateCancelResponseCannotOverwriteLatchedFault)
{
  GuardCore core;
  const auto t0 = GuardCore::TimePoint{};
  core.heartbeat(true, t0);
  ASSERT_TRUE(core.tick(t0 + 101ms, 100ms));
  ASSERT_TRUE(core.cancel_failed(GuardFault::kCancelResponseTimeout));

  EXPECT_FALSE(core.cancel_result(true, 1u));
  EXPECT_EQ(core.state(), GuardState::kFaulted);
  EXPECT_EQ(core.fault(), GuardFault::kCancelResponseTimeout);
}
