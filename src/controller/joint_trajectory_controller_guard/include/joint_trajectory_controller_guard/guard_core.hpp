// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>

namespace joint_trajectory_controller_guard
{

enum class GuardState : uint8_t
{
  kIdle = 0,
  kArmed,
  kCanceling,
  kFaulted
};
enum class GuardFault : uint8_t
{
  kNone = 0,
  kHeartbeatTimeout,
  kCancelAccepted,
  kCancelNoActiveGoal,
  kActionServerUnavailable,
  kCancelRejected,
  kCancelResponseTimeout,
};

class GuardCore
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  void heartbeat(bool active, TimePoint now) noexcept
  {
    if (!active) {
      if (state_ != GuardState::kCanceling) {
        state_ = GuardState::kIdle;
        fault_ = GuardFault::kNone;
      }
      return;
    }
    if (state_ == GuardState::kIdle || state_ == GuardState::kArmed) {
      state_ = GuardState::kArmed;
      fault_ = GuardFault::kNone;
      last_heartbeat_ = now;
      heartbeat_seen_ = true;
    }
  }

  bool tick(TimePoint now, std::chrono::duration<double> heartbeat_timeout) noexcept
  {
    if (
      state_ != GuardState::kArmed || !heartbeat_seen_ ||
      now - last_heartbeat_ <= heartbeat_timeout) {
      return false;
    }
    state_ = GuardState::kCanceling;
    fault_ = GuardFault::kHeartbeatTimeout;
    cancel_request_time_ = now;
    ++sequence_;
    return true;
  }

  bool cancel_response_expired(TimePoint now, std::chrono::duration<double> timeout) const noexcept
  {
    return state_ == GuardState::kCanceling && now - cancel_request_time_ > timeout;
  }

  bool cancel_result(bool accepted, size_t canceling_goal_count) noexcept
  {
    if (state_ != GuardState::kCanceling) {
      return false;
    }
    state_ = GuardState::kFaulted;
    if (!accepted) {
      fault_ = GuardFault::kCancelRejected;
    } else if (canceling_goal_count == 0) {
      fault_ = GuardFault::kCancelNoActiveGoal;
    } else {
      fault_ = GuardFault::kCancelAccepted;
    }
    return true;
  }

  bool cancel_failed(GuardFault fault) noexcept
  {
    if (state_ != GuardState::kCanceling) {
      return false;
    }
    state_ = GuardState::kFaulted;
    fault_ = fault;
    return true;
  }

  GuardState state() const noexcept { return state_; }
  GuardFault fault() const noexcept { return fault_; }
  uint64_t sequence() const noexcept { return sequence_; }

private:
  GuardState state_{GuardState::kIdle};
  GuardFault fault_{GuardFault::kNone};
  bool heartbeat_seen_{false};
  uint64_t sequence_{0};
  TimePoint last_heartbeat_{};
  TimePoint cancel_request_time_{};
};

}  // namespace joint_trajectory_controller_guard
