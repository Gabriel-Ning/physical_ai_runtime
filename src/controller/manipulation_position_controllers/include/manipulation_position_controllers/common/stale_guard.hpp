// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>

#include <rclcpp/time.hpp>

namespace manipulation_position_controllers::common
{

inline bool reference_is_fresh(
  const double now_s, const double reference_time_s, const double stale_timeout_s)
{
  const double age_s = now_s - reference_time_s;
  // A negative age means the active clock moved backwards (for example a
  // simulation world reset) or the producer is in a different clock domain.
  // Never let an old/future reference survive such a discontinuity.
  return age_s >= 0.0 && age_s <= stale_timeout_s;
}

inline bool update_stale_hold_state(
  const rclcpp::Time & now,
  const rclcpp::Time & last_reference_time,
  const double stale_timeout_s,
  bool & stale_hold_active,
  uint64_t & stale_hold_count)
{
  const double age_s = (now - last_reference_time).seconds();
  const bool is_stale = age_s < 0.0 || age_s > stale_timeout_s;
  if (!is_stale) {
    stale_hold_active = false;
    return false;
  }

  if (!stale_hold_active) {
    ++stale_hold_count;
    stale_hold_active = true;
  }
  return true;
}

}  // namespace manipulation_position_controllers::common
