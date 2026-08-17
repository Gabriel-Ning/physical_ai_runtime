// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>

#include <rclcpp/time.hpp>

namespace manipulation_position_controllers::common
{

inline bool update_stale_hold_state(
	const rclcpp::Time & now,
	const rclcpp::Time & last_reference_time,
	const double stale_timeout_s,
	bool & stale_hold_active,
	uint64_t & stale_hold_count)
{
	const bool is_stale = (now - last_reference_time).seconds() > stale_timeout_s;
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