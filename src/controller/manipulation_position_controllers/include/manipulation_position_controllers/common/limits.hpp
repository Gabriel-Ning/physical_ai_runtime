// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace manipulation_position_controllers::common
{

inline bool validate_limit_vectors(
	const std::vector<std::string> & joint_names,
	const std::vector<double> & lower_limits,
	const std::vector<double> & upper_limits)
{
	if (lower_limits.empty() && upper_limits.empty()) {
		return true;
	}
	if (lower_limits.size() != joint_names.size() || upper_limits.size() != joint_names.size()) {
		return false;
	}

	for (size_t index = 0; index < joint_names.size(); ++index) {
		const double lower = lower_limits[index];
		const double upper = upper_limits[index];
		if ((!std::isfinite(lower) && !std::isinf(lower)) ||
			(!std::isfinite(upper) && !std::isinf(upper)))
		{
			return false;
		}
		if (lower > upper) {
			return false;
		}
	}
	return true;
}

inline void fill_default_limits(
	const std::vector<std::string> & joint_names,
	std::vector<double> & lower_limits,
	std::vector<double> & upper_limits)
{
	if (lower_limits.empty()) {
		lower_limits.assign(joint_names.size(), -std::numeric_limits<double>::infinity());
	}
	if (upper_limits.empty()) {
		upper_limits.assign(joint_names.size(), std::numeric_limits<double>::infinity());
	}
}

}  // namespace manipulation_position_controllers::common