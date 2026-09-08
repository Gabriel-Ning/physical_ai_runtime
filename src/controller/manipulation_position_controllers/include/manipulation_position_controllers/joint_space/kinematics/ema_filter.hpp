// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <vector>

namespace manipulation_position_controllers::joint_space
{

inline void apply_ema(
	const std::vector<double> & raw_target,
	const double alpha,
	std::vector<double> & filtered_target)
{
	const auto size = raw_target.size();
	if (filtered_target.size() != size) {
		filtered_target.assign(size, 0.0);
	}
	for (size_t index = 0; index < size; ++index) {
		filtered_target[index] = alpha * raw_target[index] + (1.0 - alpha) * filtered_target[index];
	}
}

}  // namespace manipulation_position_controllers::joint_space