// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <vector>

#include "manipulation_position_controllers/joint_space/kinematics/joint_reference.hpp"

namespace manipulation_position_controllers::joint_space
{

bool sample_reference(
	const JointReference & reference,
	size_t active_reference_size,
	size_t joint_count,
	double time_s,
	std::vector<double> & target);

}  // namespace manipulation_position_controllers::joint_space