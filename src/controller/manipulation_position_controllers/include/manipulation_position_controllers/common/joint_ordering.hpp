// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace manipulation_position_controllers::common
{

/// Sentinel for "this joint was not present in the source message"
constexpr size_t kJointNotPresent = std::numeric_limits<size_t>::max();

/// Map from expected_joint_names → positions in message_joint_names.
/// @param allow_partial  if true, joints missing from the message get
///                       `kJointNotPresent` instead of returning false.
bool build_source_indices(
  const std::vector<std::string> & expected_joint_names,
  const std::vector<std::string> & message_joint_names,
  std::vector<size_t> & source_indices,
  bool allow_partial = false);

}  // namespace manipulation_position_controllers::common