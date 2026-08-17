// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include "manipulation_position_controllers/common/joint_ordering.hpp"

#include <unordered_map>

namespace manipulation_position_controllers::common
{

bool build_source_indices(
  const std::vector<std::string> & expected_joint_names,
  const std::vector<std::string> & message_joint_names,
  std::vector<size_t> & source_indices,
  bool allow_partial)
{
  std::unordered_map<std::string, size_t> index_by_name;
  index_by_name.reserve(message_joint_names.size());
  for (size_t index = 0; index < message_joint_names.size(); ++index) {
    index_by_name.emplace(message_joint_names[index], index);
  }

  source_indices.assign(expected_joint_names.size(), kJointNotPresent);
  bool found_any = false;
  for (size_t index = 0; index < expected_joint_names.size(); ++index) {
    const auto it = index_by_name.find(expected_joint_names[index]);
    if (it != index_by_name.end()) {
      source_indices[index] = it->second;
      found_any = true;
    } else if (!allow_partial) {
      return false;
    }
  }

  return found_any || allow_partial;
}

}  // namespace manipulation_position_controllers::common