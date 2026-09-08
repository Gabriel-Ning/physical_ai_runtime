// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "manipulation_position_controllers/joint_space/kinematics/joint_reference.hpp"

namespace manipulation_position_controllers::joint_space
{

enum class ReferenceReadResult : uint8_t
{
  kSuccess = 0,
  kEmpty,
  kJointLayout,
  kZeroStamp,
  kPointLayout,
  kNonfinite,
  kOutOfBounds,
  kInvalidTiming,
};

struct ReferenceMessageReaderConfig
{
  const std::vector<std::string> & expected_joint_names;
  const std::vector<double> & lower_limits;
  const std::vector<double> & upper_limits;
  size_t max_reference_points{32};
  bool reject_out_of_bounds_targets{false};
  bool reject_zero_stamped_references{true};
  bool allow_partial_joint_references{false};
  const rclcpp::Clock & clock;
};

ReferenceReadResult read_reference_message_result(
  const trajectory_msgs::msg::JointTrajectory & message,
  const ReferenceMessageReaderConfig & config,
  JointReference & reference);

inline bool read_reference_message(
  const trajectory_msgs::msg::JointTrajectory & message,
  const ReferenceMessageReaderConfig & config,
  JointReference & reference)
{
  return read_reference_message_result(message, config, reference) ==
    ReferenceReadResult::kSuccess;
}

}  // namespace manipulation_position_controllers::joint_space
