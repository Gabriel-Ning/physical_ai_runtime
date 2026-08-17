// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include "manipulation_position_controllers/joint_space/kinematics/reference_message_reader.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "manipulation_position_controllers/common/joint_ordering.hpp"

namespace manipulation_position_controllers::joint_space
{

ReferenceReadResult read_reference_message_result(
  const trajectory_msgs::msg::JointTrajectory & message,
  const ReferenceMessageReaderConfig & config,
  JointReference & reference)
{
  if (message.joint_names.empty() || message.points.empty()) {
    return ReferenceReadResult::kEmpty;
  }

  std::vector<size_t> source_indices;
  if (!common::build_source_indices(
      config.expected_joint_names, message.joint_names, source_indices,
      config.allow_partial_joint_references)) {
    return ReferenceReadResult::kJointLayout;
  }
  const bool is_partial = config.allow_partial_joint_references;

  const size_t point_count = std::min(message.points.size(), config.max_reference_points);
  reference.size = point_count;
  reference.used_zero_stamp_fallback = false;
  reference.has_velocities = false;

  rclcpp::Time start_time(message.header.stamp);
  if (start_time.nanoseconds() == 0) {
    if (config.reject_zero_stamped_references) {
      return ReferenceReadResult::kZeroStamp;
    }
    start_time = config.clock.now();
    reference.used_zero_stamp_fallback = true;
  }
  reference.receive_time_s = config.clock.now().seconds();

  bool all_points_have_velocities = true;
  double previous_time = -std::numeric_limits<double>::infinity();
  for (size_t point_index = 0; point_index < point_count; ++point_index) {
    const auto & point = message.points[point_index];
    if (point.positions.size() < message.joint_names.size()) {
      return ReferenceReadResult::kPointLayout;
    }
    all_points_have_velocities =
      all_points_have_velocities && point.velocities.size() >= message.joint_names.size();

    const rclcpp::Time point_time = start_time + rclcpp::Duration(point.time_from_start);
    const double point_time_s = point_time.seconds();
    if (!std::isfinite(point_time_s) || point_time_s < previous_time) {
      return ReferenceReadResult::kInvalidTiming;
    }
    previous_time = point_time_s;
    reference.times[point_index] = point_time_s;

    for (size_t joint_index = 0; joint_index < config.expected_joint_names.size(); ++joint_index) {
      if (is_partial && source_indices[joint_index] == common::kJointNotPresent) {
        // Partial reference: this joint was not in the message — mark as
        // "hold current position" by using NaN (caller must fill from state).
        reference.positions[point_index * kMaxJoints + joint_index] =
          std::numeric_limits<double>::quiet_NaN();
        continue;
      }
      double position = point.positions[source_indices[joint_index]];
      if (!std::isfinite(position)) {
        return ReferenceReadResult::kNonfinite;
      }
      if (position < config.lower_limits[joint_index] ||
        position > config.upper_limits[joint_index])
      {
        if (config.reject_out_of_bounds_targets) {
          return ReferenceReadResult::kOutOfBounds;
        }
        position = std::clamp(
          position, config.lower_limits[joint_index], config.upper_limits[joint_index]);
      }
      reference.positions[point_index * kMaxJoints + joint_index] = position;
    }
  }

  if (all_points_have_velocities) {
    for (size_t point_index = 0; point_index < point_count; ++point_index) {
      const auto & point = message.points[point_index];
      const size_t offset = point_index * kMaxJoints;
      for (size_t joint_index = 0; joint_index < config.expected_joint_names.size(); ++joint_index) {
        if (is_partial && source_indices[joint_index] == common::kJointNotPresent) {
          reference.velocities[offset + joint_index] =
            std::numeric_limits<double>::quiet_NaN();
          continue;
        }
        const double velocity = point.velocities[source_indices[joint_index]];
        if (!std::isfinite(velocity)) {
          return ReferenceReadResult::kNonfinite;
        }
        reference.velocities[offset + joint_index] = velocity;
      }
    }
    reference.has_velocities = true;
  } else if (point_count > 1) {
    const auto joint_count = config.expected_joint_names.size();
    for (size_t point_index = 0; point_index < point_count; ++point_index) {
      size_t left_index = 0;
      size_t right_index = 0;
      if (point_index == 0) {
        left_index = 0;
        right_index = 1;
      } else if (point_index + 1 == point_count) {
        left_index = point_index - 1;
        right_index = point_index;
      } else {
        left_index = point_index - 1;
        right_index = point_index + 1;
      }

      const double dt = reference.times[right_index] - reference.times[left_index];
      if (dt <= 0.0 || !std::isfinite(dt)) {
        return ReferenceReadResult::kInvalidTiming;
      }

      const size_t offset = point_index * kMaxJoints;
      const size_t left_offset = left_index * kMaxJoints;
      const size_t right_offset = right_index * kMaxJoints;
      for (size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
        reference.velocities[offset + joint_index] =
          (reference.positions[right_offset + joint_index] -
          reference.positions[left_offset + joint_index]) / dt;
      }
    }
    reference.has_velocities = true;
  } else {
    for (size_t joint_index = 0; joint_index < config.expected_joint_names.size(); ++joint_index) {
      reference.velocities[joint_index] = 0.0;
    }
    reference.has_velocities = true;
  }

  return ReferenceReadResult::kSuccess;
}

}  // namespace manipulation_position_controllers::joint_space
