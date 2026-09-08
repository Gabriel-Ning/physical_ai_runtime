// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// Non-RT helper: moveit_msgs/CartesianTrajectory → PoseChunk.
// Timed: header.stamp + time_from_start (strictly increasing when any
// time_from_start is non-zero). Untimed (all-zero): receive_time + i * dt.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include <builtin_interfaces/msg/duration.hpp>
#include <moveit_msgs/msg/cartesian_trajectory.hpp>

#include "manipulation_position_controllers/task_space/kinematics/pose_chunk_buffer.hpp"
#include "manipulation_position_controllers/task_space/kinematics/pose_target_buffer.hpp"

namespace manipulation_position_controllers::task_space
{

struct CartesianTrajectoryReaderConfig
{
  /// Absolute receive / header-resolved start time [s].
  double receive_time_s{0.0};

  /// Monotonic sequence stamped onto the chunk.
  uint64_t sequence{0};

  /// Max points to accept (also capped by kMaxPoseFrames).
  size_t max_pose_points{kMaxPoseFrames};

  /// Inter-frame spacing [s] when all time_from_start are zero.
  double untimed_frame_dt_s{0.02};

  /// Reject messages that do not carry a source timestamp.
  bool reject_zero_stamped_references{true};
};

inline double duration_to_seconds(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<double>(d.sec) + 1e-9 * static_cast<double>(d.nanosec);
}

/// Fill ``chunk`` from ``msg``. Returns false on empty / invalid / bad timing.
inline bool read_cartesian_trajectory(
  const moveit_msgs::msg::CartesianTrajectory & msg,
  const CartesianTrajectoryReaderConfig & config,
  PoseChunk & chunk)
{
  if (config.reject_zero_stamped_references &&
    msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0)
  {
    return false;
  }
  if (msg.points.empty() || config.max_pose_points == 0 ||
    config.untimed_frame_dt_s <= 0.0)
  {
    return false;
  }

  const size_t count = std::min(
    {msg.points.size(), config.max_pose_points, kMaxPoseFrames});

  bool any_timed = false;
  for (size_t i = 0; i < count; ++i) {
    if (duration_to_seconds(msg.points[i].time_from_start) != 0.0) {
      any_timed = true;
      break;
    }
  }

  chunk = PoseChunk{};
  chunk.size = count;
  chunk.sequence = config.sequence;
  chunk.receive_time_s = config.receive_time_s;

  double previous_time = -std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < count; ++i) {
    const auto & pose = msg.points[i].point.pose;
    const std::array<double, 3> position{
      pose.position.x, pose.position.y, pose.position.z};
    const std::array<double, 4> orientation{
      pose.orientation.x, pose.orientation.y,
      pose.orientation.z, pose.orientation.w};
    if (!is_valid_position(position) || !is_valid_quaternion(orientation)) {
      return false;
    }

    double time_s = 0.0;
    if (!any_timed) {
      time_s = config.receive_time_s +
        static_cast<double>(i) * config.untimed_frame_dt_s;
    } else {
      const double tfs = duration_to_seconds(msg.points[i].time_from_start);
      if (!std::isfinite(tfs) || tfs < 0.0) {
        return false;
      }
      time_s = config.receive_time_s + tfs;
      if (!(time_s > previous_time)) {
        return false;
      }
    }
    previous_time = time_s;

    auto & frame = chunk.frames[i];
    frame.time_s = time_s;
    frame.px = position[0];
    frame.py = position[1];
    frame.pz = position[2];
    frame.qx = orientation[0];
    frame.qy = orientation[1];
    frame.qz = orientation[2];
    frame.qw = orientation[3];
  }
  return true;
}

}  // namespace manipulation_position_controllers::task_space
