// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// Realtime-safe pose target container for task-space controllers.
// Generic: no robot-specific assumptions.

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace manipulation_position_controllers::task_space
{

/// A single pose target received via ROS subscription.
///
/// All fields are fixed-size (no dynamic allocation) so the struct is trivially
/// copyable into a realtime_tools::RealtimeBuffer.
struct PoseTarget
{
  /// Seconds since epoch from header.stamp (0.0 = unset).
  double stamp{0.0};

  /// Frame this pose is expressed in.  Empty string = invalid / never set.
  std::string frame_id;

  /// Position {x, y, z} in metres.
  std::array<double, 3> position{0.0, 0.0, 0.0};

  /// Orientation quaternion {x, y, z, w}.  Stored as received.
  std::array<double, 4> orientation{0.0, 0.0, 1.0, 0.0};

  /// Monotonic sequence number assigned on each ROS callback invocation.
  uint64_t sequence{0};

  /// True when at least one valid target has been written.
  bool valid{false};
};

/// A task-space velocity command received via TwistStamped.
///
/// Twist is expressed in base_frame with linear {x,y,z} [m/s] and angular
/// {x,y,z} [rad/s].  The controller integrates it into an internal pose target
/// and then uses the normal pose IK backend.
struct TwistCommand
{
  double stamp{0.0};
  std::string frame_id;
  std::array<double, 3> linear{0.0, 0.0, 0.0};
  std::array<double, 3> angular{0.0, 0.0, 0.0};
  uint64_t sequence{0};
  bool valid{false};
};

/// Lightweight validity check (does not involve TF or URDF).
inline bool is_valid_quaternion(const std::array<double, 4> & q)
{
  const double norm_sq =
    q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  return norm_sq > 0.9 && norm_sq < 1.1;
}

inline bool is_valid_position(const std::array<double, 3> & p)
{
  return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

inline bool is_valid_vector3(const std::array<double, 3> & v)
{
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

}  // namespace manipulation_position_controllers::task_space
