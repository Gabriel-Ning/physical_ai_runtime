// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// Realtime-safe pose chunk buffer — parallel to JointReference.
// Stores N time-indexed end-effector poses and provides receding-horizon
// sampling for the TaskSpaceKinematicPositionController update loop.

#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace manipulation_position_controllers::task_space
{

/// Maximum number of pose frames in a chunk (same ceiling as JointReference).
constexpr size_t kMaxPoseFrames = 64;

/// A single time-indexed end-effector pose.
struct PoseFrame
{
  /// Absolute time [s] (header.stamp + i*dt).
  double time_s{0.0};

  /// Position {x, y, z} [m].
  double px{0.0}, py{0.0}, pz{0.0};

  /// Orientation quaternion {x, y, z, w}.
  double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
};

/// Fixed-capacity chunk of time-indexed pose frames.
/// All storage is pre-allocated; zero heap allocation in steady state.
struct PoseChunk
{
  /// Number of valid frames in this chunk (0 = empty).
  size_t size{0};

  /// Monotonic sequence number.
  uint64_t sequence{0};

  /// Receive time of the latest chunk [s].
  double receive_time_s{0.0};

  /// Time-indexed pose frames.
  std::array<PoseFrame, kMaxPoseFrames> frames{};
};

/// True while a chunk should still drive the command.
///
/// A chunk stays "fresh" from receipt, through its own planned time horizon
/// (the last frame's time), plus a `stale_timeout_s` grace window past the
/// horizon. After that the controller treats it as stale and enters hold,
/// rather than holding the last frame forever. This mirrors the hold-and-
/// recover contract used for single-pose and twist inputs.
///
/// @param chunk           the active pose chunk.
/// @param time_s          current wall-clock time [s].
/// @param stale_timeout_s grace window past the last frame [s].
/// @return                true if the chunk is still active at time_s.
inline bool chunk_is_fresh(
  const PoseChunk & chunk, double time_s, double stale_timeout_s)
{
  if (chunk.size == 0) {
    return false;
  }
  const double end_time_s = chunk.frames[chunk.size - 1].time_s;
  return time_s <= end_time_s + stale_timeout_s;
}

/// Sample a pose from a chunk at a given wall-clock time.
///
/// @param chunk       the active pose chunk.
/// @param time_s      current wall-clock time [s].
/// @param px,py,pz    output position [m].
/// @param qx,qy,qz,qw output orientation quaternion.
/// @return            true if a valid sample was produced.
inline bool sample_pose_chunk(
  const PoseChunk & chunk,
  double time_s,
  double & px, double & py, double & pz,
  double & qx, double & qy, double & qz, double & qw)
{
  if (chunk.size == 0) {
    return false;
  }

  // Before the chunk window: hold first frame.
  if (time_s <= chunk.frames[0].time_s || chunk.size == 1) {
    const auto & f = chunk.frames[0];
    px = f.px; py = f.py; pz = f.pz;
    qx = f.qx; qy = f.qy; qz = f.qz; qw = f.qw;
    return true;
  }

  // After the chunk window: hold last frame.
  const size_t last = chunk.size - 1;
  if (time_s >= chunk.frames[last].time_s) {
    const auto & f = chunk.frames[last];
    px = f.px; py = f.py; pz = f.pz;
    qx = f.qx; qy = f.qy; qz = f.qz; qw = f.qw;
    return true;
  }

  // Within the window: linear interpolation between adjacent frames.
  size_t right = 1;
  while (right < chunk.size && chunk.frames[right].time_s < time_s) {
    ++right;
  }
  const size_t left = right - 1;
  const auto & f0 = chunk.frames[left];
  const auto & f1 = chunk.frames[right];
  const double dt = f1.time_s - f0.time_s;
  const double ratio = (dt > 1e-12) ? ((time_s - f0.time_s) / dt) : 0.0;
  const double inv = 1.0 - ratio;

  // Position: linear interpolation.
  px = inv * f0.px + ratio * f1.px;
  py = inv * f0.py + ratio * f1.py;
  pz = inv * f0.pz + ratio * f1.pz;

  // Orientation: slerp (simplified — lerp + normalize for short intervals).
  qx = inv * f0.qx + ratio * f1.qx;
  qy = inv * f0.qy + ratio * f1.qy;
  qz = inv * f0.qz + ratio * f1.qz;
  qw = inv * f0.qw + ratio * f1.qw;
  // Normalize the interpolated quaternion.
  const double n = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
  if (n > 1e-12) {
    const double inv_n = 1.0 / n;
    qx *= inv_n; qy *= inv_n; qz *= inv_n; qw *= inv_n;
  } else {
    qx = f0.qx; qy = f0.qy; qz = f0.qz; qw = f0.qw;
  }

  return true;
}

}  // namespace manipulation_position_controllers::task_space
