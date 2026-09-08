// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include "manipulation_position_controllers/joint_space/kinematics/trajectory_sampler.hpp"

namespace manipulation_position_controllers::joint_space
{

bool sample_reference(
  const JointReference & reference,
  const size_t active_reference_size,
  const size_t joint_count,
  const double time_s,
  std::vector<double> & target)
{
  if (active_reference_size == 0 || target.size() < joint_count) {
    return false;
  }

  if (active_reference_size == 1 || time_s <= reference.times[0]) {
    for (size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
      target[joint_index] = reference.positions[joint_index];
    }
    return true;
  }

  const size_t last_index = active_reference_size - 1;
  if (time_s >= reference.times[last_index]) {
    const size_t offset = last_index * kMaxJoints;
    for (size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
      target[joint_index] = reference.positions[offset + joint_index];
    }
    return true;
  }

  size_t right_index = 1;
  while (right_index < active_reference_size && reference.times[right_index] < time_s) {
    ++right_index;
  }
  const size_t left_index = right_index - 1;
  const double t0 = reference.times[left_index];
  const double t1 = reference.times[right_index];
  const double ratio = (t1 > t0) ? ((time_s - t0) / (t1 - t0)) : 0.0;

  const size_t left_offset = left_index * kMaxJoints;
  const size_t right_offset = right_index * kMaxJoints;
  for (size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
    const double q0 = reference.positions[left_offset + joint_index];
    const double q1 = reference.positions[right_offset + joint_index];
    if (reference.has_velocities) {
      const double dt = t1 - t0;
      const double ratio2 = ratio * ratio;
      const double ratio3 = ratio2 * ratio;
      const double h00 = 2.0 * ratio3 - 3.0 * ratio2 + 1.0;
      const double h10 = ratio3 - 2.0 * ratio2 + ratio;
      const double h01 = -2.0 * ratio3 + 3.0 * ratio2;
      const double h11 = ratio3 - ratio2;
      const double v0 = reference.velocities[left_offset + joint_index];
      const double v1 = reference.velocities[right_offset + joint_index];
      target[joint_index] = h00 * q0 + h10 * dt * v0 + h01 * q1 + h11 * dt * v1;
    } else {
      target[joint_index] = q0 + ratio * (q1 - q0);
    }
  }

  return true;
}

}  // namespace manipulation_position_controllers::joint_space