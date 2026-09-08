// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace manipulation_position_controllers::joint_space
{

inline constexpr size_t kMaxJoints = 16;
inline constexpr size_t kMaxReferencePoints = 64;

struct JointReference
{
  std::array<double, kMaxReferencePoints> times{};
  std::array<double, kMaxReferencePoints * kMaxJoints> positions{};
  std::array<double, kMaxReferencePoints * kMaxJoints> velocities{};
  size_t size{0};
  uint64_t sequence{0};
  bool has_velocities{false};
  bool used_zero_stamp_fallback{false};
  double receive_time_s{0.0};
};

}  // namespace manipulation_position_controllers::joint_space