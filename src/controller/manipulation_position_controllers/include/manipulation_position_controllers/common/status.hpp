// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>

namespace manipulation_position_controllers::common
{

inline constexpr size_t kStatusFieldCount = 10;
inline constexpr size_t kStatusActiveReferenceSize = 0;
inline constexpr size_t kStatusCommandAge = 1;
inline constexpr size_t kStatusLimitFactor = 2;
inline constexpr size_t kStatusInitialized = 3;
inline constexpr size_t kStatusZeroStampFallbackCount = 4;
inline constexpr size_t kStatusReferenceOverwriteCount = 5;
inline constexpr size_t kStatusCoalescedUpdateCount = 6;
inline constexpr size_t kStatusStaleHoldCount = 7;
inline constexpr size_t kStatusRemainingReferenceTime = 8;
inline constexpr size_t kStatusPreemptionJumpNorm = 9;

}  // namespace manipulation_position_controllers::common