// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

namespace manipulation_position_controllers::common
{

inline bool is_positive_finite(double value)
{
  return std::isfinite(value) && value > 0.0;
}

inline bool is_nonnegative_finite(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

inline bool is_unit_interval_finite(double value)
{
  return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

inline bool all_nonnegative_finite(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), is_nonnegative_finite);
}

inline bool all_positive_finite(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), is_positive_finite);
}

inline bool all_unique_nonempty(const std::vector<std::string> & values)
{
  std::unordered_set<std::string> seen;
  seen.reserve(values.size());
  for (const auto & value : values) {
    if (value.empty() || !seen.insert(value).second) {
      return false;
    }
  }
  return true;
}

}  // namespace manipulation_position_controllers::common
