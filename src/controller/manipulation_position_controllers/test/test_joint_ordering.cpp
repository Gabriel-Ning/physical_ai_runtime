// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "manipulation_position_controllers/common/joint_ordering.hpp"
#include "manipulation_position_controllers/common/limits.hpp"

namespace common = manipulation_position_controllers::common;

// ---------------------------------------------------------------------------
// build_source_indices
// ---------------------------------------------------------------------------

TEST(JointOrdering, ExactMatchIdentityMapping)
{
  std::vector<std::string> expected{"j1", "j2", "j3"};
  std::vector<std::string> message{"j1", "j2", "j3"};
  std::vector<size_t> indices;

  ASSERT_TRUE(common::build_source_indices(expected, message, indices));
  ASSERT_EQ(indices.size(), 3u);
  EXPECT_EQ(indices[0], 0u);
  EXPECT_EQ(indices[1], 1u);
  EXPECT_EQ(indices[2], 2u);
}

TEST(JointOrdering, ReorderedMessageMapsCorrectly)
{
  std::vector<std::string> expected{"j1", "j2", "j3"};
  std::vector<std::string> message{"j3", "j1", "j2"};
  std::vector<size_t> indices;

  ASSERT_TRUE(common::build_source_indices(expected, message, indices));
  EXPECT_EQ(indices[0], 1u);  // j1 is at message index 1
  EXPECT_EQ(indices[1], 2u);  // j2 is at message index 2
  EXPECT_EQ(indices[2], 0u);  // j3 is at message index 0
}

TEST(JointOrdering, PartialAllowedMarksMissingWithSentinel)
{
  std::vector<std::string> expected{"j1", "j2", "j3"};
  std::vector<std::string> message{"j1", "j3"};  // j2 missing
  std::vector<size_t> indices;

  ASSERT_TRUE(common::build_source_indices(expected, message, indices, /*allow_partial=*/true));
  EXPECT_EQ(indices[0], 0u);
  EXPECT_EQ(indices[1], common::kJointNotPresent);
  EXPECT_EQ(indices[2], 1u);
}

TEST(JointOrdering, PartialStrictRejectsMissing)
{
  std::vector<std::string> expected{"j1", "j2", "j3"};
  std::vector<std::string> message{"j1", "j3"};  // j2 missing
  std::vector<size_t> indices;

  EXPECT_FALSE(common::build_source_indices(expected, message, indices, /*allow_partial=*/false));
}

TEST(JointOrdering, NoOverlapAllowedReturnsAllSentinel)
{
  std::vector<std::string> expected{"j1", "j2"};
  std::vector<std::string> message{"x", "y"};
  std::vector<size_t> indices;

  ASSERT_TRUE(common::build_source_indices(expected, message, indices, /*allow_partial=*/true));
  EXPECT_EQ(indices[0], common::kJointNotPresent);
  EXPECT_EQ(indices[1], common::kJointNotPresent);
}

// ---------------------------------------------------------------------------
// validate_limit_vectors
// ---------------------------------------------------------------------------

TEST(Limits, BothEmptyIsValid)
{
  std::vector<std::string> joints{"j1", "j2"};
  EXPECT_TRUE(common::validate_limit_vectors(joints, {}, {}));
}

TEST(Limits, SizeMismatchRejected)
{
  std::vector<std::string> joints{"j1", "j2"};
  EXPECT_FALSE(common::validate_limit_vectors(joints, {-1.0}, {1.0}));
}

TEST(Limits, LowerGreaterThanUpperRejected)
{
  std::vector<std::string> joints{"j1"};
  EXPECT_FALSE(common::validate_limit_vectors(joints, {1.0}, {-1.0}));
}

TEST(Limits, InfiniteBoundsAllowed)
{
  std::vector<std::string> joints{"j1"};
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(common::validate_limit_vectors(joints, {-inf}, {inf}));
}

TEST(Limits, NaNBoundsRejected)
{
  std::vector<std::string> joints{"j1"};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(common::validate_limit_vectors(joints, {nan}, {1.0}));
}

// ---------------------------------------------------------------------------
// fill_default_limits
// ---------------------------------------------------------------------------

TEST(Limits, FillDefaultsUsesInfinityWhenEmpty)
{
  std::vector<std::string> joints{"j1", "j2"};
  std::vector<double> lower;
  std::vector<double> upper;
  common::fill_default_limits(joints, lower, upper);

  ASSERT_EQ(lower.size(), 2u);
  ASSERT_EQ(upper.size(), 2u);
  EXPECT_TRUE(std::isinf(lower[0]) && lower[0] < 0.0);
  EXPECT_TRUE(std::isinf(upper[0]) && upper[0] > 0.0);
}

TEST(Limits, FillDefaultsLeavesPopulatedVectorsUntouched)
{
  std::vector<std::string> joints{"j1"};
  std::vector<double> lower{-2.0};
  std::vector<double> upper{2.0};
  common::fill_default_limits(joints, lower, upper);

  EXPECT_DOUBLE_EQ(lower[0], -2.0);
  EXPECT_DOUBLE_EQ(upper[0], 2.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
