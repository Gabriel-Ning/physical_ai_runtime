// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "manipulation_position_controllers/common/manipulation_diagnostics.hpp"

namespace common = manipulation_position_controllers::common;

TEST(ManipulationDiagnostics, HoldRequestIsConsumedExactlyOnce)
{
  common::ManipulationDiagnostics diagnostics;

  diagnostics.request_hold(common::ManipulationFault::kZeroStampReference);

  EXPECT_EQ(diagnostics.state(), common::ManipulationSafetyState::kSafeHold);
  EXPECT_EQ(diagnostics.fault(), common::ManipulationFault::kZeroStampReference);
  EXPECT_EQ(diagnostics.sequence(), 1u);
  EXPECT_EQ(diagnostics.consume_hold_request(), common::ManipulationFault::kZeroStampReference);
  EXPECT_EQ(diagnostics.consume_hold_request(), common::ManipulationFault::kNone);
}

TEST(ManipulationDiagnostics, RepeatedSameHoldDoesNotInflateEventSequence)
{
  common::ManipulationDiagnostics diagnostics;

  diagnostics.enter_hold(common::ManipulationFault::kReferenceTimeout, 0.6, 0.5);
  diagnostics.enter_hold(common::ManipulationFault::kReferenceTimeout, 0.7, 0.5);

  EXPECT_EQ(diagnostics.sequence(), 1u);
  diagnostics.set_tracking();
  EXPECT_EQ(diagnostics.fault(), common::ManipulationFault::kNone);
  EXPECT_EQ(diagnostics.last_fault(), common::ManipulationFault::kReferenceTimeout);
  diagnostics.enter_hold(common::ManipulationFault::kReferenceTimeout, 0.6, 0.5);
  EXPECT_EQ(diagnostics.sequence(), 2u);
}

TEST(ManipulationDiagnostics, FaultAndStateNamesAreStable)
{
  EXPECT_STREQ(
    common::fault_name(common::ManipulationFault::kNonfiniteReference), "NONFINITE_REFERENCE");
  EXPECT_STREQ(common::safety_state_name(common::ManipulationSafetyState::kSafeHold), "SAFE_HOLD");
}
