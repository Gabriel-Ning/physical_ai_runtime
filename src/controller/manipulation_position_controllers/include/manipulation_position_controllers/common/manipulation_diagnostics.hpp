// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <atomic>
#include <cstdint>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <memory>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>

namespace manipulation_position_controllers::common
{

enum class ManipulationFault : uint8_t
{
  kNone = 0,
  kReferenceTimeout,
  kZeroStampReference,
  kInvalidReference,
  kFrameMismatch,
  kNonfiniteReference,
  kOutOfBoundsReference,
  kInvalidMeasuredState,
  kTrackingError,
  kInternalOutputFailure,
};

enum class ManipulationSafetyState : uint8_t
{
  kIdle = 0,
  kTracking,
  kSafeHold,
  kError,
};

const char * fault_name(ManipulationFault fault) noexcept;
const char * safety_state_name(ManipulationSafetyState state) noexcept;

/// Diagnostics bridge for manipulation-specific safety state.
///
/// Controller update() calls only the noexcept atomic methods. The
/// diagnostic_updater callback performs all string construction and DDS work
/// from a non-realtime executor callback.
class ManipulationDiagnostics
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & task_name,
    double period_s);

  void set_idle() noexcept;
  void set_tracking() noexcept;
  void request_hold(ManipulationFault fault) noexcept;
  ManipulationFault consume_hold_request() noexcept;
  void enter_hold(
    ManipulationFault fault, double observed_s = 0.0, double threshold_s = 0.0) noexcept;
  void enter_error(ManipulationFault fault) noexcept;

  ManipulationFault fault() const noexcept;
  ManipulationFault last_fault() const noexcept;
  ManipulationSafetyState state() const noexcept;
  uint64_t sequence() const noexcept;

private:
  void record(
    ManipulationSafetyState state, ManipulationFault fault, double observed_s, double threshold_s,
    bool count_event) noexcept;
  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status);

  std::unique_ptr<diagnostic_updater::Updater> updater_;
  std::atomic<uint8_t> state_{static_cast<uint8_t>(ManipulationSafetyState::kIdle)};
  std::atomic<uint8_t> fault_{static_cast<uint8_t>(ManipulationFault::kNone)};
  std::atomic<uint8_t> last_fault_{static_cast<uint8_t>(ManipulationFault::kNone)};
  std::atomic<uint8_t> pending_hold_{static_cast<uint8_t>(ManipulationFault::kNone)};
  std::atomic<uint64_t> sequence_{0};
  std::atomic<int64_t> observed_us_{0};
  std::atomic<int64_t> threshold_us_{0};
};

}  // namespace manipulation_position_controllers::common
