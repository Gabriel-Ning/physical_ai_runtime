// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include "manipulation_position_controllers/common/manipulation_diagnostics.hpp"

#include <cmath>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <functional>
#include <string>

namespace manipulation_position_controllers::common
{

namespace
{

int64_t seconds_to_microseconds(double seconds) noexcept
{
  if (!std::isfinite(seconds) || seconds <= 0.0) {
    return 0;
  }
  return static_cast<int64_t>(seconds * 1.0e6);
}

}  // namespace

const char * fault_name(ManipulationFault fault) noexcept
{
  switch (fault) {
    case ManipulationFault::kNone:
      return "NONE";
    case ManipulationFault::kReferenceTimeout:
      return "REFERENCE_TIMEOUT";
    case ManipulationFault::kZeroStampReference:
      return "ZERO_STAMP_REFERENCE";
    case ManipulationFault::kInvalidReference:
      return "INVALID_REFERENCE";
    case ManipulationFault::kFrameMismatch:
      return "FRAME_MISMATCH";
    case ManipulationFault::kNonfiniteReference:
      return "NONFINITE_REFERENCE";
    case ManipulationFault::kOutOfBoundsReference:
      return "OUT_OF_BOUNDS_REFERENCE";
    case ManipulationFault::kInvalidMeasuredState:
      return "INVALID_MEASURED_STATE";
    case ManipulationFault::kTrackingError:
      return "TRACKING_ERROR";
    case ManipulationFault::kInternalOutputFailure:
      return "INTERNAL_OUTPUT_FAILURE";
  }
  return "UNKNOWN";
}

const char * safety_state_name(ManipulationSafetyState state) noexcept
{
  switch (state) {
    case ManipulationSafetyState::kIdle:
      return "IDLE";
    case ManipulationSafetyState::kTracking:
      return "TRACKING";
    case ManipulationSafetyState::kSafeHold:
      return "SAFE_HOLD";
    case ManipulationSafetyState::kError:
      return "ERROR";
  }
  return "UNKNOWN";
}

void ManipulationDiagnostics::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & task_name,
  double period_s)
{
  updater_ = std::make_unique<diagnostic_updater::Updater>(node, period_s);
  updater_->setHardwareID("none");
  updater_->add(
    task_name,
    std::bind(&ManipulationDiagnostics::produce_diagnostics, this, std::placeholders::_1));
}

void ManipulationDiagnostics::record(
  ManipulationSafetyState state, ManipulationFault fault, double observed_s, double threshold_s,
  bool count_event) noexcept
{
  observed_us_.store(seconds_to_microseconds(observed_s), std::memory_order_relaxed);
  threshold_us_.store(seconds_to_microseconds(threshold_s), std::memory_order_relaxed);
  fault_.store(static_cast<uint8_t>(fault), std::memory_order_relaxed);
  state_.store(static_cast<uint8_t>(state), std::memory_order_release);
  if (count_event) {
    last_fault_.store(static_cast<uint8_t>(fault), std::memory_order_relaxed);
    sequence_.fetch_add(1, std::memory_order_relaxed);
  }
}

void ManipulationDiagnostics::set_idle() noexcept
{
  pending_hold_.store(static_cast<uint8_t>(ManipulationFault::kNone), std::memory_order_relaxed);
  record(ManipulationSafetyState::kIdle, ManipulationFault::kNone, 0.0, 0.0, false);
}

void ManipulationDiagnostics::set_tracking() noexcept
{
  record(ManipulationSafetyState::kTracking, ManipulationFault::kNone, 0.0, 0.0, false);
}

void ManipulationDiagnostics::request_hold(ManipulationFault fault) noexcept
{
  pending_hold_.store(static_cast<uint8_t>(fault), std::memory_order_release);
  record(ManipulationSafetyState::kSafeHold, fault, 0.0, 0.0, true);
}

ManipulationFault ManipulationDiagnostics::consume_hold_request() noexcept
{
  return static_cast<ManipulationFault>(pending_hold_.exchange(
    static_cast<uint8_t>(ManipulationFault::kNone), std::memory_order_acq_rel));
}

void ManipulationDiagnostics::enter_hold(
  ManipulationFault fault, double observed_s, double threshold_s) noexcept
{
  const auto previous_state = state();
  const auto previous_fault = this->fault();
  record(
    ManipulationSafetyState::kSafeHold, fault, observed_s, threshold_s,
    previous_state != ManipulationSafetyState::kSafeHold || previous_fault != fault);
}

void ManipulationDiagnostics::enter_error(ManipulationFault fault) noexcept
{
  const auto previous_state = state();
  const auto previous_fault = this->fault();
  record(
    ManipulationSafetyState::kError, fault, 0.0, 0.0,
    previous_state != ManipulationSafetyState::kError || previous_fault != fault);
}

ManipulationFault ManipulationDiagnostics::fault() const noexcept
{
  return static_cast<ManipulationFault>(fault_.load(std::memory_order_acquire));
}

ManipulationFault ManipulationDiagnostics::last_fault() const noexcept
{
  return static_cast<ManipulationFault>(last_fault_.load(std::memory_order_acquire));
}

ManipulationSafetyState ManipulationDiagnostics::state() const noexcept
{
  return static_cast<ManipulationSafetyState>(state_.load(std::memory_order_acquire));
}

uint64_t ManipulationDiagnostics::sequence() const noexcept
{
  return sequence_.load(std::memory_order_relaxed);
}

void ManipulationDiagnostics::produce_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  const auto current_state = state();
  const auto current_fault = fault();

  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  if (current_fault == ManipulationFault::kReferenceTimeout) {
    level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
  } else if (
    current_state == ManipulationSafetyState::kSafeHold ||
    current_state == ManipulationSafetyState::kError) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }

  status.summary(level, fault_name(current_fault));
  status.add("safety_state", safety_state_name(current_state));
  status.add("fault_code", fault_name(current_fault));
  status.add("last_fault_code", fault_name(last_fault()));
  status.add(
    "action", current_state == ManipulationSafetyState::kSafeHold
                ? "MEASURED_STATE_HOLD"
                : (current_state == ManipulationSafetyState::kError ? "RETURN_ERROR" : "NONE"));
  status.add("fault_sequence", sequence());
  status.add("observed_ms", observed_us_.load(std::memory_order_relaxed) / 1000.0);
  status.add("threshold_ms", threshold_us_.load(std::memory_order_relaxed) / 1000.0);
}

}  // namespace manipulation_position_controllers::common
