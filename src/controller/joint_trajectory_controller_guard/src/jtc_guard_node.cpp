// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include <action_msgs/srv/cancel_goal.hpp>
#include <chrono>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <functional>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <stdexcept>
#include <string>

#include "joint_trajectory_controller_guard/guard_core.hpp"

namespace joint_trajectory_controller_guard
{

namespace
{

const char * state_name(GuardState state)
{
  switch (state) {
    case GuardState::kIdle:
      return "IDLE";
    case GuardState::kArmed:
      return "ARMED";
    case GuardState::kCanceling:
      return "CANCELING";
    case GuardState::kFaulted:
      return "FAULTED";
  }
  return "UNKNOWN";
}

const char * fault_name(GuardFault fault)
{
  switch (fault) {
    case GuardFault::kNone:
      return "NONE";
    case GuardFault::kHeartbeatTimeout:
      return "TRAJECTORY_HEARTBEAT_TIMEOUT";
    case GuardFault::kCancelAccepted:
      return "TRAJECTORY_CANCEL_ACCEPTED";
    case GuardFault::kCancelNoActiveGoal:
      return "TRAJECTORY_CANCEL_NO_ACTIVE_GOAL";
    case GuardFault::kActionServerUnavailable:
      return "JTC_ACTION_SERVER_UNAVAILABLE";
    case GuardFault::kCancelRejected:
      return "TRAJECTORY_CANCEL_REJECTED";
    case GuardFault::kCancelResponseTimeout:
      return "TRAJECTORY_CANCEL_RESPONSE_TIMEOUT";
  }
  return "UNKNOWN";
}

const char * action_name(GuardState state, GuardFault fault)
{
  if (state == GuardState::kCanceling) {
    return "CANCEL_REQUESTED";
  }
  switch (fault) {
    case GuardFault::kCancelAccepted:
      return "CANCEL_ACCEPTED";
    case GuardFault::kCancelNoActiveGoal:
      return "NO_ACTIVE_GOAL";
    case GuardFault::kActionServerUnavailable:
    case GuardFault::kCancelRejected:
    case GuardFault::kCancelResponseTimeout:
      return "CANCEL_FAILED";
    case GuardFault::kNone:
    case GuardFault::kHeartbeatTimeout:
      return "NONE";
  }
  return "UNKNOWN";
}

}  // namespace

class JtcGuardNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using CancelResponse = rclcpp_action::Client<FollowJointTrajectory>::CancelResponse;

  JtcGuardNode() : Node("jtc_guard")
  {
    action_name_ = declare_parameter<std::string>("action_name", "");
    heartbeat_topic_ = declare_parameter<std::string>("heartbeat_topic", "~/heartbeat");
    heartbeat_timeout_s_ = declare_parameter<double>("heartbeat_timeout_s", 0.5);
    cancel_response_timeout_s_ = declare_parameter<double>("cancel_response_timeout_s", 0.5);
    watchdog_rate_hz_ = declare_parameter<double>("watchdog_rate_hz", 50.0);
    diagnostic_period_s_ = declare_parameter<double>("diagnostic_period_s", 0.1);

    if (
      action_name_.empty() || heartbeat_topic_.empty() || heartbeat_timeout_s_ <= 0.0 ||
      cancel_response_timeout_s_ <= 0.0 || watchdog_rate_hz_ <= 0.0 ||
      diagnostic_period_s_ <= 0.0) {
      throw std::invalid_argument(
        "action_name and heartbeat_topic must be nonempty; timeout/rate parameters must be "
        "positive");
    }

    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, action_name_);
    heartbeat_sub_ = create_subscription<std_msgs::msg::Bool>(
      heartbeat_topic_, rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::Bool::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        core_.heartbeat(message->data, std::chrono::steady_clock::now());
      });
    resolved_heartbeat_topic_ = heartbeat_sub_->get_topic_name();

    diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this, diagnostic_period_s_);
    diagnostics_->setHardwareID("none");
    diagnostics_->add(
      "liveness", std::bind(&JtcGuardNode::produce_diagnostics, this, std::placeholders::_1));

    watchdog_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / watchdog_rate_hz_),
      std::bind(&JtcGuardNode::watchdog_tick, this));
  }

private:
  void watchdog_tick()
  {
    const auto now = std::chrono::steady_clock::now();
    bool request_cancel = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (
        core_.cancel_response_expired(
          now, std::chrono::duration<double>(cancel_response_timeout_s_))) {
        core_.cancel_failed(GuardFault::kCancelResponseTimeout);
      }
      request_cancel = core_.tick(now, std::chrono::duration<double>(heartbeat_timeout_s_));
    }

    if (!request_cancel) {
      return;
    }
    if (!action_client_->action_server_is_ready()) {
      std::lock_guard<std::mutex> lock(mutex_);
      core_.cancel_failed(GuardFault::kActionServerUnavailable);
      return;
    }

    try {
      action_client_->async_cancel_all_goals([this](const CancelResponse::SharedPtr response) {
        const bool accepted =
          response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE;
        std::lock_guard<std::mutex> lock(mutex_);
        core_.cancel_result(accepted, response->goals_canceling.size());
      });
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "Failed to request JTC cancel: %s", exception.what());
      std::lock_guard<std::mutex> lock(mutex_);
      core_.cancel_failed(GuardFault::kCancelRejected);
    }
  }

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    GuardState state;
    GuardFault fault;
    uint64_t sequence;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state = core_.state();
      fault = core_.fault();
      sequence = core_.sequence();
    }

    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    if (
      fault == GuardFault::kHeartbeatTimeout || fault == GuardFault::kCancelAccepted ||
      fault == GuardFault::kCancelNoActiveGoal) {
      level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    } else if (fault != GuardFault::kNone) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }

    status.summary(level, fault_name(fault));
    status.add("guard_state", state_name(state));
    status.add("fault_code", fault_name(fault));
    status.add("action", action_name(state, fault));
    status.add("fault_sequence", sequence);
    status.add("jtc_action", action_name_);
    status.add("heartbeat_topic", resolved_heartbeat_topic_);
    status.add("heartbeat_timeout_ms", heartbeat_timeout_s_ * 1000.0);
  }

  std::string action_name_;
  std::string heartbeat_topic_;
  std::string resolved_heartbeat_topic_;
  double heartbeat_timeout_s_{0.5};
  double cancel_response_timeout_s_{0.5};
  double watchdog_rate_hz_{50.0};
  double diagnostic_period_s_{0.1};
  std::mutex mutex_;
  GuardCore core_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
};

}  // namespace joint_trajectory_controller_guard

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<joint_trajectory_controller_guard::JtcGuardNode>());
  rclcpp::shutdown();
  return 0;
}
