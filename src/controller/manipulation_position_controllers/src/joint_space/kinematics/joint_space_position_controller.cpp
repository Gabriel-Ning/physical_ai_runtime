// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include "manipulation_position_controllers/joint_space/kinematics/joint_space_position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "manipulation_position_controllers/common/limits.hpp"
#include "manipulation_position_controllers/common/parameter_validation.hpp"
#include "manipulation_position_controllers/common/status.hpp"
#include "manipulation_position_controllers/common/stale_guard.hpp"
#include "manipulation_position_controllers/joint_space/kinematics/ema_filter.hpp"
#include "manipulation_position_controllers/joint_space/kinematics/reference_message_reader.hpp"
#include "manipulation_position_controllers/joint_space/kinematics/trajectory_sampler.hpp"

namespace manipulation_position_controllers
{

namespace
{

common::ManipulationFault fault_from_read_result(joint_space::ReferenceReadResult result)
{
  switch (result) {
    case joint_space::ReferenceReadResult::kZeroStamp:
      return common::ManipulationFault::kZeroStampReference;
    case joint_space::ReferenceReadResult::kNonfinite:
      return common::ManipulationFault::kNonfiniteReference;
    case joint_space::ReferenceReadResult::kOutOfBounds:
      return common::ManipulationFault::kOutOfBoundsReference;
    default:
      return common::ManipulationFault::kInvalidReference;
  }
}

}  // namespace

controller_interface::InterfaceConfiguration
JointSpacePositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointSpacePositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

CallbackReturn JointSpacePositionController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<std::string>("input_topic", "/execution/joint_reference");
  auto_declare<int64_t>("input_qos_depth", 10);
  auto_declare<std::string>("trajectory_behavior.mode", "limiter");
  auto_declare<int64_t>("trajectory_behavior.max_points", 32);
  auto_declare<double>("trajectory_behavior.ema_alpha", 0.2);
  auto_declare<double>("trajectory_behavior.max_velocity_rad_s", 1.5);
  auto_declare<double>("trajectory_behavior.max_acceleration_rad_s2", 3.0);
  auto_declare<double>("trajectory_behavior.max_jerk_rad_s3", 10.0);
  auto_declare<double>("trajectory_behavior.ruckig_control_cycle_s", 0.002);
  auto_declare<double>("trajectory_behavior.stale_timeout_s", 0.5);
  auto_declare<double>("status_rate_hz", 10.0);
  auto_declare<bool>("reject_out_of_bounds_targets", false);
  auto_declare<bool>("reject_zero_stamped_references", true);
  auto_declare<bool>("allow_partial_joint_references", true);
  auto_declare<std::vector<double>>("lower_limits", {});
  auto_declare<std::vector<double>>("upper_limits", {});
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointSpacePositionController::load_parameters()
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  input_topic_ = get_node()->get_parameter("input_topic").as_string();
  input_qos_depth_ = get_node()->get_parameter("input_qos_depth").as_int();
  reference_behavior_mode_ = get_node()->get_parameter("trajectory_behavior.mode").as_string();
  max_reference_points_ = get_node()->get_parameter("trajectory_behavior.max_points").as_int();
  ema_alpha_ = get_node()->get_parameter("trajectory_behavior.ema_alpha").as_double();
  max_velocity_rad_s_ =
    get_node()->get_parameter("trajectory_behavior.max_velocity_rad_s").as_double();
  max_acceleration_rad_s2_ =
    get_node()->get_parameter("trajectory_behavior.max_acceleration_rad_s2").as_double();
  max_jerk_rad_s3_ =
    get_node()->get_parameter("trajectory_behavior.max_jerk_rad_s3").as_double();
  ruckig_control_cycle_s_ =
    get_node()->get_parameter("trajectory_behavior.ruckig_control_cycle_s").as_double();
  stale_timeout_s_ =
    get_node()->get_parameter("trajectory_behavior.stale_timeout_s").as_double();
  status_rate_hz_ = get_node()->get_parameter("status_rate_hz").as_double();
  reject_out_of_bounds_targets_ =
    get_node()->get_parameter("reject_out_of_bounds_targets").as_bool();
  reject_zero_stamped_references_ =
    get_node()->get_parameter("reject_zero_stamped_references").as_bool();
  allow_partial_joint_references_ =
    get_node()->get_parameter("allow_partial_joint_references").as_bool();
  lower_limits_ = get_node()->get_parameter("lower_limits").as_double_array();
  upper_limits_ = get_node()->get_parameter("upper_limits").as_double_array();
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointSpacePositionController::validate_configuration() const
{
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' must not be empty.");
    return CallbackReturn::ERROR;
  }
  if (!common::all_unique_nonempty(joint_names_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' must contain unique, nonempty names.");
    return CallbackReturn::ERROR;
  }
  if (joint_names_.size() > joint_space::kMaxJoints) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "This controller supports at most %zu joints.",
      joint_space::kMaxJoints);
    return CallbackReturn::ERROR;
  }
  if (reference_behavior_mode_ != "limiter" &&
      reference_behavior_mode_ != "ruckig") {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Unsupported trajectory_behavior.mode '%s'. Allowed: 'limiter', 'ruckig'.",
      reference_behavior_mode_.c_str());
    return CallbackReturn::ERROR;
  }
  if (input_topic_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "input_topic must not be empty.");
    return CallbackReturn::ERROR;
  }
  if (max_reference_points_ <= 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "trajectory_behavior.max_points must be positive.");
    return CallbackReturn::ERROR;
  }
  if (static_cast<size_t>(max_reference_points_) > joint_space::kMaxReferencePoints) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "trajectory_behavior.max_points must be <= %zu.",
      joint_space::kMaxReferencePoints);
    return CallbackReturn::ERROR;
  }
  if (!common::is_unit_interval_finite(ema_alpha_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "trajectory_behavior.ema_alpha must be in [0, 1].");
    return CallbackReturn::ERROR;
  }
  if (!common::is_positive_finite(max_velocity_rad_s_) ||
      !common::is_positive_finite(max_acceleration_rad_s2_)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "trajectory_behavior.max_velocity_rad_s and "
      "trajectory_behavior.max_acceleration_rad_s2 must be positive.");
    return CallbackReturn::ERROR;
  }
  if (reference_behavior_mode_ == "ruckig" &&
      !common::is_positive_finite(max_jerk_rad_s3_)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "trajectory_behavior.max_jerk_rad_s3 must be positive for ruckig mode.");
    return CallbackReturn::ERROR;
  }
  if (reference_behavior_mode_ == "ruckig" &&
      !common::is_positive_finite(ruckig_control_cycle_s_)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "trajectory_behavior.ruckig_control_cycle_s must be positive for ruckig mode.");
    return CallbackReturn::ERROR;
  }
  if (!common::validate_limit_vectors(joint_names_, lower_limits_, upper_limits_)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "lower_limits/upper_limits must be finite (or +/-inf), monotonic, and match joints size.");
    return CallbackReturn::ERROR;
  }
  if (!common::is_positive_finite(status_rate_hz_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "status_rate_hz must be positive.");
    return CallbackReturn::ERROR;
  }
  if (input_qos_depth_ <= 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "input_qos_depth must be positive.");
    return CallbackReturn::ERROR;
  }
  if (!common::is_positive_finite(stale_timeout_s_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "stale_timeout_s must be finite and positive.");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

void JointSpacePositionController::configure_limiters()
{
  limiters_.clear();
  limiters_.reserve(joint_names_.size());
  for (size_t index = 0; index < joint_names_.size(); ++index) {
    limiters_.emplace_back(
      lower_limits_[index],
      upper_limits_[index],
      -max_velocity_rad_s_,
      max_velocity_rad_s_,
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      -max_acceleration_rad_s2_,
      max_acceleration_rad_s2_);
  }
}

void JointSpacePositionController::initialize_runtime_state()
{
  common::fill_default_limits(joint_names_, lower_limits_, upper_limits_);

  const auto joint_count = joint_names_.size();
  raw_target_.assign(joint_count, 0.0);
  filtered_target_.assign(joint_count, 0.0);
  command_.assign(joint_count, 0.0);
  previous_command_.assign(joint_count, 0.0);
  ruckig_target_.assign(joint_count, 0.0);
  active_reference_ = joint_space::JointReference{};
  active_reference_size_ = 0;
  last_preemption_jump_norm_ = 0.0;
  zero_stamp_fallback_count_ = 0;
  reference_overwrite_count_ = 0;
  coalesced_update_count_ = 0;
  stale_hold_count_ = 0;
  stale_hold_active_ = false;
  configure_limiters();
  ruckig_adapter_.reset();
  if (reference_behavior_mode_ == "ruckig") {
    joint_space::RuckigConfig ruckig_config;
    ruckig_config.max_velocity_rad_s = max_velocity_rad_s_;
    ruckig_config.max_acceleration_rad_s2 = max_acceleration_rad_s2_;
    ruckig_config.max_jerk_rad_s3 = max_jerk_rad_s3_;
    ruckig_adapter_ = std::make_unique<joint_space::RuckigAdapter>();
    ruckig_adapter_->configure(joint_count, control_cycle_s_, ruckig_config);
  }
}

void JointSpacePositionController::reset_activation_state()
{
  target_initialized_ = false;
  reference_sequence_ = 0;
  last_sequence_ = 0;
  reference_buffer_.clear();
  active_reference_ = joint_space::JointReference{};
  active_reference_size_ = 0;
  last_limit_factor_ = 1.0;
  last_preemption_jump_norm_ = 0.0;
  zero_stamp_fallback_count_ = 0;
  reference_overwrite_count_ = 0;
  coalesced_update_count_ = 0;
  stale_hold_count_ = 0;
  stale_hold_active_ = false;
  publish_counter_ = 0;
  last_reference_time_ = get_node()->now();
}

CallbackReturn JointSpacePositionController::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (load_parameters() != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (validate_configuration() != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Drive Ruckig from the controller's real update rate instead of the
  // hardcoded ruckig_control_cycle_s parameter. get_update_rate() reflects the
  // rate the controller manager actually ticks this controller at; the
  // parameter remains only as a fallback when the rate is unknown (0).
  const auto update_rate = get_update_rate();
  control_cycle_s_ = (update_rate > 0)
    ? (1.0 / static_cast<double>(update_rate))
    : ruckig_control_cycle_s_;
  if (update_rate > 0 &&
      std::abs(control_cycle_s_ - ruckig_control_cycle_s_) > 1e-6) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Using controller update rate %u Hz (dt=%.5f s) for Ruckig instead of "
      "trajectory_behavior.ruckig_control_cycle_s=%.5f s.",
      update_rate, control_cycle_s_, ruckig_control_cycle_s_);
  }

  initialize_runtime_state();

  // best_effort() is a deliberate choice for a streaming realtime reference
  // input (drop late samples rather than block on reliable retransmission);
  // only the queue depth is exposed as a parameter.
  input_sub_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    input_topic_,
    rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(input_qos_depth_))).best_effort(),
    [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
      reference_callback(msg);
    });

  command_state_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    "~/commanded_joint_state", rclcpp::SystemDefaultsQoS());
  realtime_command_state_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
    command_state_pub_);
  realtime_command_state_pub_->msg_.name = joint_names_;
  realtime_command_state_pub_->msg_.position.resize(joint_names_.size(), 0.0);

  status_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/status", rclcpp::SystemDefaultsQoS());
  realtime_status_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
    status_pub_);
  realtime_status_pub_->msg_.data.resize(common::kStatusFieldCount, 0.0);

  diagnostics_.configure(
    get_node(),
    "safety",
    1.0 / status_rate_hz_);

  // Decimate ~/commanded_joint_state and ~/status from status_rate_hz_ and
  // the controller's real update rate, instead of the previous fixed
  // publish_decimation_ = 10 with no parameter path (that made the effective
  // publish rate an unconfigurable, update-rate-dependent side effect, e.g.
  // 50 Hz at a 500 Hz loop, silently different at any other loop rate).
  // Mirrors TaskSpaceKinematicPositionController::status_decimation_.
  publish_decimation_ = std::max(
    size_t{1},
    static_cast<size_t>(
      (update_rate > 0 ? static_cast<double>(update_rate) : 500.0) /
      std::max(status_rate_hz_, 1.0)));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "JointSpacePositionController configured: input_topic=%s, mode=%s, max_points=%ld, "
    "ema_alpha=%.3f, max_velocity=%.3f rad/s, max_acceleration=%.3f rad/s^2, "
    "max_jerk=%.3f rad/s^3, ruckig_dt=%.4f s",
    input_topic_.c_str(), reference_behavior_mode_.c_str(), max_reference_points_, ema_alpha_,
    max_velocity_rad_s_, max_acceleration_rad_s2_, max_jerk_rad_s3_, control_cycle_s_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointSpacePositionController::on_activate(
  const rclcpp_lifecycle::State &)
{
  const auto joint_count = joint_names_.size();
  if (state_interfaces_.size() < joint_count || command_interfaces_.size() < joint_count) {
    RCLCPP_ERROR(get_node()->get_logger(), "Not enough state/command interfaces to activate.");
    return CallbackReturn::ERROR;
  }

  for (size_t index = 0; index < joint_count; ++index) {
    const auto position = state_interfaces_[index].get_optional();
    if (!position.has_value() || !std::isfinite(position.value())) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Invalid initial position state for joint '%s'.",
        joint_names_[index].c_str());
      return CallbackReturn::ERROR;
    }

    raw_target_[index] = position.value();
    filtered_target_[index] = position.value();
    command_[index] = position.value();
    previous_command_[index] = position.value();
    (void)command_interfaces_[index].set_value(position.value());
  }

  if (reference_behavior_mode_ == "ruckig" && ruckig_adapter_) {
    ruckig_adapter_->reset();
  }

  reset_activation_state();
  diagnostics_.set_idle();

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointSpacePositionController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  target_initialized_ = false;
  return CallbackReturn::SUCCESS;
}

void JointSpacePositionController::reference_callback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  joint_space::JointReference reference;
  const joint_space::ReferenceMessageReaderConfig config{
    joint_names_,
    lower_limits_,
    upper_limits_,
    static_cast<size_t>(max_reference_points_),
    reject_out_of_bounds_targets_,
    reject_zero_stamped_references_,
    allow_partial_joint_references_,
    *get_node()->get_clock()};
  const auto result = joint_space::read_reference_message_result(*msg, config, reference);
  if (result != joint_space::ReferenceReadResult::kSuccess) {
    reference_buffer_.clear();
    diagnostics_.request_hold(fault_from_read_result(result));
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "Dropping joint reference message with invalid points.");
    return;
  }

  reference.sequence = ++reference_sequence_;
  reference_buffer_.set(reference);
}

bool JointSpacePositionController::hold_measured_position()
{
  for (size_t index = 0; index < joint_names_.size(); ++index) {
    const auto position = state_interfaces_[index].get_optional();
    if (!position.has_value() || !std::isfinite(position.value())) {
      return false;
    }
    raw_target_[index] = position.value();
    filtered_target_[index] = position.value();
    previous_command_[index] = position.value();
    command_[index] = position.value();
    (void)command_interfaces_[index].set_value(position.value());
  }
  if (ruckig_adapter_) {
    ruckig_adapter_->reset();
  }
  return true;
}


void JointSpacePositionController::patch_partial_target()
{
  for (size_t index = 0; index < raw_target_.size(); ++index) {
    if (!std::isfinite(raw_target_[index])) {
      raw_target_[index] = command_[index];
    }
  }
}
void JointSpacePositionController::publish_command_state(const rclcpp::Time & time)
{
  if (!realtime_command_state_pub_ || ++publish_counter_ < publish_decimation_) {
    return;
  }
  publish_counter_ = 0;

  if (!realtime_command_state_pub_->trylock()) {
    return;
  }

  auto & joint_state = realtime_command_state_pub_->msg_;
  joint_state.header.stamp = time;
  for (size_t index = 0; index < command_.size(); ++index) {
    joint_state.position[index] = command_[index];
  }
  realtime_command_state_pub_->unlockAndPublish();

  if (!realtime_status_pub_ || !realtime_status_pub_->trylock()) {
    return;
  }

  auto & status = realtime_status_pub_->msg_;
  status.data[common::kStatusActiveReferenceSize] = static_cast<double>(active_reference_size_);
  status.data[common::kStatusCommandAge] = (time - last_reference_time_).seconds();
  status.data[common::kStatusLimitFactor] = last_limit_factor_;
  status.data[common::kStatusInitialized] = target_initialized_ ? 1.0 : 0.0;
  status.data[common::kStatusZeroStampFallbackCount] = static_cast<double>(zero_stamp_fallback_count_);
  status.data[common::kStatusReferenceOverwriteCount] = static_cast<double>(reference_overwrite_count_);
  status.data[common::kStatusCoalescedUpdateCount] = static_cast<double>(coalesced_update_count_);
  status.data[common::kStatusStaleHoldCount] = static_cast<double>(stale_hold_count_);
  status.data[common::kStatusRemainingReferenceTime] = 0.0;
  if (active_reference_size_ > 0) {
    const size_t last_index = active_reference_size_ - 1;
    status.data[common::kStatusRemainingReferenceTime] =
      active_reference_.times[last_index] - time.seconds();
  }
  status.data[common::kStatusPreemptionJumpNorm] = last_preemption_jump_norm_;
  realtime_status_pub_->unlockAndPublish();
}

controller_interface::return_type JointSpacePositionController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  for (size_t index = 0; index < joint_names_.size(); ++index) {
    const auto position = state_interfaces_[index].get_optional();
    if (!position.has_value() || !std::isfinite(position.value())) {
      diagnostics_.enter_error(common::ManipulationFault::kInvalidMeasuredState);
      for (size_t command_index = 0; command_index < command_.size(); ++command_index) {
        (void)command_interfaces_[command_index].set_value(command_[command_index]);
      }
      return controller_interface::return_type::ERROR;
    }
  }

  const auto requested_fault = diagnostics_.consume_hold_request();
  if (requested_fault != common::ManipulationFault::kNone) {
    active_reference_size_ = 0;
    target_initialized_ = false;
    stale_hold_active_ = true;
    if (!hold_measured_position()) {
      diagnostics_.enter_error(common::ManipulationFault::kInvalidMeasuredState);
      return controller_interface::return_type::ERROR;
    }
    publish_command_state(time);
    return controller_interface::return_type::OK;
  }

  double dtau = period.seconds();
  if (dtau <= 0.0) {
    for (size_t index = 0; index < command_.size(); ++index) {
      (void)command_interfaces_[index].set_value(command_[index]);
    }
    publish_command_state(time);
    return controller_interface::return_type::OK;
  }

  const auto reference = reference_buffer_.try_get();
  if (reference.has_value() && reference->sequence != last_sequence_) {
    const bool had_active_future =
      last_sequence_ != 0 && active_reference_size_ > 0 &&
      time.seconds() < active_reference_.times[active_reference_size_ - 1];
    if (had_active_future) {
      ++reference_overwrite_count_;
    }
    if (last_sequence_ != 0 && reference->sequence > last_sequence_ + 1) {
      coalesced_update_count_ += reference->sequence - last_sequence_ - 1;
    }
    if (reference->used_zero_stamp_fallback) {
      ++zero_stamp_fallback_count_;
    }

    active_reference_ = reference.value();
    active_reference_size_ =
      std::min(active_reference_.size, static_cast<size_t>(max_reference_points_));
    last_reference_time_ = time;
    last_sequence_ = active_reference_.sequence;
    stale_hold_active_ = false;
    diagnostics_.set_tracking();

    if (had_active_future &&
      joint_space::sample_reference(
        active_reference_, active_reference_size_, joint_names_.size(), time.seconds(), raw_target_))
    {
      patch_partial_target();
      double jump_norm_sq = 0.0;
      for (size_t index = 0; index < raw_target_.size(); ++index) {
        const double diff = raw_target_[index] - command_[index];
        jump_norm_sq += diff * diff;
      }
      last_preemption_jump_norm_ = std::sqrt(jump_norm_sq);
    } else {
      last_preemption_jump_norm_ = 0.0;
    }

    if (!target_initialized_) {
      (void)joint_space::sample_reference(
        active_reference_, active_reference_size_, joint_names_.size(), time.seconds(), raw_target_);
      patch_partial_target();
      for (size_t index = 0; index < raw_target_.size(); ++index) {
        filtered_target_[index] = raw_target_[index];
      }
      target_initialized_ = true;
    }
  }

  if (!joint_space::sample_reference(
      active_reference_, active_reference_size_, joint_names_.size(), time.seconds(), raw_target_))
  {
    for (size_t index = 0; index < command_.size(); ++index) {
      (void)command_interfaces_[index].set_value(command_[index]);
    }
    publish_command_state(time);
    return controller_interface::return_type::OK;
  }
  patch_partial_target();

  const bool was_stale_hold = stale_hold_active_;
  if (common::update_stale_hold_state(
      time, last_reference_time_, stale_timeout_s_, stale_hold_active_, stale_hold_count_)) {
    if (!was_stale_hold) {
      if (!hold_measured_position()) {
        diagnostics_.enter_error(common::ManipulationFault::kInvalidMeasuredState);
        return controller_interface::return_type::ERROR;
      }
      diagnostics_.enter_hold(
        common::ManipulationFault::kReferenceTimeout,
        (time - last_reference_time_).seconds(),
        stale_timeout_s_);
    } else {
      for (size_t index = 0; index < command_.size(); ++index) {
        (void)command_interfaces_[index].set_value(command_[index]);
      }
    }
    publish_command_state(time);
    return controller_interface::return_type::OK;
  }

  if (reference_behavior_mode_ == "ruckig") {
    // One fixed-dt Ruckig step per new robot tick; if the tick spanned more
    // than dt_nom, fill the gap with constant-accel extrapolation first.
    if (ruckig_adapter_ && dtau > control_cycle_s_) {
      ruckig_adapter_->extrapolate_constant_accel(dtau - control_cycle_s_);
    }

    ruckig_target_ = raw_target_;
    if (ruckig_adapter_) {
      (void)ruckig_adapter_->update(command_, ruckig_target_, command_);
    }

    for (size_t index = 0; index < command_.size(); ++index) {
      (void)command_interfaces_[index].set_value(command_[index]);
    }
    last_limit_factor_ = 1.0;
  } else {
    last_limit_factor_ = 1.0;
    joint_space::apply_ema(raw_target_, ema_alpha_, filtered_target_);
    for (size_t index = 0; index < command_.size(); ++index) {
      double next_command = filtered_target_[index];
      const double limit_factor =
        limiters_[index].limit(next_command, command_[index], previous_command_[index], dtau);
      last_limit_factor_ = std::min(last_limit_factor_, limit_factor);

      previous_command_[index] = command_[index];
      command_[index] = next_command;
      (void)command_interfaces_[index].set_value(command_[index]);
    }
  }

  publish_command_state(time);
  return controller_interface::return_type::OK;
}

}  // namespace manipulation_position_controllers

PLUGINLIB_EXPORT_CLASS(
  manipulation_position_controllers::JointSpacePositionController,
  controller_interface::ControllerInterface)
