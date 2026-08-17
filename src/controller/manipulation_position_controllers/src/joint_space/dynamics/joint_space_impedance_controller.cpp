// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include "manipulation_position_controllers/joint_space/dynamics/joint_space_impedance_controller.hpp"

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

namespace manipulation_position_controllers::joint_space::dynamics
{

namespace
{

common::ManipulationFault fault_from_read_result(ReferenceReadResult result)
{
  switch (result) {
    case ReferenceReadResult::kZeroStamp:
      return common::ManipulationFault::kZeroStampReference;
    case ReferenceReadResult::kNonfinite:
      return common::ManipulationFault::kNonfiniteReference;
    case ReferenceReadResult::kOutOfBounds:
      return common::ManipulationFault::kOutOfBoundsReference;
    default:
      return common::ManipulationFault::kInvalidReference;
  }
}

}  // namespace

controller_interface::InterfaceConfiguration
JointSpaceImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointSpaceImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

CallbackReturn JointSpaceImpedanceController::on_init()
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

  // Impedance (industry: k_gains / d_gains / k_alpha)
  auto_declare<std::vector<double>>("kp_stiffness", {});
  auto_declare<std::vector<double>>("kd_damping", {});
  auto_declare<std::vector<double>>("max_torques", {});
  auto_declare<std::vector<double>>("max_torque_rates", {});
  auto_declare<double>("velocity_filter_alpha", 0.99);
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointSpaceImpedanceController::load_parameters()
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

  kp_stiffness_ = get_node()->get_parameter("kp_stiffness").as_double_array();
  kd_damping_ = get_node()->get_parameter("kd_damping").as_double_array();
  max_torques_ = get_node()->get_parameter("max_torques").as_double_array();
  max_torque_rates_ = get_node()->get_parameter("max_torque_rates").as_double_array();
  velocity_filter_alpha_ = get_node()->get_parameter("velocity_filter_alpha").as_double();
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointSpaceImpedanceController::validate_configuration() const
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
  if (input_qos_depth_ <= 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "input_qos_depth must be positive.");
    return CallbackReturn::ERROR;
  }
  if (max_reference_points_ <= 0 ||
      static_cast<size_t>(max_reference_points_) > joint_space::kMaxReferencePoints)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "trajectory_behavior.max_points must be in [1, %zu].",
      joint_space::kMaxReferencePoints);
    return CallbackReturn::ERROR;
  }
  if (!common::is_unit_interval_finite(ema_alpha_)) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "trajectory_behavior.ema_alpha must be finite and in [0, 1].");
    return CallbackReturn::ERROR;
  }
  if (!common::is_positive_finite(max_velocity_rad_s_) ||
      !common::is_positive_finite(max_acceleration_rad_s2_) ||
      !common::is_positive_finite(stale_timeout_s_) ||
      !common::is_positive_finite(status_rate_hz_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "reference velocity/acceleration, stale_timeout_s, and status_rate_hz must be finite and positive.");
    return CallbackReturn::ERROR;
  }
  if (reference_behavior_mode_ == "ruckig" &&
      (!common::is_positive_finite(max_jerk_rad_s3_) ||
       !common::is_positive_finite(ruckig_control_cycle_s_)))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "reference jerk and ruckig_control_cycle_s must be finite and positive in ruckig mode.");
    return CallbackReturn::ERROR;
  }
  // Require explicit gains (franka_follower / fr3_motion pattern).
  if (kp_stiffness_.size() != joint_names_.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "kp_stiffness must be set and match joints size (%zu), got %zu.",
      joint_names_.size(), kp_stiffness_.size());
    return CallbackReturn::ERROR;
  }
  if (kd_damping_.size() != joint_names_.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "kd_damping must be set and match joints size (%zu), got %zu.",
      joint_names_.size(), kd_damping_.size());
    return CallbackReturn::ERROR;
  }
  if (max_torques_.size() != joint_names_.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "max_torques must be set and match joints size (%zu), got %zu.",
      joint_names_.size(), max_torques_.size());
    return CallbackReturn::ERROR;
  }
  if (!max_torque_rates_.empty() && max_torque_rates_.size() != joint_names_.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "max_torque_rates size must match joints size (%zu), got %zu.",
      joint_names_.size(), max_torque_rates_.size());
    return CallbackReturn::ERROR;
  }
  if (!common::all_nonnegative_finite(kp_stiffness_) ||
      !common::all_nonnegative_finite(kd_damping_) ||
      !common::all_positive_finite(max_torques_) ||
      (!max_torque_rates_.empty() && !common::all_positive_finite(max_torque_rates_)))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "kp_stiffness/kd_damping must be finite and nonnegative; torque limits/rates must be finite and positive.");
    return CallbackReturn::ERROR;
  }
  if (!common::is_unit_interval_finite(velocity_filter_alpha_)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "velocity_filter_alpha must be in [0, 1] (measurement weight; 1.0 = no filter), got %.3f.",
      velocity_filter_alpha_);
    return CallbackReturn::ERROR;
  }
  if (!common::validate_limit_vectors(joint_names_, lower_limits_, upper_limits_)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "lower_limits/upper_limits must be finite (or +/-inf), monotonic, and match joints size.");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

void JointSpaceImpedanceController::initialize_runtime_state()
{
  common::fill_default_limits(joint_names_, lower_limits_, upper_limits_);

  const auto joint_count = joint_names_.size();
  raw_target_.assign(joint_count, 0.0);
  filtered_target_.assign(joint_count, 0.0);
  position_target_.assign(joint_count, 0.0);
  velocity_target_.assign(joint_count, 0.0);
  dq_filtered_.assign(joint_count, 0.0);
  prev_position_target_.assign(joint_count, 0.0);
  effort_command_.assign(joint_count, 0.0);
  active_reference_ = joint_space::JointReference{};
  active_reference_size_ = 0;
  last_preemption_jump_norm_ = 0.0;
  zero_stamp_fallback_count_ = 0;
  reference_overwrite_count_ = 0;
  coalesced_update_count_ = 0;
  stale_hold_count_ = 0;
  stale_hold_active_ = false;

  if (max_torque_rates_.empty()) {
    // Conservative default: FCI-scale rate room without being unbounded.
    max_torque_rates_.assign(joint_count, 1000.0);
  }

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

void JointSpaceImpedanceController::reset_activation_state()
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
  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0.0);
  std::fill(effort_command_.begin(), effort_command_.end(), 0.0);
}

CallbackReturn JointSpaceImpedanceController::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (load_parameters() != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (validate_configuration() != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto update_rate = get_update_rate();
  control_cycle_s_ = (update_rate > 0)
    ? (1.0 / static_cast<double>(update_rate))
    : ruckig_control_cycle_s_;

  initialize_runtime_state();

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
  realtime_command_state_pub_->msg_.velocity.resize(joint_names_.size(), 0.0);
  realtime_command_state_pub_->msg_.effort.resize(joint_names_.size(), 0.0);

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

  publish_decimation_ = std::max(
    size_t{1},
    static_cast<size_t>(
      (update_rate > 0 ? static_cast<double>(update_rate) : 500.0) /
      std::max(status_rate_hz_, 1.0)));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "JointSpaceImpedanceController configured: input_topic=%s, mode=%s, "
    "velocity_filter_alpha=%.3f, joints=%zu",
    input_topic_.c_str(), reference_behavior_mode_.c_str(),
    velocity_filter_alpha_, joint_names_.size());

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointSpaceImpedanceController::on_activate(
  const rclcpp_lifecycle::State &)
{
  const auto joint_count = joint_names_.size();
  if (state_interfaces_.size() < 2 * joint_count || command_interfaces_.size() < joint_count) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Not enough interfaces (expected >= %zu state and %zu command).",
      2 * joint_count, joint_count);
    return CallbackReturn::ERROR;
  }

  for (size_t index = 0; index < joint_count; ++index) {
    const auto position = state_interfaces_[2 * index].get_optional();
    if (!position.has_value() || !std::isfinite(position.value())) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Invalid position state for joint '%s'.",
        joint_names_[index].c_str());
      return CallbackReturn::ERROR;
    }

    // Seed references at measured q (fr3_motion / official impedance pattern).
    raw_target_[index] = position.value();
    filtered_target_[index] = position.value();
    position_target_[index] = position.value();
    prev_position_target_[index] = position.value();
    velocity_target_[index] = 0.0;
    dq_filtered_[index] = 0.0;
    effort_command_[index] = 0.0;
    (void)command_interfaces_[index].set_value(0.0);
  }

  if (reference_behavior_mode_ == "ruckig" && ruckig_adapter_) {
    ruckig_adapter_->reset();
  }

  reset_activation_state();
  diagnostics_.set_idle();

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointSpaceImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  target_initialized_ = false;
  reference_buffer_.clear();
  std::fill(effort_command_.begin(), effort_command_.end(), 0.0);
  for (auto & command_interface : command_interfaces_) {
    (void)command_interface.set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

void JointSpaceImpedanceController::reference_callback(
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
  const auto result = read_reference_message_result(*msg, config, reference);
  if (result != ReferenceReadResult::kSuccess) {
    reference_buffer_.clear();
    diagnostics_.request_hold(fault_from_read_result(result));
    return;
  }

  reference.sequence = ++reference_sequence_;
  reference_buffer_.set(reference);
}

void JointSpaceImpedanceController::patch_partial_target()
{
  for (size_t index = 0; index < raw_target_.size(); ++index) {
    if (!std::isfinite(raw_target_[index])) {
      raw_target_[index] = position_target_[index];
    }
  }
}

void JointSpaceImpedanceController::write_effort_commands()
{
  for (size_t index = 0; index < effort_command_.size(); ++index) {
    (void)command_interfaces_[index].set_value(effort_command_[index]);
  }
}

void JointSpaceImpedanceController::hold_measured_as_reference()
{
  // Soft hold: drive position/velocity error to zero so residual τ is only
  // damping of measured motion (safer than freezing a non-zero effort).
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const double q_meas = state_interfaces_[2 * i].get_optional().value_or(position_target_[i]);
    position_target_[i] = q_meas;
    prev_position_target_[i] = q_meas;
    velocity_target_[i] = 0.0;
    raw_target_[i] = q_meas;
    filtered_target_[i] = q_meas;
  }
}

void JointSpaceImpedanceController::publish_command_state(const rclcpp::Time & time)
{
  if (!realtime_command_state_pub_ || ++publish_counter_ < publish_decimation_) {
    return;
  }
  publish_counter_ = 0;

  if (realtime_command_state_pub_->trylock()) {
    auto & joint_state = realtime_command_state_pub_->msg_;
    joint_state.header.stamp = time;
    for (size_t index = 0; index < position_target_.size(); ++index) {
      joint_state.position[index] = position_target_[index];
      joint_state.velocity[index] = velocity_target_[index];
      joint_state.effort[index] = effort_command_[index];
    }
    realtime_command_state_pub_->unlockAndPublish();
  }

  if (realtime_status_pub_ && realtime_status_pub_->trylock()) {
    auto & status = realtime_status_pub_->msg_;
    status.data[common::kStatusActiveReferenceSize] = static_cast<double>(active_reference_size_);
    status.data[common::kStatusCommandAge] = (time - last_reference_time_).seconds();
    status.data[common::kStatusLimitFactor] = last_limit_factor_;
    status.data[common::kStatusInitialized] = target_initialized_ ? 1.0 : 0.0;
    status.data[common::kStatusZeroStampFallbackCount] =
      static_cast<double>(zero_stamp_fallback_count_);
    status.data[common::kStatusReferenceOverwriteCount] =
      static_cast<double>(reference_overwrite_count_);
    status.data[common::kStatusCoalescedUpdateCount] =
      static_cast<double>(coalesced_update_count_);
    status.data[common::kStatusStaleHoldCount] = static_cast<double>(stale_hold_count_);
    status.data[common::kStatusPreemptionJumpNorm] = last_preemption_jump_norm_;
    realtime_status_pub_->unlockAndPublish();
  }
}

controller_interface::return_type JointSpaceImpedanceController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto position = state_interfaces_[2 * i].get_optional();
    const auto velocity = state_interfaces_[2 * i + 1].get_optional();
    if (!position.has_value() || !std::isfinite(position.value()) ||
        !velocity.has_value() || !std::isfinite(velocity.value()))
    {
      diagnostics_.enter_error(common::ManipulationFault::kInvalidMeasuredState);
      std::fill(effort_command_.begin(), effort_command_.end(), 0.0);
      write_effort_commands();
      return controller_interface::return_type::ERROR;
    }
  }

  double dtau = period.seconds();
  if (dtau <= 0.0) {
    write_effort_commands();
    publish_command_state(time);
    return controller_interface::return_type::OK;
  }
  dtau = std::clamp(dtau, 1.0e-4, 0.1);

  const auto requested_fault = diagnostics_.consume_hold_request();
  if (requested_fault != common::ManipulationFault::kNone) {
    active_reference_size_ = 0;
    target_initialized_ = false;
    stale_hold_active_ = true;
    hold_measured_as_reference();
    if (ruckig_adapter_) {
      ruckig_adapter_->reset();
    }
  }

  const auto reference = reference_buffer_.try_get();
  if (reference.has_value() && reference->sequence != last_sequence_) {
    active_reference_ = reference.value();
    active_reference_size_ =
      std::min(active_reference_.size, static_cast<size_t>(max_reference_points_));
    last_reference_time_ = time;
    last_sequence_ = active_reference_.sequence;
    stale_hold_active_ = false;
    diagnostics_.set_tracking();

    if (!target_initialized_) {
      // Do NOT snap q_ref to raw target (avoids Kp·Δq spike). Keep measured
      // seed; Ruckig/EMA will chase the new raw_target_ from here.
      (void)joint_space::sample_reference(
        active_reference_, active_reference_size_, joint_names_.size(), time.seconds(),
        raw_target_);
      patch_partial_target();
      target_initialized_ = true;
    }
  }

  if (joint_space::sample_reference(
      active_reference_, active_reference_size_, joint_names_.size(), time.seconds(), raw_target_))
  {
    patch_partial_target();
  }

  const bool was_stale_hold = stale_hold_active_;
  if (common::update_stale_hold_state(
      time, last_reference_time_, stale_timeout_s_, stale_hold_active_, stale_hold_count_))
  {
    hold_measured_as_reference();
    if (!was_stale_hold) {
      diagnostics_.enter_hold(
        common::ManipulationFault::kReferenceTimeout,
        (time - last_reference_time_).seconds(),
        stale_timeout_s_);
    }
    if (reference_behavior_mode_ == "ruckig" && ruckig_adapter_) {
      ruckig_adapter_->reset();
    }
  }

  // --- Reference shaping ---
  if (reference_behavior_mode_ == "ruckig") {
    if (ruckig_adapter_ && dtau > control_cycle_s_) {
      ruckig_adapter_->extrapolate_constant_accel(dtau - control_cycle_s_);
    }
    ruckig_target_ = raw_target_;
    if (ruckig_adapter_) {
      (void)ruckig_adapter_->update(
        position_target_, ruckig_target_, position_target_, velocity_target_);
    }
  } else {
    // Limiter / EMA: match industry impedance followers — track position only.
    // Do not finite-difference q_ref at 1 kHz (one-tick Δq/dt spikes fight Kd).
    joint_space::apply_ema(raw_target_, ema_alpha_, position_target_);
    std::fill(velocity_target_.begin(), velocity_target_.end(), 0.0);
  }

  // --- Impedance law ---
  // τ = Kp (q_ref − q) + Kd (q̇_ref − dq_filt)
  // velocity_filter_alpha_ = measurement weight: (1-α)·prev + α·meas
  // (1.0 = no filter; smaller α = heavier smoothing). Matches common Franka
  // example k_alpha usage where 0.99 ≈ near pass-through.
  const size_t joint_count = joint_names_.size();
  const double alpha = velocity_filter_alpha_;
  for (size_t i = 0; i < joint_count; ++i) {
    const double q_meas = state_interfaces_[2 * i].get_optional().value_or(position_target_[i]);
    const double qdot_meas = state_interfaces_[2 * i + 1].get_optional().value_or(0.0);
    dq_filtered_[i] = (1.0 - alpha) * dq_filtered_[i] + alpha * qdot_meas;

    const double pos_err = position_target_[i] - q_meas;
    const double vel_err = velocity_target_[i] - dq_filtered_[i];
    const double raw_tau = kp_stiffness_[i] * pos_err + kd_damping_[i] * vel_err;

    const double max_tau_change = max_torque_rates_[i] * dtau;
    double tau = std::clamp(
      raw_tau, effort_command_[i] - max_tau_change, effort_command_[i] + max_tau_change);
    tau = std::clamp(tau, -max_torques_[i], max_torques_[i]);

    effort_command_[i] = tau;
  }

  write_effort_commands();
  publish_command_state(time);
  return controller_interface::return_type::OK;
}

}  // namespace manipulation_position_controllers::joint_space::dynamics

PLUGINLIB_EXPORT_CLASS(
  manipulation_position_controllers::joint_space::dynamics::JointSpaceImpedanceController,
  controller_interface::ControllerInterface)
