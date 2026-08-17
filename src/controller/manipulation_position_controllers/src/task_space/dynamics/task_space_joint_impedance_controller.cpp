// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include "manipulation_position_controllers/task_space/dynamics/task_space_joint_impedance_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <Eigen/Geometry>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "manipulation_position_controllers/task_space/kinematics/differential_ik/dls_solver.hpp"
#include "manipulation_position_controllers/task_space/kinematics/differential_ik/osqp_solver.hpp"
#include "manipulation_position_controllers/task_space/kinematics/differential_ik/placo_solver.hpp"
#include "manipulation_position_controllers/task_space/kinematics/cartesian_trajectory_reader.hpp"
#include "manipulation_position_controllers/common/limits.hpp"
#include "manipulation_position_controllers/common/parameter_validation.hpp"

namespace manipulation_position_controllers
{

namespace
{

std::array<double, 3> clamp_vector_norm(
  const std::array<double, 3> & input, double max_norm)
{
  auto output = input;
  const double norm = std::sqrt(
    output[0] * output[0] +
    output[1] * output[1] +
    output[2] * output[2]);
  if (norm > max_norm && norm > 1e-12) {
    const double scale = max_norm / norm;
    for (auto & value : output) {
      value *= scale;
    }
  }
  return output;
}

}  // namespace

controller_interface::InterfaceConfiguration
TaskSpaceJointImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : joint_names_) {
    config.names.push_back(name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
TaskSpaceJointImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : joint_names_) {
    config.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

CallbackReturn TaskSpaceJointImpedanceController::on_init()
{
  auto_declare<std::string>("robot_description", "");
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<std::string>("base_frame", "base_link");
  auto_declare<std::string>("tip_frame", "flange_link");
  auto_declare<std::string>("input_topic", "/execution/pose_reference");
  auto_declare<std::string>("twist_topic", "");
  auto_declare<int64_t>("trajectory_behavior.max_points", 64);
  auto_declare<double>("trajectory_behavior.untimed_frame_dt_s", 0.02);
  auto_declare<double>("stale_timeout_s", 0.2);
  auto_declare<double>("twist_stale_timeout_s", 0.2);
  auto_declare<double>("max_joint_velocity_rad_s", 1.0);
  auto_declare<double>("max_linear_velocity_m_s", 0.5);
  auto_declare<double>("max_angular_velocity_rad_s", 1.0);
  auto_declare<double>("status_rate_hz", 10.0);
  auto_declare<double>("velocity_filter_alpha", 0.99);
  auto_declare<bool>("reject_zero_stamped_references", true);

  auto_declare<std::string>("solver.backend", "osqp");
  auto_declare<double>("solver.position_gain", 4.0);
  auto_declare<double>("solver.orientation_gain", 1.0);
  auto_declare<double>("solver.damping", 0.05);
  auto_declare<double>("solver.posture_weight", 1.0e-3);
  auto_declare<double>("solver.manipulability_weight", 0.0);
  auto_declare<double>("solver.kinetic_energy_weight", 0.0);
  auto_declare<std::vector<double>>("solver.joint_motion_weights", {});
  auto_declare<double>("solver.max_joint_acceleration_rad_s2", 0.0);
  auto_declare<int>("solver.max_iterations", 1);
  auto_declare<int>("solver.osqp.max_admm_iterations", 200);
  auto_declare<double>("solver.osqp.cbf_gain", 2.0);
  auto_declare<double>("solver.osqp.slack_penalty", 1.0e4);
  auto_declare<double>("solver.osqp.abs_tolerance", 1.0e-6);
  auto_declare<double>("solver.osqp.rel_tolerance", 1.0e-6);
  auto_declare<double>("solver.osqp.rho", 0.1);

  auto_declare<std::vector<double>>("kp_stiffness", {});
  auto_declare<std::vector<double>>("kd_damping", {});
  auto_declare<std::vector<double>>("max_torques", {});
  auto_declare<std::vector<double>>("max_torque_rates", {});
  auto_declare<std::vector<double>>("lower_limits", {});
  auto_declare<std::vector<double>>("upper_limits", {});
  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceJointImpedanceController::load_parameters()
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  base_frame_ = get_node()->get_parameter("base_frame").as_string();
  tip_frame_ = get_node()->get_parameter("tip_frame").as_string();
  input_topic_ = get_node()->get_parameter("input_topic").as_string();
  twist_topic_ = get_node()->get_parameter("twist_topic").as_string();
  max_pose_points_ =
    get_node()->get_parameter("trajectory_behavior.max_points").as_int();
  untimed_frame_dt_s_ =
    get_node()->get_parameter("trajectory_behavior.untimed_frame_dt_s").as_double();
  stale_timeout_s_ = get_node()->get_parameter("stale_timeout_s").as_double();
  twist_stale_timeout_s_ =
    get_node()->get_parameter("twist_stale_timeout_s").as_double();
  max_joint_velocity_rad_s_ =
    get_node()->get_parameter("max_joint_velocity_rad_s").as_double();
  max_linear_velocity_m_s_ =
    get_node()->get_parameter("max_linear_velocity_m_s").as_double();
  max_angular_velocity_rad_s_ =
    get_node()->get_parameter("max_angular_velocity_rad_s").as_double();
  status_rate_hz_ = get_node()->get_parameter("status_rate_hz").as_double();
  velocity_filter_alpha_ =
    get_node()->get_parameter("velocity_filter_alpha").as_double();
  reject_zero_stamped_references_ =
    get_node()->get_parameter("reject_zero_stamped_references").as_bool();

  solver_config_.backend = get_node()->get_parameter("solver.backend").as_string();
  solver_config_.position_gain =
    get_node()->get_parameter("solver.position_gain").as_double();
  solver_config_.orientation_gain =
    get_node()->get_parameter("solver.orientation_gain").as_double();
  solver_config_.damping = get_node()->get_parameter("solver.damping").as_double();
  solver_config_.posture_weight =
    get_node()->get_parameter("solver.posture_weight").as_double();
  solver_config_.manipulability_weight =
    get_node()->get_parameter("solver.manipulability_weight").as_double();
  solver_config_.kinetic_energy_weight =
    get_node()->get_parameter("solver.kinetic_energy_weight").as_double();
  solver_config_.joint_motion_weights =
    get_node()->get_parameter("solver.joint_motion_weights").as_double_array();
  solver_config_.max_iterations =
    get_node()->get_parameter("solver.max_iterations").as_int();
  solver_config_.max_joint_acceleration_rad_s2 =
    get_node()->get_parameter("solver.max_joint_acceleration_rad_s2").as_double();
  solver_config_.osqp_max_admm_iterations =
    get_node()->get_parameter("solver.osqp.max_admm_iterations").as_int();
  solver_config_.osqp_cbf_gain =
    get_node()->get_parameter("solver.osqp.cbf_gain").as_double();
  solver_config_.osqp_slack_penalty =
    get_node()->get_parameter("solver.osqp.slack_penalty").as_double();
  solver_config_.osqp_abs_tolerance =
    get_node()->get_parameter("solver.osqp.abs_tolerance").as_double();
  solver_config_.osqp_rel_tolerance =
    get_node()->get_parameter("solver.osqp.rel_tolerance").as_double();
  solver_config_.osqp_rho = get_node()->get_parameter("solver.osqp.rho").as_double();
  solver_config_.max_joint_velocity_rad_s = max_joint_velocity_rad_s_;
  solver_config_.max_linear_velocity_m_s = max_linear_velocity_m_s_;
  solver_config_.max_angular_velocity_rad_s = max_angular_velocity_rad_s_;

  kp_stiffness_ = get_node()->get_parameter("kp_stiffness").as_double_array();
  kd_damping_ = get_node()->get_parameter("kd_damping").as_double_array();
  max_torques_ = get_node()->get_parameter("max_torques").as_double_array();
  max_torque_rates_ = get_node()->get_parameter("max_torque_rates").as_double_array();
  lower_limits_ = get_node()->get_parameter("lower_limits").as_double_array();
  upper_limits_ = get_node()->get_parameter("upper_limits").as_double_array();
  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceJointImpedanceController::validate_configuration() const
{
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' must not be empty.");
    return CallbackReturn::ERROR;
  }
  if (!common::all_unique_nonempty(joint_names_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' must contain unique, nonempty names.");
    return CallbackReturn::ERROR;
  }
  if (base_frame_.empty() || tip_frame_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "base_frame and tip_frame must not be empty.");
    return CallbackReturn::ERROR;
  }
  if (kp_stiffness_.size() != joint_names_.size() ||
      kd_damping_.size() != joint_names_.size() ||
      max_torques_.size() != joint_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "kp_stiffness, kd_damping, and max_torques must match joints size (%zu).",
      joint_names_.size());
    return CallbackReturn::ERROR;
  }
  if (!max_torque_rates_.empty() && max_torque_rates_.size() != joint_names_.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "max_torque_rates size must match joints size (%zu).", joint_names_.size());
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
      "velocity_filter_alpha must be in [0, 1] (measurement weight; 1.0 = no filter).");
    return CallbackReturn::ERROR;
  }
  if (!common::is_positive_finite(status_rate_hz_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "status_rate_hz must be positive.");
    return CallbackReturn::ERROR;
  }
  if (!common::validate_limit_vectors(joint_names_, lower_limits_, upper_limits_)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "lower_limits/upper_limits must be empty or match joints size, finite/±inf, and monotonic.");
    return CallbackReturn::ERROR;
  }
  if (!common::is_positive_finite(max_joint_velocity_rad_s_) ||
      !common::is_positive_finite(max_linear_velocity_m_s_) ||
      !common::is_positive_finite(max_angular_velocity_rad_s_) ||
      !common::is_positive_finite(stale_timeout_s_) ||
      !common::is_positive_finite(twist_stale_timeout_s_) ||
      !common::is_positive_finite(untimed_frame_dt_s_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "velocity limits, stale timeouts, and untimed_frame_dt_s must be finite and positive.");
    return CallbackReturn::ERROR;
  }
  if (input_topic_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "input_topic must not be empty.");
    return CallbackReturn::ERROR;
  }
  if (max_pose_points_ <= 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "trajectory_behavior.max_points must be > 0.");
    return CallbackReturn::ERROR;
  }
  if (static_cast<size_t>(max_pose_points_) > task_space::kMaxPoseFrames) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "trajectory_behavior.max_points must be <= %zu.",
      task_space::kMaxPoseFrames);
    return CallbackReturn::ERROR;
  }
  std::string solver_error;
  if (!solver_config_.joint_motion_weights.empty() &&
      solver_config_.joint_motion_weights.size() != joint_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "solver.joint_motion_weights size must match joints size (%zu).",
      joint_names_.size());
    return CallbackReturn::ERROR;
  }
  if (!task_space::validate_solver_config(solver_config_, solver_error)) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s.", solver_error.c_str());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::unique_ptr<task_space::DifferentialIkSolver>
TaskSpaceJointImpedanceController::create_solver()
{
  if (solver_config_.backend == "osqp") {
    return std::make_unique<task_space::OsqpIkSolver>();
  }
  if (solver_config_.backend == "placo") {
    return std::make_unique<task_space::PlacoKinematicSolver>();
  }
  if (solver_config_.backend == "pinocchio_dls") {
    return std::make_unique<task_space::PinocchioDlsSolver>();
  }
  RCLCPP_ERROR(
    get_node()->get_logger(),
    "Unknown solver backend '%s'.",
    solver_config_.backend.c_str());
  return nullptr;
}

CallbackReturn TaskSpaceJointImpedanceController::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (load_parameters() != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (validate_configuration() != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto joint_count = joint_names_.size();
  common::fill_default_limits(joint_names_, lower_limits_, upper_limits_);
  if (max_torque_rates_.empty()) {
    max_torque_rates_.assign(joint_count, 1000.0);
  }

  q_des_.assign(joint_count, 0.0);
  qdot_des_.assign(joint_count, 0.0);
  q_meas_.assign(joint_count, 0.0);
  dq_filtered_.assign(joint_count, 0.0);
  effort_command_.assign(joint_count, 0.0);

  solver_ = create_solver();
  if (!solver_) {
    return CallbackReturn::ERROR;
  }
  if (auto * osqp = dynamic_cast<task_space::OsqpIkSolver *>(solver_.get())) {
    osqp->set_joint_limits(lower_limits_, upper_limits_);
  }

  std::string urdf_xml = get_robot_description();
  if (urdf_xml.empty()) {
    (void)get_node()->get_parameter("robot_description", urdf_xml);
  }
  if (urdf_xml.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No robot_description available.");
    return CallbackReturn::ERROR;
  }
  if (!solver_->configure(
      urdf_xml, joint_names_, base_frame_, tip_frame_, solver_config_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Solver configure failed: %s",
      solver_->last_error().c_str());
    return CallbackReturn::ERROR;
  }

  trajectory_sub_ =
    get_node()->create_subscription<moveit_msgs::msg::CartesianTrajectory>(
    input_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const moveit_msgs::msg::CartesianTrajectory::SharedPtr msg) {
      trajectory_callback(msg);
    });
  if (!twist_topic_.empty()) {
    twist_sub_ =
      get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
      twist_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        twist_callback(msg);
      });
  }

  cmd_state_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    "~/joint_commands", rclcpp::SystemDefaultsQoS());
  realtime_cmd_state_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
      cmd_state_pub_);
  realtime_cmd_state_pub_->msg_.name = joint_names_;
  realtime_cmd_state_pub_->msg_.position.assign(joint_count, 0.0);
  realtime_cmd_state_pub_->msg_.velocity.assign(joint_count, 0.0);
  realtime_cmd_state_pub_->msg_.effort.assign(joint_count, 0.0);

  status_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/status", rclcpp::SystemDefaultsQoS());
  realtime_status_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
      status_pub_);
  realtime_status_pub_->msg_.data.assign(8, 0.0);

  diagnostics_.configure(
    get_node(),
    "safety",
    1.0 / status_rate_hz_);

  const auto update_rate = get_update_rate();
  const double loop_hz = (update_rate > 0u)
    ? static_cast<double>(update_rate)
    : 1000.0;
  status_decimation_ = std::max(
    static_cast<size_t>(1),
    static_cast<size_t>(std::round(loop_hz / std::max(status_rate_hz_, 1.0))));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "TaskSpaceJointImpedanceController configured: base=%s tip=%s joints=%zu "
    "input_topic=%s twist_topic=%s backend=%s",
    base_frame_.c_str(), tip_frame_.c_str(), joint_count,
    input_topic_.c_str(), twist_topic_.c_str(),
    solver_config_.backend.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceJointImpedanceController::on_activate(
  const rclcpp_lifecycle::State &)
{
  const auto joint_count = joint_names_.size();
  if (state_interfaces_.size() < 2 * joint_count || command_interfaces_.size() < joint_count) {
    RCLCPP_ERROR(get_node()->get_logger(), "Not enough state/command interfaces.");
    return CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < joint_count; ++i) {
    const auto pos = state_interfaces_[2 * i].get_optional();
    if (!pos.has_value() || !std::isfinite(pos.value())) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Invalid position state for joint '%s'.",
        joint_names_[i].c_str());
      return CallbackReturn::ERROR;
    }
    const double q = pos.value();
    q_des_[i] = q;
    qdot_des_[i] = 0.0;
    dq_filtered_[i] = 0.0;
    effort_command_[i] = 0.0;
  }
  solver_->reset(q_des_);
  target_initialized_ = false;
  diagnostics_.set_idle();
  stale_hold_active_ = false;
  integrated_twist_pose_valid_ = seed_integrated_twist_pose(q_des_);
  active_input_mode_ = 0;
  previous_active_input_mode_ = 0;
  last_twist_age_s_ = std::numeric_limits<double>::infinity();
  twist_sequence_ = 0;
  twist_frame_reject_count_ = 0;
  write_effort_commands();

  RCLCPP_INFO(
    get_node()->get_logger(),
    "TaskSpaceJointImpedanceController activated with %zu joints.", joint_count);
  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceJointImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (size_t i = 0; i < effort_command_.size(); ++i) {
    effort_command_[i] = 0.0;
  }
  write_effort_commands();
  return CallbackReturn::SUCCESS;
}

double TaskSpaceJointImpedanceController::resolve_stamp_seconds(
  const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec == 0 && stamp.nanosec == 0) {
    zero_stamp_fallback_count_++;
    return get_node()->now().seconds();
  }
  return static_cast<double>(stamp.sec) +
         1.0e-9 * static_cast<double>(stamp.nanosec);
}

void TaskSpaceJointImpedanceController::trajectory_callback(
  const moveit_msgs::msg::CartesianTrajectory::SharedPtr msg)
{
  if (reject_zero_stamped_references_ &&
    msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
  {
    pose_chunk_buffer_.writeFromNonRT(task_space::PoseChunk{});
    twist_buffer_.writeFromNonRT(task_space::TwistCommand{});
    diagnostics_.request_hold(common::ManipulationFault::kZeroStampReference);
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Rejected CartesianTrajectory with zero header.stamp.");
    return;
  }
  if (msg->header.frame_id != base_frame_) {
    frame_reject_count_++;
    pose_chunk_buffer_.writeFromNonRT(task_space::PoseChunk{});
    twist_buffer_.writeFromNonRT(task_space::TwistCommand{});
    diagnostics_.request_hold(common::ManipulationFault::kFrameMismatch);
    return;
  }

  task_space::PoseChunk chunk;
  const task_space::CartesianTrajectoryReaderConfig config{
    resolve_stamp_seconds(msg->header.stamp),
    static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL +
      msg->header.stamp.nanosec,
    static_cast<size_t>(max_pose_points_),
    untimed_frame_dt_s_,
    reject_zero_stamped_references_};
  if (!task_space::read_cartesian_trajectory(*msg, config, chunk)) {
    pose_chunk_buffer_.writeFromNonRT(task_space::PoseChunk{});
    twist_buffer_.writeFromNonRT(task_space::TwistCommand{});
    diagnostics_.request_hold(common::ManipulationFault::kInvalidReference);
    return;
  }

  pose_chunk_buffer_.writeFromNonRT(chunk);
  target_initialized_ = true;
}

void TaskSpaceJointImpedanceController::twist_callback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (reject_zero_stamped_references_ &&
    msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
  {
    pose_chunk_buffer_.writeFromNonRT(task_space::PoseChunk{});
    twist_buffer_.writeFromNonRT(task_space::TwistCommand{});
    diagnostics_.request_hold(common::ManipulationFault::kZeroStampReference);
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Rejected TwistStamped with zero header.stamp.");
    return;
  }
  if (msg->header.frame_id != base_frame_) {
    frame_reject_count_++;
    twist_frame_reject_count_++;
    pose_chunk_buffer_.writeFromNonRT(task_space::PoseChunk{});
    twist_buffer_.writeFromNonRT(task_space::TwistCommand{});
    diagnostics_.request_hold(common::ManipulationFault::kFrameMismatch);
    return;
  }

  task_space::TwistCommand command;
  command.linear = {
    msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z};
  command.angular = {
    msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z};
  if (!task_space::is_valid_vector3(command.linear) ||
      !task_space::is_valid_vector3(command.angular))
  {
    pose_chunk_buffer_.writeFromNonRT(task_space::PoseChunk{});
    twist_buffer_.writeFromNonRT(task_space::TwistCommand{});
    diagnostics_.request_hold(common::ManipulationFault::kNonfiniteReference);
    return;
  }
  command.stamp = resolve_stamp_seconds(msg->header.stamp);
  command.frame_id = msg->header.frame_id;
  command.sequence =
    static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL +
    msg->header.stamp.nanosec;
  command.valid = true;

  twist_buffer_.writeFromNonRT(command);
  twist_sequence_ = command.sequence;
  target_initialized_ = true;
}

bool TaskSpaceJointImpedanceController::seed_integrated_twist_pose(
  const std::vector<double> & q_seed)
{
  if (!solver_ ||
      !solver_->compute_tip_pose(q_seed, integrated_twist_pose_))
  {
    integrated_twist_pose_valid_ = false;
    return false;
  }
  integrated_twist_pose_valid_ = true;
  return true;
}

void TaskSpaceJointImpedanceController::set_integrated_twist_pose(
  const std::array<double, 7> & pose)
{
  integrated_twist_pose_ = pose;
  Eigen::Quaterniond quaternion(
    pose[6], pose[3], pose[4], pose[5]);
  quaternion.normalize();
  integrated_twist_pose_[3] = quaternion.x();
  integrated_twist_pose_[4] = quaternion.y();
  integrated_twist_pose_[5] = quaternion.z();
  integrated_twist_pose_[6] = quaternion.w();
  integrated_twist_pose_valid_ = true;
}

void TaskSpaceJointImpedanceController::integrate_base_frame_twist_target(
  const task_space::TwistCommand & twist, double period_s)
{
  const double dt = std::max(period_s, 1e-9);
  const auto linear =
    clamp_vector_norm(twist.linear, max_linear_velocity_m_s_);
  const auto angular =
    clamp_vector_norm(twist.angular, max_angular_velocity_rad_s_);
  for (size_t i = 0; i < 3; ++i) {
    integrated_twist_pose_[i] += linear[i] * dt;
  }

  Eigen::Quaterniond quaternion(
    integrated_twist_pose_[6],
    integrated_twist_pose_[3],
    integrated_twist_pose_[4],
    integrated_twist_pose_[5]);
  quaternion.normalize();
  const double angular_norm = std::sqrt(
    angular[0] * angular[0] +
    angular[1] * angular[1] +
    angular[2] * angular[2]);
  if (angular_norm > 1e-12) {
    const Eigen::Vector3d axis(
      angular[0] / angular_norm,
      angular[1] / angular_norm,
      angular[2] / angular_norm);
    quaternion =
      Eigen::Quaterniond(Eigen::AngleAxisd(angular_norm * dt, axis)) *
      quaternion;
    quaternion.normalize();
  }
  integrated_twist_pose_[3] = quaternion.x();
  integrated_twist_pose_[4] = quaternion.y();
  integrated_twist_pose_[5] = quaternion.z();
  integrated_twist_pose_[6] = quaternion.w();
}

void TaskSpaceJointImpedanceController::write_effort_commands()
{
  for (size_t i = 0; i < command_interfaces_.size() && i < effort_command_.size(); ++i) {
    (void)command_interfaces_[i].set_value(effort_command_[i]);
  }
}

void TaskSpaceJointImpedanceController::hold_measured_as_reference()
{
  const size_t joint_count = joint_names_.size();
  for (size_t i = 0; i < joint_count; ++i) {
    const double q = state_interfaces_[2 * i].get_optional().value_or(q_des_[i]);
    q_des_[i] = q;
    qdot_des_[i] = 0.0;
  }
}

void TaskSpaceJointImpedanceController::publish_command_state(const rclcpp::Time & time)
{
  if (!realtime_cmd_state_pub_ || ++publish_counter_ < status_decimation_) {
    return;
  }
  publish_counter_ = 0;

  if (realtime_cmd_state_pub_->trylock()) {
    auto & js = realtime_cmd_state_pub_->msg_;
    js.header.stamp = time;
    for (size_t i = 0; i < q_des_.size(); ++i) {
      js.position[i] = q_des_[i];
      js.velocity[i] = qdot_des_[i];
      js.effort[i] = effort_command_[i];
    }
    realtime_cmd_state_pub_->unlockAndPublish();
  }

  if (realtime_status_pub_ && realtime_status_pub_->trylock()) {
    auto & d = realtime_status_pub_->msg_.data;
    d[0] = target_initialized_ ? 1.0 : 0.0;
    d[1] = static_cast<double>(solve_success_count_);
    d[2] = static_cast<double>(solve_fail_count_);
    d[3] = static_cast<double>(stale_hold_count_);
    d[4] = last_task_position_error_norm_;
    d[5] = last_task_orientation_error_norm_;
    d[6] = last_solve_latency_ms_;
    d[7] = stale_hold_active_ ? 1.0 : 0.0;
    realtime_status_pub_->unlockAndPublish();
  }
}

controller_interface::return_type TaskSpaceJointImpedanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
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

  const auto joint_count = joint_names_.size();
  for (size_t i = 0; i < joint_count; ++i) {
    q_meas_[i] = state_interfaces_[2 * i].get_optional().value_or(q_des_[i]);
    const double qdot_meas = state_interfaces_[2 * i + 1].get_optional().value_or(0.0);
    // velocity_filter_alpha_ = measurement weight: (1-α)·prev + α·meas.
    // 1.0 = no filter; smaller α = heavier smoothing.
    dq_filtered_[i] =
      (1.0 - velocity_filter_alpha_) * dq_filtered_[i] +
      velocity_filter_alpha_ * qdot_meas;
  }

  const auto requested_fault = diagnostics_.consume_hold_request();
  if (requested_fault != common::ManipulationFault::kNone) {
    hold_measured_as_reference();
    stale_hold_active_ = true;
  }

  previous_active_input_mode_ = active_input_mode_;
  // Present: first-point only. Freshness = receive_time + stale_timeout.
  const auto * chunk = pose_chunk_buffer_.readFromRT();
  const bool has_chunk = chunk && chunk->size > 0;
  const double traj_age_s = has_chunk
    ? (time.seconds() - chunk->receive_time_s)
    : std::numeric_limits<double>::infinity();
  const bool fresh_traj = has_chunk && traj_age_s <= stale_timeout_s_;
  const auto * twist = twist_buffer_.readFromRT();
  const bool has_twist =
    twist && twist->valid && twist->frame_id == base_frame_;
  const double twist_age_s = has_twist
    ? (time.seconds() - twist->stamp)
    : std::numeric_limits<double>::infinity();
  const bool fresh_twist =
    has_twist && twist_age_s <= twist_stale_timeout_s_;
  last_twist_age_s_ = twist_age_s;

  if (!fresh_traj && !fresh_twist) {
    active_input_mode_ = 0;
    if (target_initialized_ && !stale_hold_active_) {
      stale_hold_count_++;
      stale_hold_active_ = true;
      diagnostics_.enter_hold(
        common::ManipulationFault::kReferenceTimeout,
        std::min(traj_age_s, twist_age_s),
        std::min(stale_timeout_s_, twist_stale_timeout_s_));
    }
    hold_measured_as_reference();
  } else {
    stale_hold_active_ = false;
    last_valid_target_time_ = time;

    std::array<double, 7> pose{};
    bool have_pose = false;
    if (fresh_traj) {
      const auto & f = chunk->frames[0];
      pose[0] = f.px; pose[1] = f.py; pose[2] = f.pz;
      pose[3] = f.qx; pose[4] = f.qy; pose[5] = f.qz; pose[6] = f.qw;
      have_pose = true;
      active_input_mode_ = 1;
      set_integrated_twist_pose(pose);
    }
    if (!have_pose && fresh_twist) {
      active_input_mode_ = 2;
      if (previous_active_input_mode_ != 2) {
        integrated_twist_pose_valid_ = false;
      }
      if (!integrated_twist_pose_valid_ &&
          !seed_integrated_twist_pose(q_des_))
      {
        solve_fail_count_++;
        diagnostics_.enter_hold(common::ManipulationFault::kInternalOutputFailure);
        hold_measured_as_reference();
      } else {
        integrate_base_frame_twist_target(*twist, dtau);
        pose = integrated_twist_pose_;
        have_pose = true;
      }
    }

    if (have_pose) {
      const auto t0 = std::chrono::steady_clock::now();
      const auto & result = solver_->solve(q_des_, pose, dtau);
      last_solve_latency_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();

      if (result.success) {
        const bool output_is_finite = std::all_of(
          result.q_command.begin(), result.q_command.end(),
          [](double value) {return std::isfinite(value);});
        if (!output_is_finite) {
          solve_fail_count_++;
          hold_measured_as_reference();
          diagnostics_.enter_hold(common::ManipulationFault::kInternalOutputFailure);
        } else {
          solve_success_count_++;
          diagnostics_.set_tracking();
          last_task_position_error_norm_ = result.task_position_error_norm;
          last_task_orientation_error_norm_ = result.task_orientation_error_norm;
          for (size_t i = 0; i < joint_count; ++i) {
            q_des_[i] = result.q_command[i];
            if (i < result.joint_velocity_estimate.size() &&
              std::isfinite(result.joint_velocity_estimate[i]))
            {
              qdot_des_[i] = std::clamp(
                result.joint_velocity_estimate[i],
                -max_joint_velocity_rad_s_, max_joint_velocity_rad_s_);
            } else {
              qdot_des_[i] = 0.0;
            }
          }
        }
      } else {
        solve_fail_count_++;
        hold_measured_as_reference();
        diagnostics_.enter_hold(common::ManipulationFault::kInternalOutputFailure);
      }
    } else {
      hold_measured_as_reference();
    }
  }

  for (size_t i = 0; i < joint_count; ++i) {
    const double pos_err = q_des_[i] - q_meas_[i];
    const double vel_err = qdot_des_[i] - dq_filtered_[i];
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

}  // namespace manipulation_position_controllers

PLUGINLIB_EXPORT_CLASS(
  manipulation_position_controllers::TaskSpaceJointImpedanceController,
  controller_interface::ControllerInterface)
