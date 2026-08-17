// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include "manipulation_position_controllers/task_space/kinematics/task_space_kinematic_position_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <Eigen/Geometry>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "manipulation_position_controllers/task_space/kinematics/differential_ik/osqp_solver.hpp"
#include "manipulation_position_controllers/task_space/kinematics/differential_ik/dls_solver.hpp"
#include "manipulation_position_controllers/task_space/kinematics/differential_ik/placo_solver.hpp"
#include "manipulation_position_controllers/task_space/kinematics/cartesian_trajectory_reader.hpp"
#include "manipulation_position_controllers/common/limits.hpp"
#include "manipulation_position_controllers/common/parameter_validation.hpp"

namespace manipulation_position_controllers
{

namespace
{

// Tracking-resync completion tolerance (rad). Once the rate-limited resync
// step (max_joint_velocity_rad_s_ * period) drives the per-cycle residual
// below this, command_ is treated as having caught up to q_current and the
// resync exits back to normal target tracking. This is a numerical/joint-unit
// epsilon (radians are already robot-scale-invariant), not a per-robot tuning
// knob, so it stays a compile-time constant rather than a YAML parameter.
constexpr double kTrackingResyncDoneToleranceRad = 1.0e-4;

std::array<double, 3> clamp_vector_norm(
  const std::array<double, 3> & input,
  double max_norm)
{
  std::array<double, 3> output = input;
  const double norm = std::sqrt(
    output[0] * output[0] +
    output[1] * output[1] +
    output[2] * output[2]);
  if (norm > max_norm && norm > 1e-12) {
    const double scale = max_norm / norm;
    output[0] *= scale;
    output[1] *= scale;
    output[2] *= scale;
  }
  return output;
}

}  // namespace

// ---------------------------------------------------------------------------
// Interface configuration
// ---------------------------------------------------------------------------

controller_interface::InterfaceConfiguration
TaskSpaceKinematicPositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : joint_names_) {
    config.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

controller_interface::InterfaceConfiguration
TaskSpaceKinematicPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : joint_names_) {
    config.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_init
// ---------------------------------------------------------------------------

CallbackReturn TaskSpaceKinematicPositionController::on_init()
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
  auto_declare<double>("max_command_tracking_error_rad", 0.35);
  auto_declare<double>("status_rate_hz", 10.0);
  auto_declare<bool>("reject_zero_stamped_references", true);

  auto_declare<std::string>("solver.backend", "pinocchio_dls");
  auto_declare<double>("solver.position_gain", 4.0);
  auto_declare<double>("solver.orientation_gain", 1.0);
  auto_declare<double>("solver.damping", 0.05);
  auto_declare<double>("solver.posture_weight", 1.0e-3);
  auto_declare<double>("solver.manipulability_weight", 0.0);
  auto_declare<double>("solver.kinetic_energy_weight", 0.0);
  auto_declare<std::vector<double>>("solver.joint_motion_weights", {});
  auto_declare<double>("solver.max_joint_acceleration_rad_s2", 0.0);
  auto_declare<int>("solver.max_iterations", 1);

  // OSQP backend only (ignored by pinocchio_dls / placo).
  auto_declare<int>("solver.osqp.max_admm_iterations", 200);
  auto_declare<double>("solver.osqp.cbf_gain", 2.0);
  auto_declare<double>("solver.osqp.slack_penalty", 1.0e4);
  auto_declare<double>("solver.osqp.abs_tolerance", 1.0e-6);
  auto_declare<double>("solver.osqp.rel_tolerance", 1.0e-6);
  auto_declare<double>("solver.osqp.rho", 0.1);

  auto_declare<std::vector<double>>("lower_limits", {});
  auto_declare<std::vector<double>>("upper_limits", {});

  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Parameter loading
// ---------------------------------------------------------------------------

CallbackReturn TaskSpaceKinematicPositionController::load_parameters()
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  base_frame_  = get_node()->get_parameter("base_frame").as_string();
  tip_frame_   = get_node()->get_parameter("tip_frame").as_string();
  input_topic_ = get_node()->get_parameter("input_topic").as_string();
  twist_topic_ = get_node()->get_parameter("twist_topic").as_string();
  max_pose_points_ =
    get_node()->get_parameter("trajectory_behavior.max_points").as_int();
  untimed_frame_dt_s_ =
    get_node()->get_parameter("trajectory_behavior.untimed_frame_dt_s").as_double();
  stale_timeout_s_ =
    get_node()->get_parameter("stale_timeout_s").as_double();
  twist_stale_timeout_s_ =
    get_node()->get_parameter("twist_stale_timeout_s").as_double();
  max_joint_velocity_rad_s_ =
    get_node()->get_parameter("max_joint_velocity_rad_s").as_double();
  max_linear_velocity_m_s_ =
    get_node()->get_parameter("max_linear_velocity_m_s").as_double();
  max_angular_velocity_rad_s_ =
    get_node()->get_parameter("max_angular_velocity_rad_s").as_double();
  max_command_tracking_error_rad_ =
    get_node()->get_parameter("max_command_tracking_error_rad").as_double();
  status_rate_hz_ =
    get_node()->get_parameter("status_rate_hz").as_double();
  reject_zero_stamped_references_ =
    get_node()->get_parameter("reject_zero_stamped_references").as_bool();

  solver_config_.backend =
    get_node()->get_parameter("solver.backend").as_string();
  solver_config_.position_gain =
    get_node()->get_parameter("solver.position_gain").as_double();
  solver_config_.orientation_gain =
    get_node()->get_parameter("solver.orientation_gain").as_double();
  solver_config_.damping =
    get_node()->get_parameter("solver.damping").as_double();
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
  solver_config_.osqp_rho =
    get_node()->get_parameter("solver.osqp.rho").as_double();
  solver_config_.max_joint_velocity_rad_s = max_joint_velocity_rad_s_;
  solver_config_.max_linear_velocity_m_s = max_linear_velocity_m_s_;
  solver_config_.max_angular_velocity_rad_s = max_angular_velocity_rad_s_;

  lower_limits_ = get_node()->get_parameter("lower_limits").as_double_array();
  upper_limits_ = get_node()->get_parameter("upper_limits").as_double_array();

  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceKinematicPositionController::validate_configuration() const
{
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' must not be empty.");
    return CallbackReturn::ERROR;
  }
  if (!common::all_unique_nonempty(joint_names_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' must contain unique, nonempty names.");
    return CallbackReturn::ERROR;
  }
  if (base_frame_.empty() || tip_frame_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "base_frame and tip_frame must be set.");
    return CallbackReturn::ERROR;
  }
  if (!common::is_positive_finite(stale_timeout_s_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "stale_timeout_s must be > 0.");
    return CallbackReturn::ERROR;
  }
  if (!common::is_positive_finite(twist_stale_timeout_s_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "twist_stale_timeout_s must be > 0.");
    return CallbackReturn::ERROR;
  }
  if (!common::is_positive_finite(untimed_frame_dt_s_)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "trajectory_behavior.untimed_frame_dt_s must be > 0.");
    return CallbackReturn::ERROR;
  }
  if (max_pose_points_ <= 0) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "trajectory_behavior.max_points must be > 0.");
    return CallbackReturn::ERROR;
  }
  if (static_cast<size_t>(max_pose_points_) > task_space::kMaxPoseFrames) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "trajectory_behavior.max_points must be <= %zu.",
      task_space::kMaxPoseFrames);
    return CallbackReturn::ERROR;
  }
  if (input_topic_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "input_topic must not be empty.");
    return CallbackReturn::ERROR;
  }
  if (!common::is_positive_finite(max_joint_velocity_rad_s_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "max_joint_velocity_rad_s must be > 0.");
    return CallbackReturn::ERROR;
  }
  if (!common::is_positive_finite(max_linear_velocity_m_s_) ||
      !common::is_positive_finite(max_angular_velocity_rad_s_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "max_linear_velocity_m_s and max_angular_velocity_rad_s must be > 0.");
    return CallbackReturn::ERROR;
  }
  if (!common::is_positive_finite(max_command_tracking_error_rad_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "max_command_tracking_error_rad must be > 0.");
    return CallbackReturn::ERROR;
  }
  if (!common::is_positive_finite(status_rate_hz_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "status_rate_hz must be finite and positive.");
    return CallbackReturn::ERROR;
  }
  if (!common::validate_limit_vectors(joint_names_, lower_limits_, upper_limits_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "lower_limits/upper_limits must be empty or match joints size, contain no NaN, and be monotonic.");
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

// ---------------------------------------------------------------------------
// Solver factory — easy PlaCo swap point
// ---------------------------------------------------------------------------

std::unique_ptr<task_space::DifferentialIkSolver>
TaskSpaceKinematicPositionController::create_solver()
{
  if (solver_config_.backend == "osqp") {
    return std::make_unique<task_space::OsqpIkSolver>();
  }
  if (solver_config_.backend == "pinocchio_dls") {
    return std::make_unique<task_space::PinocchioDlsSolver>();
  }
  if (solver_config_.backend == "placo") {
    return std::make_unique<task_space::PlacoKinematicSolver>();
  }

  RCLCPP_ERROR(get_node()->get_logger(),
    "Unknown solver backend '%s'.",
    solver_config_.backend.c_str());
  return nullptr;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_configure
// ---------------------------------------------------------------------------

CallbackReturn TaskSpaceKinematicPositionController::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (load_parameters() != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (validate_configuration() != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto joint_count = joint_names_.size();
  command_.assign(joint_count, 0.0);
  hold_position_.assign(joint_count, 0.0);
  q_current_.assign(joint_count, 0.0);

  // Fill default limits.
  if (lower_limits_.empty()) {
    lower_limits_.assign(joint_count, -std::numeric_limits<double>::infinity());
  }
  if (upper_limits_.empty()) {
    upper_limits_.assign(joint_count, std::numeric_limits<double>::infinity());
  }

  // Build solver.
  solver_ = create_solver();
  if (!solver_) {
    return CallbackReturn::ERROR;
  }

  // Pass joint limits to QP-based solvers that enforce them as constraints.
  if (auto * osqp = dynamic_cast<task_space::OsqpIkSolver *>(solver_.get())) {
    osqp->set_joint_limits(lower_limits_, upper_limits_);
  }

  // Get URDF. Primary source: the controller manager (Jazzy passes the robot
  // description it received to every controller via init()). Fallback: a
  // robot_description parameter set directly on the controller node.
  std::string urdf_xml = get_robot_description();
  if (urdf_xml.empty()) {
    (void)get_node()->get_parameter("robot_description", urdf_xml);
  }
  if (urdf_xml.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "No robot description available: controller manager did not provide "
      "one and the 'robot_description' parameter is empty.");
    return CallbackReturn::ERROR;
  }

  if (!solver_->configure(urdf_xml, joint_names_, base_frame_, tip_frame_,
                          solver_config_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Solver configuration failed: %s",
      solver_->last_error().empty() ? "unknown error" :
      solver_->last_error().c_str());
    return CallbackReturn::ERROR;
  }

  // CartesianTrajectory pose input (required). 1 point ≡ former PoseStamped.
  trajectory_sub_ =
    get_node()->create_subscription<moveit_msgs::msg::CartesianTrajectory>(
    input_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const moveit_msgs::msg::CartesianTrajectory::SharedPtr msg) {
      trajectory_callback(msg);
    });
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Subscribed to pose trajectory topic: %s (max_points=%ld)",
    input_topic_.c_str(), max_pose_points_);

  if (!twist_topic_.empty()) {
    twist_sub_ =
      get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
      twist_topic_,
      rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        twist_callback(msg);
      });
    RCLCPP_INFO(get_node()->get_logger(),
      "Subscribed to twist topic: %s", twist_topic_.c_str());
  }

  // Commanded joint state publisher (for RViz visualization).
  cmd_state_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    "~/commanded_joint_state", rclcpp::SystemDefaultsQoS());
  realtime_cmd_state_pub_ = std::make_unique<
    realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
    cmd_state_pub_);
  realtime_cmd_state_pub_->msg_.name = joint_names_;
  realtime_cmd_state_pub_->msg_.position.resize(joint_count, 0.0);

  // Status publisher.
  status_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/status", rclcpp::SystemDefaultsQoS());
  realtime_status_pub_ = std::make_unique<
    realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
    status_pub_);
  realtime_status_pub_->msg_.data.resize(
    task_space_status::kFieldCount, 0.0);

  diagnostics_.configure(
    get_node(),
    "safety",
    1.0 / status_rate_hz_);

  // Drive the decimation off the controller's real update rate instead of a
  // hardcoded 500 Hz assumption -- mirrors the JointSpacePositionController
  // Ruckig control_cycle fix (get_update_rate() reflects the rate
  // controller_manager actually ticks this controller at). Falls back to
  // 500 Hz only if the rate is unknown (0).
  const auto update_rate = get_update_rate();
  const double loop_hz = (update_rate > 0) ? static_cast<double>(update_rate) : 500.0;
  status_decimation_ = std::max(
    size_t{1},
    static_cast<size_t>(loop_hz / std::max(status_rate_hz_, 1.0)));

  RCLCPP_INFO(get_node()->get_logger(),
    "TaskSpaceKinematicPositionController configured: "
    "base=%s tip=%s joints=%zu input_topic=%s twist_topic=%s backend=%s "
    "pos_gain=%.1f ori_gain=%.1f damping=%.3f max_joint_vel=%.2f rad/s "
    "max_linear_vel=%.2f m/s max_angular_vel=%.2f rad/s "
    "max_tracking_err=%.2f rad",
    base_frame_.c_str(), tip_frame_.c_str(), joint_count,
    input_topic_.c_str(), twist_topic_.c_str(), solver_config_.backend.c_str(),
    solver_config_.position_gain, solver_config_.orientation_gain,
    solver_config_.damping, max_joint_velocity_rad_s_,
    max_linear_velocity_m_s_, max_angular_velocity_rad_s_,
    max_command_tracking_error_rad_);

  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_activate
// ---------------------------------------------------------------------------

CallbackReturn TaskSpaceKinematicPositionController::on_activate(
  const rclcpp_lifecycle::State &)
{
  const auto joint_count = joint_names_.size();
  if (state_interfaces_.size() < joint_count || command_interfaces_.size() < joint_count) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Not enough state/command interfaces for %zu joints.", joint_count);
    return CallbackReturn::ERROR;
  }

  // Initialise hold position and command from current state.
  for (size_t i = 0; i < joint_count; ++i) {
    const auto pos = state_interfaces_[i].get_optional();
    if (!pos.has_value() || !std::isfinite(pos.value())) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Invalid initial position for joint '%s'.", joint_names_[i].c_str());
      return CallbackReturn::ERROR;
    }
    hold_position_[i] = pos.value();
    command_[i] = pos.value();
  }

  solver_->reset(hold_position_);
  reset_runtime_state();
  diagnostics_.set_idle();
  if (!seed_integrated_twist_pose(hold_position_)) {
    RCLCPP_WARN(get_node()->get_logger(),
      "Could not seed initial twist integrated pose from FK; it will be "
      "re-seeded from command state when the first twist command arrives.");
  }

  RCLCPP_INFO(get_node()->get_logger(),
    "TaskSpaceKinematicPositionController activated with %zu joints.",
    joint_count);

  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_deactivate
// ---------------------------------------------------------------------------

CallbackReturn TaskSpaceKinematicPositionController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(),
    "TaskSpaceKinematicPositionController deactivated.");
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Runtime helpers
// ---------------------------------------------------------------------------

void TaskSpaceKinematicPositionController::reset_runtime_state()
{
  target_initialized_ = false;
  active_stale_hold_ = false;
  tracking_resync_active_ = false;
  integrated_twist_pose_valid_ = false;
  integrated_twist_pose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  solve_success_count_ = 0;
  solve_fail_count_ = 0;
  stale_hold_count_ = 0;
  frame_reject_count_ = 0;
  invalid_quat_count_ = 0;
  zero_stamp_fallback_count_ = 0;
  last_solve_latency_ms_ = 0.0;
  last_task_position_error_norm_ = 0.0;
  last_task_orientation_error_norm_ = 0.0;
  last_command_tracking_error_rad_ = 0.0;
  last_target_age_s_ = std::numeric_limits<double>::infinity();
  // Twist observability.
  last_twist_age_s_ = std::numeric_limits<double>::infinity();
  last_twist_linear_norm_ = 0.0;
  last_twist_angular_norm_ = 0.0;
  twist_sequence_ = 0;
  twist_stale_count_ = 0;
  twist_frame_reject_count_ = 0;
  active_input_mode_ = 0;
  status_counter_ = 0;
  command_publish_counter_ = 0;
  last_valid_target_time_ = get_node()->now();
  last_status_time_ = get_node()->now();
}

double TaskSpaceKinematicPositionController::resolve_stamp_seconds(
  const builtin_interfaces::msg::Time & stamp)
{
  const rclcpp::Time t(stamp);
  if (t.nanoseconds() == 0) {
    zero_stamp_fallback_count_++;
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(),
      *get_node()->get_clock(), 5000,
      "Received reference with zero header.stamp; using receive time instead "
      "(total %lu). Stamp your references to get accurate staleness.",
      zero_stamp_fallback_count_.load());
    return get_node()->now().seconds();
  }
  return t.seconds();
}

bool TaskSpaceKinematicPositionController::seed_integrated_twist_pose(
  const std::vector<double> & q_seed)
{
  if (!solver_ ||
      !solver_->compute_tip_pose(q_seed, integrated_twist_pose_)) {
    integrated_twist_pose_valid_ = false;
    return false;
  }

  integrated_twist_pose_valid_ = true;
  return true;
}

void TaskSpaceKinematicPositionController::set_integrated_twist_pose(
  const std::array<double, 7> & pose)
{
  integrated_twist_pose_ = pose;
  Eigen::Quaterniond quat(
    integrated_twist_pose_[6],
    integrated_twist_pose_[3],
    integrated_twist_pose_[4],
    integrated_twist_pose_[5]);
  quat.normalize();
  integrated_twist_pose_[3] = quat.x();
  integrated_twist_pose_[4] = quat.y();
  integrated_twist_pose_[5] = quat.z();
  integrated_twist_pose_[6] = quat.w();
  integrated_twist_pose_valid_ = true;
}

void TaskSpaceKinematicPositionController::integrate_base_frame_twist_target(
  const task_space::TwistCommand & twist,
  double period_s)
{
  const double dt = std::max(period_s, 1e-9);
  const auto linear = clamp_vector_norm(
    twist.linear, max_linear_velocity_m_s_);
  const auto angular = clamp_vector_norm(
    twist.angular, max_angular_velocity_rad_s_);

  integrated_twist_pose_[0] += linear[0] * dt;
  integrated_twist_pose_[1] += linear[1] * dt;
  integrated_twist_pose_[2] += linear[2] * dt;

  Eigen::Quaterniond current_quat(
    integrated_twist_pose_[6],
    integrated_twist_pose_[3],
    integrated_twist_pose_[4],
    integrated_twist_pose_[5]);
  current_quat.normalize();

  const double omega_norm = std::sqrt(
    angular[0] * angular[0] +
    angular[1] * angular[1] +
    angular[2] * angular[2]);
  if (omega_norm > 1e-12) {
    const double angle = omega_norm * dt;
    const Eigen::Vector3d axis(
      angular[0] / omega_norm,
      angular[1] / omega_norm,
      angular[2] / omega_norm);
    const Eigen::Quaterniond delta_quat(Eigen::AngleAxisd(angle, axis));
    current_quat = delta_quat * current_quat;
    current_quat.normalize();
  }

  integrated_twist_pose_[3] = current_quat.x();
  integrated_twist_pose_[4] = current_quat.y();
  integrated_twist_pose_[5] = current_quat.z();
  integrated_twist_pose_[6] = current_quat.w();
}

// ---------------------------------------------------------------------------
// Trajectory callback (non-realtime — ROS subscriber thread)
// ---------------------------------------------------------------------------

void TaskSpaceKinematicPositionController::trajectory_callback(
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

void TaskSpaceKinematicPositionController::twist_callback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  task_space::TwistCommand command;

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
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(),
      *get_node()->get_clock(), 5000,
      "TwistStamped frame '%s' != expected '%s' (rejected %lu total, %lu twist).",
      msg->header.frame_id.c_str(), base_frame_.c_str(),
      frame_reject_count_.load(), twist_frame_reject_count_.load());
    return;
  }

  command.linear = {
    msg->twist.linear.x,
    msg->twist.linear.y,
    msg->twist.linear.z};
  command.angular = {
    msg->twist.angular.x,
    msg->twist.angular.y,
    msg->twist.angular.z};

  if (!task_space::is_valid_vector3(command.linear) ||
      !task_space::is_valid_vector3(command.angular)) {
    pose_chunk_buffer_.writeFromNonRT(task_space::PoseChunk{});
    twist_buffer_.writeFromNonRT(task_space::TwistCommand{});
    diagnostics_.request_hold(common::ManipulationFault::kNonfiniteReference);
    return;
  }

  command.stamp = resolve_stamp_seconds(msg->header.stamp);
  command.frame_id = msg->header.frame_id;
  command.sequence = msg->header.stamp.sec * 1000000000ULL +
                     msg->header.stamp.nanosec;
  command.valid = true;

  twist_buffer_.writeFromNonRT(command);
  twist_sequence_ = command.sequence;
  target_initialized_ = true;
}

// ---------------------------------------------------------------------------
// Realtime update loop (rate set by controller_manager's update_rate)
// ---------------------------------------------------------------------------

controller_interface::return_type
TaskSpaceKinematicPositionController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // --- 0. Capture previous-cycle mode for transition detection ---
  previous_active_input_mode_ = active_input_mode_;

  // --- 1. Read current joint state ---
  const auto joint_count = joint_names_.size();
  for (size_t i = 0; i < joint_count; ++i) {
    const auto pos = state_interfaces_[i].get_optional();
    if (!pos.has_value() || !std::isfinite(pos.value())) {
      diagnostics_.enter_error(common::ManipulationFault::kInvalidMeasuredState);
      for (size_t j = 0; j < joint_count; ++j) {
        (void)command_interfaces_[j].set_value(hold_position_[j]);
      }
      return controller_interface::return_type::ERROR;
    }
    q_current_[i] = pos.value();
  }

  const auto requested_fault = diagnostics_.consume_hold_request();
  if (requested_fault != common::ManipulationFault::kNone) {
    for (size_t i = 0; i < joint_count; ++i) {
      command_[i] = q_current_[i];
      hold_position_[i] = q_current_[i];
      (void)command_interfaces_[i].set_value(q_current_[i]);
    }
    solver_->reset(q_current_);
    integrated_twist_pose_valid_ = false;
    active_stale_hold_ = true;
    publish_command_state(time);
    publish_status(time, /*state=*/2);
    return controller_interface::return_type::OK;
  }
  last_command_tracking_error_rad_ = 0.0;
  for (size_t i = 0; i < joint_count; ++i) {
    last_command_tracking_error_rad_ = std::max(
      last_command_tracking_error_rad_, std::abs(command_[i] - q_current_[i]));
  }

  // --- 2. Read latest target (trajectory > twist) ---
  // Present trajectory_behavior: first-point only (JSPC-style hook for future
  // modes). Freshness is receive_time + stale_timeout, not full horizon play.
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
  last_target_age_s_ = std::min(traj_age_s, twist_age_s);

  last_twist_age_s_ = twist_age_s;
  if (has_twist) {
    last_twist_linear_norm_ = std::sqrt(
      twist->linear[0] * twist->linear[0] +
      twist->linear[1] * twist->linear[1] +
      twist->linear[2] * twist->linear[2]);
    last_twist_angular_norm_ = std::sqrt(
      twist->angular[0] * twist->angular[0] +
      twist->angular[1] * twist->angular[1] +
      twist->angular[2] * twist->angular[2]);
  }

  // --- 3. Handle no-target / stale ---
  if (!fresh_traj && !fresh_twist) {
    active_input_mode_ = 0;
    if (!target_initialized_) {
      for (size_t i = 0; i < joint_count; ++i) {
        (void)command_interfaces_[i].set_value(hold_position_[i]);
      }
      publish_command_state(time);
      publish_status(time, /*state=*/0);
      return controller_interface::return_type::OK;
    }

    if (!active_stale_hold_) {
      stale_hold_count_++;
      if (has_twist && twist_age_s > twist_stale_timeout_s_) {
        twist_stale_count_++;
      }
      active_stale_hold_ = true;
      for (size_t i = 0; i < joint_count; ++i) {
        command_[i] = q_current_[i];
        hold_position_[i] = q_current_[i];
      }
      solver_->reset(q_current_);
      integrated_twist_pose_valid_ = false;
      diagnostics_.enter_hold(
        common::ManipulationFault::kReferenceTimeout,
        last_target_age_s_,
        std::min(stale_timeout_s_, twist_stale_timeout_s_));
    }
    for (size_t i = 0; i < joint_count; ++i) {
      (void)command_interfaces_[i].set_value(hold_position_[i]);
    }
    publish_command_state(time);
    publish_status(time, /*state=*/2);  // STALE_HOLD
    return controller_interface::return_type::OK;
  }

  // --- 4. Solve IK ---
  active_stale_hold_ = false;

  const bool tracking_over_limit =
    last_command_tracking_error_rad_ > max_command_tracking_error_rad_;
  if (tracking_over_limit || tracking_resync_active_) {
    if (tracking_over_limit && !tracking_resync_active_) {
      solve_fail_count_++;
      diagnostics_.enter_hold(common::ManipulationFault::kTrackingError);
    }

    const double max_step = max_joint_velocity_rad_s_ * period.seconds();
    double residual_error = 0.0;
    for (size_t i = 0; i < joint_count; ++i) {
      const double err = q_current_[i] - command_[i];
      const double step = std::clamp(err, -max_step, max_step);
      command_[i] += step;
      hold_position_[i] = command_[i];
      (void)command_interfaces_[i].set_value(command_[i]);
      residual_error = std::max(
        residual_error, std::abs(q_current_[i] - command_[i]));
    }

    tracking_resync_active_ = residual_error > kTrackingResyncDoneToleranceRad;
    if (!tracking_resync_active_) {
      last_command_tracking_error_rad_ = 0.0;
    }
    integrated_twist_pose_valid_ = false;
    publish_command_state(time);
    publish_status(time, /*state=*/3);  // IK_FAIL_HOLD / tracking guard
    return controller_interface::return_type::OK;
  }

  // Pack target: prefer pose trajectory (first point only), then TwistStamped.
  std::array<double, 7> target_pose{};
  bool have_pose = false;

  if (fresh_traj) {
    const auto & f = chunk->frames[0];
    target_pose[0] = f.px; target_pose[1] = f.py; target_pose[2] = f.pz;
    target_pose[3] = f.qx; target_pose[4] = f.qy;
    target_pose[5] = f.qz; target_pose[6] = f.qw;
    have_pose = true;
    active_input_mode_ = 1;
    set_integrated_twist_pose(target_pose);
  }

  if (!have_pose && fresh_twist) {
    active_input_mode_ = 2;
    if (previous_active_input_mode_ != 2) {
      integrated_twist_pose_valid_ = false;
    }
    if (!integrated_twist_pose_valid_ &&
        !seed_integrated_twist_pose(command_)) {
      solve_fail_count_++;
      diagnostics_.enter_hold(common::ManipulationFault::kInternalOutputFailure);
      for (size_t i = 0; i < joint_count; ++i) {
        (void)command_interfaces_[i].set_value(hold_position_[i]);
      }
      publish_command_state(time);
      publish_status(time, /*state=*/3);  // IK_FAIL_HOLD / FK seed fail
      return controller_interface::return_type::OK;
    }
    integrate_base_frame_twist_target(*twist, period.seconds());
    target_pose = integrated_twist_pose_;
    have_pose = true;
  }

  const auto t0 = std::chrono::steady_clock::now();
  // Use command_ (not q_current) so IK integrates from previous command,
  // avoiding the lag of waiting for physical feedback to catch up.
  const task_space::SolveResult * result = nullptr;
  if (have_pose) {
    result = &solver_->solve(command_, target_pose, period.seconds());
  }
  const auto t1 = std::chrono::steady_clock::now();
  last_solve_latency_ms_ =
    std::chrono::duration<double, std::milli>(t1 - t0).count();

  // --- 5. Process result ---
  if (!result || !result->success) {
    solve_fail_count_++;
    diagnostics_.enter_hold(common::ManipulationFault::kInternalOutputFailure);
    for (size_t i = 0; i < joint_count; ++i) {
      (void)command_interfaces_[i].set_value(hold_position_[i]);
    }
    publish_command_state(time);
    publish_status(time, /*state=*/3);  // IK_FAIL_HOLD
    return controller_interface::return_type::OK;
  }

  solve_success_count_++;
  diagnostics_.set_tracking();
  last_task_position_error_norm_ = result->task_position_error_norm;
  last_task_orientation_error_norm_ = result->task_orientation_error_norm;

  // --- 6. Joint-limit clamp + per-cycle velocity step cap ---
  const double max_step = max_joint_velocity_rad_s_ * period.seconds();
  for (size_t i = 0; i < joint_count; ++i) {
    double q_target = result->q_command[i];
    if (!std::isfinite(q_target)) {
      for (size_t j = 0; j < joint_count; ++j) {
        command_[j] = q_current_[j];
        hold_position_[j] = q_current_[j];
        (void)command_interfaces_[j].set_value(q_current_[j]);
      }
      diagnostics_.enter_hold(common::ManipulationFault::kInternalOutputFailure);
      publish_command_state(time);
      publish_status(time, /*state=*/3);
      return controller_interface::return_type::OK;
    }

    if (i < lower_limits_.size() && std::isfinite(lower_limits_[i])) {
      q_target = std::max(q_target, lower_limits_[i]);
    }
    if (i < upper_limits_.size() && std::isfinite(upper_limits_[i])) {
      q_target = std::min(q_target, upper_limits_[i]);
    }

    double delta = q_target - command_[i];
    if (max_step > 0.0) {
      delta = std::clamp(delta, -max_step, max_step);
    }
    const double q_cmd = command_[i] + delta;

    command_[i] = q_cmd;
    hold_position_[i] = q_cmd;
    (void)command_interfaces_[i].set_value(q_cmd);
  }

  last_valid_target_time_ = time;
  publish_command_state(time);
  publish_status(time, /*state=*/1);  // ACTIVE
  return controller_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// Status publisher (rate-limited)
// ---------------------------------------------------------------------------

void TaskSpaceKinematicPositionController::publish_command_state(
  const rclcpp::Time & time)
{
  if (!realtime_cmd_state_pub_ || ++command_publish_counter_ < status_decimation_) {
    return;
  }
  command_publish_counter_ = 0;

  if (!realtime_cmd_state_pub_->trylock()) {
    return;
  }

  auto & joint_state = realtime_cmd_state_pub_->msg_;
  joint_state.header.stamp = time;
  for (size_t i = 0; i < command_.size(); ++i) {
    joint_state.position[i] = command_[i];
  }
  realtime_cmd_state_pub_->unlockAndPublish();
}

void TaskSpaceKinematicPositionController::publish_status(
  const rclcpp::Time &, int state)
{
  status_counter_++;
  if (status_counter_ < status_decimation_ && state == 1) {
    return;
  }
  status_counter_ = 0;

  if (!realtime_status_pub_ || !realtime_status_pub_->trylock()) {
    return;
  }

  auto & data = realtime_status_pub_->msg_.data;
  data[task_space_status::kState] = static_cast<double>(state);

  data[task_space_status::kTargetAgeS] = last_target_age_s_;

  data[task_space_status::kSolveSuccessCount] =
    static_cast<double>(solve_success_count_);
  data[task_space_status::kSolveFailCount] =
    static_cast<double>(solve_fail_count_);
  data[task_space_status::kStaleHoldCount] =
    static_cast<double>(stale_hold_count_);
  data[task_space_status::kFrameRejectCount] =
    static_cast<double>(frame_reject_count_.load());
  data[task_space_status::kInvalidQuatCount] =
    static_cast<double>(invalid_quat_count_.load());
  data[task_space_status::kTaskPositionErrorNorm] = last_task_position_error_norm_;
  data[task_space_status::kTaskOrientationErrorNorm] = last_task_orientation_error_norm_;
  data[task_space_status::kLastSolveLatencyMs] = last_solve_latency_ms_;
  data[task_space_status::kCommandTrackingErrorRad] = last_command_tracking_error_rad_;

  // Twist-specific observability (6.2).
  data[task_space_status::kActiveInputMode] = static_cast<double>(active_input_mode_);
  data[task_space_status::kTwistAgeS] = last_twist_age_s_;
  data[task_space_status::kTwistLinearVelocityNorm] = last_twist_linear_norm_;
  data[task_space_status::kTwistAngularVelocityNorm] = last_twist_angular_norm_;
  data[task_space_status::kTwistSequence] = static_cast<double>(twist_sequence_.load());
  data[task_space_status::kIntegratedTwistPoseValid] =
    integrated_twist_pose_valid_ ? 1.0 : 0.0;
  data[task_space_status::kTwistStaleCount] = static_cast<double>(twist_stale_count_);
  data[task_space_status::kTwistFrameRejectCount] = static_cast<double>(twist_frame_reject_count_.load());

  realtime_status_pub_->unlockAndPublish();
}

}  // namespace manipulation_position_controllers

PLUGINLIB_EXPORT_CLASS(
  manipulation_position_controllers::TaskSpaceKinematicPositionController,
  controller_interface::ControllerInterface)
