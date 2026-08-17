// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// Task × Dynamic (joint impedance under normalized task-space inputs).
// CartesianTrajectory / TwistStamped → Diff-IK → q_des → τ = Kp e + Kd ė
//
// Distinct from a future TaskSpaceCartesianImpedanceController
// (operational-space / Cartesian stiffness). This controller keeps the
// impedance law in joint space after Diff-IK.

#pragma once

#include <array>
#include <atomic>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/msg/cartesian_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "manipulation_position_controllers/task_space/kinematics/differential_ik/solver_interface.hpp"
#include "manipulation_position_controllers/common/manipulation_diagnostics.hpp"
#include "manipulation_position_controllers/task_space/kinematics/pose_chunk_buffer.hpp"
#include "manipulation_position_controllers/task_space/kinematics/pose_target_buffer.hpp"

namespace manipulation_position_controllers
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TaskSpaceJointImpedanceController
  : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  CallbackReturn load_parameters();
  CallbackReturn validate_configuration() const;
  std::unique_ptr<task_space::DifferentialIkSolver> create_solver();
  void trajectory_callback(
    const moveit_msgs::msg::CartesianTrajectory::SharedPtr msg);
  void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  double resolve_stamp_seconds(const builtin_interfaces::msg::Time & stamp);
  bool seed_integrated_twist_pose(const std::vector<double> & q_seed);
  void set_integrated_twist_pose(const std::array<double, 7> & pose);
  void integrate_base_frame_twist_target(
    const task_space::TwistCommand & twist, double period_s);
  void write_effort_commands();
  void hold_measured_as_reference();
  void publish_command_state(const rclcpp::Time & time);

  std::vector<std::string> joint_names_;
  std::string base_frame_;
  std::string tip_frame_;
  std::string input_topic_;
  std::string twist_topic_;
  double untimed_frame_dt_s_{0.02};
  int64_t max_pose_points_{64};
  double stale_timeout_s_{0.2};
  double twist_stale_timeout_s_{0.2};
  double max_joint_velocity_rad_s_{1.0};
  double max_linear_velocity_m_s_{0.5};
  double max_angular_velocity_rad_s_{1.0};
  double status_rate_hz_{10.0};
  double velocity_filter_alpha_{0.99};
  bool reject_zero_stamped_references_{true};
  task_space::SolverConfig solver_config_;

  std::vector<double> kp_stiffness_;
  std::vector<double> kd_damping_;
  std::vector<double> max_torques_;
  std::vector<double> max_torque_rates_;
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;

  rclcpp::Subscription<moveit_msgs::msg::CartesianTrajectory>::SharedPtr
    trajectory_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>
    realtime_cmd_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr status_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>
    realtime_status_pub_;

  realtime_tools::RealtimeBuffer<task_space::PoseChunk> pose_chunk_buffer_;
  realtime_tools::RealtimeBuffer<task_space::TwistCommand> twist_buffer_;
  common::ManipulationDiagnostics diagnostics_;
  std::unique_ptr<task_space::DifferentialIkSolver> solver_;

  std::vector<double> q_des_;
  std::vector<double> qdot_des_;
  std::vector<double> q_meas_;
  std::vector<double> dq_filtered_;
  std::vector<double> effort_command_;
  std::array<double, 7> integrated_twist_pose_{
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

  std::atomic<bool> target_initialized_{false};
  std::atomic<uint64_t> frame_reject_count_{0};
  std::atomic<uint64_t> invalid_quat_count_{0};
  std::atomic<uint64_t> zero_stamp_fallback_count_{0};
  std::atomic<uint64_t> twist_sequence_{0};
  std::atomic<uint64_t> twist_frame_reject_count_{0};
  uint64_t solve_success_count_{0};
  uint64_t solve_fail_count_{0};
  uint64_t stale_hold_count_{0};
  bool stale_hold_active_{false};
  bool integrated_twist_pose_valid_{false};
  int active_input_mode_{0};
  int previous_active_input_mode_{0};
  double last_twist_age_s_{std::numeric_limits<double>::infinity()};
  double last_task_position_error_norm_{0.0};
  double last_task_orientation_error_norm_{0.0};
  double last_solve_latency_ms_{0.0};
  rclcpp::Time last_valid_target_time_{0, 0, RCL_ROS_TIME};
  size_t status_decimation_{10};
  size_t publish_counter_{0};
};

}  // namespace manipulation_position_controllers
