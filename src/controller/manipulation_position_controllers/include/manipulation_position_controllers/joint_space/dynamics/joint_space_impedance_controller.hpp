// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "manipulation_position_controllers/common/bounded_reference_buffer.hpp"
#include "manipulation_position_controllers/common/manipulation_diagnostics.hpp"
#include "manipulation_position_controllers/joint_space/kinematics/joint_reference.hpp"
#include "manipulation_position_controllers/joint_space/kinematics/ruckig_adapter.hpp"

namespace manipulation_position_controllers
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace joint_space::dynamics
{

/// Joint-space impedance controller (Joint × Dynamic in the package taxonomy).
///
/// External contract: `trajectory_msgs/JointTrajectory` position references
/// (same as JointSpacePositionController). Internal command interface:
/// `<joint>/effort`.
///
/// Control law (aligned with frankarobotics / fr3_motion / ohrc impedance):
///   τ = Kp (q_ref − q) + Kd (q̇_ref − dq_filt)   [+ torque clamps]
/// where q_ref / q̇_ref come from Ruckig (or EMA limiter), and dq_filt is a
/// low-pass on measured joint velocity (`velocity_filter_alpha`).
///
/// On activate, q_ref is seeded to measured q (hold current pose). There is no
/// leader-follower SYNCING phase — upstream teleop (marker/planner) already
/// initializes from the current end-effector pose.
///
/// Gravity compensation is assumed from the hardware stack (e.g. libfranka).
class JointSpaceImpedanceController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  CallbackReturn load_parameters();
  CallbackReturn validate_configuration() const;
  void initialize_runtime_state();
  void reset_activation_state();
  void reference_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void patch_partial_target();
  void publish_command_state(const rclcpp::Time & time);
  void write_effort_commands();
  void hold_measured_as_reference();

  std::vector<std::string> joint_names_;
  std::string input_topic_;
  int64_t input_qos_depth_{10};
  std::string reference_behavior_mode_;
  int64_t max_reference_points_{32};
  double ema_alpha_{0.2};
  double max_velocity_rad_s_{1.5};
  double max_acceleration_rad_s2_{3.0};
  double max_jerk_rad_s3_{10.0};
  double ruckig_control_cycle_s_{0.002};
  double control_cycle_s_{0.002};
  double stale_timeout_s_{0.5};
  double status_rate_hz_{10.0};
  bool reject_out_of_bounds_targets_{false};
  bool reject_zero_stamped_references_{true};
  bool allow_partial_joint_references_{true};

  std::vector<double> kp_stiffness_;
  std::vector<double> kd_damping_;
  std::vector<double> max_torques_;
  std::vector<double> max_torque_rates_;
  double velocity_filter_alpha_{0.99};

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr input_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>
    realtime_command_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr status_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>
    realtime_status_pub_;

  common::BoundedReferenceBuffer<joint_space::JointReference> reference_buffer_;
  common::ManipulationDiagnostics diagnostics_;
  uint64_t reference_sequence_{0};
  uint64_t last_sequence_{0};

  joint_space::JointReference active_reference_;
  size_t active_reference_size_{0};
  double last_limit_factor_{1.0};
  double last_preemption_jump_norm_{0.0};
  uint64_t zero_stamp_fallback_count_{0};
  uint64_t reference_overwrite_count_{0};
  uint64_t coalesced_update_count_{0};
  uint64_t stale_hold_count_{0};
  bool stale_hold_active_{false};

  size_t publish_decimation_{10};
  size_t publish_counter_{0};
  std::vector<double> raw_target_;
  std::vector<double> filtered_target_;
  std::vector<double> position_target_;
  std::vector<double> velocity_target_;
  std::vector<double> dq_filtered_;
  std::vector<double> prev_position_target_;
  std::vector<double> effort_command_;
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;
  std::unique_ptr<joint_space::RuckigAdapter> ruckig_adapter_;
  std::vector<double> ruckig_target_;
  rclcpp::Time last_reference_time_{0, 0, RCL_ROS_TIME};
  bool target_initialized_{false};
};

}  // namespace joint_space::dynamics
}  // namespace manipulation_position_controllers
