// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// Task-space kinematic position controller plugin for ros2_control.
//
// Exactly two command inputs (JSPC parity for pose):
//   - input_topic  ← moveit_msgs/CartesianTrajectory (1 pt ≡ old PoseStamped)
//   - twist_topic  ← geometry_msgs/TwistStamped (spatial velocity)
// Present: command first trajectory point only. trajectory_behavior.* is the
// JSPC-analogue hook for future richer consume strategies. Priority: traj > twist.

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
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "manipulation_position_controllers/task_space/kinematics/differential_ik/solver_interface.hpp"
#include "manipulation_position_controllers/common/manipulation_diagnostics.hpp"
#include "manipulation_position_controllers/task_space/kinematics/pose_chunk_buffer.hpp"
#include "manipulation_position_controllers/task_space/kinematics/pose_target_buffer.hpp"

namespace manipulation_position_controllers
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// Status field indices for the Float64MultiArray status publisher.
namespace task_space_status
{
enum : size_t
{
  kState = 0,                // 0=IDLE, 1=ACTIVE, 2=STALE_HOLD, 3=IK_FAIL_HOLD
  kTargetAgeS = 1,           // seconds since last valid target
  kSolveSuccessCount = 2,    // cumulative successful solve steps
  kSolveFailCount = 3,       // cumulative failed solve steps
  kStaleHoldCount = 4,       // cumulative stale-hold events
  kFrameRejectCount = 5,     // cumulative wrong-frame rejections
  kInvalidQuatCount = 6,     // cumulative invalid-quaternion rejections
  kTaskPositionErrorNorm = 7,// current position error norm [m]
  kTaskOrientationErrorNorm=8,// current orientation error norm (approx)
  kLastSolveLatencyMs = 9,   // last solve duration [ms]
  kCommandTrackingErrorRad = 10, // max |command - feedback| [rad]
  kActiveInputMode = 11,          // 0=none, 1=pose_trajectory, 2=twist
  kTwistAgeS = 12,               // seconds since last valid twist command
  kTwistLinearVelocityNorm = 13, // norm of current twist linear velocity [m/s]
  kTwistAngularVelocityNorm = 14,// norm of current twist angular velocity [rad/s]
  kTwistSequence = 15,           // last twist sequence counter
  kIntegratedTwistPoseValid = 16,// 1.0 if integrated twist pose is seeded valid
  kTwistStaleCount = 17,         // cumulative twist-specific stale-hold events
  kTwistFrameRejectCount = 18,   // cumulative twist wrong-frame rejections
  kFieldCount
};
}  // namespace task_space_status

class TaskSpaceKinematicPositionController
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
  void reset_runtime_state();

  void trajectory_callback(
    const moveit_msgs::msg::CartesianTrajectory::SharedPtr msg);
  void twist_callback(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  void publish_status(const rclcpp::Time & time, int state);
  void publish_command_state(const rclcpp::Time & time);

  std::unique_ptr<task_space::DifferentialIkSolver> create_solver();

  bool seed_integrated_twist_pose(const std::vector<double> & q_seed);
  void set_integrated_twist_pose(const std::array<double, 7> & pose);

  double resolve_stamp_seconds(const builtin_interfaces::msg::Time & stamp);
  void integrate_base_frame_twist_target(
    const task_space::TwistCommand & twist,
    double period_s);

  // --- Parameters ---
  std::vector<std::string> joint_names_;
  std::string base_frame_;
  std::string tip_frame_;
  /// CartesianTrajectory pose input (1 or N points). JSPC-analogue name.
  std::string input_topic_;
  std::string twist_topic_;
  /// trajectory_behavior.max_points
  int64_t max_pose_points_{64};
  /// trajectory_behavior.untimed_frame_dt_s
  double untimed_frame_dt_s_{0.02};
  double stale_timeout_s_{0.2};
  double twist_stale_timeout_s_{0.2};
  double max_joint_velocity_rad_s_{1.0};
  double max_linear_velocity_m_s_{0.5};
  double max_angular_velocity_rad_s_{1.0};
  double max_command_tracking_error_rad_{0.35};
  double status_rate_hz_{10.0};
  bool reject_zero_stamped_references_{true};
  task_space::SolverConfig solver_config_;

  // --- ROS communication ---
  rclcpp::Subscription<moveit_msgs::msg::CartesianTrajectory>::SharedPtr
    trajectory_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<
    sensor_msgs::msg::JointState>> realtime_cmd_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<
    std_msgs::msg::Float64MultiArray>> realtime_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr status_pub_;

  // --- Realtime data ---
  realtime_tools::RealtimeBuffer<task_space::PoseChunk> pose_chunk_buffer_;
  realtime_tools::RealtimeBuffer<task_space::TwistCommand> twist_buffer_;
  common::ManipulationDiagnostics diagnostics_;
  std::unique_ptr<task_space::DifferentialIkSolver> solver_;

  // --- Working buffers (allocated once) ---
  std::vector<double> command_;
  std::vector<double> hold_position_;
  std::vector<double> q_current_;
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;
  std::array<double, 7> integrated_twist_pose_{
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

  // --- Counters and state ---
  uint64_t solve_success_count_{0};
  uint64_t solve_fail_count_{0};
  uint64_t stale_hold_count_{0};
  std::atomic<uint64_t> frame_reject_count_{0};
  std::atomic<uint64_t> invalid_quat_count_{0};
  std::atomic<uint64_t> zero_stamp_fallback_count_{0};
  double last_solve_latency_ms_{0.0};
  double last_task_position_error_norm_{0.0};
  double last_task_orientation_error_norm_{0.0};
  double last_command_tracking_error_rad_{0.0};
  double last_target_age_s_{std::numeric_limits<double>::infinity()};
  rclcpp::Time last_valid_target_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  size_t status_decimation_{5};
  size_t status_counter_{0};
  size_t command_publish_counter_{0};
  std::atomic<bool> target_initialized_{false};
  bool active_stale_hold_{false};
  bool tracking_resync_active_{false};
  bool integrated_twist_pose_valid_{false};

  double last_twist_age_s_{std::numeric_limits<double>::infinity()};
  double last_twist_linear_norm_{0.0};
  double last_twist_angular_norm_{0.0};
  std::atomic<uint64_t> twist_sequence_{0};
  uint64_t twist_stale_count_{0};
  std::atomic<uint64_t> twist_frame_reject_count_{0};
  int active_input_mode_{0};  // 0=none, 1=pose_trajectory, 2=twist
  int previous_active_input_mode_{0};
};

}  // namespace manipulation_position_controllers
