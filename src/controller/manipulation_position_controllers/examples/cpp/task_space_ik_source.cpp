// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// IK-solver-as-source node for A/B testing: consumes PoseStamped, runs DLS IK
// at a lower rate, publishes single-point JointTrajectory for JTC to track.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "manipulation_position_controllers/task_space/kinematics/differential_ik/solver_interface.hpp"
#include "manipulation_position_controllers/task_space/kinematics/differential_ik/dls_solver.hpp"

namespace manipulation_position_controllers
{

class TaskSpaceIkSource : public rclcpp::Node
{
public:
  TaskSpaceIkSource() : Node("task_space_ik_source")
  {
    // Parameters
    declare_parameter<std::string>("robot_description", "");
    declare_parameter<std::vector<std::string>>("joints", {});
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("tip_frame", "flange_link");
    declare_parameter<std::string>("pose_topic", "/test/pose_reference");
    declare_parameter<std::string>("joint_state_topic", "/joint_states");
    declare_parameter<std::string>("trajectory_topic", "/ik_joint_trajectory");
    declare_parameter<double>("ik_rate_hz", 50.0);
    declare_parameter<double>("position_gain", 4.0);
    declare_parameter<double>("orientation_gain", 1.0);
    declare_parameter<double>("damping", 0.05);
    declare_parameter<double>("max_joint_velocity_rad_s", 1.0);

    joints_ = get_parameter("joints").as_string_array();
    base_frame_ = get_parameter("base_frame").as_string();
    tip_frame_ = get_parameter("tip_frame").as_string();

    // Build solver
    task_space::SolverConfig config;
    config.backend = "pinocchio_dls";
    config.position_gain = get_parameter("position_gain").as_double();
    config.orientation_gain = get_parameter("orientation_gain").as_double();
    config.damping = get_parameter("damping").as_double();
    config.max_joint_velocity_rad_s = get_parameter("max_joint_velocity_rad_s").as_double();

    solver_ = std::make_unique<task_space::PinocchioDlsSolver>();
    std::string urdf = get_parameter("robot_description").as_string();
    if (!solver_->configure(urdf, joints_, base_frame_, tip_frame_, config)) {
      RCLCPP_ERROR(get_logger(), "Solver configure failed: %s",
                   solver_->last_error().c_str());
      throw std::runtime_error("Solver configure failed");
    }

    // Current joint state
    q_current_.assign(joints_.size(), 0.0);
    solver_->reset(q_current_);

    // Subscriptions
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("pose_topic").as_string(), rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        latest_pose_ = msg;
        has_pose_ = true;
      });

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      get_parameter("joint_state_topic").as_string(), rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < joints_.size(); ++i) {
          auto it = std::find(msg->name.begin(), msg->name.end(), joints_[i]);
          if (it != msg->name.end()) {
            q_current_[i] = msg->position[it - msg->name.begin()];
          }
        }
      });

    // Publisher
    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      get_parameter("trajectory_topic").as_string(), rclcpp::SystemDefaultsQoS());

    // IK timer (low rate)
    const double ik_rate = get_parameter("ik_rate_hz").as_double();
    const int ik_period_ms = static_cast<int>(1000.0 / ik_rate);
    ik_timer_ = create_wall_timer(
      std::chrono::milliseconds(ik_period_ms),
      [this]() { ik_callback(); });

    RCLCPP_INFO(get_logger(),
      "TaskSpaceIkSource ready: %zu joints, IK @ %.1f Hz, topic=%s",
      joints_.size(), ik_rate,
      get_parameter("trajectory_topic").as_string().c_str());
  }

private:
  void ik_callback()
  {
    if (!has_pose_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "No pose received yet.");
      return;
    }

    // Pack target
    std::array<double, 7> pose_target{};
    pose_target[0] = latest_pose_->pose.position.x;
    pose_target[1] = latest_pose_->pose.position.y;
    pose_target[2] = latest_pose_->pose.position.z;
    pose_target[3] = latest_pose_->pose.orientation.x;
    pose_target[4] = latest_pose_->pose.orientation.y;
    pose_target[5] = latest_pose_->pose.orientation.z;
    pose_target[6] = latest_pose_->pose.orientation.w;

    // Solve IK
    // Use a period large enough that the single-step IK converges close to
    // the target.  The solver internally applies velocity limits.
    const double period = 1.0 / get_parameter("ik_rate_hz").as_double();
    auto result = solver_->solve(q_current_, pose_target, period);

    if (!result.success) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "IK solve failed.");
      return;
    }

    // Publish as single-point JointTrajectory
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = latest_pose_->header.stamp;
    traj.joint_names = joints_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = result.q_command;
    point.time_from_start = rclcpp::Duration::from_seconds(period);
    traj.points.push_back(point);

    traj_pub_->publish(traj);
  }

  std::unique_ptr<task_space::PinocchioDlsSolver> solver_;
  std::vector<std::string> joints_;
  std::string base_frame_, tip_frame_;
  std::vector<double> q_current_;
  bool has_pose_{false};
  geometry_msgs::msg::PoseStamped::SharedPtr latest_pose_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::TimerBase::SharedPtr ik_timer_;
};

}  // namespace manipulation_position_controllers

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<manipulation_position_controllers::TaskSpaceIkSource>());
  rclcpp::shutdown();
  return 0;
}
