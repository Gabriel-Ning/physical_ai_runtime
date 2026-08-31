// Copyright 2026 Physical AI Runtime contributors
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace gamepad_teleop {

class GamepadTeleopNode final : public rclcpp::Node {
public:
  GamepadTeleopNode() : Node("gamepad_teleop") {
    // ── Clutch / Preemption ──────────────────────────────────────────────────
    // Default 9 is SDL Left Shoulder (L1/LB); 4 is raw Linux joydev L1
    clutch_button_ = declare_parameter<int>("clutch_button", 9);
    clutch_axis_ = declare_parameter<int>("clutch_axis", -1);
    require_clutch_ = declare_parameter<bool>("require_clutch", true);
    require_clutch_for_gripper_ =
        declare_parameter<bool>("require_clutch_for_gripper", false);

    // Turbo mode (L2 trigger or button)
    turbo_button_ = declare_parameter<int>("turbo_button", -1);
    turbo_axis_ = declare_parameter<int>("turbo_axis", 4);  // SDL Axis 4 = L2
    turbo_multiplier_ = declare_parameter<double>("turbo_multiplier", 2.0);
    trigger_threshold_ = declare_parameter<double>("trigger_threshold", 0.2);

    // ── 6-DoF Cartesian Twist Mapping (TSKPC) ────────────────────────────────
    axis_linear_x_ = declare_parameter<int>("axis_linear.x", 1);
    axis_linear_y_ = declare_parameter<int>("axis_linear.y", 0);
    axis_linear_z_ = declare_parameter<int>("axis_linear.z", 3);

    axis_angular_yaw_ = declare_parameter<int>("axis_angular.yaw", 2);
    axis_angular_pitch_ = declare_parameter<int>("axis_angular.pitch", -1);
    axis_angular_roll_ = declare_parameter<int>("axis_angular.roll", -1);

    axis_dpad_x_ = declare_parameter<int>("axis_dpad_x", 6);
    axis_dpad_y_ = declare_parameter<int>("axis_dpad_y", 7);

    scale_linear_x_ = declare_parameter<double>("scale_linear.x", 0.15);
    scale_linear_y_ = declare_parameter<double>("scale_linear.y", 0.15);
    scale_linear_z_ = declare_parameter<double>("scale_linear.z", 0.10);

    scale_angular_yaw_ = declare_parameter<double>("scale_angular.yaw", 0.35);
    scale_angular_pitch_ = declare_parameter<double>("scale_angular.pitch", 0.35);
    scale_angular_roll_ = declare_parameter<double>("scale_angular.roll", 0.35);

    deadzone_ = declare_parameter<double>("deadzone", 0.05);
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");

    // ── Continuous Gripper Control ───────────────────────────────────────────
    // Circle (SDL Button 1) = Open, Triangle (SDL Button 3 / joydev 2) = Close
    gripper_open_button_ = declare_parameter<int>("gripper_open_button", 1);
    gripper_close_button_ = declare_parameter<int>("gripper_close_button", 3);
    gripper_close_axis_ = declare_parameter<int>("gripper_close_axis", -1);
    gripper_joint_name_ =
        declare_parameter<std::string>("gripper_joint_name", "gripper_joint1");
    gripper_speed_m_per_s_ =
        declare_parameter<double>("gripper_speed_m_per_s", 0.025);
    gripper_min_width_ = declare_parameter<double>("gripper_min_width", 0.000);
    gripper_max_width_ = declare_parameter<double>("gripper_max_width", 0.040);
    gripper_initial_width_ =
        declare_parameter<double>("gripper_initial_width", 0.000);
    current_gripper_width_ = std::clamp(gripper_initial_width_,
                                        gripper_min_width_, gripper_max_width_);

    // ── Output Topics & Rate ─────────────────────────────────────────────────
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);
    autostart_ = declare_parameter<bool>("autostart", true);
    enabled_ = autostart_;

    twist_topic_ = declare_parameter<std::string>(
        "twist_topic", "/action_sources/gamepad/arm/twist");
    gripper_topic_ = declare_parameter<std::string>(
        "gripper_topic",
        "/action_sources/gamepad/end_effector/joint_reference");
    clutch_topic_ = declare_parameter<std::string>("clutch_topic",
                                                   "/teleop/gamepad/clutch");
    status_topic_ = declare_parameter<std::string>("status_topic",
                                                   "/teleop/gamepad/status");
    joy_topic_ = declare_parameter<std::string>("joy_topic", "/joy");

    if (publish_rate_hz_ <= 0.0 || !std::isfinite(publish_rate_hz_)) {
      throw std::invalid_argument("publish_rate_hz must be positive");
    }

    // ── Publishers & Subscribers ─────────────────────────────────────────────
    const auto command_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

    twist_pub_ =
        create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_, command_qos);
    gripper_pub_ =
        create_publisher<trajectory_msgs::msg::JointTrajectory>(gripper_topic_, command_qos);
    clutch_pub_ = create_publisher<std_msgs::msg::Bool>(clutch_topic_, command_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
        status_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_, rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
        std::bind(&GamepadTeleopNode::onJoy, this, std::placeholders::_1));

    // ── Service ──────────────────────────────────────────────────────────────
    enable_service_ = create_service<std_srvs::srv::SetBool>(
        "~/enable",
        std::bind(&GamepadTeleopNode::onEnable, this, std::placeholders::_1,
                  std::placeholders::_2));

    // ── Timer Loop ───────────────────────────────────────────────────────────
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&GamepadTeleopNode::publishLoop, this));

    RCLCPP_INFO(get_logger(),
                "gamepad_teleop initialized (rate=%.1f Hz, autostart=%s, gripper: Circle=Open, Triangle=Close).",
                publish_rate_hz_, autostart_ ? "true" : "false");
  }

private:
  void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_joy_msg_ = *msg;
    last_joy_time_ = now();
    has_joy_msg_ = true;
  }

  void onEnable(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_ = req->data;
    res->success = true;
    res->message = enabled_ ? "gamepad_teleop enabled" : "gamepad_teleop disabled";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  bool isButtonPressed(const std::vector<int32_t> &buttons, int index) const {
    if (index < 0 || index >= static_cast<int>(buttons.size())) {
      return false;
    }
    return buttons[index] != 0;
  }

  bool isAxisTriggered(const std::vector<float> &axes, int index, double threshold) const {
    if (index < 0 || index >= static_cast<int>(axes.size())) {
      return false;
    }
    return static_cast<double>(axes[index]) > threshold;
  }

  bool isClutchActive(const sensor_msgs::msg::Joy &msg) const {
    if (!require_clutch_) {
      return true;
    }
    if (clutch_button_ >= 0 && isButtonPressed(msg.buttons, clutch_button_)) {
      return true;
    }
    if (clutch_axis_ >= 0 && isAxisTriggered(msg.axes, clutch_axis_, trigger_threshold_)) {
      return true;
    }
    // Auto-adapter for driver differences (SDL has >=15 buttons where L1=9; joydev has <15 where L1=4)
    if (clutch_button_ == 9 && msg.buttons.size() < 15 && isButtonPressed(msg.buttons, 4)) {
      return true;
    }
    if (clutch_button_ == 4 && msg.buttons.size() >= 15 && isButtonPressed(msg.buttons, 9)) {
      return true;
    }
    return false;
  }

  bool isTurboActive(const sensor_msgs::msg::Joy &msg) const {
    if (turbo_button_ >= 0 && isButtonPressed(msg.buttons, turbo_button_)) {
      return true;
    }
    if (turbo_axis_ >= 0 && isAxisTriggered(msg.axes, turbo_axis_, trigger_threshold_)) {
      return true;
    }
    if (msg.buttons.size() < 15 && isButtonPressed(msg.buttons, 6)) {
      return true;
    }
    return false;
  }

  bool isGripperOpenActive(const sensor_msgs::msg::Joy &msg) const {
    return gripper_open_button_ >= 0 &&
           isButtonPressed(msg.buttons, gripper_open_button_);
  }

  bool isGripperCloseActive(const sensor_msgs::msg::Joy &msg) const {
    if (gripper_close_button_ >= 0 &&
        isButtonPressed(msg.buttons, gripper_close_button_)) {
      return true;
    }
    return gripper_close_axis_ >= 0 &&
           isAxisTriggered(msg.axes, gripper_close_axis_, trigger_threshold_);
  }

  double processAxis(const std::vector<float> &axes, int axis_index, double scale) const {
    if (axis_index < 0 || axis_index >= static_cast<int>(axes.size())) {
      return 0.0;
    }
    double raw = static_cast<double>(axes[axis_index]);
    double abs_val = std::abs(raw);
    if (abs_val <= deadzone_) {
      return 0.0;
    }
    double sign = (raw > 0.0) ? 1.0 : -1.0;
    double normalized = (abs_val - deadzone_) / (1.0 - deadzone_);
    return sign * normalized * scale;
  }

  double computePitch(const sensor_msgs::msg::Joy &msg) const {
    if (axis_angular_pitch_ >= 0) {
      double val = processAxis(msg.axes, axis_angular_pitch_, scale_angular_pitch_);
      if (std::abs(val) > 1e-5) {
        return val;
      }
    }
    // D-pad Up (SDL 11) -> +Pitch, D-pad Down (SDL 12) -> -Pitch
    if (isButtonPressed(msg.buttons, 11)) {
      return scale_angular_pitch_;
    }
    if (isButtonPressed(msg.buttons, 12)) {
      return -scale_angular_pitch_;
    }
    // Linux joydev D-pad Y axis (axis 7, up is +1.0)
    if (axis_dpad_y_ >= 0 && axis_dpad_y_ < static_cast<int>(msg.axes.size())) {
      double raw = msg.axes[axis_dpad_y_];
      if (std::abs(raw) > deadzone_) {
        return (raw > 0.0 ? 1.0 : -1.0) * scale_angular_pitch_;
      }
    }
    return 0.0;
  }

  double computeRoll(const sensor_msgs::msg::Joy &msg) const {
    if (axis_angular_roll_ >= 0) {
      double val = processAxis(msg.axes, axis_angular_roll_, scale_angular_roll_);
      if (std::abs(val) > 1e-5) {
        return val;
      }
    }
    // D-pad Left (SDL 13) -> -Roll, D-pad Right (SDL 14) -> +Roll
    if (isButtonPressed(msg.buttons, 13)) {
      return -scale_angular_roll_;
    }
    if (isButtonPressed(msg.buttons, 14)) {
      return scale_angular_roll_;
    }
    // Linux joydev D-pad X axis (axis 6, left is +1.0)
    if (axis_dpad_x_ >= 0 && axis_dpad_x_ < static_cast<int>(msg.axes.size())) {
      double raw = msg.axes[axis_dpad_x_];
      if (std::abs(raw) > deadzone_) {
        return (raw > 0.0 ? -1.0 : 1.0) * scale_angular_roll_;
      }
    }
    return 0.0;
  }

  void publishLoop() {
    sensor_msgs::msg::Joy joy_snapshot;
    bool joy_valid = false;
    bool is_enabled = false;

    const rclcpp::Time now_time = now();
    const double dt = 1.0 / publish_rate_hz_;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      is_enabled = enabled_;
      if (has_joy_msg_) {
        const double age_s = (now_time - last_joy_time_).seconds();
        if (age_s < 1.0) {
          joy_snapshot = latest_joy_msg_;
          joy_valid = true;
        }
      }
    }

    bool clutch_active = false;
    bool turbo_active = false;
    double vx = 0.0, vy = 0.0, vz = 0.0;
    double wx = 0.0, wy = 0.0, wz = 0.0;

    if (is_enabled && joy_valid) {
      clutch_active = isClutchActive(joy_snapshot);
      turbo_active = isTurboActive(joy_snapshot);

      const double mult = turbo_active ? turbo_multiplier_ : 1.0;
      const bool can_move_arm = !require_clutch_ || clutch_active;

      if (can_move_arm) {
        // Linear velocities
        vx = processAxis(joy_snapshot.axes, axis_linear_x_, scale_linear_x_) * mult;
        vy = processAxis(joy_snapshot.axes, axis_linear_y_, scale_linear_y_) * mult;
        vz = processAxis(joy_snapshot.axes, axis_linear_z_, scale_linear_z_) * mult;

        // Angular velocities (Full 6-DoF Roll/Pitch/Yaw)
        wz = processAxis(joy_snapshot.axes, axis_angular_yaw_, scale_angular_yaw_) * mult;
        wy = computePitch(joy_snapshot) * mult;
        wx = computeRoll(joy_snapshot) * mult;
      }

      // Gripper continuous servoing (Triangle = Close, Circle = Open)
      const bool can_move_gripper = !require_clutch_for_gripper_ || clutch_active;
      if (can_move_gripper) {
        const bool open_active = isGripperOpenActive(joy_snapshot);
        const bool close_active = isGripperCloseActive(joy_snapshot);

        if (open_active && !close_active) {
          current_gripper_width_ += gripper_speed_m_per_s_ * dt;
        } else if (close_active && !open_active) {
          current_gripper_width_ -= gripper_speed_m_per_s_ * dt;
        }
        current_gripper_width_ = std::clamp(
            current_gripper_width_, gripper_min_width_, gripper_max_width_);
      }
    }

    // ── 1. Publish TwistStamped ──────────────────────────────────────────────
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = now_time;
    twist_msg.header.frame_id = frame_id_;
    twist_msg.twist.linear.x = vx;
    twist_msg.twist.linear.y = vy;
    twist_msg.twist.linear.z = vz;
    twist_msg.twist.angular.x = wx;
    twist_msg.twist.angular.y = wy;
    twist_msg.twist.angular.z = wz;
    twist_pub_->publish(twist_msg);

    // ── 2. Publish Gripper Trajectory ────────────────────────────────────────
    trajectory_msgs::msg::JointTrajectory gripper_msg;
    gripper_msg.header.stamp = now_time;
    gripper_msg.joint_names = {gripper_joint_name_};
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = {current_gripper_width_};
    pt.time_from_start = rclcpp::Duration::from_seconds(0.0);
    gripper_msg.points.push_back(pt);
    gripper_pub_->publish(gripper_msg);

    // ── 3. Publish Clutch Signal ─────────────────────────────────────────────
    std_msgs::msg::Bool clutch_msg;
    clutch_msg.data = is_enabled && clutch_active;
    clutch_pub_->publish(clutch_msg);

    // ── 4. Publish Status Diagnostics ────────────────────────────────────────
    std::ostringstream json;
    json << "{"
         << "\"enabled\":" << (is_enabled ? "true" : "false") << ","
         << "\"joy_connected\":" << (joy_valid ? "true" : "false") << ","
         << "\"clutch\":" << (clutch_active ? "true" : "false") << ","
         << "\"turbo\":" << (turbo_active ? "true" : "false") << ","
         << "\"gripper_width\":" << current_gripper_width_ << ","
         << "\"twist\":{"
         << "\"linear\":[" << vx << "," << vy << "," << vz << "],"
         << "\"angular\":[" << wx << "," << wy << "," << wz << "]"
         << "}"
         << "}";
    std_msgs::msg::String status_msg;
    status_msg.data = json.str();
    status_pub_->publish(status_msg);
  }

  // Parameters
  int clutch_button_;
  int clutch_axis_;
  bool require_clutch_;
  bool require_clutch_for_gripper_;
  int turbo_button_;
  int turbo_axis_;
  double turbo_multiplier_;
  double trigger_threshold_;

  int axis_linear_x_;
  int axis_linear_y_;
  int axis_linear_z_;
  int axis_angular_yaw_;
  int axis_angular_pitch_;
  int axis_angular_roll_;
  int axis_dpad_x_;
  int axis_dpad_y_;

  double scale_linear_x_;
  double scale_linear_y_;
  double scale_linear_z_;
  double scale_angular_yaw_;
  double scale_angular_pitch_;
  double scale_angular_roll_;

  double deadzone_;
  std::string frame_id_;

  int gripper_open_button_;
  int gripper_close_button_;
  int gripper_close_axis_;
  std::string gripper_joint_name_;
  double gripper_speed_m_per_s_;
  double gripper_min_width_;
  double gripper_max_width_;
  double gripper_initial_width_;

  double publish_rate_hz_;
  bool autostart_;
  std::atomic<bool> enabled_{true};

  std::string twist_topic_;
  std::string gripper_topic_;
  std::string clutch_topic_;
  std::string status_topic_;
  std::string joy_topic_;

  // State
  std::mutex mutex_;
  sensor_msgs::msg::Joy latest_joy_msg_;
  rclcpp::Time last_joy_time_{0, 0, RCL_ROS_TIME};
  bool has_joy_msg_{false};
  double current_gripper_width_{0.000};

  // ROS entities
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clutch_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace gamepad_teleop

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gamepad_teleop::GamepadTeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
