// Copyright 2026 Physical AI Runtime contributors
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <piper/control_types.h>
#include <piper/mit_config.h>
#include <piper/model.h>
#include <piper/robot.h>
#include <piper/teaching_pendant.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace piper_leader_teleop {

using namespace std::chrono_literals;

namespace {
constexpr double kFingerTravelPerOpeningWidth = 0.5;
constexpr double kMaxOpeningWidthM = 0.08;

enum class LeaderMode {
  kDisabled = 0,
  kShadowTracking = 1,       // Position servoing tracking follower joints (Non-preempting)
  kPassiveGravityComp = 2,   // 500Hz MIT gravity comp 0-G float (Non-preempting)
  kActivePreempt = 3,        // 500Hz MIT gravity comp + Active Teleop Streaming (Preempting)
};

const char *modeToString(LeaderMode mode) {
  switch (mode) {
    case LeaderMode::kDisabled:
      return "disabled";
    case LeaderMode::kShadowTracking:
      return "shadow_tracking";
    case LeaderMode::kPassiveGravityComp:
      return "passive_gravity_comp";
    case LeaderMode::kActivePreempt:
      return "active_preempt";
  }
  return "unknown";
}

LeaderMode modeFromString(const std::string &str) {
  if (str == "shadow" || str == "shadow_tracking") {
    return LeaderMode::kShadowTracking;
  }
  if (str == "passive" || str == "passive_gravity" || str == "passive_gravity_comp") {
    return LeaderMode::kPassiveGravityComp;
  }
  if (str == "preempt" || str == "active_preempt") {
    return LeaderMode::kActivePreempt;
  }
  return LeaderMode::kDisabled;
}
}  // namespace

class PiperLeaderNode final : public rclcpp::Node {
public:
  PiperLeaderNode() : Node("piper_leader") {
    can_interface_ = declare_parameter<std::string>("can_interface", "can0");
    leader_robot_description_ =
        declare_parameter<std::string>("leader_robot_description", "");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 200.0);
    feedback_timeout_s_ = declare_parameter<double>("feedback_timeout_s", 0.1);
    mit_kd_effort_damping_ =
        declare_parameter<double>("mit_kd_effort_damping", 0.0);
    restore_pendant_servo_on_stop_ =
        declare_parameter<bool>("restore_pendant_servo_on_stop", false);

    // Follower tracking parameters (Shadow mode)
    follower_joint_topic_ = declare_parameter<std::string>(
        "follower_joint_state_topic", "/joint_states");
    follower_joint_names_ = declare_parameter<std::vector<std::string>>(
        "follower_joint_names",
        {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    default_mode_str_ = declare_parameter<std::string>("default_mode", "shadow");
    fallback_mode_str_ = declare_parameter<std::string>("fallback_mode", "shadow");
    shadow_velocity_limit_rad_s_ =
        declare_parameter<double>("shadow_velocity_limit_rad_s", 2.0);
    autostart_ = declare_parameter<bool>("autostart", true);

    // Output Teleop Topics
    joint_topic_ = declare_parameter<std::string>(
        "joint_reference_topic",
        "/action_sources/piper_leader/arm/joint_reference");
    pendant_topic_ = declare_parameter<std::string>(
        "pendant_state_topic", "/teleop/piper_leader/pendant_state");
    gripper_topic_ = declare_parameter<std::string>(
        "gripper_reference_topic",
        "/action_sources/piper_leader/end_effector/joint_reference");
    status_topic_ = declare_parameter<std::string>(
        "status_topic", "/teleop/piper_leader/status");
    gripper_joint_name_ =
        declare_parameter<std::string>("gripper_joint_name", "gripper_joint1");
    follower_gripper_joint_name_ = declare_parameter<std::string>(
        "follower_gripper_joint_name", gripper_joint_name_);
    joint_names_ = declare_parameter<std::vector<std::string>>(
        "joint_names",
        {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});

    if (joint_names_.size() != piper::kNumJoints || publish_rate_hz_ <= 0.0 ||
        !std::isfinite(publish_rate_hz_) || feedback_timeout_s_ <= 0.0 ||
        !std::isfinite(feedback_timeout_s_) || mit_kd_effort_damping_ < 0.0 ||
        !std::isfinite(mit_kd_effort_damping_)) {
      throw std::invalid_argument("invalid Piper leader parameters");
    }

    fallback_mode_ = modeFromString(fallback_mode_str_);

    const auto command_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    joint_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        joint_topic_, command_qos);
    gripper_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        gripper_topic_, command_qos);
    pendant_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        pendant_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    status_pub_ = create_publisher<std_msgs::msg::String>(
        status_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    // Subscriber to follower joint states for Shadow Tracking mode
    follower_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        follower_joint_topic_, rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
        std::bind(&PiperLeaderNode::onFollowerJointState, this, std::placeholders::_1));

    // Lifecycle Services
    enable_service_ = create_service<std_srvs::srv::SetBool>(
        "~/enable", std::bind(&PiperLeaderNode::onEnable, this,
                              std::placeholders::_1, std::placeholders::_2));
    preempt_service_ = create_service<std_srvs::srv::SetBool>(
        "~/preempt", std::bind(&PiperLeaderNode::onPreempt, this,
                               std::placeholders::_1, std::placeholders::_2));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&PiperLeaderNode::publishLatest, this));

    if (autostart_) {
      try {
        LeaderMode target = modeFromString(default_mode_str_);
        if (target == LeaderMode::kDisabled) {
          target = LeaderMode::kShadowTracking;
        }
        setModeLocked(target);
        RCLCPP_INFO(get_logger(), "Piper leader autostarted in %s mode.",
                    modeToString(mode_));
      } catch (const std::exception &error) {
        RCLCPP_WARN(
            get_logger(),
            "Piper leader autostart deferred (waiting for ~/enable): %s",
            error.what());
      }
    } else {
      RCLCPP_INFO(get_logger(),
                  "Piper leader initialized. Call '%s/enable' to activate.",
                  get_fully_qualified_name());
    }
  }

  ~PiperLeaderNode() override {
    std::lock_guard<std::mutex> lock(state_mutex_);
    stopLocked();
  }

private:
  void onFollowerJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!msg || msg->name.empty() || msg->position.empty()) {
      return;
    }
    std::lock_guard<std::mutex> lock(follower_mutex_);
    std::array<double, piper::kNumJoints> target_q{};
    bool all_found = true;

    for (size_t i = 0; i < follower_joint_names_.size() && i < piper::kNumJoints; ++i) {
      const auto &name = follower_joint_names_[i];
      auto it = std::find(msg->name.begin(), msg->name.end(), name);
      if (it != msg->name.end()) {
        size_t idx = std::distance(msg->name.begin(), it);
        if (idx < msg->position.size()) {
          target_q[i] = msg->position[idx];
        } else {
          all_found = false;
        }
      } else {
        all_found = false;
      }
    }

    if (all_found) {
      latest_follower_q_ = target_q;
      has_follower_q_ = true;
      last_follower_update_time_ = std::chrono::steady_clock::now();
    }

    if (!follower_gripper_joint_name_.empty()) {
      auto it_g = std::find(msg->name.begin(), msg->name.end(), follower_gripper_joint_name_);
      if (it_g != msg->name.end()) {
        size_t g_idx = std::distance(msg->name.begin(), it_g);
        if (g_idx < msg->position.size()) {
          latest_follower_gripper_pos_ = msg->position[g_idx];
          has_follower_gripper_pos_ = true;
        }
      }
    }
  }

  void onEnable(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
      if (request->data) {
        LeaderMode target = modeFromString(default_mode_str_);
        if (target == LeaderMode::kDisabled) {
          target = LeaderMode::kShadowTracking;
        }
        setModeLocked(target);
        response->message = std::string("Piper leader enabled in mode: ") + modeToString(mode_);
      } else {
        setModeLocked(LeaderMode::kDisabled);
        response->message = "Piper leader disabled and stopped";
      }
      response->success = true;
    } catch (const std::exception &error) {
      setModeLocked(LeaderMode::kDisabled);
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "Piper leader enable failed: %s", error.what());
    }
  }

  void onPreempt(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                 std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
      if (mode_ == LeaderMode::kDisabled) {
        response->success = false;
        response->message = "Cannot preempt while Piper leader is disabled. Enable first.";
        return;
      }

      if (request->data) {
        // Enter ACTIVE_PREEMPT mode (Instantaneous if from Shadow mode)
        setModeLocked(LeaderMode::kActivePreempt);
        response->message = "Piper leader active in 0-G Preempt Teleop mode";
      } else {
        // Exit ACTIVE_PREEMPT mode -> Fall back to fallback_mode (Shadow or Passive)
        std::string current_fallback = fallback_mode_str_;
        get_parameter("fallback_mode", current_fallback);
        fallback_mode_ = modeFromString(current_fallback);
        setModeLocked(fallback_mode_);
        response->message = std::string("Piper leader released preempt, returned to: ") + modeToString(mode_);
      }
      response->success = true;
    } catch (const std::exception &error) {
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "Piper leader preempt transition failed: %s", error.what());
    }
  }

  void setModeLocked(LeaderMode target_mode) {
    if (mode_ == target_mode) {
      return;
    }

    RCLCPP_INFO(get_logger(), "Transitioning Piper leader mode: %s -> %s",
                modeToString(mode_), modeToString(target_mode));

    if (target_mode == LeaderMode::kDisabled) {
      stopLocked();
      mode_ = LeaderMode::kDisabled;
      return;
    }

    // Ensure hardware and model are initialized
    ensureHardwareInitializedLocked();

    const LeaderMode old_mode = mode_;
    mode_ = target_mode;

    // Handle switching between position control (Shadow) and torque control (Gravity comp / Preempt)
    if (target_mode == LeaderMode::kShadowTracking) {
      last_commanded_pendant_width_ = -1.0;
      startShadowTrackingControlLocked();
    } else if (target_mode == LeaderMode::kPassiveGravityComp || target_mode == LeaderMode::kActivePreempt) {
      if (pendant_) {
        try {
          pendant_->enableFingerFreeMove();
        } catch (...) {}
      }
      if (old_mode != LeaderMode::kPassiveGravityComp && old_mode != LeaderMode::kActivePreempt) {
        startTorqueControlLocked();
      }
    }
  }

  void ensureHardwareInitializedLocked() {
    if (robot_ && model_ && pendant_) {
      return;
    }

    if (leader_robot_description_.empty() ||
        leader_robot_description_.find("<robot") == std::string::npos) {
      throw std::runtime_error(
          "leader_robot_description is empty or is not an expanded URDF");
    }

    const auto unique_suffix = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
    temporary_model_path_ = std::filesystem::temp_directory_path() /
                            ("piper_leader_model_" + unique_suffix + ".urdf");
    {
      std::ofstream model_file(temporary_model_path_,
                               std::ios::out | std::ios::trunc);
      if (!model_file) {
        throw std::runtime_error("failed to create temporary leader URDF");
      }
      model_file << leader_robot_description_;
    }

    robot_ = std::make_unique<piper::Robot>(
        can_interface_, piper::RealtimeConfig::kIgnore, false);
    model_ = std::make_unique<piper::Model>(
        robot_->loadModel(temporary_model_path_.string()));
    if (!model_->isLoaded()) {
      throw std::runtime_error("Piper leader gravity model failed to load");
    }

    piper::MitGains gains{};
    gains.kd_effort_damping = mit_kd_effort_damping_;
    robot_->setMitGains(gains);

    const auto initial_state = robot_->readOnce();
    if (!initial_state.valid || !initial_state.coherent ||
        !isFresh(initial_state.feedback_receive_time)) {
      throw std::runtime_error(
          "Piper leader has no fresh coherent joint feedback");
    }
    robot_->enableArm();

    pendant_ = std::make_unique<piper::TeachingPendant>(can_interface_, false);
    pendant_->enableFingerFreeMove();

    last_arm_sequence_ = 0;
    last_pendant_sequence_ = 0;
  }

  void startShadowTrackingControlLocked() {
    if (!robot_) return;
    try {
      robot_->stop();
    } catch (...) {}

    current_shadow_target_ = robot_->readLatest().q;

    robot_->controlAsync([this](const piper::RobotState &state,
                                piper::Duration dt) -> piper::JointPositions {
      piper::JointPositions command{state.q};
      if (mode_ != LeaderMode::kShadowTracking) {
        command.motion_finished = true;
        return command;
      }

      std::array<double, piper::kNumJoints> desired_q = state.q;
      {
        std::lock_guard<std::mutex> lock(follower_mutex_);
        if (has_follower_q_) {
          desired_q = latest_follower_q_;
        }
      }

      // Safe velocity-limited rate ramp towards desired follower pose
      const double dt_s = std::max(0.001, dt.toSec());
      const double max_step = shadow_velocity_limit_rad_s_ * dt_s;

      for (size_t i = 0; i < piper::kNumJoints; ++i) {
        double delta = desired_q[i] - current_shadow_target_[i];
        delta = std::clamp(delta, -max_step, max_step);
        current_shadow_target_[i] += delta;
        command.q[i] = current_shadow_target_[i];
      }

      return command;
    }, piper::ControlType::kInternalJointPos);
  }

  void startTorqueControlLocked() {
    if (!robot_) return;
    try {
      robot_->stop();
    } catch (...) {}

    robot_->controlAsync([this](const piper::RobotState &state,
                                piper::Duration) -> piper::Torques {
      piper::Torques command{model_->gravity(state)};
      if (mode_ != LeaderMode::kPassiveGravityComp && mode_ != LeaderMode::kActivePreempt) {
        command.motion_finished = true;
        return command;
      }
      return command;
    });
  }

  void stopLocked() noexcept {
    mode_ = LeaderMode::kDisabled;
    if (robot_) {
      try {
        robot_->stop();
      } catch (const std::exception &error) {
        RCLCPP_WARN(get_logger(), "Piper leader stop failed: %s", error.what());
      }
    }
    if (pendant_ && restore_pendant_servo_on_stop_) {
      try {
        pendant_->disableFingerFreeMove();
      } catch (const std::exception &error) {
        RCLCPP_WARN(get_logger(), "Pendant servo restore failed: %s",
                    error.what());
      }
    }
    pendant_.reset();
    model_.reset();
    robot_.reset();
    if (!temporary_model_path_.empty()) {
      std::error_code error;
      std::filesystem::remove(temporary_model_path_, error);
      temporary_model_path_.clear();
    }
  }

  template <typename TimePoint> bool isFresh(TimePoint receive_time) const {
    if (receive_time == TimePoint{}) {
      return false;
    }
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         receive_time)
               .count() <= feedback_timeout_s_;
  }

  void publishLatest() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (mode_ == LeaderMode::kDisabled || !robot_ || !pendant_) {
      publishStatus(nullptr, nullptr);
      return;
    }

    const auto arm = robot_->readLatest();
    const auto pendant = pendant_->readLatest();
    const bool arm_usable =
        arm.valid && arm.coherent && isFresh(arm.feedback_receive_time) &&
        arm.device_status != piper::FeedbackDeviceStatus::kError;
    const bool pendant_usable =
        pendant.valid && isFresh(pendant.feedback_receive_time) &&
        pendant.device_status != piper::FeedbackDeviceStatus::kError;
    const auto stamp = now();

    // In SHADOW_TRACKING mode, servo the teaching pendant to mirror follower gripper
    if (mode_ == LeaderMode::kShadowTracking) {
      if (pendant_usable && has_follower_gripper_pos_) {
        double target_width = latest_follower_gripper_pos_;
        if (kFingerTravelPerOpeningWidth > 1e-6) {
          target_width = latest_follower_gripper_pos_ / kFingerTravelPerOpeningWidth;
        }
        target_width = std::clamp(target_width, 0.0, kMaxOpeningWidthM);

        if (std::abs(target_width - last_commanded_pendant_width_) > 0.0005) {
          try {
            pendant_->command(target_width, 1.5);
            last_commanded_pendant_width_ = target_width;
          } catch (...) {}
        }
      }
    }

    // Stream action commands only when in ACTIVE_PREEMPT mode
    if (mode_ == LeaderMode::kActivePreempt) {
      if (arm_usable && arm.feedback_sequence != last_arm_sequence_) {
        trajectory_msgs::msg::JointTrajectory message;
        message.header.stamp = stamp;
        message.joint_names = joint_names_;
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.assign(arm.q.begin(), arm.q.end());
        point.velocities.assign(arm.dq.begin(), arm.dq.end());
        point.effort.assign(arm.tau_J.begin(), arm.tau_J.end());
        message.points.push_back(std::move(point));
        joint_pub_->publish(message);
        last_arm_sequence_ = arm.feedback_sequence;
      }

      if (pendant_usable && pendant.feedback_sequence != last_pendant_sequence_) {
        const double opening_width =
            std::clamp(pendant.width, 0.0, kMaxOpeningWidthM);
        const double finger_position =
            opening_width * kFingerTravelPerOpeningWidth;

        sensor_msgs::msg::JointState raw_message;
        raw_message.header.stamp = stamp;
        raw_message.name = {gripper_joint_name_};
        raw_message.position = {pendant.width};
        raw_message.effort = {pendant.force};
        pendant_pub_->publish(raw_message);

        trajectory_msgs::msg::JointTrajectory gripper_message;
        gripper_message.header.stamp = stamp;
        gripper_message.joint_names = {gripper_joint_name_};
        trajectory_msgs::msg::JointTrajectoryPoint gripper_point;
        gripper_point.positions = {finger_position};
        gripper_message.points.push_back(std::move(gripper_point));
        gripper_pub_->publish(gripper_message);

        last_pendant_sequence_ = pendant.feedback_sequence;
      }
    }

    publishStatus(&arm, &pendant);
  }

  void publishStatus(const piper::RobotState *arm,
                     const piper::TeachingPendantState *pendant) {
    std::ostringstream json;
    json << "{\"schema_version\":2"
         << ",\"mode\":\"" << modeToString(mode_) << "\""
         << ",\"active\":" << (mode_ != LeaderMode::kDisabled ? "true" : "false")
         << ",\"preempted\":" << (mode_ == LeaderMode::kActivePreempt ? "true" : "false")
         << ",\"fallback_mode\":\"" << modeToString(fallback_mode_) << "\"";

    if (arm) {
      const double age =
          arm->feedback_receive_time == std::chrono::steady_clock::time_point{}
              ? -1.0
              : std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                              arm->feedback_receive_time)
                    .count();
      json << ",\"arm_sequence\":" << arm->feedback_sequence
           << ",\"arm_valid\":" << (arm->valid ? "true" : "false")
           << ",\"arm_coherent\":" << (arm->coherent ? "true" : "false")
           << ",\"arm_source_age_s\":" << age;
    }
    if (pendant) {
      const double age =
          pendant->feedback_receive_time ==
                  std::chrono::steady_clock::time_point{}
              ? -1.0
              : std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                              pendant->feedback_receive_time)
                    .count();
      json << ",\"pendant_sequence\":" << pendant->feedback_sequence
           << ",\"pendant_valid\":" << (pendant->valid ? "true" : "false")
           << ",\"pendant_source_age_s\":" << age;
    }
    json << '}';
    std_msgs::msg::String message;
    message.data = json.str();
    status_pub_->publish(message);
  }

  std::mutex state_mutex_;
  LeaderMode mode_{LeaderMode::kDisabled};
  LeaderMode fallback_mode_{LeaderMode::kShadowTracking};
  std::string default_mode_str_;
  std::string fallback_mode_str_;
  bool autostart_{true};

  std::unique_ptr<piper::Robot> robot_;
  std::unique_ptr<piper::Model> model_;
  std::unique_ptr<piper::TeachingPendant> pendant_;

  // Follower Shadow tracking state
  std::mutex follower_mutex_;
  std::array<double, piper::kNumJoints> latest_follower_q_{};
  std::array<double, piper::kNumJoints> current_shadow_target_{};
  bool has_follower_q_{false};
  std::chrono::steady_clock::time_point last_follower_update_time_{};
  double shadow_velocity_limit_rad_s_{2.0};
  std::string follower_joint_topic_;
  std::vector<std::string> follower_joint_names_;
  std::string follower_gripper_joint_name_;
  double latest_follower_gripper_pos_{0.0};
  bool has_follower_gripper_pos_{false};
  double last_commanded_pendant_width_{-1.0};

  std::string can_interface_;
  std::string leader_robot_description_;
  std::filesystem::path temporary_model_path_;
  double publish_rate_hz_{200.0};
  double feedback_timeout_s_{0.1};
  double mit_kd_effort_damping_{0.0};
  bool restore_pendant_servo_on_stop_{false};
  std::string joint_topic_;
  std::string pendant_topic_;
  std::string gripper_topic_;
  std::string status_topic_;
  std::string gripper_joint_name_;
  std::vector<std::string> joint_names_;
  uint64_t last_arm_sequence_{0};
  uint64_t last_pendant_sequence_{0};

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pendant_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr follower_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr preempt_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

} // namespace piper_leader_teleop

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<piper_leader_teleop::PiperLeaderNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("piper_leader"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
