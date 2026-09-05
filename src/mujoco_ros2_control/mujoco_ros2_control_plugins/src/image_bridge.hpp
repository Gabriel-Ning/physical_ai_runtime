// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <set>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "camera_shm_reader.hpp"

namespace mujoco_ros2_control_plugins
{
class ImageBridge : public rclcpp::Node
{
public:
  explicit ImageBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("mujoco_image_bridge", options)
  {
    // use_sim_time is owned by rclcpp. The wall timers also work when it is true.
    const auto names = declare_parameter<std::vector<std::string>>("camera_names", std::vector<std::string>{});
    prefix_ = declare_parameter<std::string>("shm_prefix", "/pai_mj_cam_");
    const double poll_hz = declare_parameter<double>("poll_hz", 120.0);
    const auto reliability = declare_parameter<std::string>("reliability", "best_effort");
    if (prefix_.size() < 2 || prefix_[0] != '/' || prefix_.find('/', 1) != std::string::npos ||
        !std::isfinite(poll_hz) || poll_hz <= 0 || poll_hz > 10000)
    {
      throw std::invalid_argument("Invalid shm_prefix or poll_hz (expected 0 < poll_hz <= 10000)");
    }
    if (reliability == "best_effort") { qos_.best_effort(); }
    else if (reliability == "reliable") { qos_.reliable(); }
    else { throw std::invalid_argument("reliability must be best_effort or reliable"); }
    std::set<std::string> unique;
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / poll_hz));
    for (const auto& name : names)
    {
      if (name.empty() || name.find('/') != std::string::npos || !unique.insert(name).second)
      {
        throw std::invalid_argument("camera_names must be unique POSIX SHM suffixes without slashes");
      }
      auto camera = std::make_unique<Camera>();
      camera->name = name;
      camera->group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      camera->timer = create_wall_timer(period, [this, ptr = camera.get()] { poll(*ptr); }, camera->group);
      cameras_.push_back(std::move(camera));
    }
    if (names.empty()) { RCLCPP_WARN(get_logger(), "No cameras configured for SHM bridging"); }
  }

private:
  friend struct ImageBridgeTestAccess;
  using Clock = std::chrono::steady_clock;
  struct Camera
  {
    std::string name;
    CameraShmReader reader;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub, depth_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
    sensor_msgs::msg::Image color, depth;
    sensor_msgs::msg::CameraInfo info;
    rclcpp::CallbackGroup::SharedPtr group;
    rclcpp::TimerBase::SharedPtr timer;
    Clock::time_point next_attach_check{}, next_warning{};
  };

  void configure_publishers(Camera& camera)
  {
    const auto& header = camera.reader.header();
    camera.color_pub = create_publisher<sensor_msgs::msg::Image>(header.image_topic, qos_);
    camera.depth_pub = create_publisher<sensor_msgs::msg::Image>(header.depth_topic, qos_);
    camera.info_pub = create_publisher<sensor_msgs::msg::CameraInfo>(header.info_topic, qos_);
    camera.color.header.frame_id = camera.depth.header.frame_id = camera.info.header.frame_id = header.frame_id;
    camera.color.width = camera.depth.width = camera.info.width = header.width;
    camera.color.height = camera.depth.height = camera.info.height = header.height;
    camera.color.encoding = "rgb8";
    camera.depth.encoding = "32FC1";
    camera.color.step = header.width * 3;
    camera.depth.step = header.width * 4;
    camera.color.is_bigendian = camera.depth.is_bigendian = false;
    camera.color.data.resize(header.color_bytes);
    camera.depth.data.resize(header.depth_bytes);
    camera.info.distortion_model = header.distortion_model;
    std::copy(std::begin(header.k), std::end(header.k), camera.info.k.begin());
    std::copy(std::begin(header.p), std::end(header.p), camera.info.p.begin());
    camera.info.d.assign(std::begin(header.d), std::end(header.d));
    camera.info.r = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
  }

  template <class Publisher> static bool wanted(const Publisher& pub)
  {
    return pub->get_subscription_count() + pub->get_intra_process_subscription_count() != 0;
  }

  void poll(Camera& camera)
  {
    const auto now = Clock::now();
    // Check inode replacement even with no subscribers. On restart the old mmap
    // must be closed before attaching to the new producer's segment.
    if (now >= camera.next_attach_check)
    {
      camera.next_attach_check = now + std::chrono::milliseconds(500);
      if (!camera.reader.current())
      {
        camera.reader.close();
        camera.color_pub.reset();
        camera.depth_pub.reset();
        camera.info_pub.reset();
        std::string reason;
        if (!camera.reader.open(prefix_ + camera.name, reason))
        {
          if (now >= camera.next_warning)
          {
            RCLCPP_WARN(get_logger(), "Waiting for camera SHM %s%s: %s. Check CameraPlugin output: shm and camera policy.",
                prefix_.c_str(), camera.name.c_str(), reason.c_str());
            camera.next_warning = now + std::chrono::seconds(5);
          }
          return;
        }
        configure_publishers(camera);
        RCLCPP_INFO(get_logger(), "Attached %s%s (%ux%u, layout v%u) -> %s", prefix_.c_str(), camera.name.c_str(),
            camera.reader.header().width, camera.reader.header().height, camera.reader.header().version,
            camera.reader.header().image_topic);
      }
    }
    if (!camera.reader.attached()) { return; }
    const bool color = wanted(camera.color_pub), depth = wanted(camera.depth_pub), info = wanted(camera.info_pub);
    if (!color && !depth && !info) { return; }
    int32_t sec;
    uint32_t nsec;
    if (!camera.reader.read_latest(color ? &camera.color.data : nullptr, depth ? &camera.depth.data : nullptr, sec, nsec))
    {
      return;
    }
    // The captured simulation stamp is copied unchanged. Polling uses wall time.
    camera.color.header.stamp.sec = camera.depth.header.stamp.sec = camera.info.header.stamp.sec = sec;
    camera.color.header.stamp.nanosec = camera.depth.header.stamp.nanosec = camera.info.header.stamp.nanosec = nsec;
    if (color) { camera.color_pub->publish(camera.color); }
    if (depth) { camera.depth_pub->publish(camera.depth); }
    if (info) { camera.info_pub->publish(camera.info); }
  }

  std::string prefix_;
  rclcpp::QoS qos_{ 1 };
  std::vector<std::unique_ptr<Camera>> cameras_;
};
}  // namespace mujoco_ros2_control_plugins
