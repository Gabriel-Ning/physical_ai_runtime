// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
#include <atomic>
#include <thread>
#include <gtest/gtest.h>
#include "image_bridge.hpp"
#include "camera_shm_test_writer.hpp"

namespace mujoco_ros2_control_plugins
{
struct ImageBridgeTestAccess
{
  // Call only while executor is stopped, to avoid racing its timer callback.
  static uint64_t consumed(const ImageBridge& bridge) { return bridge.cameras_.front()->reader.last_sequence(); }
};
}

using namespace std::chrono_literals;
using namespace mujoco_ros2_control_plugins;

TEST(ImageBridgeTest, LazyFanoutUsesCapturedStampAndReattachesAfterRestart)
{
  rclcpp::init(0, nullptr);
  CameraShmTestWriter writer;
  const auto split = writer.name.find_last_of('_') + 1;
  auto bridge = std::make_shared<ImageBridge>(rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("camera_names", std::vector<std::string>{ writer.name.substr(split) }),
      rclcpp::Parameter("shm_prefix", writer.name.substr(0, split)),
      // No /clock is published: wall polling must still attach and forward.
      rclcpp::Parameter("use_sim_time", true) }));
  auto consumer = std::make_shared<rclcpp::Node>("shm_bridge_consumer");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge);
  executor.add_node(consumer);
  auto spin_for = [&](auto duration) {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end)
    {
      executor.spin_some();
      std::this_thread::sleep_for(2ms);
    }
  };
  writer.write(7);
  spin_for(100ms);
  EXPECT_EQ(ImageBridgeTestAccess::consumed(*bridge), 0u);  // no subscribers: no payload read
  auto qos = rclcpp::QoS(1).best_effort();
  sensor_msgs::msg::Image::ConstSharedPtr received_a, received_b, received_depth;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr received_info;
  auto a = consumer->create_subscription<sensor_msgs::msg::Image>("/shm_test/color", qos,
      [&](sensor_msgs::msg::Image::ConstSharedPtr msg) { received_a = msg; });
  auto b = consumer->create_subscription<sensor_msgs::msg::Image>("/shm_test/color", qos,
      [&](sensor_msgs::msg::Image::ConstSharedPtr msg) { received_b = msg; });
  auto d = consumer->create_subscription<sensor_msgs::msg::Image>("/shm_test/depth", qos,
      [&](sensor_msgs::msg::Image::ConstSharedPtr msg) { received_depth = msg; });
  auto i = consumer->create_subscription<sensor_msgs::msg::CameraInfo>("/shm_test/info", qos,
      [&](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { received_info = msg; });
  // Keep a new sample available during DDS discovery.
  for (unsigned attempt = 0; attempt < 50 && !(received_a && received_b && received_depth && received_info); ++attempt)
  {
    writer.write(8);
    spin_for(20ms);
  }
  EXPECT_TRUE(received_a && received_b && received_depth && received_info);
  if (received_a && received_b && received_depth && received_info)
  {
    EXPECT_EQ(*received_a, *received_b);
    EXPECT_EQ(received_a->data, writer.color);
    EXPECT_EQ(received_depth->data, writer.depth);
    EXPECT_EQ(received_a->header.stamp.sec, 8);
    EXPECT_EQ(received_a->header.stamp.nanosec, 123u);
    EXPECT_EQ(received_a->header, received_depth->header);
    EXPECT_EQ(received_a->header, received_info->header);
    EXPECT_EQ(received_a->encoding, "rgb8");
    EXPECT_EQ(received_depth->encoding, "32FC1");
    EXPECT_EQ(received_info->k[0], 123.5);
  }
  const auto publishers = consumer->get_publishers_info_by_topic("/shm_test/color");
  EXPECT_EQ(publishers.size(), 1u);
  if (!publishers.empty()) { EXPECT_EQ(publishers[0].node_name(), "mujoco_image_bridge"); }
  received_a.reset();
  writer.recreate(4, 2);  // same name, new inode, different dimensions, reset sequence
  for (unsigned attempt = 0; attempt < 100 && (!received_a || received_a->width != 4); ++attempt)
  {
    writer.write(9);
    spin_for(20ms);
  }
  EXPECT_TRUE(received_a && received_a->width == 4 && received_a->height == 2);
  if (received_a) { EXPECT_EQ(received_a->header.stamp.sec, 9); }
  executor.remove_node(consumer);
  executor.remove_node(bridge);
  rclcpp::shutdown();
}
