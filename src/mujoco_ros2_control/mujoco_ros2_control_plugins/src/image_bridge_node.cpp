// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
#include "image_bridge.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try
  {
    auto node = std::make_shared<mujoco_ros2_control_plugins::ImageBridge>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("mujoco_image_bridge"), "%s", e.what());
    result = 1;
  }
  if (rclcpp::ok()) { rclcpp::shutdown(); }
  return result;
}
