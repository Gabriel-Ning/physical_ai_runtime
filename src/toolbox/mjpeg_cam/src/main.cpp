#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mjpeg_cam/node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mjpeg_cam::MjpegCamNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
