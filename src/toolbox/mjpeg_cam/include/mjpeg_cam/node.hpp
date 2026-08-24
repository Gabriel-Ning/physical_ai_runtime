#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace mjpeg_cam {

class MjpegCamNode : public rclcpp::Node
{
public:
  explicit MjpegCamNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~MjpegCamNode() override;

  MjpegCamNode(const MjpegCamNode &) = delete;
  MjpegCamNode &operator=(const MjpegCamNode &) = delete;

private:
  struct Buffer
  {
    void *start{nullptr};
    std::size_t length{0};
  };

  bool open_device();
  void close_device();
  void capture_loop();

  std::string video_device_;
  std::string frame_id_;
  std::string format_;
  int width_{0};
  int height_{0};
  double framerate_{30.0};

  int fd_{-1};
  std::vector<Buffer> buffers_;
  std::atomic<bool> running_{false};
  std::thread capture_thread_;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
};

}  // namespace mjpeg_cam
