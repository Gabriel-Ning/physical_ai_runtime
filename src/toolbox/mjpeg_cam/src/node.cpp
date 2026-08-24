#include "mjpeg_cam/node.hpp"

#include "mjpeg_cam/jpeg.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>

#include <linux/videodev2.h>

namespace mjpeg_cam {
namespace {

int xioctl(int fd, unsigned long request, void *arg)
{
  int result = 0;
  do {
    result = ioctl(fd, request, arg);
  } while (result == -1 && errno == EINTR);
  return result;
}

std::string fourcc_to_string(std::uint32_t fourcc)
{
  char text[5] = {
    static_cast<char>(fourcc & 0xFFu),
    static_cast<char>((fourcc >> 8) & 0xFFu),
    static_cast<char>((fourcc >> 16) & 0xFFu),
    static_cast<char>((fourcc >> 24) & 0xFFu),
    '\0',
  };
  return std::string(text);
}

}  // namespace

MjpegCamNode::MjpegCamNode(const rclcpp::NodeOptions &options)
: Node("camera", options)
{
  video_device_ = declare_parameter("video_device", std::string("/dev/video0"));
  frame_id_ = declare_parameter("frame_id", std::string("camera_optical_frame"));
  format_ = declare_parameter("format", std::string("jpeg"));
  width_ = static_cast<int>(declare_parameter("image_width", 1280));
  height_ = static_cast<int>(declare_parameter("image_height", 720));
  framerate_ = declare_parameter("framerate", 30.0);
  const auto topic = declare_parameter("compressed_topic", std::string("image/compressed"));
  declare_parameter("camera_name", std::string(""));

  pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
    topic, rclcpp::SensorDataQoS());

  if (!open_device()) {
    throw std::runtime_error("mjpeg_cam failed to open " + video_device_);
  }
  running_ = true;
  capture_thread_ = std::thread(&MjpegCamNode::capture_loop, this);
  RCLCPP_INFO(
    get_logger(),
    "publishing original MJPEG from %s (%dx%d @ %.1f Hz) on %s",
    video_device_.c_str(), width_, height_, framerate_, topic.c_str());
}

MjpegCamNode::~MjpegCamNode()
{
  running_ = false;
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  close_device();
}

bool MjpegCamNode::open_device()
{
  fd_ = ::open(video_device_.c_str(), O_RDWR | O_NONBLOCK, 0);
  if (fd_ < 0) {
    RCLCPP_ERROR(
      get_logger(), "open(%s) failed: %s", video_device_.c_str(), std::strerror(errno));
    return false;
  }

  v4l2_capability cap{};
  if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
    RCLCPP_ERROR(get_logger(), "VIDIOC_QUERYCAP failed: %s", std::strerror(errno));
    close_device();
    return false;
  }
  if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0 ||
    (cap.capabilities & V4L2_CAP_STREAMING) == 0)
  {
    RCLCPP_ERROR(get_logger(), "%s is not a streaming capture device", video_device_.c_str());
    close_device();
    return false;
  }

  v4l2_format fmt{};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = static_cast<std::uint32_t>(width_);
  fmt.fmt.pix.height = static_cast<std::uint32_t>(height_);
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
    RCLCPP_ERROR(get_logger(), "VIDIOC_S_FMT MJPEG failed: %s", std::strerror(errno));
    close_device();
    return false;
  }
  if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG) {
    RCLCPP_ERROR(
      get_logger(),
      "device negotiated %s instead of MJPEG",
      fourcc_to_string(fmt.fmt.pix.pixelformat).c_str());
    close_device();
    return false;
  }
  width_ = static_cast<int>(fmt.fmt.pix.width);
  height_ = static_cast<int>(fmt.fmt.pix.height);

  v4l2_streamparm parm{};
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_G_PARM, &parm) == 0) {
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator =
      static_cast<std::uint32_t>(framerate_ + 0.5);
    if (parm.parm.capture.timeperframe.denominator == 0) {
      parm.parm.capture.timeperframe.denominator = 30;
    }
    if (xioctl(fd_, VIDIOC_S_PARM, &parm) < 0) {
      RCLCPP_WARN(get_logger(), "VIDIOC_S_PARM failed: %s", std::strerror(errno));
    }
  }

  v4l2_requestbuffers req{};
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0 || req.count < 2) {
    RCLCPP_ERROR(get_logger(), "VIDIOC_REQBUFS failed: %s", std::strerror(errno));
    close_device();
    return false;
  }

  buffers_.resize(req.count);
  for (std::uint32_t index = 0; index < req.count; ++index) {
    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;
    if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
      RCLCPP_ERROR(get_logger(), "VIDIOC_QUERYBUF failed: %s", std::strerror(errno));
      close_device();
      return false;
    }
    buffers_[index].length = buf.length;
    buffers_[index].start = mmap(
      nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
    if (buffers_[index].start == MAP_FAILED) {
      RCLCPP_ERROR(get_logger(), "mmap failed: %s", std::strerror(errno));
      buffers_[index].start = nullptr;
      close_device();
      return false;
    }
    if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
      RCLCPP_ERROR(get_logger(), "VIDIOC_QBUF failed: %s", std::strerror(errno));
      close_device();
      return false;
    }
  }

  auto type = static_cast<int>(V4L2_BUF_TYPE_VIDEO_CAPTURE);
  if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
    RCLCPP_ERROR(get_logger(), "VIDIOC_STREAMON failed: %s", std::strerror(errno));
    close_device();
    return false;
  }
  return true;
}

void MjpegCamNode::close_device()
{
  if (fd_ >= 0) {
    auto type = static_cast<int>(V4L2_BUF_TYPE_VIDEO_CAPTURE);
    xioctl(fd_, VIDIOC_STREAMOFF, &type);
  }
  for (auto &buffer : buffers_) {
    if (buffer.start != nullptr && buffer.start != MAP_FAILED) {
      munmap(buffer.start, buffer.length);
    }
    buffer.start = nullptr;
  }
  buffers_.clear();
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void MjpegCamNode::capture_loop()
{
  while (running_ && rclcpp::ok()) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd_, &fds);
    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 200000;
    const int ready = select(fd_ + 1, &fds, nullptr, nullptr, &timeout);
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_ERROR(get_logger(), "select failed: %s", std::strerror(errno));
      break;
    }
    if (ready == 0) {
      continue;
    }

    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
      if (errno == EAGAIN) {
        continue;
      }
      RCLCPP_ERROR(get_logger(), "VIDIOC_DQBUF failed: %s", std::strerror(errno));
      break;
    }

    if (buf.index < buffers_.size() && buffers_[buf.index].start != nullptr &&
      buf.bytesused >= 4)
    {
      const auto *data = static_cast<const std::uint8_t *>(buffers_[buf.index].start);
      const std::size_t used = jpeg_payload_size(data, buf.bytesused);
      auto msg = sensor_msgs::msg::CompressedImage();
      msg.header.stamp = now();
      msg.header.frame_id = frame_id_;
      msg.format = format_;
      msg.data.assign(data, data + used);
      pub_->publish(std::move(msg));
    }

    if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
      RCLCPP_ERROR(get_logger(), "VIDIOC_QBUF failed: %s", std::strerror(errno));
      break;
    }
  }
}

}  // namespace mjpeg_cam
