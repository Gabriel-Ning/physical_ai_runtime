// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <stdexcept>
#include "camera_shm_reader.hpp"

struct CameraShmTestWriter
{
  explicit CameraShmTestWriter(uint32_t width = 3, uint32_t height = 1)
  {
    static unsigned instance = 0;
    name = "/pai_camera_test_" + std::to_string(getpid()) + "_" + std::to_string(++instance);
    recreate(width, height);
  }
  ~CameraShmTestWriter() { release(); shm_unlink(name.c_str()); }
  void release()
  {
    if (mapping) { munmap(mapping, size); mapping = nullptr; }
    if (fd >= 0) { ::close(fd); fd = -1; }
  }
  void recreate(uint32_t width = 3, uint32_t height = 1)
  {
    using namespace mujoco_ros2_control_plugins;
    release();
    shm_unlink(name.c_str());
    fd = shm_open(name.c_str(), O_RDWR | O_CREAT | O_EXCL, 0600);
    size = camera_shm_total_bytes(width * height * 3, width * height * 4);
    if (fd < 0 || ftruncate(fd, size) != 0) { throw std::runtime_error("cannot create test SHM"); }
    mapping = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (mapping == MAP_FAILED) { mapping = nullptr; throw std::runtime_error("cannot map test SHM"); }
    std::memset(mapping, 0, size);
    auto& h = header();
    h.version = kCameraShmVersion;
    h.width = width; h.height = height;
    h.color_bytes = width * height * 3; h.depth_bytes = width * height * 4;
    h.num_slots = kCameraShmNumSlots;
    h.slot_bytes = camera_shm_slot_bytes(h.color_bytes, h.depth_bytes);
    std::strcpy(h.frame_id, "test_optical_frame");
    std::strcpy(h.image_topic, "/shm_test/color");
    std::strcpy(h.depth_topic, "/shm_test/depth");
    std::strcpy(h.info_topic, "/shm_test/info");
    std::strcpy(h.distortion_model, "plumb_bob");
    h.k[0] = 123.5; h.p[0] = 123.5;
    color.resize(h.color_bytes); depth.resize(h.depth_bytes);
    sequence = 0;
    camera_shm_set_magic(mapping, kCameraShmMagic);
  }
  mujoco_ros2_control_plugins::CameraShmStatic& header()
  {
    return *static_cast<mujoco_ros2_control_plugins::CameraShmStatic*>(mapping);
  }
  void write(uint8_t value)
  {
    std::fill(color.begin(), color.end(), value);
    std::fill(depth.begin(), depth.end(), value);
    mujoco_ros2_control_plugins::camera_shm_write_frame(mapping, sequence, value, 123,
        color.data(), depth.data());
  }
  std::string name;
  int fd{ -1 };
  void* mapping{ nullptr };
  size_t size{ 0 };
  uint64_t sequence{ 0 };
  std::vector<uint8_t> color, depth;
};
