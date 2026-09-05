// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cerrno>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "camera_shm.hpp"

namespace mujoco_ros2_control_plugins
{
// One mapping and fd per camera, kept together until disconnect/replacement.
class CameraShmReader
{
public:
  CameraShmReader() = default;
  CameraShmReader(const CameraShmReader&) = delete;
  CameraShmReader& operator=(const CameraShmReader&) = delete;
  ~CameraShmReader() { close(); }

  void close()
  {
    if (mapping_) { munmap(mapping_, size_); }
    if (fd_ >= 0) { ::close(fd_); }
    mapping_ = nullptr;
    fd_ = -1;
    size_ = 0;
    last_sequence_ = 0;
  }

  bool open(const std::string& name, std::string& error)
  {
    close();
    path_ = "/dev/shm" + name;
    fd_ = shm_open(name.c_str(), O_RDONLY | O_CLOEXEC, 0);
    if (fd_ < 0) { error = std::strerror(errno); return false; }
    if (fstat(fd_, &identity_) != 0 || identity_.st_size < static_cast<off_t>(sizeof(CameraShmStatic)))
    {
      error = "SHM header is not ready"; close(); return false;
    }
    size_ = static_cast<size_t>(identity_.st_size);
    mapping_ = mmap(nullptr, size_, PROT_READ, MAP_SHARED, fd_, 0);
    if (mapping_ == MAP_FAILED)
    {
      mapping_ = nullptr; error = std::strerror(errno); close(); return false;
    }
    if (camera_shm_magic(mapping_) != kCameraShmMagic)
    {
      error = "SHM metadata is not committed"; close(); return false;
    }
    std::memcpy(&header_, mapping_, sizeof(header_));
    const uint64_t pixels = static_cast<uint64_t>(header_.width) * header_.height;
    if (header_.version != kCameraShmVersion || pixels == 0 ||
        pixels > std::numeric_limits<uint32_t>::max() / 7u ||
        header_.color_bytes != pixels * 3 || header_.depth_bytes != pixels * 4 ||
        header_.num_slots != kCameraShmNumSlots ||
        header_.slot_bytes != camera_shm_slot_bytes(header_.color_bytes, header_.depth_bytes) ||
        size_ != camera_shm_total_bytes(header_.color_bytes, header_.depth_bytes) ||
        !valid_string(header_.frame_id) || !valid_string(header_.image_topic) ||
        !valid_string(header_.depth_topic) || !valid_string(header_.info_topic) ||
        !valid_string(header_.distortion_model))
    {
      error = "invalid SHM layout/version/metadata (rebuild producer and bridge together)";
      close(); return false;
    }
    if (!current()) { error = "producer replaced SHM during attach"; close(); return false; }
    return true;
  }

  bool attached() const { return mapping_ != nullptr; }
  const CameraShmStatic& header() const { return header_; }
  uint64_t last_sequence() const { return last_sequence_; }

  // An unlinked old mapping stays valid in memory. Detect replacement by inode,
  // not magic alone: a restarted producer uses the same magic/version and name.
  bool current() const
  {
    struct stat now{};
    return mapping_ && camera_shm_magic(mapping_) == kCameraShmMagic &&
        stat(path_.c_str(), &now) == 0 && now.st_dev == identity_.st_dev &&
        now.st_ino == identity_.st_ino && now.st_size == identity_.st_size;
  }

  uint64_t latest_sequence() const
  {
    uint64_t latest = 0;
    if (!mapping_ || camera_shm_magic(mapping_) != kCameraShmMagic) { return 0; }
    for (uint32_t i = 0; i < header_.num_slots; ++i)
    {
      const auto sequence = camera_shm_sequence(slot(i));
      if (!(sequence & 1u) && sequence > latest) { latest = sequence; }
    }
    return latest;
  }

  // Inspect only slot headers first; copy the latest payload once, independent
  // of DDS reader count. Null buffers implement per-stream lazy conversion.
  bool read_latest(std::vector<uint8_t>* color, std::vector<uint8_t>* depth, int32_t& sec, uint32_t& nsec)
  {
    if (!mapping_ || camera_shm_magic(mapping_) != kCameraShmMagic) { return false; }
    for (int attempt = 0; attempt < 3; ++attempt)
    {
      uint64_t newest = last_sequence_;
      const uint8_t* selected = nullptr;
      for (uint32_t i = 0; i < header_.num_slots; ++i)
      {
        const auto sequence = camera_shm_sequence(slot(i));
        if (!(sequence & 1u) && sequence > newest)
        {
          newest = sequence;
          selected = slot(i);
        }
      }
      if (!selected) { return false; }
      std::memcpy(&sec, selected + 8, sizeof(sec));
      std::memcpy(&nsec, selected + 12, sizeof(nsec));
      if (color)
      {
        color->resize(header_.color_bytes);
        std::memcpy(color->data(), selected + sizeof(CameraShmSlotHeader), header_.color_bytes);
      }
      if (depth)
      {
        depth->resize(header_.depth_bytes);
        std::memcpy(depth->data(), selected + sizeof(CameraShmSlotHeader) + header_.color_bytes, header_.depth_bytes);
      }
      // Complete all payload reads before validating the sequence again.
      __atomic_thread_fence(__ATOMIC_SEQ_CST);
      if (camera_shm_sequence(selected) == newest && camera_shm_magic(mapping_) == kCameraShmMagic)
      {
        if (nsec >= 1000000000u) { return false; }
        last_sequence_ = newest;
        return true;
      }
    }
    return false;
  }

private:
  template <size_t N> static bool valid_string(const char (&text)[N])
  {
    return text[0] != '\0' && std::memchr(text, '\0', N) != nullptr;
  }
  const uint8_t* slot(uint32_t index) const
  {
    return static_cast<const uint8_t*>(mapping_) + sizeof(CameraShmStatic) + index * header_.slot_bytes;
  }
  int fd_{ -1 };
  void* mapping_{ nullptr };
  size_t size_{ 0 };
  struct stat identity_{};
  std::string path_;
  CameraShmStatic header_{};
  uint64_t last_sequence_{ 0 };
};
}  // namespace mujoco_ros2_control_plugins
