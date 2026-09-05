// Shared-memory layout for CameraPlugin -> mujoco_image_bridge.
// Canonical packed POSIX SHM ABI shared by the producer and native bridge.

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace mujoco_ros2_control_plugins
{

inline constexpr uint32_t kCameraShmMagic = 0x4D4A4331u;  // 'MJC1'
// v2: globally increasing per-camera sequence and 8-byte aligned slots.
// v1 used per-slot counters, which made every other frame look like a duplicate.
inline constexpr uint32_t kCameraShmVersion = 2u;
inline constexpr uint32_t kCameraShmNumSlots = 2u;
inline constexpr std::size_t kCameraShmFrameIdLen = 64u;
inline constexpr std::size_t kCameraShmTopicLen = 128u;
inline constexpr std::size_t kCameraShmDistortionModelLen = 32u;

#pragma pack(push, 1)
struct CameraShmStatic
{
  uint32_t magic;
  uint32_t version;
  uint32_t width;
  uint32_t height;
  uint32_t color_bytes;
  uint32_t depth_bytes;
  uint32_t num_slots;
  uint32_t slot_bytes;  // seq+stamp + color + depth
  char frame_id[kCameraShmFrameIdLen];
  char image_topic[kCameraShmTopicLen];
  char depth_topic[kCameraShmTopicLen];
  char info_topic[kCameraShmTopicLen];
  double k[9];
  double p[12];
  double d[5];
  char distortion_model[kCameraShmDistortionModelLen];
};

struct CameraShmSlotHeader
{
  // Globally increasing per-camera seqlock (not a counter local to this slot).
  // Odd => writer in progress, even => stable. Access with shm_load/store helpers.
  uint64_t seq;
  int32_t stamp_sec;
  uint32_t stamp_nsec;
};
#pragma pack(pop)

inline std::size_t camera_shm_slot_bytes(uint32_t color_bytes, uint32_t depth_bytes)
{
  const auto bytes = sizeof(CameraShmSlotHeader) + static_cast<std::size_t>(color_bytes) + depth_bytes;
  return (bytes + 7u) & ~std::size_t(7u);
}

inline std::size_t camera_shm_total_bytes(uint32_t color_bytes, uint32_t depth_bytes)
{
  return sizeof(CameraShmStatic) +
         kCameraShmNumSlots * camera_shm_slot_bytes(color_bytes, depth_bytes);
}

static_assert(sizeof(CameraShmStatic) == 720, "CameraShmStatic ABI size must remain stable");
static_assert(sizeof(CameraShmSlotHeader) == 16, "CameraShmSlotHeader ABI size must remain stable");

// Linux POSIX mmap IPC: aligned, lock-free compiler atomics provide process-shared
// sequence ordering. No std::mutex or process-local atomic fallback is permitted.
static_assert(__atomic_always_lock_free(8, nullptr), "SHM requires lock-free 64-bit atomics");
inline uint64_t camera_shm_sequence(const void* slot)
{
  return __atomic_load_n(static_cast<const uint64_t*>(slot), __ATOMIC_ACQUIRE);
}

inline uint32_t camera_shm_magic(const void* mapping)
{
  return __atomic_load_n(static_cast<const uint32_t*>(mapping), __ATOMIC_ACQUIRE);
}

inline void camera_shm_set_magic(void* mapping, uint32_t magic)
{
  __atomic_store_n(static_cast<uint32_t*>(mapping), magic, __ATOMIC_RELEASE);
}

inline void camera_shm_write_frame(void* mapping, uint64_t& sequence, int32_t sec, uint32_t nsec,
                                   const uint8_t* color, const uint8_t* depth)
{
  const auto* header = static_cast<const CameraShmStatic*>(mapping);
  const auto slot_index = (sequence / 2) % header->num_slots;
  auto* slot = static_cast<uint8_t*>(mapping) + sizeof(CameraShmStatic) + slot_index * header->slot_bytes;
  sequence += 2;
  // Full barrier: the odd marker becomes visible before payload writes begin.
  __atomic_store_n(reinterpret_cast<uint64_t*>(slot), sequence - 1, __ATOMIC_SEQ_CST);
  std::memcpy(slot + 8, &sec, sizeof(sec));
  std::memcpy(slot + 12, &nsec, sizeof(nsec));
  std::memcpy(slot + sizeof(CameraShmSlotHeader), color, header->color_bytes);
  std::memcpy(slot + sizeof(CameraShmSlotHeader) + header->color_bytes, depth, header->depth_bytes);
  __atomic_store_n(reinterpret_cast<uint64_t*>(slot), sequence, __ATOMIC_RELEASE);
}

}  // namespace mujoco_ros2_control_plugins
