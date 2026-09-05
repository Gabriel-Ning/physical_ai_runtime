// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <sys/wait.h>
#include <gtest/gtest.h>
#include "camera_shm_test_writer.hpp"

using namespace mujoco_ros2_control_plugins;

TEST(CameraShmTest, EveryAlternatingSlotHasANewGlobalSequence)
{
  CameraShmTestWriter writer;
  CameraShmReader reader;
  std::string error;
  ASSERT_TRUE(reader.open(writer.name, error)) << error;
  EXPECT_EQ(sizeof(CameraShmStatic), 720u);
  EXPECT_EQ(sizeof(CameraShmSlotHeader), 16u);
  EXPECT_EQ(writer.header().slot_bytes % 8, 0u);  // odd resolution must remain aligned
  std::vector<uint8_t> color, depth;
  int32_t sec; uint32_t nsec;
  for (uint8_t frame = 1; frame <= 8; ++frame)
  {
    writer.write(frame);
    ASSERT_TRUE(reader.read_latest(&color, &depth, sec, nsec));
    EXPECT_EQ(reader.last_sequence(), 2u * frame);
    EXPECT_EQ(color, writer.color);
    EXPECT_EQ(depth, writer.depth);
    EXPECT_EQ(sec, frame);
    EXPECT_EQ(nsec, 123u);
    EXPECT_FALSE(reader.read_latest(&color, &depth, sec, nsec));
  }
}

TEST(CameraShmTest, ReadsOnlyNewestAndCanReadMetadataWithoutPixels)
{
  CameraShmTestWriter writer;
  CameraShmReader reader;
  std::string error;
  ASSERT_TRUE(reader.open(writer.name, error));
  writer.write(1); writer.write(2); writer.write(3);
  int32_t sec; uint32_t nsec;
  ASSERT_TRUE(reader.read_latest(nullptr, nullptr, sec, nsec));
  EXPECT_EQ(sec, 3);
  EXPECT_EQ(reader.last_sequence(), 6u);
}

TEST(CameraShmTest, RejectsUncommittedAndOldLayouts)
{
  CameraShmTestWriter writer;
  CameraShmReader reader;
  std::string error;
  camera_shm_set_magic(writer.mapping, 0);
  EXPECT_FALSE(reader.open(writer.name, error));
  camera_shm_set_magic(writer.mapping, kCameraShmMagic);
  writer.header().version = 1;
  EXPECT_FALSE(reader.open(writer.name, error));
  writer.header().version = kCameraShmVersion;
  writer.header().color_bytes = 0;
  EXPECT_FALSE(reader.open(writer.name, error));
}

TEST(CameraShmTest, ReattachesAfterProducerRestartWithSequenceReset)
{
  CameraShmTestWriter writer;
  CameraShmReader reader;
  std::string error;
  ASSERT_TRUE(reader.open(writer.name, error));
  writer.write(1); writer.write(2);
  int32_t sec; uint32_t nsec;
  ASSERT_TRUE(reader.read_latest(nullptr, nullptr, sec, nsec));
  writer.recreate(4, 2);
  EXPECT_FALSE(reader.current());
  ASSERT_TRUE(reader.open(writer.name, error)) << error;
  writer.write(9);
  ASSERT_TRUE(reader.read_latest(nullptr, nullptr, sec, nsec));
  EXPECT_EQ(sec, 9);
  EXPECT_EQ(reader.last_sequence(), 2u);
  EXPECT_EQ(reader.header().width, 4u);
}

TEST(CameraShmTest, ConcurrentProcessNeverDeliversTornPixelsOrStamp)
{
  CameraShmTestWriter writer(320, 240);
  CameraShmReader reader;
  std::string error;
  ASSERT_TRUE(reader.open(writer.name, error));
  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0)
  {
    for (unsigned frame = 1; frame <= 500; ++frame)
    {
      writer.write(static_cast<uint8_t>(frame % 251));
      usleep(100);
    }
    _exit(0);
  }
  unsigned received = 0;
  int status;
  std::vector<uint8_t> color, depth;
  int32_t sec; uint32_t nsec;
  do
  {
    if (reader.read_latest(&color, &depth, sec, nsec))
    {
      ++received;
      EXPECT_TRUE(std::all_of(color.begin(), color.end(), [sec](uint8_t value) { return value == sec; }));
      EXPECT_TRUE(std::all_of(depth.begin(), depth.end(), [sec](uint8_t value) { return value == sec; }));
      EXPECT_EQ(nsec, 123u);
    }
  } while (waitpid(child, &status, WNOHANG) == 0);
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0);
  EXPECT_GT(received, 5u);
}
