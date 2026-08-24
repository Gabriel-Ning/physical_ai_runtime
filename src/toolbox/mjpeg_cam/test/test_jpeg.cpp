#include <cstdint>
#include <vector>

#include "gtest/gtest.h"
#include "mjpeg_cam/jpeg.hpp"

TEST(JpegPayload, TrimsPaddingAfterEoi)
{
  std::vector<std::uint8_t> buffer = {0xFF, 0xD8, 0x01, 0x02, 0xFF, 0xD9, 0x00, 0x00};
  EXPECT_EQ(mjpeg_cam::jpeg_payload_size(buffer.data(), buffer.size()), 6u);
}

TEST(JpegPayload, KeepsExactPacket)
{
  std::vector<std::uint8_t> buffer = {0xFF, 0xD8, 0xAA, 0xFF, 0xD9};
  EXPECT_EQ(mjpeg_cam::jpeg_payload_size(buffer.data(), buffer.size()), buffer.size());
}

TEST(JpegPayload, UsesLastEoi)
{
  std::vector<std::uint8_t> buffer = {
    0xFF, 0xD8, 0xFF, 0xD9, 0x11, 0xFF, 0xD9, 0x00};
  EXPECT_EQ(mjpeg_cam::jpeg_payload_size(buffer.data(), buffer.size()), 7u);
}

TEST(JpegPayload, LeavesNonJpegUnchanged)
{
  std::vector<std::uint8_t> buffer = {0x00, 0x01, 0x02, 0x03};
  EXPECT_EQ(mjpeg_cam::jpeg_payload_size(buffer.data(), buffer.size()), buffer.size());
}

TEST(JpegPayload, HandlesNullAndShort)
{
  EXPECT_EQ(mjpeg_cam::jpeg_payload_size(nullptr, 16), 16u);
  const std::uint8_t tiny[] = {0xFF, 0xD8};
  EXPECT_EQ(mjpeg_cam::jpeg_payload_size(tiny, 2), 2u);
}
