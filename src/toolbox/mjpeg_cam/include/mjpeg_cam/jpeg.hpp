#pragma once

#include <cstddef>
#include <cstdint>

namespace mjpeg_cam {

/// Return the JPEG payload length inside a V4L2 MJPEG buffer.
///
/// UVC drivers often set ``bytesused`` to ``sizeimage`` (padded). The real
/// packet ends at the last EOI marker (0xFF 0xD9). If SOI/EOI are missing,
/// ``bytesused`` is returned unchanged.
inline std::size_t jpeg_payload_size(
  const std::uint8_t *data, std::size_t bytesused)
{
  if (data == nullptr || bytesused < 4) {
    return bytesused;
  }
  if (data[0] != 0xFFu || data[1] != 0xD8u) {
    return bytesused;
  }
  for (std::size_t i = bytesused; i >= 2; --i) {
    if (data[i - 2] == 0xFFu && data[i - 1] == 0xD9u) {
      return i;
    }
  }
  return bytesused;
}

}  // namespace mjpeg_cam
