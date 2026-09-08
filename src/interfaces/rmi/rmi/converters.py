"""Ready-made ROS message converters for ``Context.make_camera`` / ``make_sensor``.

``Camera`` and ``Sensor`` accept any ``Callable[[Message], Value]``; these are
the conversions every RMI application needs, so they ship with the SDK instead
of being copied into each backend adapter.
"""

from __future__ import annotations

from typing import Any

import numpy as np

__all__ = ["ros_image_to_numpy"]

_IMAGE_CHANNELS = {
    "rgb8": 3,
    "bgr8": 3,
    "rgba8": 4,
    "bgra8": 4,
    "mono8": 1,
    "8uc1": 1,
    "8uc3": 3,
    "8sc3": 3,
}


def ros_image_to_numpy(message: Any) -> np.ndarray:
    """Decode a raw ``sensor_msgs/Image`` into contiguous HWC RGB ``uint8``."""
    encoding = str(getattr(message, "encoding", "")).lower()
    channels = _IMAGE_CHANNELS.get(encoding)
    if channels is None:
        raise ValueError(f"unsupported image encoding {encoding!r}")
    height, width = int(message.height), int(message.width)
    required = height * int(message.step)
    source = np.frombuffer(message.data, dtype=np.uint8)
    if source.size < required:
        raise ValueError(f"image data is truncated: {source.size} < {required}")
    image = source[:required].reshape(height, int(message.step))
    image = image[:, : width * channels].reshape(height, width, channels)
    if channels == 1:
        return np.repeat(image, 3, axis=2).copy()
    if encoding in {"bgr8", "bgra8"}:
        return image[..., :3][..., ::-1].copy()
    return np.ascontiguousarray(image[..., :3])
