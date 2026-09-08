"""Timestamped observation sensors: ROS topics, cameras, and TF poses.

All share one Robot attachment surface: ``Observation.sensors[name]`` holds
``TimestampedSample[...]``. Context only snapshots ``latest`` — no cross-stream
time sync.

TCP poses come from TF today (``TfTcpPoseSensor``). A future FoundationPose
estimator can publish into a topic-backed pose sensor with the same
``PoseSample`` value type and sensor-name contract.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Generic, TypeVar

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from .config import CameraSensorConfig

ValueT = TypeVar("ValueT")


@dataclass(frozen=True)
class TimestampedSample(Generic[ValueT]):
    """One immutable sensor sample with source and local receive time."""

    value: ValueT
    source_time_s: float
    receive_time_s: float
    sequence: int
    frame_id: str = ""


class SampleBuffer(Generic[ValueT]):
    """Thread-safe bounded history used by synchronous sensor APIs."""

    def __init__(self, capacity: int) -> None:
        if capacity <= 0:
            raise ValueError("sample buffer capacity must be positive")
        self._samples: deque[TimestampedSample[ValueT]] = deque(maxlen=capacity)
        self._condition = threading.Condition()

    def add(self, sample: TimestampedSample[ValueT]) -> None:
        with self._condition:
            self._samples.append(sample)
            self._condition.notify_all()

    def latest(self) -> TimestampedSample[ValueT] | None:
        with self._condition:
            return self._samples[-1] if self._samples else None

    def snapshot(self) -> tuple[TimestampedSample[ValueT], ...]:
        with self._condition:
            return tuple(self._samples)

    def wait_next(
        self,
        *,
        after_sequence: int | None = None,
        timeout: float = 1.0,
    ) -> TimestampedSample[ValueT]:
        deadline = time.monotonic() + timeout
        with self._condition:
            if after_sequence is None:
                after_sequence = self._samples[-1].sequence if self._samples else 0
            while True:
                latest = self._samples[-1] if self._samples else None
                if latest is not None and latest.sequence > after_sequence:
                    return latest
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError("timed out waiting for the next sensor sample")
                self._condition.wait(remaining)


class Sensor(Generic[ValueT]):
    """Synchronous latest/history API for one ROS sensor stream."""

    def __init__(
        self,
        *,
        name: str,
        node: Any,
        topic: str,
        message_type: Any,
        converter: Callable[[Any], ValueT] | None = None,
        history_size: int = 8,
        qos: Any = qos_profile_sensor_data,
    ) -> None:
        if not name:
            raise ValueError("sensor name must not be empty")
        if not topic:
            raise ValueError("sensor topic must not be empty")
        self.name = name
        self.topic = topic
        self._node = node
        self._converter = converter if converter is not None else _identity
        self._buffer: SampleBuffer[ValueT] = SampleBuffer(history_size)
        self._sequence = 0
        self._subscription = node.create_subscription(
            message_type,
            topic,
            self._on_message,
            qos,
        )

    @property
    def value(self) -> ValueT:
        return self.latest.value

    @property
    def latest(self) -> TimestampedSample[ValueT]:
        sample = self._buffer.latest()
        if sample is None:
            raise RuntimeError(
                f"sensor {self.name!r} has no sample; call wait_until_ready()"
            )
        return sample

    @property
    def history(self) -> tuple[TimestampedSample[ValueT], ...]:
        return self._buffer.snapshot()

    def is_ready(self) -> bool:
        return self._buffer.latest() is not None

    def wait_until_ready(self, timeout: float = 10.0) -> None:
        if self.is_ready():
            return
        self._buffer.wait_next(timeout=timeout)

    def wait_next(
        self,
        *,
        after_sequence: int | None = None,
        timeout: float = 1.0,
    ) -> TimestampedSample[ValueT]:
        return self._buffer.wait_next(
            after_sequence=after_sequence,
            timeout=timeout,
        )

    def close(self) -> None:
        if hasattr(self._node, "destroy_subscription"):
            self._node.destroy_subscription(self._subscription)

    def _on_message(self, message: Any) -> None:
        receive_time_s = _node_now_s(self._node)
        source_time_s, frame_id = _header_metadata(message, receive_time_s)
        value = self._converter(message)
        self._sequence += 1
        self._buffer.add(
            TimestampedSample(
                value=value,
                source_time_s=source_time_s,
                receive_time_s=receive_time_s,
                sequence=self._sequence,
                frame_id=frame_id,
            )
        )


class Camera(Sensor[ValueT]):
    """Timestamp-preserving camera stream.

    The default value is the original ROS image message to avoid an implicit
    full-frame copy. ``encoding: jpeg`` (also ``jpg`` / ``mjpeg``) subscribes
    to ``sensor_msgs/CompressedImage``; otherwise ``sensor_msgs/Image``.
    Applications may inject a converter for NumPy, Torch, or another
    policy-native representation.
    """

    def __init__(
        self,
        config: CameraSensorConfig,
        node: Any,
        *,
        converter: Callable[[Any], ValueT] | None = None,
        history_size: int = 8,
    ) -> None:
        self.config = config
        super().__init__(
            name=config.name,
            node=node,
            topic=config.ros_topic,
            message_type=_camera_message_type(config.encoding),
            converter=converter,
            history_size=history_size,
            qos=qos_profile_sensor_data,
        )

    @property
    def frame(self) -> TimestampedSample[ValueT]:
        return self.latest


@dataclass(frozen=True)
class PoseSample:
    """One SE(3) sample in a parent frame (quaternion is wxyz)."""

    position_xyz: tuple[float, float, float]
    orientation_wxyz: tuple[float, float, float, float]
    frame_id: str
    child_frame_id: str = ""


class TfTcpPoseSensor:
    """Pull latest ``base_frame`` → ``tcp_frame`` from a shared TF buffer.

    Implements the same ``name`` / ``latest`` / ``is_ready`` /
    ``wait_until_ready`` surface as ``Sensor`` so ``Robot`` can attach it.
    """

    def __init__(
        self,
        *,
        name: str,
        base_frame: str,
        tcp_frame: str,
        buffer: Any,
        node: Any,
        lookup_timeout_s: float = 0.05,
    ) -> None:
        if not name:
            raise ValueError("pose sensor name must not be empty")
        if not base_frame:
            raise ValueError("base_frame must not be empty")
        if not tcp_frame:
            raise ValueError("tcp_frame must not be empty")
        if lookup_timeout_s < 0.0:
            raise ValueError("lookup_timeout_s must be non-negative")
        self.name = name
        self.base_frame = base_frame
        self.tcp_frame = tcp_frame
        self._buffer = buffer
        self._node = node
        self._lookup_timeout_s = float(lookup_timeout_s)
        self._sequence = 0

    @property
    def value(self) -> PoseSample:
        return self.latest.value

    @property
    def latest(self) -> TimestampedSample[PoseSample]:
        from rclpy.duration import Duration
        from rclpy.time import Time

        receive_time_s = _node_now_s(self._node)
        try:
            transform = self._buffer.lookup_transform(
                self.base_frame,
                self.tcp_frame,
                Time(),
                timeout=Duration(seconds=self._lookup_timeout_s),
            )
        except Exception as exc:
            raise RuntimeError(
                f"TF lookup failed for {self.base_frame!r} → {self.tcp_frame!r}: {exc}"
            ) from exc
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        stamp = transform.header.stamp
        source_time_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if source_time_s <= 0.0:
            source_time_s = receive_time_s
        self._sequence += 1
        return TimestampedSample(
            value=PoseSample(
                position_xyz=(
                    float(translation.x),
                    float(translation.y),
                    float(translation.z),
                ),
                orientation_wxyz=(
                    float(rotation.w),
                    float(rotation.x),
                    float(rotation.y),
                    float(rotation.z),
                ),
                frame_id=self.base_frame,
                child_frame_id=self.tcp_frame,
            ),
            source_time_s=source_time_s,
            receive_time_s=receive_time_s,
            sequence=self._sequence,
            frame_id=self.base_frame,
        )

    def is_ready(self) -> bool:
        from rclpy.duration import Duration
        from rclpy.time import Time

        try:
            return bool(
                self._buffer.can_transform(
                    self.base_frame,
                    self.tcp_frame,
                    Time(),
                    timeout=Duration(seconds=0.0),
                )
            )
        except Exception:
            return False

    def wait_until_ready(self, timeout: float = 10.0) -> None:
        if timeout <= 0.0:
            raise ValueError("timeout must be positive")
        deadline = time.monotonic() + timeout
        from rclpy.duration import Duration
        from rclpy.time import Time

        poll = Duration(seconds=0.05)
        while time.monotonic() < deadline:
            try:
                if self._buffer.can_transform(
                    self.base_frame,
                    self.tcp_frame,
                    Time(),
                    timeout=poll,
                ):
                    return
            except Exception:
                pass
            time.sleep(0.05)
        raise TimeoutError(
            f"timed out waiting for TF {self.base_frame!r} → {self.tcp_frame!r}"
        )

    def close(self) -> None:
        """TF buffer is owned by Context; nothing to destroy per sensor."""


def _identity(value: ValueT) -> ValueT:
    return value


_COMPRESSED_ENCODINGS = frozenset({"jpeg", "jpg", "mjpeg"})


def _camera_message_type(encoding: str) -> Any:
    if encoding.lower() in _COMPRESSED_ENCODINGS:
        return CompressedImage
    return Image


def _node_now_s(node: Any) -> float:
    now = node.get_clock().now()
    nanoseconds = getattr(now, "nanoseconds", None)
    if nanoseconds is not None:
        return float(nanoseconds) * 1e-9
    message = now.to_msg()
    return float(message.sec) + float(message.nanosec) * 1e-9


def _header_metadata(message: Any, receive_time_s: float) -> tuple[float, str]:
    header = getattr(message, "header", None)
    if header is None:
        return receive_time_s, ""
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return receive_time_s, str(getattr(header, "frame_id", ""))
    source_time_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    if source_time_s <= 0.0:
        source_time_s = receive_time_s
    return source_time_s, str(getattr(header, "frame_id", ""))


__all__ = [
    "Camera",
    "PoseSample",
    "SampleBuffer",
    "Sensor",
    "TfTcpPoseSensor",
    "TimestampedSample",
]
