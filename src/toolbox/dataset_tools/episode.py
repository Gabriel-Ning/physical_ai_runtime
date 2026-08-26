"""Asynchronous-topic synchronization for offline dataset conversion."""

from __future__ import annotations

import bisect
import math
from collections.abc import Iterator
from dataclasses import dataclass
from typing import Any

import numpy as np

from .contract import DatasetContract
from .mcap_reader import McapReader


@dataclass(frozen=True)
class Sample:
    timestamp_ns: int
    value: dict[str, float]


@dataclass(frozen=True)
class EpisodeFrame:
    timestamp: float
    values: dict[str, Any]


class ProfileEpisodeReader:
    """Synchronize native-rate streams onto a regular Profile-defined timeline."""

    def __init__(self, reader: McapReader, contract: DatasetContract) -> None:
        self.reader = reader
        self.contract = contract

    def frames(self) -> Iterator[EpisodeFrame]:
        topic_types = self.reader.topic_types()
        missing = sorted(set(self.contract.topics) - set(topic_types))
        if missing:
            raise ValueError(f"MCAP is missing required Profile topics: {missing}")

        numeric, image_times = self._collect_index(topic_types)
        start_ns = max(times[0] for times in image_times.values())
        end_ns = min(times[-1] for times in image_times.values())
        period_ns = 1_000_000_000 / self.contract.policy.frequency
        count = math.floor((end_ns - start_ns) / period_ns) + 1
        if count < 2:
            raise ValueError("camera streams have no usable common time range")
        targets = [round(start_ns + index * period_ns) for index in range(count)]
        numeric_times = {
            topic: [sample.timestamp_ns for sample in samples]
            for topic, samples in numeric.items()
        }
        image_indices = {
            feature: [_nearest_index(times, target) for target in targets]
            for feature, times in image_times.items()
        }
        image_streams = {
            feature: self._selected_images(
                topic_types,
                feature,
                image_indices[feature],
            )
            for feature in self.contract.policy.camera_shapes
        }

        state_samples = numeric[self.contract.state_topic]
        for timestamp_ns in targets:
            state = _nearest(
                state_samples,
                numeric_times[self.contract.state_topic],
                timestamp_ns,
            ).value
            action_values = dict(state)
            for group in self.contract.policy.action_groups:
                topic = self.contract.action_topics[group.part]
                latest = _latest(
                    numeric[topic], numeric_times[topic], timestamp_ns
                )
                if latest is not None:
                    action_values.update(latest)
            values: dict[str, Any] = {
                "observation.state": _ordered_vector(
                    state, self.contract.policy.state_feature_names
                ),
                "action": _ordered_vector(
                    action_values, self.contract.policy.action_feature_names
                ),
            }
            for feature_name in self.contract.policy.camera_shapes:
                values[feature_name] = next(image_streams[feature_name])
            yield EpisodeFrame(timestamp_ns * 1e-9, values)

    def _collect_index(
        self, topic_types: dict[str, str]
    ) -> tuple[dict[str, list[Sample]], dict[str, list[int]]]:
        numeric_topics = {
            self.contract.state_topic,
            *self.contract.action_topics.values(),
        }
        camera_by_topic = {
            topic: feature
            for feature, topic in self.contract.policy.camera_topics.items()
        }
        part_by_topic = {
            topic: part for part, topic in self.contract.action_topics.items()
        }
        groups = {group.part: group for group in self.contract.policy.action_groups}
        numeric = {topic: [] for topic in numeric_topics}
        image_times = {feature: [] for feature in self.contract.policy.camera_shapes}
        for topic, serialized, timestamp_ns in self.reader.raw_messages(
            self.contract.topics
        ):
            if topic in camera_by_topic:
                image_times[camera_by_topic[topic]].append(timestamp_ns)
                continue
            message = self.reader.deserialize(topic, serialized, topic_types)
            value = (
                _joint_positions(message)
                if topic == self.contract.state_topic
                else _command_positions(
                    message, groups[part_by_topic[topic]].joint_names
                )
            )
            if value:
                numeric[topic].append(Sample(timestamp_ns, value))
        empty = [topic for topic, samples in numeric.items() if not samples]
        empty.extend(feature for feature, times in image_times.items() if not times)
        if empty:
            raise ValueError(f"required streams contain no samples: {empty}")
        return numeric, image_times

    def _selected_images(
        self,
        topic_types: dict[str, str],
        feature_name: str,
        source_indices: list[int],
    ) -> Iterator[np.ndarray]:
        """Decode selected frames lazily, reusing one image for repeated targets."""
        topic = self.contract.policy.camera_topics[feature_name]
        expected_shape = self.contract.policy.camera_shapes[feature_name]
        target_index = 0
        for source_index, (read_topic, serialized, _) in enumerate(
            self.reader.raw_messages((topic,))
        ):
            if source_index != source_indices[target_index]:
                continue
            message = self.reader.deserialize(read_topic, serialized, topic_types)
            image = ros_image_to_numpy(message)
            if tuple(image.shape) != expected_shape:
                raise ValueError(
                    f"camera {feature_name!r} on {topic!r} has shape "
                    f"{tuple(image.shape)}, which does not match Profile "
                    f"{expected_shape}"
                )
            while (
                target_index < len(source_indices)
                and source_indices[target_index] == source_index
            ):
                target_index += 1
                yield image
            if target_index == len(source_indices):
                return
        raise RuntimeError(f"camera {feature_name!r} ended before selected frames")


def _nearest_index(timestamps: list[int], target: int) -> int:
    index = bisect.bisect_left(timestamps, target)
    if index == 0:
        return 0
    if index == len(timestamps):
        return len(timestamps) - 1
    before, after = timestamps[index - 1], timestamps[index]
    return index - 1 if target - before <= after - target else index


def _nearest(
    samples: list[Sample], timestamps: list[int], timestamp_ns: int
) -> Sample:
    return samples[_nearest_index(timestamps, timestamp_ns)]


def _latest(
    samples: list[Sample], timestamps: list[int], timestamp_ns: int
) -> dict[str, float] | None:
    index = bisect.bisect_right(timestamps, timestamp_ns) - 1
    return None if index < 0 else samples[index].value


def _ordered_vector(values: dict[str, float], names: tuple[str, ...]) -> np.ndarray:
    joints = (name.removesuffix(".pos") for name in names)
    try:
        return np.asarray([values[name] for name in joints], dtype=np.float32)
    except KeyError as exc:
        raise ValueError(f"sample is missing joint {exc.args[0]!r}") from exc


def _joint_positions(message: Any) -> dict[str, float]:
    return {
        str(name): float(position)
        for name, position in zip(message.name, message.position, strict=False)
    }


def _command_positions(
    message: Any, expected_names: tuple[str, ...]
) -> dict[str, float]:
    points = getattr(message, "points", None)
    if points:
        names = tuple(getattr(message, "joint_names", ())) or expected_names
        return dict(zip(names, map(float, points[0].positions), strict=False))
    data = getattr(message, "data", None)
    if data is not None:
        return dict(zip(expected_names, map(float, data), strict=False))
    positions = getattr(message, "positions", None)
    if positions is not None:
        return dict(zip(expected_names, map(float, positions), strict=False))
    raise TypeError(f"unsupported action message {type(message).__name__}")


def ros_image_to_numpy(message: Any) -> np.ndarray:
    """Decode supported raw ROS Image encodings into contiguous HWC RGB."""
    encoding = str(getattr(message, "encoding", "")).lower()
    channels = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1,
        "8uc1": 1,
        "8uc3": 3,
        "8sc3": 3,
    }.get(encoding)
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
    return image[..., :3].copy()
