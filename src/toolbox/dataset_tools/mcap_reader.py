"""ROSbag2 MCAP access for offline dataset conversion."""

from __future__ import annotations

from collections.abc import Iterator, Sequence
from pathlib import Path
from typing import Any


class McapReader:
    """Re-openable raw MCAP reader with lazy ROS type deserialization."""

    def __init__(self, episode: str | Path) -> None:
        self.episode = Path(episode)
        self.mcap = self._resolve_file(self.episode)
        self._message_types: dict[str, Any] = {}

    @staticmethod
    def _resolve_file(path: Path) -> Path:
        if path.is_file() and path.suffix == ".mcap":
            return path
        if not path.is_dir():
            raise FileNotFoundError(path)
        preferred = path / "episode.mcap"
        files = [preferred] if preferred.is_file() else sorted(path.glob("*.mcap"))
        if len(files) != 1:
            raise ValueError(f"expected exactly one MCAP in {path}, found {len(files)}")
        return files[0]

    def _open(self) -> Any:
        import rosbag2_py

        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(self.mcap.parent), storage_id="mcap"),
            rosbag2_py.ConverterOptions("", ""),
        )
        return reader

    def topic_types(self) -> dict[str, str]:
        reader = self._open()
        return {item.name: item.type for item in reader.get_all_topics_and_types()}

    def raw_messages(self, topics: Sequence[str]) -> Iterator[tuple[str, bytes, int]]:
        selected = set(topics)
        reader = self._open()
        while reader.has_next():
            topic, serialized, timestamp_ns = reader.read_next()
            if topic in selected:
                yield topic, serialized, int(timestamp_ns)

    def deserialize(
        self, topic: str, serialized: bytes, topic_types: dict[str, str]
    ) -> Any:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        message_type = self._message_types.get(topic)
        if message_type is None:
            message_type = get_message(topic_types[topic])
            self._message_types[topic] = message_type
        return deserialize_message(serialized, message_type)
