"""Replay support for a Replay Agent role.

Replay is a concrete :class:`~rmi.Agent` role (peer of Policy / Teleop / Planner).
This module supplies recorded-action sources and 1x timeline pacing; ownership
and command admission still go through ``agent.run()`` / ``session.act()``.

Typical application form::

    replay = context.make_agent("Replay", robot=robot)
    player = ReplayPlayer(McapActionSource(uri, topic), ...)
    with replay.run(robot) as session:
        player.open()
        for message in player:
            session.act(Action(part, command, message), observation=session.observe())
        player.close()
"""

from __future__ import annotations

import time
from collections.abc import Callable, Iterator
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import CartesianTrajectory
from trajectory_msgs.msg import JointTrajectory


@dataclass(frozen=True)
class RecordedAction:
    """One decoded native action with an explicit selected source time."""

    source_time_ns: int
    payload: Any


class ReplayActionSource(Protocol):
    """Adapter boundary for MCAP or another action memory."""

    def open(self) -> None: ...
    def reset(self) -> None: ...
    def close(self) -> None: ...
    def actions(self) -> Iterator[RecordedAction]: ...


class ActionTimestampRebaser:
    """Translate one recorded 1x timeline onto current ROS and steady clocks."""

    def __init__(self) -> None:
        self._source_origin_ns: int | None = None
        self._ros_origin_ns = 0
        self._steady_origin_ns = 0
        self._last_source_ns: int | None = None

    def start(
        self, *, source_origin_ns: int, ros_origin_ns: int, steady_origin_ns: int
    ) -> None:
        if source_origin_ns < 0 or ros_origin_ns < 0 or steady_origin_ns < 0:
            raise ValueError("timeline origins must be non-negative")
        self._source_origin_ns = source_origin_ns
        self._ros_origin_ns = ros_origin_ns
        self._steady_origin_ns = steady_origin_ns
        self._last_source_ns = None

    def rewrite(self, message: Any, source_time_ns: int) -> tuple[Any, int]:
        if self._source_origin_ns is None:
            raise RuntimeError("timestamp rebaser has not been started")
        if source_time_ns < self._source_origin_ns:
            raise ValueError("action precedes replay origin")
        if self._last_source_ns is not None and source_time_ns < self._last_source_ns:
            raise ValueError("action source times must be monotonic")
        if not isinstance(
            message, (JointTrajectory, CartesianTrajectory, TwistStamped)
        ):
            raise TypeError(f"unsupported replay action type {type(message).__name__}")
        self._last_source_ns = source_time_ns
        relative_ns = source_time_ns - self._source_origin_ns
        result = deepcopy(message)
        stamp_ns = self._ros_origin_ns + relative_ns
        result.header.stamp.sec = stamp_ns // 1_000_000_000
        result.header.stamp.nanosec = stamp_ns % 1_000_000_000
        return result, self._steady_origin_ns + relative_ns


class McapActionSource:
    """Read one ROS-native action topic using MCAP record time."""

    def __init__(
        self,
        episode_uri: str,
        topic: str,
        *,
        reader_factory: Callable[[], Any] | None = None,
        message_type_resolver: Callable[[str], type] | None = None,
        deserializer: Callable[[bytes, type], Any] | None = None,
    ) -> None:
        self._episode_uri = str(Path(episode_uri))
        self._topic = topic
        self._reader_factory = reader_factory
        self._message_type_resolver = message_type_resolver
        self._deserializer = deserializer
        self._reader: Any = None
        self._message_type: type | None = None

    def open(self) -> None:
        self._open_reader()

    def reset(self) -> None:
        self._open_reader()

    def close(self) -> None:
        self._reader = None
        self._message_type = None

    def actions(self) -> Iterator[RecordedAction]:
        if self._reader is None or self._message_type is None:
            raise RuntimeError("MCAP action source is not open")
        while self._reader.has_next():
            topic, serialized, record_time_ns = self._reader.read_next()
            if topic != self._topic:
                continue
            yield RecordedAction(
                source_time_ns=int(record_time_ns),
                payload=self._deserializer(serialized, self._message_type),
            )

    def _open_reader(self) -> None:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        reader_factory = self._reader_factory or rosbag2_py.SequentialReader
        resolve = self._message_type_resolver or get_message
        deserialize = self._deserializer or deserialize_message
        self._deserializer = deserialize
        reader = reader_factory()
        reader.open(
            rosbag2_py.StorageOptions(uri=self._episode_uri, storage_id="mcap"),
            rosbag2_py.ConverterOptions("", ""),
        )
        types = {item.name: item.type for item in reader.get_all_topics_and_types()}
        if self._topic not in types:
            raise ValueError(f"episode has no action topic {self._topic!r}")
        reader.set_filter(rosbag2_py.StorageFilter(topics=[self._topic]))
        self._message_type = resolve(types[self._topic])
        self._reader = reader


class ReplayPlayer:
    """Pace recorded actions at 1x for use under a Replay Agent session."""

    def __init__(
        self,
        source: ReplayActionSource,
        *,
        ros_clock_ns: Callable[[], int],
        steady_clock_ns: Callable[[], int],
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._source = source
        self._ros_clock_ns = ros_clock_ns
        self._steady_clock_ns = steady_clock_ns
        self._sleep = sleep
        self._opened = False

    def open(self) -> None:
        self._source.open()
        self._opened = True

    def close(self) -> None:
        self._source.close()
        self._opened = False

    def __iter__(self) -> Iterator[Any]:
        if not self._opened:
            raise RuntimeError("ReplayPlayer is not open")
        self._source.reset()
        rebaser = ActionTimestampRebaser()
        first = True
        for action in self._source.actions():
            if first:
                rebaser.start(
                    source_origin_ns=action.source_time_ns,
                    ros_origin_ns=self._ros_clock_ns(),
                    steady_origin_ns=self._steady_clock_ns(),
                )
                first = False
            message, deadline_ns = rebaser.rewrite(
                action.payload, action.source_time_ns
            )
            delay_ns = deadline_ns - self._steady_clock_ns()
            if delay_ns > 0:
                self._sleep(delay_ns / 1_000_000_000)
            yield message


__all__ = [
    "ActionTimestampRebaser",
    "McapActionSource",
    "RecordedAction",
    "ReplayActionSource",
    "ReplayPlayer",
]
