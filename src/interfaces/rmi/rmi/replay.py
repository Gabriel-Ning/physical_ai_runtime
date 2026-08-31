"""Recorded-action sources and timeline pacing for a Replay Node.

Typical application form::

    replay = context.make_node("Replay")
    player = ReplayPlayer(McapActionSource(uri, topic), ...)
    player.open()
    for message in player:
        replay.submit(Action(part, command, message))
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

from .contracts import Action


@dataclass(frozen=True)
class EpisodeReplayInfo:
    """Facts discovered while loading one recorded execution timeline."""

    frame_count: int
    rate_hz: float
    source: str
    intervention_detected: bool


class EpisodeReplayPolicy:
    """Policy that returns the next realized or commanded episode action."""

    def __init__(
        self,
        arm_part: str,
        gripper_part: str | None,
        arm_frames: list[list[float]],
        gripper_frames: list[list[float]],
        info: EpisodeReplayInfo,
    ) -> None:
        self.arm_part = arm_part
        self.gripper_part = gripper_part
        self.arm_frames = arm_frames
        self.gripper_frames = gripper_frames
        self.info = info
        self.step = 0

    @classmethod
    def from_mcap(
        cls,
        episode: str | Path,
        profile: Any,
        *,
        side: str = "left",
        source: str = "auto",
    ) -> EpisodeReplayPolicy:
        """Load a single-arm replay policy from an episode MCAP."""
        if source not in {"auto", "joint_states", "commands"}:
            raise ValueError("source must be auto, joint_states, or commands")
        mcap_path = _resolve_mcap(episode)
        arm_part, gripper_part = _resolve_manipulation_parts(profile, side)
        arm_joints = tuple(profile.parts[arm_part].joint_names)
        gripper_joint = (
            profile.parts[gripper_part].joint_names[0]
            if gripper_part and profile.parts[gripper_part].joint_names
            else None
        )
        arm_frames, gripper_frames, timestamps, intervention, selected_source = (
            _load_episode_frames(
                mcap_path,
                arm_part=arm_part,
                gripper_part=gripper_part,
                arm_joints=arm_joints,
                gripper_joint=gripper_joint,
                source=source,
            )
        )
        if not arm_frames:
            raise RuntimeError("episode contains no replayable arm frames")
        if len(timestamps) < 2:
            raise RuntimeError("episode needs at least two timestamped frames")
        duration_s = (timestamps[-1] - timestamps[0]) * 1e-9
        if duration_s <= 0.0:
            raise RuntimeError("episode replay timestamps have no positive duration")
        rate_hz = (len(timestamps) - 1) / duration_s
        return cls(
            arm_part,
            gripper_part,
            arm_frames,
            gripper_frames,
            EpisodeReplayInfo(
                frame_count=len(arm_frames),
                rate_hz=rate_hz,
                source=selected_source,
                intervention_detected=intervention,
            ),
        )

    @property
    def done(self) -> bool:
        return self.step >= len(self.arm_frames)

    def select_action(self, observation: Any) -> list[Action] | None:
        del observation
        if self.done:
            return None
        actions = [Action(self.arm_part, "joint_reference", self.arm_frames[self.step])]
        if (
            self.gripper_part
            and self.step < len(self.gripper_frames)
            and self.gripper_frames[self.step]
        ):
            actions.append(
                Action(
                    self.gripper_part,
                    "joint_reference",
                    self.gripper_frames[self.step],
                )
            )
        self.step += 1
        return actions


def _resolve_mcap(episode: str | Path) -> Path:
    path = Path(episode)
    if path.is_dir():
        files = sorted(path.glob("*.mcap"))
        if len(files) != 1:
            raise ValueError(f"episode directory must contain one MCAP file: {path}")
        path = files[0]
    if not path.is_file():
        raise FileNotFoundError(f"MCAP file not found: {path}")
    return path


def _resolve_manipulation_parts(profile: Any, side: str) -> tuple[str, str | None]:
    if "arm" in profile.parts:
        return "arm", "end_effector" if "end_effector" in profile.parts else None
    arm_part = f"{side}_arm"
    gripper_part = f"{side}_gripper"
    if arm_part not in profile.parts:
        raise KeyError(f"profile has no arm part for side {side!r}")
    return arm_part, gripper_part if gripper_part in profile.parts else None


def _load_episode_frames(
    path: Path,
    *,
    arm_part: str,
    gripper_part: str | None,
    arm_joints: tuple[str, ...],
    gripper_joint: str | None,
    source: str,
) -> tuple[list[list[float]], list[list[float]], list[int], bool, str]:
    from mcap.reader import make_reader
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray

    joint_topic = f"/execution/{arm_part}/joint_reference"
    twist_topic = f"/execution/{arm_part}/twist_reference"
    pose_topic = f"/execution/{arm_part}/pose_reference"
    gripper_topic = (
        f"/execution/{gripper_part}/joint_reference" if gripper_part else None
    )
    command_frames: list[list[float]] = []
    command_times: list[int] = []
    command_gripper: list[list[float]] = []
    state_frames: list[list[float]] = []
    state_times: list[int] = []
    state_gripper: list[list[float]] = []
    intervention = False
    topics = [joint_topic, twist_topic, pose_topic, "/joint_states"]
    if gripper_topic:
        topics.append(gripper_topic)

    with path.open("rb") as stream:
        for _, channel, message in make_reader(stream).iter_messages(topics=topics):
            if channel.topic == joint_topic:
                trajectory = deserialize_message(message.data, JointTrajectory)
                if trajectory.points and len(trajectory.points[-1].positions) >= len(
                    arm_joints
                ):
                    command_frames.append(
                        list(trajectory.points[-1].positions[: len(arm_joints)])
                    )
                    command_times.append(message.log_time)
            elif channel.topic in {twist_topic, pose_topic}:
                intervention = True
            elif gripper_topic and channel.topic == gripper_topic:
                command = deserialize_message(message.data, Float64MultiArray)
                if command.data:
                    command_gripper.append([float(command.data[0])])
            elif channel.topic == "/joint_states":
                state = deserialize_message(message.data, JointState)
                positions = dict(zip(state.name, state.position, strict=False))
                if all(name in positions for name in arm_joints):
                    state_frames.append([positions[name] for name in arm_joints])
                    state_times.append(message.log_time)
                    state_gripper.append(
                        [positions[gripper_joint]]
                        if gripper_joint and gripper_joint in positions
                        else []
                    )

    use_states = (
        source == "joint_states"
        or (source == "auto" and intervention)
        or not command_frames
    )
    if use_states:
        return state_frames, state_gripper, state_times, intervention, "joint_states"
    return command_frames, command_gripper, command_times, intervention, "commands"


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


class ReplayClockJumpError(RuntimeError):
    """Raised when simulated time moves backwards during an active replay."""


class ReplayPacer:
    """Pace replay against monotonic wall time or the ROS simulation clock."""

    def __init__(
        self,
        *,
        ros_clock_ns: Callable[[], int],
        use_sim_time: bool,
        steady_clock_ns: Callable[[], int] = time.monotonic_ns,
        sleep: Callable[[float], None] = time.sleep,
        keep_running: Callable[[], bool] = lambda: True,
        poll_interval_s: float = 0.01,
    ) -> None:
        if poll_interval_s <= 0.0:
            raise ValueError("poll_interval_s must be positive")
        self._ros_clock_ns = ros_clock_ns
        self._use_sim_time = use_sim_time
        self._steady_clock_ns = steady_clock_ns
        self._sleep = sleep
        self._keep_running = keep_running
        self._poll_interval_s = poll_interval_s
        self._origin_ns: int | None = None
        self._last_clock_ns: int | None = None

    @classmethod
    def from_node(cls, node: Any, **kwargs: Any) -> ReplayPacer:
        """Create a pacer from a ROS node's ``use_sim_time`` and clock."""
        use_sim_time = bool(node.get_parameter("use_sim_time").value)
        context = getattr(node, "context", None)
        keep_running = (
            context.ok
            if context is not None and hasattr(context, "ok")
            else lambda: True
        )
        return cls(
            ros_clock_ns=lambda: int(node.get_clock().now().nanoseconds),
            use_sim_time=use_sim_time,
            keep_running=keep_running,
            **kwargs,
        )

    @property
    def use_sim_time(self) -> bool:
        return self._use_sim_time

    def start(self) -> int:
        """Start a timeline and return its ROS timestamp origin."""
        if self._use_sim_time:
            now_ns = self._wait_for_valid_sim_time()
            self._origin_ns = now_ns
            self._last_clock_ns = now_ns
            return now_ns
        self._origin_ns = self._steady_clock_ns()
        self._last_clock_ns = self._origin_ns
        return self._ros_clock_ns()

    def wait_until(self, relative_time_s: float) -> None:
        """Wait until ``relative_time_s`` on the selected replay timeline."""
        if self._origin_ns is None:
            raise RuntimeError("replay pacer has not been started")
        if relative_time_s < 0.0:
            raise ValueError("relative replay time must be non-negative")
        target_ns = self._origin_ns + int(relative_time_s * 1_000_000_000)
        if not self._use_sim_time:
            delay_ns = target_ns - self._steady_clock_ns()
            if delay_ns > 0:
                self._sleep(delay_ns / 1_000_000_000)
            return

        while self._keep_running():
            now_ns = self._ros_clock_ns()
            if self._last_clock_ns is not None and now_ns < self._last_clock_ns:
                raise ReplayClockJumpError(
                    "simulation clock moved backwards during replay; restart the replay"
                )
            self._last_clock_ns = now_ns
            if now_ns >= target_ns:
                return
            self._sleep(self._poll_interval_s)
        raise RuntimeError("ROS context stopped while waiting for replay time")

    def _wait_for_valid_sim_time(self) -> int:
        while self._keep_running():
            now_ns = self._ros_clock_ns()
            if now_ns > 0:
                return now_ns
            self._sleep(self._poll_interval_s)
        raise RuntimeError(
            "ROS context stopped before a valid simulation clock arrived"
        )


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
    """Pace recorded actions at their native timeline for Node submission."""

    def __init__(
        self,
        source: ReplayActionSource,
        *,
        ros_clock_ns: Callable[[], int],
        steady_clock_ns: Callable[[], int],
        sleep: Callable[[float], None] = time.sleep,
        use_sim_time: bool = False,
        keep_running: Callable[[], bool] = lambda: True,
        poll_interval_s: float = 0.01,
    ) -> None:
        self._source = source
        self._ros_clock_ns = ros_clock_ns
        self._steady_clock_ns = steady_clock_ns
        self._sleep = sleep
        self._use_sim_time = use_sim_time
        self._keep_running = keep_running
        self._poll_interval_s = poll_interval_s
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
        pacer = ReplayPacer(
            ros_clock_ns=self._ros_clock_ns,
            use_sim_time=self._use_sim_time,
            steady_clock_ns=self._steady_clock_ns,
            sleep=self._sleep,
            keep_running=self._keep_running,
            poll_interval_s=self._poll_interval_s,
        )
        first = True
        source_origin_ns = 0
        for action in self._source.actions():
            if first:
                source_origin_ns = action.source_time_ns
                ros_origin_ns = pacer.start()
                rebaser.start(
                    source_origin_ns=source_origin_ns,
                    ros_origin_ns=ros_origin_ns,
                    steady_origin_ns=self._steady_clock_ns(),
                )
                first = False
            message, _ = rebaser.rewrite(
                action.payload, action.source_time_ns
            )
            pacer.wait_until((action.source_time_ns - source_origin_ns) * 1e-9)
            yield message


__all__ = [
    "ActionTimestampRebaser",
    "EpisodeReplayInfo",
    "EpisodeReplayPolicy",
    "McapActionSource",
    "RecordedAction",
    "ReplayActionSource",
    "ReplayClockJumpError",
    "ReplayPacer",
    "ReplayPlayer",
]
