"""Application-facing robot state facade."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable, Mapping
from types import MappingProxyType
from typing import Any

from sensor_msgs.msg import JointState as RosJointState

from .config import EmbodimentConfig
from .contracts import Observation
from .selection import AuthorityClient


class RobotResource:
    """Ordered observation view over one physical part or compound group."""

    def __init__(self, robot: Robot, name: str) -> None:
        self.robot = robot
        self.name = name
        self.parts = tuple(robot.config.get_part_names(name))
        if not self.parts:
            raise KeyError(f"unknown robot resource {name!r}")
        self.joint_names = tuple(robot.config.get_part_joints(name))

    def get_observation(self) -> Observation:
        observation = self.robot.get_observation()
        source_names = list(observation.joint_names)
        index = {name: offset for offset, name in enumerate(source_names)}
        missing = [name for name in self.joint_names if name not in index]
        if missing:
            raise RuntimeError(
                f"joint state is missing {self.name!r} joints: {missing}"
            )

        data = dict(observation.data)
        data["joint_names"] = self.joint_names
        for field in ("joint_positions", "joint_velocities", "joint_efforts"):
            values = list(observation.data.get(field) or [])
            if len(values) == len(source_names):
                data[field] = tuple(values[index[name]] for name in self.joint_names)
            else:
                data[field] = ()
        return Observation(
            data=MappingProxyType(data),
            source_time_s=observation.source_time_s,
            receive_time_s=observation.receive_time_s,
            allocations=MappingProxyType(
                {
                    part: observation.allocations[part]
                    for part in self.parts
                    if part in observation.allocations
                }
            ),
            sensors=observation.sensors,
        )


class Robot:
    """Application-facing robot observation facade."""

    def __init__(
        self,
        config: EmbodimentConfig,
        selection: AuthorityClient,
        *,
        clock: Callable[[], float] | None = None,
    ) -> None:
        self.name = config.name
        self.config = config
        self.selection = selection
        self._clock = clock if clock is not None else time.time
        self._state_lock = threading.Lock()
        self._latest_state: dict[str, Any] | None = None
        self._source_time_s = 0.0
        self._receive_time_s = 0.0
        self._sensors: dict[str, Any] = {}

    def __getitem__(self, name: str) -> RobotResource:
        return RobotResource(self, name)

    @property
    def state(self) -> Observation:
        with self._state_lock:
            if self._latest_state is None:
                raise RuntimeError(
                    "robot state is unavailable; call wait_until_ready()"
                )
            data = dict(self._latest_state)
            source_time_s = self._source_time_s
            receive_time_s = self._receive_time_s
        # ExecutionManagerClient serves this from its status-topic cache. State
        # observation never performs an Execution Manager service request.
        allocations = {
            part: MappingProxyType(dict(allocation))
            for part, allocation in self.selection.get_allocations().items()
            if isinstance(allocation, Mapping)
        }
        return Observation(
            data=MappingProxyType(data),
            source_time_s=source_time_s,
            receive_time_s=receive_time_s,
            allocations=MappingProxyType(allocations),
            sensors=MappingProxyType(
                {name: sensor.latest for name, sensor in self._sensors.items()}
            ),
        )

    def get_observation(self) -> Observation:
        return self.state

    def _attach_sensor(self, sensor: Any) -> None:
        self._sensors[sensor.name] = sensor

    def is_ready(self) -> bool:
        with self._state_lock:
            return self._latest_state is not None

    def wait_until_ready(
        self,
        timeout: float = 10.0,
        check_frequency: float = 50.0,
        check_hardware: bool = False,
    ) -> None:
        """Wait for robot state and query hardware health without changing it."""
        deadline = time.monotonic() + timeout
        period = 1.0 / check_frequency
        while not self.is_ready():
            if time.monotonic() >= deadline:
                raise TimeoutError("timed out waiting for robot joint state")
            time.sleep(period)

        if check_hardware:
            get_diagnostics = getattr(
                self.selection, "get_hardware_diagnostics", None
            )
            if get_diagnostics is None:
                raise RuntimeError(
                    "hardware diagnostics are not supported by this authority client"
                )
            diagnostics = get_diagnostics()
            if diagnostics:
                details = "; ".join(diagnostics)
                raise RuntimeError(f"robot hardware is not ready: {details}")

    def update_joint_state(
        self, message: RosJointState, *, receive_time_s: float | None = None
    ) -> None:
        stamp = message.header.stamp
        source_time_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        received = receive_time_s if receive_time_s is not None else self._clock()
        if source_time_s <= 0.0:
            source_time_s = received
        state = {
            "joint_names": tuple(message.name),
            "joint_positions": tuple(message.position),
            "joint_velocities": tuple(message.velocity),
            "joint_efforts": tuple(message.effort),
        }
        with self._state_lock:
            self._latest_state = state
            self._source_time_s = source_time_s
            self._receive_time_s = received
