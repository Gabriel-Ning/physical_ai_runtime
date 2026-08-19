"""Workstation-side clients for one Action Provider's EM gateway endpoints.

:class:`ActionProviderClient` is the **command-plane** client: it publishes
native ROS commands to deployment-declared gateway endpoints under one
provider name. It is not :class:`~rmi.ProviderLifecycle` (the EM-side
``start``/``stop``/``reset`` hooks).
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from itertools import pairwise
from types import MappingProxyType
from typing import Any

from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import (
    CartesianPoint,
    CartesianTrajectory,
    CartesianTrajectoryPoint,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .config import ControllerConfig, EmbodimentConfig
from .controllers import make_controller_client_factory


class ActionProviderClient:
    """Native ROS command clients bound to one profile-declared provider.

    Unlike :class:`rmi.Robot`, these clients target Execution Manager source
    endpoints. They therefore preserve arbitration and generation fencing
    instead of commanding ros2_control controllers directly.
    """

    def __init__(
        self,
        name: str,
        node: Any,
        controllers: Mapping[str, Any],
        commands: Mapping[str, frozenset[str]],
        *,
        profile: EmbodimentConfig | None = None,
    ) -> None:
        self.name = name
        self._node = node
        self.controllers = MappingProxyType(dict(controllers))
        self.commands = MappingProxyType(dict(commands))
        self.profile = profile

    @classmethod
    def from_profile(
        cls,
        profile: EmbodimentConfig,
        provider: str,
        node: Any,
        timeout_sec: float = 5.0,
        *,
        action_client_factory: Any | None = None,
    ) -> ActionProviderClient:
        try:
            provider_config = profile.execution["providers"][provider]
        except KeyError as exc:
            raise KeyError(f"unknown action provider {provider!r}") from exc

        endpoints: dict[str, dict[str, dict[str, str]]] = {}
        for source in profile.execution.get("sources", []):
            if source["provider"] != provider:
                continue
            part_endpoints = endpoints.setdefault(
                source["part"], {"actions": {}, "topics": {}}
            )
            if source["command"] == "joint_trajectory":
                part_endpoints["actions"]["follow_joint_trajectory"] = source[
                    "action"
                ]
            else:
                part_endpoints["topics"][source["command"]] = source["topic"]

        configured_parts = set(provider_config["controllers"])
        missing = configured_parts - set(endpoints)
        if missing:
            raise ValueError(
                f"provider {provider!r} has no sources for parts {sorted(missing)!r}"
            )

        factory = make_controller_client_factory(
            node,
            timeout_sec,
            *(() if action_client_factory is None else (action_client_factory,)),
        )
        controllers: dict[str, Any] = {}
        commands: dict[str, frozenset[str]] = {}
        for part, contract in provider_config["controllers"].items():
            route = endpoints[part]
            topics = dict(route["topics"])
            part_controller = profile.parts[part].controllers[contract]
            heartbeat = part_controller.ros_topics.get("trajectory_guard_heartbeat")
            if heartbeat:
                topics["trajectory_guard_heartbeat"] = heartbeat
            source_config = ControllerConfig(
                name=f"{provider}:{part}",
                implementation=part_controller.implementation,
                command_interface=part_controller.command_interface,
                ros_actions=route["actions"],
                ros_topics=topics,
            )
            controllers[part] = factory(part, contract, source_config)
            commands[part] = frozenset(
                source["command"]
                for source in profile.execution["sources"]
                if source["provider"] == provider and source["part"] == part
            )
        return cls(provider, node, controllers, commands, profile=profile)

    def send_joint_reference(
        self,
        part: str,
        joint_names: list[str],
        positions: list[list[float]],
        times_from_start_sec: list[float],
    ) -> None:
        if not positions or len(positions) != len(times_from_start_sec):
            raise ValueError("positions and times_from_start_sec must align")
        if not joint_names or any(len(values) != len(joint_names) for values in positions):
            raise ValueError("joint reference positions must align with joint_names")
        message = JointTrajectory()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.joint_names = list(joint_names)
        for values, time_sec in zip(positions, times_from_start_sec):
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in values]
            _set_duration(point.time_from_start, time_sec)
            message.points.append(point)
        self.send(part, "joint_reference", message)

    def send_pose_reference(
        self,
        part: str,
        positions: list[list[float]],
        orientations: list[list[float]],
        times_from_start_sec: list[float],
        frame_id: str,
    ) -> None:
        if not positions or not (
            len(positions) == len(orientations) == len(times_from_start_sec)
        ):
            raise ValueError(
                "positions, orientations, and times_from_start_sec must align"
            )
        message = CartesianTrajectory()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        for position, orientation, time_sec in zip(
            positions, orientations, times_from_start_sec
        ):
            point = CartesianTrajectoryPoint()
            point.point = CartesianPoint()
            point.point.pose.position.x = float(position[0])
            point.point.pose.position.y = float(position[1])
            point.point.pose.position.z = float(position[2])
            point.point.pose.orientation.x = float(orientation[0])
            point.point.pose.orientation.y = float(orientation[1])
            point.point.pose.orientation.z = float(orientation[2])
            point.point.pose.orientation.w = float(orientation[3])
            _set_duration(point.time_from_start, time_sec)
            message.points.append(point)
        self.send(part, "pose_reference", message)

    def send_twist_reference(
        self,
        part: str,
        linear: list[float],
        angular: list[float],
        frame_id: str,
    ) -> None:
        if len(linear) != 3 or len(angular) != 3:
            raise ValueError("Cartesian twist linear and angular must be length 3")
        message = TwistStamped()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = linear
        message.twist.angular.x, message.twist.angular.y, message.twist.angular.z = angular
        self.send(part, "twist_reference", message)

    async def execute_joint_trajectory(
        self,
        part: str,
        trajectory: Any,
        joint_names: list[str],
        timeout_sec: float,
    ) -> Any:
        """Execute a complete trajectory and wait for its terminal result."""
        handle = await self.start_joint_trajectory(part, trajectory, joint_names)
        return await self.wait_joint_trajectory(part, handle, timeout_sec)

    async def start_joint_trajectory(
        self,
        part: str,
        trajectory: Any,
        joint_names: list[str],
        feedback_callback: Any | None = None,
    ) -> Any:
        """Send and accept a trajectory without claiming terminal success."""
        client = self._client(part, "joint_trajectory")
        message = _joint_trajectory_from_spec(trajectory, joint_names)
        return await client.send(message, feedback_callback=feedback_callback)

    async def wait_joint_trajectory(
        self, part: str, handle: Any, timeout_sec: float
    ) -> Any:
        """Wait for a previously accepted trajectory's terminal result."""
        client = self._client(part, "joint_trajectory")
        return await client.wait_for_result(handle, timeout_sec=timeout_sec)

    async def cancel_joint_trajectory(self, part: str) -> None:
        """Cancel this provider's active trajectory for ``part``."""
        await self._client(part, "joint_trajectory").cancel()

    def send(self, part: str, command: str, message: Any) -> Any:
        """Send one native ROS command through this provider's EM gateway."""
        client = self._client(part, command)
        now_msg = self._node.get_clock().now().to_msg()
        if command == "joint_trajectory":
            if hasattr(message, "header") and message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
                message.header.stamp = now_msg
            return client.send(message)
        if command == "joint_reference":
            if not isinstance(message, JointTrajectory):
                part_joints = (
                    self.profile.parts[part].joint_names
                    if self.profile and part in self.profile.parts
                    else []
                )
                message = _joint_trajectory_from_spec(
                    message, part_joints, require_increasing_time=False
                )
            if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
                message.header.stamp = now_msg
            return client.send(message)
        if command == "pose_reference":
            if not isinstance(message, CartesianTrajectory):
                base_frame = (
                    self.profile.parts[part].base_frame
                    if self.profile and part in self.profile.parts
                    else "base_link"
                )
                tcp_frame = (
                    self.profile.parts[part].tcp_frame
                    if self.profile and part in self.profile.parts
                    else ""
                )
                message = _cartesian_trajectory_from_spec(
                    message, base_frame, tcp_frame
                )
            if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
                message.header.stamp = now_msg
            return client.send_pose(message)
        if command == "twist_reference":
            if not isinstance(message, TwistStamped):
                base_frame = (
                    self.profile.parts[part].base_frame
                    if self.profile and part in self.profile.parts
                    else "base_link"
                )
                message = _twist_stamped_from_spec(message, base_frame)
            if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
                message.header.stamp = now_msg
            return client.send_twist(message)
        raise KeyError(f"unsupported provider command {command!r}")

    def _client(self, part: str, command: str) -> Any:
        try:
            client = self.controllers[part]
        except KeyError as exc:
            raise KeyError(f"provider {self.name!r} does not control part {part!r}") from exc
        if command not in self.commands[part]:
            raise KeyError(
                f"provider {self.name!r} has no {command!r} source for part {part!r}"
            )
        return client


def _set_duration(duration: Any, seconds: float) -> None:
    nanoseconds = round(float(seconds) * 1e9)
    duration.sec = nanoseconds // 1_000_000_000
    duration.nanosec = nanoseconds % 1_000_000_000


def _joint_trajectory_from_spec(
    trajectory: Any,
    default_joint_names: list[str],
    *,
    require_increasing_time: bool = True,
) -> JointTrajectory:
    if isinstance(trajectory, JointTrajectory):
        message = trajectory
        if not message.joint_names:
            message.joint_names = list(default_joint_names)
        return message

    # Object with .points (JointHorizonResult, PlanResult)
    if hasattr(trajectory, "points") and not isinstance(trajectory, JointTrajectory):
        in_names = list(
            getattr(trajectory, "joint_names", default_joint_names)
            or default_joint_names
        )
        target_names = list(default_joint_names) if default_joint_names else in_names
        indices = (
            [in_names.index(n) for n in target_names]
            if set(target_names).issubset(set(in_names))
            else list(range(len(in_names)))
        )
        out_names = [in_names[i] for i in indices]

        message = JointTrajectory()
        message.joint_names = out_names
        for p in trajectory.points:
            point = JointTrajectoryPoint()
            pos = list(getattr(p, "positions", []))
            point.positions = [float(pos[i]) for i in indices] if pos else []
            vel = list(getattr(p, "velocities", []) or [])
            point.velocities = [float(vel[i]) for i in indices] if vel else []
            acc = list(getattr(p, "accelerations", []) or [])
            point.accelerations = [float(acc[i]) for i in indices] if acc else []
            _set_duration(point.time_from_start, getattr(p, "time_from_start_s", 0.0))
            message.points.append(point)
        return message

    # Object with .positions (ResolveResult)
    if hasattr(trajectory, "positions") and not hasattr(trajectory, "points"):
        in_names = list(
            getattr(trajectory, "joint_names", default_joint_names)
            or default_joint_names
        )
        target_names = list(default_joint_names) if default_joint_names else in_names
        indices = (
            [in_names.index(n) for n in target_names]
            if set(target_names).issubset(set(in_names))
            else list(range(len(in_names)))
        )
        out_names = [in_names[i] for i in indices]

        message = JointTrajectory()
        message.joint_names = out_names
        point = JointTrajectoryPoint()
        pos = list(trajectory.positions)
        point.positions = [float(pos[i]) for i in indices] if pos else []
        vel = list(getattr(trajectory, "velocities", []) or [])
        point.velocities = [float(vel[i]) for i in indices] if vel else []
        acc = list(getattr(trajectory, "accelerations", []) or [])
        point.accelerations = [float(acc[i]) for i in indices] if acc else []
        _set_duration(point.time_from_start, getattr(trajectory, "time_from_start_s", 0.0))
        message.points.append(point)
        return message

    if isinstance(trajectory, (list, tuple)):
        message = JointTrajectory()
        message.joint_names = list(default_joint_names)
        if trajectory and isinstance(trajectory[0], (list, tuple)):
            # 2D list: list of waypoints [[q1, q2, ...], [q1, q2, ...]]
            for i, row in enumerate(trajectory):
                point = JointTrajectoryPoint()
                point.positions = [
                    float(x) for x in row[: len(default_joint_names)]
                ]
                _set_duration(point.time_from_start, (i + 1) * 0.01)
                message.points.append(point)
        else:
            # 1D list: single waypoint [q1, q2, ...]
            point = JointTrajectoryPoint()
            point.positions = [
                float(x) for x in trajectory[: len(default_joint_names)]
            ]
            message.points.append(point)
        return message

    if isinstance(trajectory, dict):
        message = JointTrajectory()
        message.joint_names = list(
            trajectory.get("joint_names") or default_joint_names
        )
        points = trajectory.get("points")
        if not points:
            raise ValueError("trajectory must include non-empty points")
        for value in points:
            if isinstance(value, JointTrajectoryPoint):
                point = value
            else:
                if not isinstance(value, dict):
                    raise TypeError("trajectory points must be mappings or ROS points")
                point = JointTrajectoryPoint()
                point.positions = [float(item) for item in value["positions"]]
                point.velocities = [float(item) for item in value.get("velocities", [])]
                point.accelerations = [
                    float(item) for item in value.get("accelerations", [])
                ]
                _set_duration(
                    point.time_from_start, value.get("time_from_start_s", 0.0)
                )
            message.points.append(point)
        return message

    raise TypeError(f"cannot convert {type(trajectory)} to JointTrajectory")


def _cartesian_trajectory_from_spec(
    trajectory: Any,
    default_frame_id: str = "base_link",
    default_child_frame: str = "",
) -> CartesianTrajectory:
    if isinstance(trajectory, CartesianTrajectory):
        return trajectory

    message = CartesianTrajectory()
    message.header.frame_id = default_frame_id
    if default_child_frame:
        message.tracked_frame = default_child_frame

    # PoseHorizonResult
    if hasattr(trajectory, "points"):
        message.header.frame_id = getattr(trajectory, "frame_id", default_frame_id) or default_frame_id
        for p in trajectory.points:
            pt = CartesianTrajectoryPoint()
            pos = getattr(p, "position_xyz", getattr(p, "positions", None))
            if pos is not None:
                pt.point.pose.position.x = float(pos[0])
                pt.point.pose.position.y = float(pos[1])
                pt.point.pose.position.z = float(pos[2])
            ori = getattr(p, "orientation_wxyz", getattr(p, "orientations", None))
            if ori is not None:
                pt.point.pose.orientation.w = float(ori[0])
                pt.point.pose.orientation.x = float(ori[1])
                pt.point.pose.orientation.y = float(ori[2])
                pt.point.pose.orientation.z = float(ori[3])
            _set_duration(pt.time_from_start, getattr(p, "time_from_start_s", 0.0))
            message.points.append(pt)
        return message

    # CartesianState
    if hasattr(trajectory, "position_xyz"):
        pt = CartesianTrajectoryPoint()
        pos = trajectory.position_xyz
        pt.point.pose.position.x = float(pos[0])
        pt.point.pose.position.y = float(pos[1])
        pt.point.pose.position.z = float(pos[2])
        ori = trajectory.orientation_wxyz
        if ori is not None:
            pt.point.pose.orientation.w = float(ori[0])
            pt.point.pose.orientation.x = float(ori[1])
            pt.point.pose.orientation.y = float(ori[2])
            pt.point.pose.orientation.z = float(ori[3])
        message.points.append(pt)
        return message

    # dict format
    if isinstance(trajectory, dict):
        message.header.frame_id = trajectory.get("frame_id", default_frame_id)
        for val in trajectory.get("points", []):
            pt = CartesianTrajectoryPoint()
            if "position" in val:
                p = val["position"]
                pt.point.pose.position.x = float(p[0])
                pt.point.pose.position.y = float(p[1])
                pt.point.pose.position.z = float(p[2])
            if "orientation" in val:
                q = val["orientation"]
                pt.point.pose.orientation.w = float(q[0])
                pt.point.pose.orientation.x = float(q[1])
                pt.point.pose.orientation.y = float(q[2])
                pt.point.pose.orientation.z = float(q[3])
            _set_duration(pt.time_from_start, val.get("time_from_start_s", 0.0))
            message.points.append(pt)
        return message

    raise TypeError(f"cannot convert {type(trajectory)} to CartesianTrajectory")


def _twist_stamped_from_spec(
    twist: Any, default_frame_id: str = "base_link"
) -> TwistStamped:
    if isinstance(twist, TwistStamped):
        return twist

    message = TwistStamped()
    message.header.frame_id = default_frame_id

    if isinstance(twist, (list, tuple)) and len(twist) >= 6:
        message.twist.linear.x = float(twist[0])
        message.twist.linear.y = float(twist[1])
        message.twist.linear.z = float(twist[2])
        message.twist.angular.x = float(twist[3])
        message.twist.angular.y = float(twist[4])
        message.twist.angular.z = float(twist[5])
        return message

    if isinstance(twist, dict):
        message.header.frame_id = twist.get("frame_id", default_frame_id)
        lin = twist.get("linear", [0.0, 0.0, 0.0])
        ang = twist.get("angular", [0.0, 0.0, 0.0])
        message.twist.linear.x = float(lin[0])
        message.twist.linear.y = float(lin[1])
        message.twist.linear.z = float(lin[2])
        message.twist.angular.x = float(ang[0])
        message.twist.angular.y = float(ang[1])
        message.twist.angular.z = float(ang[2])
        return message

    raise TypeError(f"cannot convert {type(twist)} to TwistStamped")
