"""Lease-bound command client used by one RMI ControlSession."""

from __future__ import annotations

import asyncio
import math
from collections.abc import Mapping
from types import SimpleNamespace
from typing import Any

from action_msgs.msg import GoalStatus
from execution_manager_interfaces.action import LeasedFollowJointTrajectory
from execution_manager_interfaces.msg import (
    LeasedJointReference,
    LeasedPoseReference,
    LeasedTwistReference,
)
from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import (
    CartesianPoint,
    CartesianTrajectory,
    CartesianTrajectoryPoint,
)
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .config import EmbodimentConfig
from .contracts import PoseHorizonResult, ResolveResult
from .errors import ControllerClientError, TrajectoryCanceledError
from .selection import EndpointBinding, LeaseGrant


def _command_qos() -> QoSProfile:
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)


class ActionProviderClient:
    """Publishes typed leased commands only to endpoints returned by EM."""

    def __init__(
        self,
        name: str,
        node: Any,
        resources: Mapping[str, str],
        *,
        profile: EmbodimentConfig | None = None,
        timeout_sec: float = 5.0,
        action_client_factory: Any = ActionClient,
    ) -> None:
        self.name = name
        self._node = node
        self.resources = dict(resources)
        self.profile = profile
        self._timeout_sec = timeout_sec
        # Context passes None when no override is configured; keep rclpy ActionClient.
        self._action_client_factory = (
            ActionClient if action_client_factory is None else action_client_factory
        )
        self._lease_id = ""
        self._bindings: dict[tuple[str, str], EndpointBinding] = {}
        self._publishers: dict[tuple[str, str], Any] = {}
        self._action_clients: dict[str, Any] = {}
        self._goal_handles: dict[str, Any] = {}

    def fork(self) -> ActionProviderClient:
        """Create an independent lease binding for a concurrent Session."""
        return ActionProviderClient(
            self.name,
            self._node,
            self.resources,
            profile=self.profile,
            timeout_sec=self._timeout_sec,
            action_client_factory=self._action_client_factory,
        )

    def bind(self, grant: LeaseGrant) -> None:
        if self._lease_id:
            raise RuntimeError("command client is already bound to a lease")
        self._lease_id = grant.lease_id
        self._bindings = dict(grant.endpoints)

    def unbind(self) -> None:
        if hasattr(self._node, "destroy_publisher"):
            for publisher in self._publishers.values():
                self._node.destroy_publisher(publisher)
        for client in self._action_clients.values():
            destroy = getattr(client, "destroy", None)
            if destroy is not None:
                destroy()
        self._lease_id = ""
        self._bindings.clear()
        self._publishers.clear()
        self._action_clients.clear()
        self._goal_handles.clear()

    def send_joint_reference(
        self,
        part: str,
        joint_names: list[str],
        positions: list[list[float]],
        times_from_start_sec: list[float],
    ) -> None:
        message = JointTrajectory()
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
        message = CartesianTrajectory()
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
        self, part: str, linear: list[float], angular: list[float], frame_id: str
    ) -> None:
        if len(linear) != 3 or len(angular) != 3:
            raise ValueError("Cartesian twist linear and angular must be length 3")
        message = TwistStamped()
        message.header.frame_id = frame_id
        message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = linear
        message.twist.angular.x, message.twist.angular.y, message.twist.angular.z = angular
        self.send(part, "twist_reference", message)

    def send(self, part: str, command: str, message: Any) -> None:
        self._require_binding(part, command)
        _reject_invalid_result(message)
        if isinstance(message, ResolveResult) and command != "joint_reference":
            raise ValueError("ResolveResult requires command='joint_reference'")
        if isinstance(message, PoseHorizonResult) and command != "pose_reference":
            raise ValueError("PoseHorizonResult requires command='pose_reference'")
        now = self._node.get_clock().now().to_msg()
        if command == "joint_reference":
            payload = (
                message
                if isinstance(message, JointTrajectory)
                else _joint_trajectory_from_spec(message, self._joint_names(part), False)
            )
            envelope = LeasedJointReference()
        elif command == "pose_reference":
            payload = (
                message
                if isinstance(message, CartesianTrajectory)
                else _cartesian_trajectory_from_spec(
                    message, self._base_frame(part), self._tcp_frame(part)
                )
            )
            envelope = LeasedPoseReference()
        elif command == "twist_reference":
            payload = (
                message
                if isinstance(message, TwistStamped)
                else _twist_stamped_from_spec(message, self._base_frame(part))
            )
            envelope = LeasedTwistReference()
        else:
            raise KeyError(f"unsupported streaming command {command!r}")
        # EM admission uses the envelope stamp; controllers use the payload
        # stamp. Streaming references are dispatched now, so stamp both at the
        # same send boundary.
        if hasattr(payload, "header"):
            payload.header.stamp = now
        envelope.header.stamp = now
        envelope.lease_id = self._lease_id
        envelope.command = payload
        self._publisher(part, command, type(envelope)).publish(envelope)

    async def start_joint_trajectory(
        self,
        part: str,
        trajectory: Any,
        joint_names: list[str],
        feedback_callback: Any | None = None,
    ) -> Any:
        binding = self._require_binding(part, "joint_trajectory")
        trajectory = _joint_trajectory_from_spec(trajectory, joint_names, True)
        client = self._action_clients.get(part)
        if client is None:
            client = self._action_client_factory(
                self._node, LeasedFollowJointTrajectory, binding.endpoint
            )
            self._action_clients[part] = client
        await _wait_ready(client, self._timeout_sec)
        goal = LeasedFollowJointTrajectory.Goal()
        goal.header.stamp = self._node.get_clock().now().to_msg()
        goal.lease_id = self._lease_id
        goal.resource = part
        goal.trajectory = trajectory
        future = (
            client.send_goal_async(goal)
            if feedback_callback is None
            else client.send_goal_async(goal, feedback_callback=feedback_callback)
        )
        handle = await _bounded(future, self._timeout_sec, "send leased trajectory")
        if not handle.accepted:
            raise ControllerClientError("Execution Manager rejected trajectory")
        self._goal_handles[part] = handle
        return handle

    async def wait_joint_trajectory(
        self, part: str, handle: Any, timeout_sec: float
    ) -> Any:
        wrapped = await _bounded(
            handle.get_result_async(), timeout_sec, "wait for leased trajectory"
        )
        self._goal_handles.pop(part, None)
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            return wrapped.result
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            raise TrajectoryCanceledError("trajectory goal was canceled")
        raise ControllerClientError(
            wrapped.result.error_string or f"trajectory status {wrapped.status}"
        )

    async def cancel_joint_trajectory(self, part: str) -> None:
        handle = self._goal_handles.pop(part, None)
        if handle is not None:
            await _bounded(
                handle.cancel_goal_async(), self._timeout_sec, "cancel trajectory"
            )

    def _publisher(self, part: str, command: str, message_type: Any) -> Any:
        key = (part, command)
        publisher = self._publishers.get(key)
        if publisher is None:
            publisher = self._node.create_publisher(
                message_type, self._bindings[key].endpoint, _command_qos()
            )
            self._publishers[key] = publisher
        return publisher

    def _require_binding(self, part: str, command: str) -> EndpointBinding:
        if not self._lease_id:
            raise RuntimeError("command client has no active lease")
        try:
            return self._bindings[(part, command)]
        except KeyError as exc:
            raise KeyError(
                f"lease has no {command!r} binding for resource {part!r}"
            ) from exc

    def _joint_names(self, part: str) -> list[str]:
        return list(self.profile.parts[part].joint_names) if self.profile else []

    def _base_frame(self, part: str) -> str:
        return (self.profile.parts[part].base_frame or "base_link") if self.profile else "base_link"

    def _tcp_frame(self, part: str) -> str:
        return (self.profile.parts[part].tcp_frame or "") if self.profile else ""


async def _wait_ready(client: Any, timeout: float) -> None:
    deadline = asyncio.get_running_loop().time() + timeout
    while not client.server_is_ready():
        if asyncio.get_running_loop().time() >= deadline:
            raise TimeoutError("leased trajectory action unavailable")
        await asyncio.sleep(0.01)


async def _bounded(future: Any, timeout: float, operation: str) -> Any:
    deadline = asyncio.get_running_loop().time() + timeout
    while not future.done():
        if asyncio.get_running_loop().time() >= deadline:
            future.cancel()
            raise TimeoutError(f"{operation} timed out")
        await asyncio.sleep(0.01)
    exception = future.exception()
    if exception is not None:
        raise exception
    return future.result()


def _reject_invalid_result(value: Any) -> None:
    if value is not None and hasattr(value, "valid") and not bool(value.valid):
        raise ValueError(f"cannot send invalid planning result: {getattr(value, 'reason', '')}")


def _validate_finite(value: float, name: str = "value") -> float:
    v = float(value)
    if math.isnan(v) or math.isinf(v):
        raise ValueError(f"{name} contains NaN or infinity: {value}")
    return v


def _set_duration(duration: Any, seconds: float) -> None:
    nanoseconds = round(float(seconds) * 1e9)
    duration.sec = nanoseconds // 1_000_000_000
    duration.nanosec = nanoseconds % 1_000_000_000


def _joint_trajectory_from_spec(
    trajectory: Any,
    default_joint_names: list[str],
    require_increasing_time: bool,
) -> JointTrajectory:
    if isinstance(trajectory, JointTrajectory):
        message = trajectory
    elif isinstance(trajectory, (list, tuple)):
        message = JointTrajectory()
        message.joint_names = list(default_joint_names)
        point = JointTrajectoryPoint()
        point.positions = [_validate_finite(value, "joint position") for value in trajectory]
        message.points.append(point)
    else:
        message = JointTrajectory()
        message.joint_names = list(getattr(trajectory, "joint_names", None) or default_joint_names)
        for value in getattr(trajectory, "points", []):
            point = JointTrajectoryPoint()
            point.positions = [_validate_finite(x, "joint position") for x in value.positions]
            if getattr(value, "velocities", None) is not None:
                point.velocities = [_validate_finite(x, "joint velocity") for x in value.velocities]
            if getattr(value, "accelerations", None) is not None:
                point.accelerations = [_validate_finite(x, "joint acceleration") for x in value.accelerations]
            _set_duration(point.time_from_start, getattr(value, "time_from_start_s", 0.0))
            message.points.append(point)
    if not message.joint_names or not message.points:
        raise ValueError("joint trajectory requires joint_names and points")
    previous = -1.0
    for point in message.points:
        if len(point.positions) != len(message.joint_names):
            raise ValueError("joint trajectory positions do not match joint_names")
        current = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9
        if require_increasing_time and current <= previous:
            raise ValueError("trajectory time_from_start must be strictly increasing")
        previous = current
    return message


def _cartesian_trajectory_from_spec(
    value: Any, base_frame: str, tcp_frame: str
) -> CartesianTrajectory:
    if isinstance(value, CartesianTrajectory):
        message = value
        if not message.header.frame_id:
            message.header.frame_id = base_frame
        if not message.tracked_frame:
            message.tracked_frame = tcp_frame
        return message
    message = CartesianTrajectory()
    message.header.frame_id = base_frame
    message.tracked_frame = tcp_frame
    items = getattr(value, "points", None)
    if items is None and hasattr(value, "position_xyz") and hasattr(
        value, "orientation_wxyz"
    ):
        items = [value]
    elif items is None and isinstance(value, Mapping) and "position" in value and "orientation" in value:
        items = [SimpleNamespace(position_xyz=value["position"], orientation_wxyz=value["orientation"])]
    for item in items or []:
        point = CartesianTrajectoryPoint()
        pos = [_validate_finite(p, "cartesian position") for p in item.position_xyz]
        point.point.pose.position.x, point.point.pose.position.y, point.point.pose.position.z = pos
        ori = [_validate_finite(q, "cartesian orientation") for q in item.orientation_wxyz]
        w, x, y, z = ori
        point.point.pose.orientation.w = w
        point.point.pose.orientation.x = x
        point.point.pose.orientation.y = y
        point.point.pose.orientation.z = z
        _set_duration(point.time_from_start, getattr(item, "time_from_start_s", 0.0))
        message.points.append(point)
    if not message.points:
        raise ValueError("pose reference requires points")
    return message


def _twist_stamped_from_spec(value: Any, base_frame: str) -> TwistStamped:
    if isinstance(value, TwistStamped):
        message = value
        if not message.header.frame_id:
            message.header.frame_id = base_frame
        return message
    message = TwistStamped()
    message.header.frame_id = base_frame
    if isinstance(value, (list, tuple)) and len(value) == 6:
        linear = value[:3]
        angular = value[3:]
    else:
        linear = getattr(value, "linear", (0.0, 0.0, 0.0))
        angular = getattr(value, "angular", (0.0, 0.0, 0.0))
    message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = [
        _validate_finite(x, "twist linear") for x in linear
    ]
    message.twist.angular.x, message.twist.angular.y, message.twist.angular.z = [
        _validate_finite(x, "twist angular") for x in angular
    ]
    return message
