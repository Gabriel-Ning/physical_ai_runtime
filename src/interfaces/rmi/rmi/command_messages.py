"""Validate RMI command values and encode their ROS message payloads."""

from __future__ import annotations

import math
from collections.abc import Mapping
from types import SimpleNamespace
from typing import Any

from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import (
    CartesianTrajectory,
    CartesianTrajectoryPoint,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def reject_invalid_result(value: Any) -> None:
    if value is not None and hasattr(value, "valid") and not bool(value.valid):
        raise ValueError(
            f"cannot send invalid planning result: {getattr(value, 'reason', '')}"
        )


def _validate_finite(value: float, name: str = "value") -> float:
    result = float(value)
    if math.isnan(result) or math.isinf(result):
        raise ValueError(f"{name} contains NaN or infinity: {value}")
    return result


def _set_duration(duration: Any, seconds: float) -> None:
    nanoseconds = round(float(seconds) * 1e9)
    duration.sec = nanoseconds // 1_000_000_000
    duration.nanosec = nanoseconds % 1_000_000_000


def joint_trajectory_from_spec(
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
        point.positions = [
            _validate_finite(value, "joint position") for value in trajectory
        ]
        message.points.append(point)
    else:
        message = JointTrajectory()
        message.joint_names = list(
            getattr(trajectory, "joint_names", None) or default_joint_names
        )
        for value in getattr(trajectory, "points", []):
            point = JointTrajectoryPoint()
            point.positions = [
                _validate_finite(item, "joint position")
                for item in value.positions
            ]
            if getattr(value, "velocities", None) is not None:
                point.velocities = [
                    _validate_finite(item, "joint velocity")
                    for item in value.velocities
                ]
            if getattr(value, "accelerations", None) is not None:
                point.accelerations = [
                    _validate_finite(item, "joint acceleration")
                    for item in value.accelerations
                ]
            _set_duration(
                point.time_from_start,
                getattr(value, "time_from_start_s", 0.0),
            )
            message.points.append(point)
    if not message.joint_names or not message.points:
        raise ValueError("joint trajectory requires joint_names and points")
    previous = -1.0
    for point in message.points:
        if len(point.positions) != len(message.joint_names):
            raise ValueError("joint trajectory positions do not match joint_names")
        current = float(point.time_from_start.sec) + float(
            point.time_from_start.nanosec
        ) * 1e-9
        if require_increasing_time and current <= previous:
            raise ValueError("trajectory time_from_start must be strictly increasing")
        previous = current
    return message


def cartesian_trajectory_from_spec(
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
    elif (
        items is None
        and isinstance(value, Mapping)
        and "position" in value
        and "orientation" in value
    ):
        items = [
            SimpleNamespace(
                position_xyz=value["position"],
                orientation_wxyz=value["orientation"],
            )
        ]
    for item in items or []:
        point = CartesianTrajectoryPoint()
        position = [
            _validate_finite(component, "cartesian position")
            for component in item.position_xyz
        ]
        (
            point.point.pose.position.x,
            point.point.pose.position.y,
            point.point.pose.position.z,
        ) = position
        orientation = [
            _validate_finite(component, "cartesian orientation")
            for component in item.orientation_wxyz
        ]
        w, x, y, z = orientation
        point.point.pose.orientation.w = w
        point.point.pose.orientation.x = x
        point.point.pose.orientation.y = y
        point.point.pose.orientation.z = z
        _set_duration(
            point.time_from_start, getattr(item, "time_from_start_s", 0.0)
        )
        message.points.append(point)
    if not message.points:
        raise ValueError("pose reference requires points")
    return message


def twist_stamped_from_spec(value: Any, base_frame: str) -> TwistStamped:
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
        _validate_finite(item, "twist linear") for item in linear
    ]
    message.twist.angular.x, message.twist.angular.y, message.twist.angular.z = [
        _validate_finite(item, "twist angular") for item in angular
    ]
    return message
