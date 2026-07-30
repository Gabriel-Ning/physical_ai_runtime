#!/usr/bin/env python3
"""Dry-run `/action_sources/policy/joint_chunk` without policy server or hardware."""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Callable

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory


EXAMPLES_DIR = Path(__file__).resolve().parents[1]
if str(EXAMPLES_DIR) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_DIR))

import cosmos_droid_policy_example as policy_entry  # noqa: E402


class FakeCosmosDroidChunkPolicy:
    """Small deterministic policy stub for ROS topic contract validation."""

    def __init__(self, horizon: int, action_dt_s: float, server_url: str) -> None:
        self.horizon = horizon
        self.action_dt_s = action_dt_s
        self.server_url = server_url
        self.last_observation: dict | None = None

    def predict(self, observation: dict) -> np.ndarray:
        self.last_observation = observation
        joint_position = np.asarray(observation["observation.state"], dtype=np.float32)
        offsets = np.arange(self.horizon, dtype=np.float32)[:, None] * 0.001
        return joint_position[None, :] + offsets

    def close(self) -> None:
        pass


def _make_joint_state(joint_names: list[str]) -> JointState:
    msg = JointState()
    msg.name = joint_names
    msg.position = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
    return msg


def _make_gripper_state() -> JointState:
    msg = JointState()
    msg.name = ["panda_finger_joint1", "panda_finger_joint2"]
    msg.position = [0.04, 0.04]
    return msg


def _make_rgb_image(red: int, green: int, blue: int) -> Image:
    msg = Image()
    msg.height = 2
    msg.width = 2
    msg.encoding = "rgb8"
    msg.is_bigendian = False
    msg.step = msg.width * 3
    pixel = np.asarray([red, green, blue], dtype=np.uint8)
    msg.data = np.tile(pixel, msg.height * msg.width).tobytes()
    return msg


def _spin_until(
    executor: SingleThreadedExecutor,
    predicate: Callable[[], bool],
    timeout_s: float,
) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def _assert_joint_chunk(
    msg: JointTrajectory,
    joint_names: list[str],
    horizon: int,
    action_dt_s: float,
) -> None:
    if msg.joint_names != joint_names:
        raise AssertionError(f"joint_names mismatch: {msg.joint_names}")
    if len(msg.points) != horizon:
        raise AssertionError(f"expected {horizon} points, got {len(msg.points)}")

    base_position = np.asarray(_make_joint_state(joint_names).position, dtype=np.float32)
    for index, point in enumerate(msg.points, start=1):
        positions = np.asarray(point.positions, dtype=np.float32)
        expected_positions = base_position + (index - 1) * 0.001
        if positions.shape != base_position.shape:
            raise AssertionError(f"point {index} position shape mismatch: {positions}")
        if not np.allclose(positions, expected_positions):
            raise AssertionError(
                f"point {index} positions mismatch: {positions} != {expected_positions}"
            )

        expected_ns = round(index * action_dt_s * 1e9)
        actual_ns = point.time_from_start.sec * 1_000_000_000
        actual_ns += point.time_from_start.nanosec
        if actual_ns != expected_ns:
            raise AssertionError(
                f"point {index} time mismatch: {actual_ns} ns != {expected_ns} ns"
            )


def main() -> None:
    policy_entry.CosmosDroidChunkPolicy = FakeCosmosDroidChunkPolicy

    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "joint_state_topic:=/cosmos_droid_dry_run/joint_states",
            "-p",
            "gripper_state_topic:=/cosmos_droid_dry_run/gripper_joint_states",
            "-p",
            "camera_topics:=[/cosmos_droid_dry_run/cam_0,/cosmos_droid_dry_run/cam_1,/cosmos_droid_dry_run/cam_2,/cosmos_droid_dry_run/cam_3,/cosmos_droid_dry_run/cam_4,/cosmos_droid_dry_run/cam_5,/cosmos_droid_dry_run/cam_6,/cosmos_droid_dry_run/cam_7]",
        ]
    )
    executor = SingleThreadedExecutor()
    policy_node = None
    driver_node = None
    probe_node = None
    try:
        policy_node = policy_entry.CosmosDroidPolicyExample()
        driver_node = Node("cosmos_droid_dry_run_driver")
        probe_node = Node("cosmos_droid_dry_run_probe")

        received: list[JointTrajectory] = []
        probe_node.create_subscription(
            JointTrajectory,
            "/action_sources/policy/joint_chunk",
            lambda msg: received.append(msg),
            1,
        )

        joint_pub = driver_node.create_publisher(
            JointState,
            "/cosmos_droid_dry_run/joint_states",
            qos_profile_sensor_data,
        )
        gripper_pub = driver_node.create_publisher(
            JointState,
            "/cosmos_droid_dry_run/gripper_joint_states",
            qos_profile_sensor_data,
        )
        wrist_pub = driver_node.create_publisher(
            Image,
            "/cosmos_droid_dry_run/cam_0",
            qos_profile_sensor_data,
        )
        exterior_1_pub = driver_node.create_publisher(
            Image,
            "/cosmos_droid_dry_run/cam_1",
            qos_profile_sensor_data,
        )
        exterior_2_pub = driver_node.create_publisher(
            Image,
            "/cosmos_droid_dry_run/cam_2",
            qos_profile_sensor_data,
        )

        executor.add_node(policy_node)
        executor.add_node(driver_node)
        executor.add_node(probe_node)

        joint_msg = _make_joint_state(policy_node.joint_names)
        gripper_msg = _make_gripper_state()
        wrist_msg = _make_rgb_image(255, 0, 0)
        exterior_1_msg = _make_rgb_image(0, 255, 0)
        exterior_2_msg = _make_rgb_image(0, 0, 255)

        def observation_ready() -> bool:
            joint_pub.publish(joint_msg)
            gripper_pub.publish(gripper_msg)
            wrist_pub.publish(wrist_msg)
            exterior_1_pub.publish(exterior_1_msg)
            exterior_2_pub.publish(exterior_2_msg)
            return policy_node.observation_cache.ready()

        if not _spin_until(executor, observation_ready, timeout_s=3.0):
            missing = ", ".join(policy_node.observation_cache.missing())
            raise TimeoutError(f"observation cache not ready, missing: {missing}")

        policy_node.infer()
        if not _spin_until(executor, lambda: bool(received), timeout_s=3.0):
            raise TimeoutError("did not receive /action_sources/policy/joint_chunk")

        _assert_joint_chunk(
            received[-1],
            policy_node.joint_names,
            policy_node.horizon,
            policy_node.action_dt_s,
        )
        print(
            "PASS dry-run: received /action_sources/policy/joint_chunk "
            f"with {len(received[-1].points)} points and "
            f"{len(received[-1].joint_names)} joints"
        )
    finally:
        if policy_node is not None and hasattr(policy_node.policy, "close"):
            policy_node.policy.close()
        for node in (policy_node, driver_node, probe_node):
            if node is not None:
                executor.remove_node(node)
                node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
