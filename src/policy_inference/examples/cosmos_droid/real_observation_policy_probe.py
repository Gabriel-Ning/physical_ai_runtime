#!/usr/bin/env python3
"""Probe real ROS observations and optionally call the Cosmos-DROID policy."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState


EXAMPLES_DIR = Path(__file__).resolve().parents[1]
if str(EXAMPLES_DIR) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_DIR))

from cosmos_droid.policy import CosmosDroidChunkPolicy  # noqa: E402
from cosmos_droid.ros_observation import CosmosDroidObservationCache  # noqa: E402


FR3_JOINTS = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]


class RealObservationProbe(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("cosmos_droid_real_observation_probe")
        self.args = args
        self.cache = CosmosDroidObservationCache(args.joint_names)
        self.image_encodings: dict[str, str] = {}

        self.create_subscription(
            JointState,
            args.joint_state_topic,
            self.cache.on_joint_state,
            qos_profile_sensor_data,
        )
        if args.gripper_state_topic:
            self.create_subscription(
                JointState,
                args.gripper_state_topic,
                self._on_gripper_state,
                qos_profile_sensor_data,
            )
        self.create_subscription(
            Image,
            args.wrist_image_topic,
            lambda msg: self._on_image("wrist", self.cache.on_wrist_image, msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            args.exterior_image_1_topic,
            lambda msg: self._on_image(
                "exterior_1",
                self.cache.on_exterior_image_1,
                msg,
            ),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            args.exterior_image_2_topic,
            lambda msg: self._on_image(
                "exterior_2",
                self.cache.on_exterior_image_2,
                msg,
            ),
            qos_profile_sensor_data,
        )

    def set_fake_gripper_if_needed(self) -> None:
        if self.args.fake_gripper is None:
            return
        if "gripper_position" not in self.cache.missing():
            return
        self.cache.on_gripper_position(float(self.args.fake_gripper))

    def _on_gripper_state(self, msg: JointState) -> None:
        try:
            self.cache.on_gripper_position(msg)
        except ValueError as exc:
            self.get_logger().warning(f"ignoring gripper state: {exc}")

    def _on_image(self, name: str, callback, msg: Image) -> None:
        try:
            callback(msg)
            self.image_encodings[name] = msg.encoding
        except ValueError as exc:
            self.get_logger().warning(f"ignoring {name} image: {exc}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--gripper-state-topic", default="/franka_gripper/joint_states")
    parser.add_argument("--wrist-image-topic", default="/cameras/cam_0/image_raw")
    parser.add_argument("--exterior-image-1-topic", default="/cameras/cam_1/image_raw")
    parser.add_argument("--exterior-image-2-topic", default="/cameras/cam_2/image_raw")
    parser.add_argument("--joint-names", nargs="+", default=FR3_JOINTS)
    parser.add_argument("--fake-gripper", type=float, default=0.04)
    parser.add_argument("--task", default="hold position")
    parser.add_argument("--horizon", type=int, default=5)
    parser.add_argument("--action-dt-s", type=float, default=0.1)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument(
        "--server-url",
        default="",
        help="If set, call the real Cosmos policy server. Empty means shape-only fake policy.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    executor = SingleThreadedExecutor()
    node = RealObservationProbe(args)
    executor.add_node(node)
    try:
        deadline = time.monotonic() + args.timeout_s
        while time.monotonic() < deadline:
            node.set_fake_gripper_if_needed()
            executor.spin_once(timeout_sec=0.05)
            if node.cache.ready():
                break

        observation = node.cache.get_observation(args.task)
        if observation is None:
            missing = ", ".join(node.cache.missing())
            raise TimeoutError(f"observation not ready, missing: {missing}")

        print("PASS observation ready")
        print("joint:", observation["observation.state"].shape, observation["observation.state"].dtype)
        print("gripper:", observation["observation.gripper"].shape, observation["observation.gripper"].dtype)
        for key in ("wrist", "exterior_1", "exterior_2"):
            image = observation[f"observation.images.{key}"]
            encoding = node.image_encodings.get(key, "unknown")
            print(f"{key}: source={encoding} rgb={image.shape} {image.dtype}")

        if args.server_url:
            policy = CosmosDroidChunkPolicy(
                horizon=args.horizon,
                action_dt_s=args.action_dt_s,
                server_url=args.server_url,
            )
            try:
                action = policy.predict(observation)
            finally:
                policy.close()
        else:
            state = np.asarray(observation["observation.state"], dtype=np.float32)
            action = np.repeat(state[None, :], args.horizon, axis=0)

        print("PASS action inferred:", action.shape, action.dtype)
        print("first action:", np.asarray(action[0], dtype=np.float32).tolist())
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__": 
    main()
