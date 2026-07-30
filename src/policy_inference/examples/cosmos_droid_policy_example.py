#!/usr/bin/env python3
"""Cosmos-DROID policy node: ROS observation -> action chunk -> EM."""

from __future__ import annotations

import json
import time
from pathlib import Path
from threading import Lock
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cosmos_droid.policy import CosmosDroidChunkPolicy
from cosmos_droid.record_8camera_dataset import (
    DEFAULT_CAMERA_TOPICS,
    FR3_JOINTS,
    ImageSample,
    _compose_roboarena_concat_view,
    _make_episode_dir,
    _stamp_to_seconds,
)
from cosmos_droid.ros_observation import CosmosDroidObservationCache, _image_msg_to_rgb8


POLICY_CAMERA_NAMES = ("cam_0", "cam_1", "cam_2")


class CosmosDroidPolicyExample(Node):
    def __init__(self) -> None:
        super().__init__("cosmos_droid_policy_example")
        self.declare_parameter("server_url", "ws://127.0.0.1:8000/")
        self.declare_parameter("joint_names", FR3_JOINTS)
        self.declare_parameter("joint_state_topic", "/franka/joint_states")
        self.declare_parameter("gripper_state_topic", "/franka_gripper/joint_states")
        self.declare_parameter("camera_topics", DEFAULT_CAMERA_TOPICS)  # 默认订阅 8 路相机
        self.declare_parameter("fake_gripper", 0.04)
        self.declare_parameter("task", "hold position")
        self.declare_parameter("target_fps", 15.0)
        self.declare_parameter("rate_hz", 15.0)  # 表示 policy node 多久推理一次 每秒推理 15 次
        self.declare_parameter("horizon", 5)
        self.declare_parameter("action_dt_s", 0.1)  # 表示 action chunk 里面 每两个动作点之间隔多久
        self.declare_parameter("action_topic", "/action_sources/policy/joint_chunk")
        self.declare_parameter("record_data", False)
        self.declare_parameter("record_output_dir", "data")
        self.declare_parameter("record_episode_name", "")
        self.declare_parameter("record_compressed", False)
        self.declare_parameter("max_camera_skew_s", 0.25)
        self.declare_parameter("max_state_age_s", 1.0)

        self.joint_names = list(self.get_parameter("joint_names").value)
        self.camera_topics = list(self.get_parameter("camera_topics").value)
        if len(self.camera_topics) != 8:
            raise ValueError(f"camera_topics must contain 8 topics, got {len(self.camera_topics)}")
        self.camera_names = [f"cam_{index}" for index in range(len(self.camera_topics))]
        self.task = str(self.get_parameter("task").value)
        self.horizon = int(self.get_parameter("horizon").value)
        self.action_dt_s = float(self.get_parameter("action_dt_s").value)
        self.target_fps = float(self.get_parameter("target_fps").value)
        if self.target_fps <= 0:
            self.target_fps = float(self.get_parameter("rate_hz").value)
        if self.target_fps <= 0:
            raise ValueError("target_fps must be positive")
        self.fake_gripper = float(self.get_parameter("fake_gripper").value)
        self.max_camera_skew_s = float(self.get_parameter("max_camera_skew_s").value)
        self.max_state_age_s = float(self.get_parameter("max_state_age_s").value)
        self.record_data = bool(self.get_parameter("record_data").value)
        self.record_compressed = bool(self.get_parameter("record_compressed").value)
        self._image_lock = Lock()
        self._latest_images: dict[str, ImageSample] = {}
        self._latest_joint_stamp_s: float | None = None
        self._latest_gripper_stamp_s: float | None = None
        self._latest_gripper_is_fake = False
        self._record_frame_index = 0
        self._last_record_sample_time_s: float | None = None
        self._manifest = None
        self._episode_dir: Path | None = None

        self.observation_cache = CosmosDroidObservationCache(self.joint_names)
        self.policy = CosmosDroidChunkPolicy(
            horizon=self.horizon,
            action_dt_s=self.action_dt_s,
            server_url=str(self.get_parameter("server_url").value),
        )

        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self.on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("gripper_state_topic").value),
            self.on_gripper_state,
            qos_profile_sensor_data,
        )
        for name, topic in zip(self.camera_names, self.camera_topics, strict=True):
            self.create_subscription(
                Image,
                topic,
                lambda msg, camera_name=name: self.on_camera_image(camera_name, msg),
                qos_profile_sensor_data,
            )
        self.publisher = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter("action_topic").value),
            1,
        )
        if self.record_data:
            self._open_recording()
        self.timer = self.create_timer(1.0 / self.target_fps, self.infer)

    def on_joint_state(self, msg: JointState) -> None:
        self.observation_cache.on_joint_state(msg)
        self._latest_joint_stamp_s = _stamp_to_seconds(
            msg.header.stamp,
            self.get_clock().now().nanoseconds / 1e9,
        )

    def on_gripper_state(self, msg: JointState) -> None:
        try:
            self.observation_cache.on_gripper_position(msg)
            self._latest_gripper_stamp_s = _stamp_to_seconds(
                msg.header.stamp,
                self.get_clock().now().nanoseconds / 1e9,
            )
            self._latest_gripper_is_fake = False
        except ValueError as exc:
            self.get_logger().warning(f"忽略 gripper 状态: {exc}")

    def on_camera_image(self, camera_name: str, msg: Image) -> None:
        try:
            rgb = _image_msg_to_rgb8(msg)
        except ValueError as exc:
            self.get_logger().warning(f"忽略 {camera_name} 图像: {exc}")
            return

        sample = ImageSample(
            image=rgb,
            stamp_s=_stamp_to_seconds(
                msg.header.stamp,
                self.get_clock().now().nanoseconds / 1e9,
            ),
            encoding=msg.encoding,
            width=int(msg.width),
            height=int(msg.height),
        )
        with self._image_lock:
            self._latest_images[camera_name] = sample

        if camera_name == POLICY_CAMERA_NAMES[0]:
            self.observation_cache._set_image("wrist", rgb)
        elif camera_name == POLICY_CAMERA_NAMES[1]:
            self.observation_cache._set_image("exterior_1", rgb)
        elif camera_name == POLICY_CAMERA_NAMES[2]:
            self.observation_cache._set_image("exterior_2", rgb)

    def infer(self) -> None:
        self._set_fake_gripper_if_needed()
        observation = self.observation_cache.get_observation(self.task)
        if observation is None:
            missing = ", ".join(self.observation_cache.missing())
            self.get_logger().debug(f"等待 observation: {missing}")
            return

        self._record_observation_if_enabled(observation)
        try:
            action = self.policy.predict(observation)
            self.publisher.publish(self.action_chunk_to_ros(action))
        except Exception as exc:
            self.get_logger().error(f"Cosmos-DROID 推理/发布失败: {exc}")

    def action_chunk_to_ros(self, action: np.ndarray) -> JointTrajectory:
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (self.horizon, len(self.joint_names)):
            raise ValueError(f"Expected action shape [T,D], got {action.shape}")
        if not np.isfinite(action).all():
            raise ValueError("Action contains NaN or Inf")

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        for index, positions in enumerate(action, start=1):
            point = JointTrajectoryPoint()
            point.positions = positions.astype(float).tolist()
            nanoseconds = round(index * self.action_dt_s * 1e9)
            point.time_from_start.sec = nanoseconds // 1_000_000_000
            point.time_from_start.nanosec = nanoseconds % 1_000_000_000
            msg.points.append(point)
        return msg

    def close(self) -> None:
        if self._manifest is not None:
            self._manifest.close()
            self._manifest = None
        if hasattr(self.policy, "close"):
            self.policy.close()

    def _set_fake_gripper_if_needed(self) -> None:
        if "gripper_position" not in self.observation_cache.missing():
            return
        self.observation_cache.on_gripper_position(self.fake_gripper)
        self._latest_gripper_stamp_s = self.get_clock().now().nanoseconds / 1e9
        self._latest_gripper_is_fake = True

    def _record_observation_if_enabled(self, observation: dict[str, Any]) -> None:
        if not self.record_data or self._manifest is None or self._episode_dir is None:
            return

        now_s = self.get_clock().now().nanoseconds / 1e9
        with self._image_lock:
            missing_cameras = [name for name in self.camera_names if name not in self._latest_images]
            if missing_cameras:
                self.get_logger().warning(f"跳过保存: 缺少相机 {missing_cameras}")
                return
            images = {name: self._latest_images[name] for name in self.camera_names}

        camera_stamps = np.asarray([sample.stamp_s for sample in images.values()], dtype=np.float64)
        sample_time_s = float(camera_stamps.mean())
        sample_dt_s = (
            None
            if self._last_record_sample_time_s is None
            else sample_time_s - self._last_record_sample_time_s
        )
        camera_skew_s = float(camera_stamps.max() - camera_stamps.min())
        gripper_stamp_s = float(self._latest_gripper_stamp_s or now_s)
        joint_stamp_s = float(self._latest_joint_stamp_s or now_s)
        max_state_age_s = max(
            abs(now_s - joint_stamp_s),
            abs(now_s - gripper_stamp_s),
            *(abs(now_s - sample.stamp_s) for sample in images.values()),
        )
        if camera_skew_s > self.max_camera_skew_s:
            self.get_logger().warning(
                f"跳过保存: camera skew {camera_skew_s:.3f}s > {self.max_camera_skew_s:.3f}s"
            )
            return
        if max_state_age_s > self.max_state_age_s:
            self.get_logger().warning(
                f"跳过保存: state age {max_state_age_s:.3f}s > {self.max_state_age_s:.3f}s"
            )
            return

        frame_path = self._episode_dir / "frames" / f"frame_{self._record_frame_index:06d}.npz"
        frame_data = {
            "prompt": np.asarray(self.task),
            "observation/joint_position": observation["observation.state"][None, :].astype(np.float32),
            "observation/gripper_position": observation["observation.gripper"].reshape(1, 1).astype(np.float32),
            "observation/image": _compose_roboarena_concat_view(
                images["cam_0"].image,
                images["cam_1"].image,
                images["cam_2"].image,
            ),
            "observation/wrist_image_left": images["cam_0"].image,
            "observation/exterior_image_1_left": images["cam_1"].image,
            "observation/exterior_image_2_left": images["cam_2"].image,
            "joint_names": np.asarray(self.joint_names),
            "camera_names": np.asarray(self.camera_names),
        }
        for name, sample in images.items():
            frame_data[f"observation/images/{name}"] = sample.image
            frame_data[f"timestamp/images/{name}"] = np.asarray(sample.stamp_s, dtype=np.float64)
        if self.record_compressed:
            np.savez_compressed(frame_path, **frame_data)
        else:
            np.savez(frame_path, **frame_data)

        row = {
            "frame_index": self._record_frame_index,
            "file": str(frame_path.relative_to(self._episode_dir)),
            "task": self.task,
            "camera_topics": dict(zip(self.camera_names, self.camera_topics, strict=True)),
            "camera_encodings": {name: images[name].encoding for name in self.camera_names},
            "camera_stamps_s": {name: images[name].stamp_s for name in self.camera_names},
            "sample_time_s": sample_time_s,
            "sample_dt_s": sample_dt_s,
            "target_fps": self.target_fps,
            "camera_skew_s": camera_skew_s,
            "joint_stamp_s": joint_stamp_s,
            "gripper_stamp_s": gripper_stamp_s,
            "gripper_is_fake": self._latest_gripper_is_fake,
            "system_time_s": time.time(),
        }
        self._manifest.write(json.dumps(row, ensure_ascii=False) + "\n")
        self._manifest.flush()
        self._last_record_sample_time_s = sample_time_s
        self._record_frame_index += 1

    def _open_recording(self) -> None:
        output_dir = Path(str(self.get_parameter("record_output_dir").value))
        episode_name = str(self.get_parameter("record_episode_name").value)
        self._episode_dir = _make_episode_dir(output_dir, episode_name)
        (self._episode_dir / "frames").mkdir(parents=True, exist_ok=False)
        self._manifest = (self._episode_dir / "metadata.jsonl").open("w", encoding="utf-8")
        metadata = {
            "type": "cosmos_droid_policy_realtime_recording",
            "task": self.task,
            "target_fps": self.target_fps,
            "camera_topics": dict(zip(self.camera_names, self.camera_topics, strict=True)),
            "policy_cameras": {
                "wrist_image_left": "cam_0",
                "exterior_image_1_left": "cam_1",
                "exterior_image_2_left": "cam_2",
            },
            "all_cameras_saved": self.camera_names,
            "joint_state_topic": str(self.get_parameter("joint_state_topic").value),
            "gripper_state_topic": str(self.get_parameter("gripper_state_topic").value),
            "joint_names": self.joint_names,
            "fake_gripper_default": self.fake_gripper,
        }
        (self._episode_dir / "episode_metadata.json").write_text(
            json.dumps(metadata, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        self.get_logger().info(f"recording 8-camera data into {self._episode_dir}")


def main() -> None:
    rclpy.init()
    node = CosmosDroidPolicyExample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
