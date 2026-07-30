#!/usr/bin/env python3
"""Record 8 ROS camera streams into Cosmos-DROID-style frame samples."""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from threading import Lock
from typing import Any

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState


EXAMPLES_DIR = Path(__file__).resolve().parents[1]
if str(EXAMPLES_DIR) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_DIR))

from cosmos_droid.ros_observation import _image_msg_to_rgb8  # noqa: E402


FR3_JOINTS = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]
DEFAULT_CAMERA_TOPICS = [f"/cameras/cam_{index}/image_raw" for index in range(8)]


@dataclass
class ImageSample:
    image: np.ndarray
    stamp_s: float
    encoding: str
    width: int
    height: int


class CosmosDroid8CameraRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("cosmos_droid_8camera_recorder")
        self.args = args
        self.camera_names = [f"cam_{index}" for index in range(len(args.camera_topics))]
        self._lock = Lock()
        self._images: dict[str, ImageSample] = {}
        self._joint_position: np.ndarray | None = None
        self._joint_stamp_s: float | None = None
        self._gripper_position: np.ndarray | None = None
        self._gripper_stamp_s: float | None = None
        self._frame_index = 0
        self._last_saved_sample_time_s: float | None = None
        self._last_status_log_s = 0.0
        self.done = False

        self.episode_dir = _make_episode_dir(args.output_dir, args.episode_name)
        self.frames_dir = self.episode_dir / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=False)
        self.manifest_path = self.episode_dir / "metadata.jsonl"
        self._manifest = self.manifest_path.open("w", encoding="utf-8")
        self._write_episode_metadata()

        for name, topic in zip(self.camera_names, args.camera_topics, strict=True):
            self.create_subscription(
                Image,
                topic,
                lambda msg, camera_name=name: self._on_image(camera_name, msg),
                qos_profile_sensor_data,
            )

        self.create_subscription(
            JointState,
            args.joint_state_topic,
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        if args.gripper_state_topic:
            self.create_subscription(
                JointState,
                args.gripper_state_topic,
                self._on_gripper_state,
                qos_profile_sensor_data,
            )

        self.create_timer(1.0 / args.target_fps, self._record_latest_frame)
        self.get_logger().info(f"recording Cosmos-DROID samples into {self.episode_dir}")

    def close(self) -> None:
        self._manifest.close()

    def _on_image(self, camera_name: str, msg: Image) -> None:
        try:
            rgb = _image_msg_to_rgb8(msg)
        except ValueError as exc:
            self.get_logger().warning(f"ignoring {camera_name}: {exc}")
            return

        sample = ImageSample(
            image=rgb,
            stamp_s=_stamp_to_seconds(msg.header.stamp, self.get_clock().now().nanoseconds / 1e9),
            encoding=msg.encoding,
            width=int(msg.width),
            height=int(msg.height),
        )
        with self._lock:
            self._images[camera_name] = sample

    def _on_joint_state(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position, strict=False))
        if not all(name in positions for name in self.args.joint_names):
            return

        joint_position = np.asarray(
            [positions[name] for name in self.args.joint_names],
            dtype=np.float32,
        )
        with self._lock:
            self._joint_position = joint_position
            self._joint_stamp_s = _stamp_to_seconds(
                msg.header.stamp,
                self.get_clock().now().nanoseconds / 1e9,
            )

    def _on_gripper_state(self, msg: JointState) -> None:
        if not msg.position:
            return
        with self._lock:
            self._gripper_position = np.asarray([float(msg.position[0])], dtype=np.float32)
            self._gripper_stamp_s = _stamp_to_seconds(
                msg.header.stamp,
                self.get_clock().now().nanoseconds / 1e9,
            )

    def _record_latest_frame(self) -> None:
        if self.done:
            return

        now_s = self.get_clock().now().nanoseconds / 1e9
        with self._lock:
            missing = self._missing_locked()
            if missing:
                self._log_waiting_status(now_s, missing)
                return

            assert self._joint_position is not None
            assert self._joint_stamp_s is not None
            images = {name: self._images[name] for name in self.camera_names}
            joint_position = self._joint_position.copy()
            joint_stamp_s = float(self._joint_stamp_s)

            gripper_is_fake = False
            if self._gripper_position is not None:
                gripper_position = self._gripper_position.copy()
                gripper_stamp_s = float(self._gripper_stamp_s or now_s)
            elif self.args.fake_gripper is not None:
                gripper_position = np.asarray([self.args.fake_gripper], dtype=np.float32)
                gripper_stamp_s = now_s
                gripper_is_fake = True
            else:
                self._log_waiting_status(now_s, ["gripper_position"])
                return

        camera_stamps = np.asarray([sample.stamp_s for sample in images.values()], dtype=np.float64)
        sample_time_s = float(camera_stamps.mean())
        if self._last_saved_sample_time_s is None:
            sample_dt_s = None
        else:
            sample_dt_s = sample_time_s - self._last_saved_sample_time_s
        camera_skew_s = float(camera_stamps.max() - camera_stamps.min())
        max_state_age_s = max(
            abs(now_s - joint_stamp_s),
            abs(now_s - gripper_stamp_s),
            *(abs(now_s - sample.stamp_s) for sample in images.values()),
        )
        if camera_skew_s > self.args.max_camera_skew_s:
            self.get_logger().warning(
                f"skip frame: camera skew {camera_skew_s:.3f}s > {self.args.max_camera_skew_s:.3f}s"
            )
            return
        if max_state_age_s > self.args.max_state_age_s:
            self.get_logger().warning(
                f"skip frame: state age {max_state_age_s:.3f}s > {self.args.max_state_age_s:.3f}s"
            )
            return

        frame_path = self.frames_dir / f"frame_{self._frame_index:06d}.npz"
        frame_data = self._build_frame_data(images, joint_position, gripper_position)
        if self.args.compressed:
            np.savez_compressed(frame_path, **frame_data)
        else:
            np.savez(frame_path, **frame_data)

        row = {
            "frame_index": self._frame_index,
            "file": str(frame_path.relative_to(self.episode_dir)),
            "task": self.args.task,
            "camera_names": self.camera_names,
            "camera_topics": dict(zip(self.camera_names, self.args.camera_topics, strict=True)),
            "camera_encodings": {name: images[name].encoding for name in self.camera_names},
            "camera_stamps_s": {name: images[name].stamp_s for name in self.camera_names},
            "sample_time_s": sample_time_s,
            "sample_dt_s": sample_dt_s,
            "target_fps": self.args.target_fps,
            "camera_skew_s": camera_skew_s,
            "joint_stamp_s": joint_stamp_s,
            "gripper_stamp_s": gripper_stamp_s,
            "gripper_is_fake": gripper_is_fake,
            "system_time_s": time.time(),
            "cosmos_keys": [
                "prompt",
                "observation/joint_position",
                "observation/gripper_position",
                "observation/image",
            ],
        }
        self._manifest.write(json.dumps(row, ensure_ascii=False) + "\n")
        self._manifest.flush()

        self._last_saved_sample_time_s = sample_time_s
        self._frame_index += 1
        self.get_logger().info(f"saved {frame_path} ({self._frame_index}/{self.args.max_frames})")
        if self.args.max_frames and self._frame_index >= self.args.max_frames:
            self.done = True

    def _build_frame_data(
        self,
        images: dict[str, ImageSample],
        joint_position: np.ndarray,
        gripper_position: np.ndarray,
    ) -> dict[str, Any]:
        wrist = images[self.args.wrist_camera].image
        exterior_1 = images[self.args.exterior_image_1_camera].image
        exterior_2 = images[self.args.exterior_image_2_camera].image
        concat_view = _compose_roboarena_concat_view(wrist, exterior_1, exterior_2)

        data: dict[str, Any] = {
            "prompt": np.asarray(self.args.task),
            "observation/joint_position": joint_position[None, :].astype(np.float32),
            "observation/gripper_position": gripper_position.reshape(1, 1).astype(np.float32),
            "observation/image": concat_view,
            "observation/wrist_image_left": wrist,
            "observation/exterior_image_1_left": exterior_1,
            "observation/exterior_image_2_left": exterior_2,
            "joint_names": np.asarray(self.args.joint_names),
            "camera_names": np.asarray(self.camera_names),
        }
        for name, sample in images.items():
            data[f"observation/images/{name}"] = sample.image
            data[f"timestamp/images/{name}"] = np.asarray(sample.stamp_s, dtype=np.float64)
        return data

    def _missing_locked(self) -> list[str]:
        missing = [name for name in self.camera_names if name not in self._images]
        if self._joint_position is None:
            missing.append("joint_position")
        if self._gripper_position is None and self.args.fake_gripper is None:
            missing.append("gripper_position")
        return missing

    def _log_waiting_status(self, now_s: float, missing: list[str]) -> None:
        if now_s - self._last_status_log_s < self.args.status_period_s:
            return
        self._last_status_log_s = now_s
        received_cameras = sorted(set(self.camera_names) - set(missing))
        self.get_logger().info(
            "waiting for inputs: "
            f"missing={missing}, received_cameras={received_cameras}, "
            f"frames_saved={self._frame_index}"
        )

    def _write_episode_metadata(self) -> None:
        metadata = {
            "type": "cosmos_droid_8camera_episode",
            "task": self.args.task,
            "output_format": "one npz per sampled frame",
            "camera_topics": dict(zip(self.camera_names, self.args.camera_topics, strict=True)),
            "cosmos_view_cameras": {
                "wrist_image_left": self.args.wrist_camera,
                "exterior_image_1_left": self.args.exterior_image_1_camera,
                "exterior_image_2_left": self.args.exterior_image_2_camera,
            },
            "joint_state_topic": self.args.joint_state_topic,
            "gripper_state_topic": self.args.gripper_state_topic,
            "joint_names": self.args.joint_names,
            "target_fps": self.args.target_fps,
            "max_camera_skew_s": self.args.max_camera_skew_s,
            "max_state_age_s": self.args.max_state_age_s,
            "fake_gripper_default": self.args.fake_gripper,
        }
        (self.episode_dir / "episode_metadata.json").write_text(
            json.dumps(metadata, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )


def _compose_roboarena_concat_view(
    wrist_image_left: np.ndarray,
    exterior_image_1_left: np.ndarray,
    exterior_image_2_left: np.ndarray,
) -> np.ndarray:
    wrist = _ensure_rgb_uint8_image(wrist_image_left)
    left_raw = _ensure_rgb_uint8_image(exterior_image_1_left)
    right_raw = _ensure_rgb_uint8_image(exterior_image_2_left)
    half_h, half_w = wrist.shape[0] // 2, wrist.shape[1] // 2
    left = _resize_rgb_uint8(left_raw, (half_h, half_w))
    right = _resize_rgb_uint8(right_raw, (half_h, half_w))
    bottom = np.concatenate([left, right], axis=1)
    return np.ascontiguousarray(np.concatenate([wrist, bottom], axis=0))


def _ensure_rgb_uint8_image(image: np.ndarray) -> np.ndarray:
    image = np.asarray(image)
    if image.ndim != 3 or image.shape[-1] != 3:
        raise ValueError(f"image must be [H,W,3], got {image.shape}")
    if image.dtype != np.uint8:
        image = image.astype(np.uint8)
    return np.ascontiguousarray(image)


def _resize_rgb_uint8(image: np.ndarray, size_hw: tuple[int, int]) -> np.ndarray:
    height, width = size_hw
    resized = PILImage.fromarray(image, mode="RGB").resize(
        (width, height),
        resample=PILImage.BILINEAR,
    )
    return np.asarray(resized, dtype=np.uint8)


def _stamp_to_seconds(stamp: Any, fallback_s: float) -> float:
    stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    if stamp_s <= 0.0:
        return fallback_s
    return stamp_s


def _make_episode_dir(output_dir: Path, episode_name: str) -> Path:
    if episode_name:
        episode_dir = output_dir / episode_name
    else:
        episode_dir = output_dir / time.strftime("episode_%Y%m%d_%H%M%S")
    episode_dir.mkdir(parents=True, exist_ok=False)
    return episode_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=Path("data"))
    parser.add_argument("--episode-name", default="")
    parser.add_argument("--camera-topics", nargs="+", default=DEFAULT_CAMERA_TOPICS)
    parser.add_argument("--wrist-camera", default="cam_0")
    parser.add_argument("--exterior-image-1-camera", default="cam_1")
    parser.add_argument("--exterior-image-2-camera", default="cam_2")
    parser.add_argument("--joint-state-topic", default="/franka/joint_states")
    parser.add_argument("--gripper-state-topic", default="/franka_gripper/joint_states")
    parser.add_argument("--joint-names", nargs="+", default=FR3_JOINTS)
    parser.add_argument("--fake-gripper", type=float, default=0.04)
    parser.add_argument("--task", default="hold position")
    parser.add_argument("--target-fps", "--rate-hz", dest="target_fps", type=float, default=15.0)
    parser.add_argument("--max-frames", type=int, default=10)
    parser.add_argument("--max-camera-skew-s", type=float, default=0.25)
    parser.add_argument("--max-state-age-s", type=float, default=1.0)
    parser.add_argument("--status-period-s", type=float, default=2.0)
    parser.add_argument("--compressed", action="store_true")
    args = parser.parse_args()

    if len(args.camera_topics) != 8:
        raise ValueError(f"expected exactly 8 camera topics, got {len(args.camera_topics)}")
    camera_names = {f"cam_{index}" for index in range(len(args.camera_topics))}
    for name in (args.wrist_camera, args.exterior_image_1_camera, args.exterior_image_2_camera):
        if name not in camera_names:
            raise ValueError(f"unknown Cosmos view camera {name!r}, valid cameras: {sorted(camera_names)}")
    if args.target_fps <= 0:
        raise ValueError("--target-fps must be positive")
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = CosmosDroid8CameraRecorder(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"episode saved: {node.episode_dir}")


if __name__ == "__main__":
    main()
