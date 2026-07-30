"""ROS observation cache for the Cosmos-DROID policy example."""

from __future__ import annotations

from dataclasses import dataclass
from threading import Lock
from typing import Any

import numpy as np
import cv2
from sensor_msgs.msg import Image, JointState


@dataclass(frozen=True)
class CosmosDroidObservation:
    """Snapshot of the latest state required by Cosmos-DROID."""

    joint_position: np.ndarray
    gripper_position: np.ndarray
    wrist_image: np.ndarray
    exterior_image_1: np.ndarray
    exterior_image_2: np.ndarray

    def as_policy_dict(self, task: str) -> dict[str, Any]:
        return {
            "task": task,
            "observation.state": self.joint_position.copy(),
            "observation.gripper": self.gripper_position.copy(),
            "observation.images.wrist": self.wrist_image.copy(),
            "observation.images.exterior_1": self.exterior_image_1.copy(),
            "observation.images.exterior_2": self.exterior_image_2.copy(),
        }


class CosmosDroidObservationCache:
    """Caches latest joint, gripper, and three-camera RGB observations."""

    def __init__(self, joint_names: list[str]) -> None:
        if not joint_names:
            raise ValueError("joint_names must not be empty")

        self.joint_names = list(joint_names)
        self._lock = Lock()
        self._joint_position: np.ndarray | None = None
        self._gripper_position: np.ndarray | None = None
        self._wrist_image: np.ndarray | None = None
        self._exterior_image_1: np.ndarray | None = None
        self._exterior_image_2: np.ndarray | None = None

    def on_joint_state(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position, strict=False))
        if not all(name in positions for name in self.joint_names):
            return

        joint_position = np.asarray(
            [positions[name] for name in self.joint_names],
            dtype=np.float32,
        )
        with self._lock:
            self._joint_position = joint_position

    def on_gripper_position(self, msg: Any) -> None:
        gripper_position = np.asarray([_extract_gripper_scalar(msg)], dtype=np.float32)
        with self._lock:
            self._gripper_position = gripper_position

    def on_wrist_image(self, msg: Image) -> None:
        self._set_image("wrist", _image_msg_to_rgb8(msg))

    def on_exterior_image_1(self, msg: Image) -> None:
        self._set_image("exterior_1", _image_msg_to_rgb8(msg))

    def on_exterior_image_2(self, msg: Image) -> None:
        self._set_image("exterior_2", _image_msg_to_rgb8(msg))

    def ready(self) -> bool:
        return not self.missing()

    def missing(self) -> list[str]:
        with self._lock:
            missing = []
            if self._joint_position is None:
                missing.append("joint_position")
            if self._gripper_position is None:
                missing.append("gripper_position")
            if self._wrist_image is None:
                missing.append("wrist_image")
            if self._exterior_image_1 is None:
                missing.append("exterior_image_1")
            if self._exterior_image_2 is None:
                missing.append("exterior_image_2")
            return missing

    def snapshot(self) -> CosmosDroidObservation | None:
        with self._lock:
            if (
                self._joint_position is None
                or self._gripper_position is None
                or self._wrist_image is None
                or self._exterior_image_1 is None
                or self._exterior_image_2 is None
            ):
                return None

            return CosmosDroidObservation(
                joint_position=self._joint_position.copy(),
                gripper_position=self._gripper_position.copy(),
                wrist_image=self._wrist_image.copy(),
                exterior_image_1=self._exterior_image_1.copy(),
                exterior_image_2=self._exterior_image_2.copy(),
            )

    def get_observation(self, task: str) -> dict[str, Any] | None:
        snapshot = self.snapshot()
        if snapshot is None:
            return None
        return snapshot.as_policy_dict(task)

    def _set_image(self, name: str, image: np.ndarray) -> None:
        with self._lock:
            if name == "wrist":
                self._wrist_image = image
            elif name == "exterior_1":
                self._exterior_image_1 = image
            elif name == "exterior_2":
                self._exterior_image_2 = image
            else:
                raise ValueError(f"unknown image cache name: {name}")


_BAYER_TO_RGB_CONVERSION = {
    "bayer_rggb8": cv2.COLOR_BayerRGGB2RGB,  # ros2 topic echo /cameras/cam_0/image_raw --once --field encoding可得
    "bayer_bggr8": cv2.COLOR_BayerBGGR2RGB,
    "bayer_gbrg8": cv2.COLOR_BayerGBRG2RGB,
    "bayer_grbg8": cv2.COLOR_BayerGRBG2RGB,
}


def _image_msg_to_rgb8(msg: Image) -> np.ndarray:
    encoding = msg.encoding.lower()
    if encoding in _BAYER_TO_RGB_CONVERSION:
        return _bayer_msg_to_rgb8(msg, encoding)

    if encoding not in {"rgb8", "bgr8", "rgba8", "bgra8", "mono8"}:
        raise ValueError(f"unsupported image encoding: {msg.encoding}")

    channels = 1 if encoding == "mono8" else 4 if encoding in {"rgba8", "bgra8"} else 3
    expected_step = msg.width * channels
    if msg.step < expected_step:
        raise ValueError(
            f"image step {msg.step} is smaller than width * channels {expected_step}"
        )

    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    image = rows[:, :expected_step].reshape(msg.height, msg.width, channels)

    if encoding == "rgb8":
        rgb = image
    elif encoding == "bgr8":
        rgb = image[..., ::-1]
    elif encoding == "rgba8":
        rgb = image[..., :3]
    elif encoding == "bgra8":
        rgb = image[..., 2::-1]
    else:
        rgb = np.repeat(image, 3, axis=2)

    return np.ascontiguousarray(rgb)


def _bayer_msg_to_rgb8(msg: Image, encoding: str) -> np.ndarray:
    if msg.step < msg.width:
        raise ValueError(f"image step {msg.step} is smaller than width {msg.width}")

    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    bayer = np.ascontiguousarray(rows[:, : msg.width])
    rgb = cv2.cvtColor(bayer, _BAYER_TO_RGB_CONVERSION[encoding])
    return np.ascontiguousarray(rgb)


def _extract_gripper_scalar(msg: Any) -> float:
    if isinstance(msg, JointState):
        if msg.position:
            return float(msg.position[0])
        raise ValueError("gripper JointState has no position")

    if hasattr(msg, "data"):
        return float(msg.data)
    if hasattr(msg, "position"):
        position = msg.position
        if isinstance(position, (list, tuple)):
            if not position:
                raise ValueError("gripper position sequence is empty")
            return float(position[0])
        return float(position)

    return float(msg)
