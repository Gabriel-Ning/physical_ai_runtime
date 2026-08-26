"""The two explicit data bridges between RMI and native LeRobot inference."""

from __future__ import annotations

from typing import Any

import numpy as np
from rmi import Action, Observation

from ..common.contract import PolicyIOContract


class RmiToLeRobotObservationBridge:
    """Project one timestamped RMI observation into LeRobot hardware values."""

    def __init__(
        self,
        contract: PolicyIOContract,
        *,
        max_stream_skew_s: float = 0.5,
    ) -> None:
        if max_stream_skew_s <= 0.0:
            raise ValueError("max_stream_skew_s must be positive")
        self.contract = contract
        self.max_stream_skew_s = max_stream_skew_s

    def encode(self, observation: Observation) -> dict[str, Any]:
        positions = dict(
            zip(
                observation.joint_names,
                observation.joint_positions,
                strict=False,
            )
        )
        missing_joints = tuple(
            name.removesuffix(".pos")
            for name in self.contract.state_feature_names
            if name.removesuffix(".pos") not in positions
        )
        if missing_joints:
            raise RuntimeError(f"RMI observation is missing joints {missing_joints}")

        values: dict[str, Any] = {
            name: float(positions[name.removesuffix(".pos")])
            for name in self.contract.state_feature_names
        }
        receive_times = [observation.receive_time_s]
        for feature_name, sensor_name in self.contract.camera_sources.items():
            try:
                sample = observation.sensors[sensor_name]
            except KeyError as exc:
                raise RuntimeError(
                    f"RMI observation is missing sensor {sensor_name!r} "
                    f"for {feature_name!r}"
                ) from exc
            value = sample.value
            expected_shape = self.contract.camera_shapes[feature_name]
            if tuple(value.shape) != expected_shape:
                raise ValueError(
                    f"camera {feature_name!r} shape {tuple(value.shape)} "
                    f"does not match Profile {expected_shape}"
                )
            values[feature_name.removeprefix("observation.images.")] = value
            receive_times.append(float(sample.receive_time_s))

        if max(receive_times) - min(receive_times) > self.max_stream_skew_s:
            raise RuntimeError(
                "RMI observation streams exceed the configured freshness window"
            )
        return values


class LeRobotToRmiActionBridge:
    """Validate and split one postprocessed LeRobot action into native RMI actions."""

    def __init__(self, contract: PolicyIOContract) -> None:
        self.contract = contract

    def decode(self, action: Any) -> tuple[Action, ...]:
        if hasattr(action, "detach"):
            action = action.detach()
        if hasattr(action, "cpu"):
            action = action.cpu()
        values = np.asarray(action, dtype=np.float64)
        if values.shape != (self.contract.action_dim,):
            raise ValueError(
                f"LeRobot action shape {values.shape} does not match "
                f"Profile ({self.contract.action_dim},)"
            )
        if not np.isfinite(values).all():
            raise ValueError("LeRobot action contains NaN or Inf")
        return tuple(
            Action(part=group.part, command=group.command, value=group_values)
            for group, group_values in self.contract.split_action(values)
        )


def ros_image_to_numpy(message: Any) -> np.ndarray:
    """Decode a raw ROS Image into contiguous HWC RGB for an RMI Camera sample."""
    encoding = str(getattr(message, "encoding", "")).lower()
    channels = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1,
        "8uc1": 1,
        "8uc3": 3,
        "8sc3": 3,
    }.get(encoding)
    if channels is None:
        raise ValueError(f"unsupported image encoding {encoding!r}")
    height, width = int(message.height), int(message.width)
    required = height * int(message.step)
    source = np.frombuffer(message.data, dtype=np.uint8)
    if source.size < required:
        raise ValueError(f"image data is truncated: {source.size} < {required}")
    image = source[:required].reshape(height, int(message.step))
    image = image[:, : width * channels].reshape(height, width, channels)
    if channels == 1:
        return np.repeat(image, 3, axis=2).copy()
    if encoding in {"bgr8", "bgra8"}:
        return image[..., :3][..., ::-1].copy()
    return np.ascontiguousarray(image[..., :3])
