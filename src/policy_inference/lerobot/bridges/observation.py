"""RMI Observation -> LeRobot frame values (dual of ``bridges.action``)."""

from __future__ import annotations

from typing import Any

import numpy as np
from rmi import Observation, PolicyLayout

from ..geometry import gripper_joint_values, pack_libero_ee_state

__all__ = ["ObservationEncoder"]


class ObservationEncoder:
    """Project one timestamped RMI observation into LeRobot hardware values.

    Output keys follow the checkpoint schema: ``"<joint>.pos"`` scalars plus one
    bare camera name per Profile image feature. Camera payloads are passed
    through by reference, so RMI stays the single owner of image buffers.

    Cartesian layouts pack LIBERO-style EE state
    (``ee_pos + axis_angle + gripper``) from ``Observation.sensors[pose_part]``
    into the Profile state feature slots.

    When ``normalize_gripper`` is set (joint layouts), Piper meters
    ``[0, gripper_max_width]`` are mapped to Aloha/RoboTwin ``[0, 1]`` before
    the policy sees them.
    """

    def __init__(
        self,
        layout: PolicyLayout,
        *,
        max_stream_skew_s: float = 0.5,
        normalize_gripper: bool = False,
        gripper_max_width: float = 0.04,
    ) -> None:
        if max_stream_skew_s <= 0.0:
            raise ValueError("max_stream_skew_s must be positive")
        if gripper_max_width < 0.0:
            raise ValueError("gripper_max_width must be non-negative")
        self.layout = layout
        self.max_stream_skew_s = max_stream_skew_s
        self.normalize_gripper = bool(normalize_gripper)
        self.gripper_max_width = float(gripper_max_width)

    def encode(self, observation: Observation) -> dict[str, Any]:
        """Encode joints/TCP + cameras; gate on receive-time freshness."""
        receive_times = [float(observation.receive_time_s)]
        if self.layout.control_mode == "cartesian":
            values, pose_receive_s = self._encode_cartesian_state(observation)
            receive_times.append(pose_receive_s)
        else:
            values = self._encode_joint_state(observation)

        for feature_name, sensor_name in self.layout.camera_sources.items():
            try:
                sample = observation.sensors[sensor_name]
            except KeyError as exc:
                raise RuntimeError(
                    f"RMI observation is missing sensor {sensor_name!r} "
                    f"for {feature_name!r}"
                ) from exc
            value = sample.value
            expected_shape = self.layout.camera_shapes[feature_name]
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

    def _encode_joint_state(self, observation: Observation) -> dict[str, Any]:
        feature_names = self.layout.state_feature_names
        positions = dict(
            zip(
                observation.joint_names,
                observation.joint_positions,
                strict=False,
            )
        )
        missing_joints = tuple(
            name.removesuffix(".pos")
            for name in feature_names
            if name.removesuffix(".pos") not in positions
        )
        if missing_joints:
            raise RuntimeError(f"RMI observation is missing joints {missing_joints}")
        encoded: dict[str, Any] = {}
        for name in feature_names:
            joint = name.removesuffix(".pos")
            value = float(positions[joint])
            if self.normalize_gripper and "gripper" in joint.lower():
                if self.gripper_max_width > 0.0:
                    value = float(
                        np.clip(value / self.gripper_max_width, 0.0, 1.0)
                    )
                else:
                    value = 0.0
            encoded[name] = value
        return encoded

    def _encode_cartesian_state(
        self,
        observation: Observation,
    ) -> tuple[dict[str, Any], float]:
        pose_part = self.layout.pose_part
        if not pose_part:
            raise RuntimeError("cartesian layout is missing pose_part")
        try:
            sample = observation.sensors[pose_part]
        except KeyError as exc:
            raise RuntimeError(
                f"RMI observation is missing TCP pose sensor {pose_part!r}"
            ) from exc
        pose = sample.value
        grip_names = _gripper_joint_names(self.layout)
        grips = gripper_joint_values(observation, grip_names)
        state = pack_libero_ee_state(
            pose.position_xyz,
            pose.orientation_wxyz,
            grips,
        )
        feature_names = self.layout.state_feature_names
        if state.shape != (len(feature_names),):
            raise ValueError(
                f"packed EE state shape {state.shape} != state feature count "
                f"({len(feature_names)},)"
            )
        return (
            dict(zip(feature_names, map(float, state), strict=True)),
            float(sample.receive_time_s),
        )


def _gripper_joint_names(layout: PolicyLayout) -> tuple[str, ...]:
    names: list[str] = []
    for group in layout.joints.groups:
        if group.part in layout.gripper_parts:
            names.extend(group.joint_names)
    if not names:
        raise RuntimeError(
            "cartesian layout has no gripper joints for EE proprio packing"
        )
    return tuple(names)
