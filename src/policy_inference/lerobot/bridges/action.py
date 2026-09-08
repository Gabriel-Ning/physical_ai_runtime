"""LeRobot action -> native RMI actions (dual of ``bridges.observation``).

One base contract, one decoder per Profile ``control_mode``::

    ActionDecoder                  # shape / finiteness checks, pose hook
    ├── JointActionDecoder         # control_mode="joint"     -> joint_reference
    └── CartesianActionDecoder     # control_mode="cartesian" -> pose_reference
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

import numpy as np
from rmi import Action, PolicyLayout

from ..geometry import (
    axis_angle_to_quat_wxyz,
    normalize_quat_wxyz,
    quat_multiply_wxyz,
)

__all__ = [
    "ActionDecoder",
    "CartesianActionDecoder",
    "JointActionDecoder",
]


class ActionDecoder(ABC):
    """Turn one postprocessed LeRobot action vector into native RMI actions."""

    def __init__(self, layout: PolicyLayout) -> None:
        self.layout = layout

    @abstractmethod
    def decode(self, action: Any) -> tuple[Action, ...]:
        """Return one RMI Action per Profile resource owned by the policy node."""

    def set_current_pose(self, position_xyz: Any, orientation_wxyz: Any) -> None:
        """Anchor hook for decoders that integrate relative motion; else no-op."""

    def _as_vector(self, action: Any) -> np.ndarray:
        if hasattr(action, "detach"):
            action = action.detach()
        if hasattr(action, "cpu"):
            action = action.cpu()
        values = np.asarray(action, dtype=np.float64).reshape(-1)
        if not np.isfinite(values).all():
            raise ValueError("LeRobot action contains NaN or Inf")
        if values.shape != (self.layout.action_dimension,):
            raise ValueError(
                f"LeRobot action shape {values.shape} does not match "
                f"Profile ({self.layout.action_dimension},)"
            )
        return values


class JointActionDecoder(ActionDecoder):
    """Split the action vector along the Profile joint layout.

    When ``normalize_gripper`` is set, Aloha/RoboTwin-style gripper commands in
    ``[0, 1]`` (0=closed, 1=open) are mapped to meters
    ``[0, gripper_max_width]`` before submit.
    """

    def __init__(
        self,
        layout: PolicyLayout,
        *,
        normalize_gripper: bool = False,
        gripper_max_width: float = 0.04,
    ) -> None:
        if gripper_max_width < 0.0:
            raise ValueError("gripper_max_width must be non-negative")
        super().__init__(layout)
        self.normalize_gripper = bool(normalize_gripper)
        self.gripper_max_width = float(gripper_max_width)

    def decode(self, action: Any) -> tuple[Action, ...]:
        values = self._as_vector(action)
        actions: list[Action] = []
        for group, group_values in self.layout.joints.split_values(values):
            mapped = list(group_values)
            if self.normalize_gripper and _is_gripper_part(group.part):
                mapped = [
                    float(
                        np.clip(
                            float(value) * self.gripper_max_width,
                            0.0,
                            self.gripper_max_width,
                        )
                    )
                    for value in mapped
                ]
            actions.append(
                Action(part=group.part, command=group.command, value=mapped)
            )
        return tuple(actions)


def _is_gripper_part(part: str) -> bool:
    return "gripper" in part.lower()


class CartesianActionDecoder(ActionDecoder):
    """Map 6D EE (+ gripper) actions to abs pose_reference + gripper targets.

    Orientation uses axis-angle. Relative actions are integrated against the
    latest TCP pose supplied via ``set_current_pose``. Gripper values in
    ``[-1, 1]`` map to ``[max_width, 0]`` (LIBERO-style open/close).

    LIBERO checkpoints store EE deltas in a near-[-1, 1] controller space after
    MIN_MAX unnormalization — not meters. Set ``position_scale`` (typically
    ``0.05``) so ``Δp_m = scale * a_xyz`` before integrating.
    """

    def __init__(
        self,
        layout: PolicyLayout,
        *,
        action_space: str | None = None,
        gripper_max_width: float = 0.045,
        position_scale: float = 0.05,
        orientation_scale: float = 0.5,
    ) -> None:
        if layout.control_mode != "cartesian":
            raise ValueError("CartesianActionDecoder requires cartesian PolicyLayout")
        space = (action_space or layout.action_space).lower()
        if space not in {"abs", "rel"}:
            raise ValueError(f"action_space must be abs or rel, got {space!r}")
        if gripper_max_width < 0.0:
            raise ValueError("gripper_max_width must be non-negative")
        if position_scale < 0.0:
            raise ValueError("position_scale must be non-negative")
        if orientation_scale < 0.0:
            raise ValueError("orientation_scale must be non-negative")
        super().__init__(layout)
        self.action_space = space
        self.gripper_max_width = float(gripper_max_width)
        self.position_scale = float(position_scale)
        self.orientation_scale = float(orientation_scale)
        self._position: np.ndarray | None = None
        self._orientation_wxyz: np.ndarray | None = None

    def set_current_pose(self, position_xyz: Any, orientation_wxyz: Any) -> None:
        position = np.asarray(position_xyz, dtype=np.float64).reshape(3)
        orientation = normalize_quat_wxyz(orientation_wxyz)
        if not np.isfinite(position).all():
            raise ValueError("current EE position contains NaN or Inf")
        self._position = position
        self._orientation_wxyz = orientation

    def decode(self, action: Any) -> tuple[Action, ...]:
        values = self._as_vector(action)
        pose_delta = values[:6].copy()
        pose_delta[:3] *= self.position_scale
        pose_delta[3:6] *= self.orientation_scale
        position, orientation = self._integrate(pose_delta)
        self._position = position
        self._orientation_wxyz = orientation
        return (
            Action(
                part=str(self.layout.pose_part),
                command="pose_reference",
                value={
                    "position": position.tolist(),
                    "orientation": orientation.tolist(),
                },
            ),
            *self._gripper_actions(values[6:]),
        )

    def _integrate(self, pose_delta: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if self.action_space == "abs":
            return pose_delta[:3].copy(), axis_angle_to_quat_wxyz(pose_delta[3:6])
        if self._position is None or self._orientation_wxyz is None:
            raise RuntimeError(
                "relative cartesian actions require set_current_pose() first"
            )
        return (
            self._position + pose_delta[:3],
            normalize_quat_wxyz(
                quat_multiply_wxyz(
                    self._orientation_wxyz,
                    axis_angle_to_quat_wxyz(pose_delta[3:6]),
                )
            ),
        )

    def _gripper_actions(self, gripper_values: np.ndarray) -> tuple[Action, ...]:
        actions: list[Action] = []
        offset = 0
        for group in self.layout.joints.groups:
            if group.part not in self.layout.gripper_parts:
                continue
            width = len(group.joint_names)
            chunk = gripper_values[offset : offset + width]
            offset += width
            mapped = [
                float(
                    np.clip(
                        self.gripper_max_width * (1.0 - float(value)) * 0.5,
                        0.0,
                        self.gripper_max_width,
                    )
                )
                for value in chunk
            ]
            actions.append(Action(part=group.part, command=group.command, value=mapped))
        return tuple(actions)
