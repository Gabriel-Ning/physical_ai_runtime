"""Geometry and state packing helpers for cartesian / LIBERO paths.

SO(3) conversions are shared by action bridges and proprio packing.
``pack_libero_ee_state`` / ``gripper_joint_values`` are used by
``ObservationEncoder`` for cartesian layouts.
"""

from __future__ import annotations

from typing import Any, Sequence

import numpy as np

__all__ = [
    "axis_angle_to_quat_wxyz",
    "gripper_joint_values",
    "normalize_quat_wxyz",
    "pack_libero_ee_state",
    "quat_multiply_wxyz",
    "quat_wxyz_to_axis_angle",
]


# --- SO(3) -----------------------------------------------------------------


def normalize_quat_wxyz(quat: np.ndarray | Sequence[float]) -> np.ndarray:
    values = np.asarray(quat, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(values))
    if norm < 1e-10:
        raise ValueError("orientation quaternion has near-zero norm")
    return values / norm


def axis_angle_to_quat_wxyz(axis_angle: np.ndarray | Sequence[float]) -> np.ndarray:
    vector = np.asarray(axis_angle, dtype=np.float64).reshape(3)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    axis = vector / angle
    half = 0.5 * angle
    s = np.sin(half)
    return np.array(
        [np.cos(half), axis[0] * s, axis[1] * s, axis[2] * s], dtype=np.float64
    )


def quat_wxyz_to_axis_angle(orientation_wxyz: Sequence[float]) -> np.ndarray:
    """Convert quaternion (w, x, y, z) to axis-angle rotation vector."""
    quat = normalize_quat_wxyz(orientation_wxyz)
    w, x, y, z = quat
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    w = float(np.clip(w, -1.0, 1.0))
    angle = 2.0 * float(np.arccos(w))
    s = float(np.sqrt(max(0.0, 1.0 - w * w)))
    if s < 1e-8:
        return np.zeros(3, dtype=np.float64)
    return np.array([x, y, z], dtype=np.float64) * (angle / s)


def quat_multiply_wxyz(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


# --- LIBERO / OpenVLA observation.state ------------------------------------


def pack_libero_ee_state(
    position_xyz: Sequence[float],
    orientation_wxyz: Sequence[float],
    gripper_joints: Sequence[float],
) -> np.ndarray:
    """Pack TCP pose + gripper into LIBERO observation.state (shape (8,)).

    Layout: ee_pos(3) + ee_axis_angle(3) + gripper_qpos(2).
    Single-DoF grippers are mirrored as ``[g, -g]``.
    """
    position = np.asarray(position_xyz, dtype=np.float64).reshape(3)
    axis_angle = quat_wxyz_to_axis_angle(orientation_wxyz)
    grips = [float(value) for value in gripper_joints]
    if not grips:
        raise ValueError("gripper_joints must not be empty")
    if len(grips) == 1:
        gripper = np.array([grips[0], -grips[0]], dtype=np.float64)
    else:
        gripper = np.asarray(grips[:2], dtype=np.float64)
    state = np.concatenate([position, axis_angle, gripper])
    if state.shape != (8,):
        raise ValueError(f"expected packed state shape (8,), got {state.shape}")
    if not np.isfinite(state).all():
        raise ValueError("packed EE state contains NaN or Inf")
    return state


def gripper_joint_values(
    observation: Any,
    gripper_joint_names: Sequence[str],
) -> list[float]:
    """Read gripper joints from an RMI observation by name."""
    positions = dict(
        zip(observation.joint_names, observation.joint_positions, strict=False)
    )
    missing = [name for name in gripper_joint_names if name not in positions]
    if missing:
        raise RuntimeError(f"observation is missing gripper joints: {missing}")
    return [float(positions[name]) for name in gripper_joint_names]
