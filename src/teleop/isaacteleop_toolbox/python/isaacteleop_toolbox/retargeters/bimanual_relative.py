"""Relative bimanual target retargeting with world-frame rotation deltas."""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np
from isaacteleop.retargeting_engine.interface import (
    BaseRetargeter,
    OptionalTensorGroup,
    RetargeterIOType,
)
from isaacteleop.retargeting_engine.interface.retargeter_core_types import RetargeterIO
from isaacteleop.retargeting_engine.interface.tensor_group_type import (
    OptionalType,
    TensorGroupType,
)
from isaacteleop.retargeting_engine.tensor_types import (
    ControllerInput,
    ControllerInputIndex,
    DLDataType,
    NDArrayType,
)
from scipy.spatial.transform import Rotation, Slerp

from isaacteleop_toolbox.filters import OneEuroFilterSE3


@dataclass
class ControllerPose:
    position: np.ndarray
    rotation: Rotation


@dataclass
class SideState:
    source_initial: ControllerPose | None = None
    target_initial: ControllerPose | None = None
    previous_target: ControllerPose | None = None
    filter: OneEuroFilterSE3 | None = None
    active: bool = False


@dataclass(frozen=True)
class BimanualSnapshot:
    """Clutch snapshot latched on the most recent deadman-press transition."""

    seq: int
    left_controller: ControllerPose | None
    left_ee: ControllerPose | None
    right_controller: ControllerPose | None
    right_ee: ControllerPose | None


@dataclass(frozen=True)
class BimanualRelativeConfig:
    pose_source: str
    deadman_source: str
    deadman_threshold: float
    require_both_deadman: bool
    linear_scale: float
    angular_scale: float
    lowpass_alpha: float
    max_linear_step_m: float
    max_angular_step_rad: float
    openxr_to_base_rotation_xyzw: list[float]
    filter_type: str = "one_euro"  # "one_euro", "lowpass", "none"
    one_euro_min_cutoff: float = 1.0
    one_euro_beta: float = 0.007
    one_euro_d_cutoff: float = 1.0
    # Gripper parameters
    gripper_min_width: float = 0.0
    gripper_max_width: float = 0.04
    gripper_speed_mps: float = 0.025
    require_clutch_for_gripper: bool = False


def _slerp(start: Rotation, end: Rotation, alpha: float) -> Rotation:
    return Slerp([0.0, 1.0], Rotation.concatenate([start, end]))([alpha])[0]


def _clamp_rotation_step(
    previous: Rotation, target: Rotation, max_step_rad: float
) -> Rotation:
    if max_step_rad <= 0.0:
        return target
    delta = target * previous.inv()
    rotvec = delta.as_rotvec()
    angle = float(np.linalg.norm(rotvec))
    if angle <= max_step_rad or angle == 0.0:
        return target
    limited_delta = Rotation.from_rotvec(rotvec * (max_step_rad / angle))
    return limited_delta * previous


def _apply_basis_change(pose: ControllerPose, basis_change: Rotation) -> ControllerPose:
    return ControllerPose(
        position=basis_change.apply(pose.position),
        rotation=basis_change * pose.rotation,
    )


def _controller_pose(ctrl: OptionalTensorGroup, pose_source: str) -> ControllerPose:
    if pose_source == "grip":
        position_index = ControllerInputIndex.GRIP_POSITION
        orientation_index = ControllerInputIndex.GRIP_ORIENTATION
    elif pose_source == "aim":
        position_index = ControllerInputIndex.AIM_POSITION
        orientation_index = ControllerInputIndex.AIM_ORIENTATION
    else:
        raise ValueError("pose_source must be 'grip' or 'aim'")

    return ControllerPose(
        position=np.from_dlpack(ctrl[position_index]),
        rotation=Rotation.from_quat(np.from_dlpack(ctrl[orientation_index])),
    )


def _scalar_value(value) -> float:
    if hasattr(value, "__dlpack__"):
        return float(np.from_dlpack(value))
    return float(value)


def _pose_valid(ctrl: OptionalTensorGroup, pose_source: str) -> bool:
    if ctrl.is_none:
        return False
    if pose_source == "grip":
        return bool(_scalar_value(ctrl[ControllerInputIndex.GRIP_IS_VALID]))
    if pose_source == "aim":
        return bool(_scalar_value(ctrl[ControllerInputIndex.AIM_IS_VALID]))
    raise ValueError("pose_source must be 'grip' or 'aim'")


def _deadman_pressed(ctrl: OptionalTensorGroup, source: str, threshold: float) -> bool:
    if ctrl.is_none:
        return False
    if source == "none":
        return True
    if source == "squeeze":
        return _scalar_value(ctrl[ControllerInputIndex.SQUEEZE_VALUE]) >= threshold
    if source == "trigger":
        return _scalar_value(ctrl[ControllerInputIndex.TRIGGER_VALUE]) >= threshold
    if source == "primary":
        return _scalar_value(ctrl[ControllerInputIndex.PRIMARY_CLICK]) >= threshold
    if source == "secondary":
        return _scalar_value(ctrl[ControllerInputIndex.SECONDARY_CLICK]) >= threshold
    if source == "thumbstick":
        return _scalar_value(ctrl[ControllerInputIndex.THUMBSTICK_CLICK]) >= threshold
    raise ValueError(
        "deadman_source must be one of: none, squeeze, trigger, primary, secondary, thumbstick"
    )


def _button_pressed(
    ctrl: OptionalTensorGroup, index: ControllerInputIndex, threshold: float = 0.5
) -> bool:
    if ctrl.is_none:
        return False
    return _scalar_value(ctrl[index]) >= threshold


class BimanualRelativeRetargeter(BaseRetargeter):
    """Pipeline-integrated stateful clutch-relative mapping with One Euro filter."""

    def __init__(
        self,
        config: BimanualRelativeConfig,
        on_activate_fn=None,
        name: str = "bimanual_relative",
    ) -> None:
        self.config = config
        self.on_activate_fn = on_activate_fn
        self._left = SideState(
            filter=OneEuroFilterSE3(
                min_cutoff_pos=config.one_euro_min_cutoff,
                beta_pos=config.one_euro_beta,
                min_cutoff_rot=config.one_euro_min_cutoff,
                beta_rot=config.one_euro_beta,
                d_cutoff=config.one_euro_d_cutoff,
            )
        )
        self._right = SideState(
            filter=OneEuroFilterSE3(
                min_cutoff_pos=config.one_euro_min_cutoff,
                beta_pos=config.one_euro_beta,
                min_cutoff_rot=config.one_euro_min_cutoff,
                beta_rot=config.one_euro_beta,
                d_cutoff=config.one_euro_d_cutoff,
            )
        )
        self._active = False
        self.snapshot_seq = 0
        self._openxr_to_base = Rotation.from_quat(
            np.asarray(config.openxr_to_base_rotation_xyzw, dtype=float)
        )
        self._left_gripper_pos = float(config.gripper_max_width)
        self._right_gripper_pos = float(config.gripper_max_width)
        self._last_time: float | None = None
        self.last_reject_reason: str = ""
        super().__init__(name=name)

    @property
    def snapshot(self) -> BimanualSnapshot:
        return BimanualSnapshot(
            seq=self.snapshot_seq,
            left_controller=self._left.source_initial,
            left_ee=self._left.target_initial,
            right_controller=self._right.source_initial,
            right_ee=self._right.target_initial,
        )

    def input_spec(self) -> RetargeterIOType:
        return {
            "controller_left": OptionalType(ControllerInput()),
            "controller_right": OptionalType(ControllerInput()),
        }

    def output_spec(self) -> RetargeterIOType:
        return {
            "left_ee_pose": TensorGroupType(
                "left_ee_pose",
                [
                    NDArrayType(
                        "pose", shape=(7,), dtype=DLDataType.FLOAT, dtype_bits=32
                    )
                ],
            ),
            "right_ee_pose": TensorGroupType(
                "right_ee_pose",
                [
                    NDArrayType(
                        "pose", shape=(7,), dtype=DLDataType.FLOAT, dtype_bits=32
                    )
                ],
            ),
            "left_gripper": TensorGroupType(
                "left_gripper",
                [
                    NDArrayType(
                        "command", shape=(1,), dtype=DLDataType.FLOAT, dtype_bits=32
                    )
                ],
            ),
            "right_gripper": TensorGroupType(
                "right_gripper",
                [
                    NDArrayType(
                        "command", shape=(1,), dtype=DLDataType.FLOAT, dtype_bits=32
                    )
                ],
            ),
            "active": TensorGroupType(
                "active",
                [NDArrayType("flag", shape=(1,), dtype=DLDataType.INT, dtype_bits=32)],
            ),
            "left_active": TensorGroupType(
                "left_active",
                [NDArrayType("flag", shape=(1,), dtype=DLDataType.INT, dtype_bits=32)],
            ),
            "right_active": TensorGroupType(
                "right_active",
                [NDArrayType("flag", shape=(1,), dtype=DLDataType.INT, dtype_bits=32)],
            ),
        }

    def _get_align_poses(
        self,
    ) -> tuple[ControllerPose | None, ControllerPose | None, bool, str]:
        if self.on_activate_fn is None:
            return None, None, False, "no_tcp_feedback"
        align_res = self.on_activate_fn()
        if isinstance(align_res, tuple) and len(align_res) == 4:
            return align_res
        if isinstance(align_res, tuple) and len(align_res) == 2:
            return align_res[0], align_res[1], True, ""
        return None, None, False, "invalid_align_fn_return"

    def _compute_fn(self, inputs: RetargeterIO, outputs: RetargeterIO, context) -> None:
        if context.execution_events.reset:
            self._active = False
            self._left = SideState(
                filter=OneEuroFilterSE3(
                    min_cutoff_pos=self.config.one_euro_min_cutoff,
                    beta_pos=self.config.one_euro_beta,
                    min_cutoff_rot=self.config.one_euro_min_cutoff,
                    beta_rot=self.config.one_euro_beta,
                    d_cutoff=self.config.one_euro_d_cutoff,
                )
            )
            self._right = SideState(
                filter=OneEuroFilterSE3(
                    min_cutoff_pos=self.config.one_euro_min_cutoff,
                    beta_pos=self.config.one_euro_beta,
                    min_cutoff_rot=self.config.one_euro_min_cutoff,
                    beta_rot=self.config.one_euro_beta,
                    d_cutoff=self.config.one_euro_d_cutoff,
                )
            )
            self._left_gripper_pos = float(self.config.gripper_max_width)
            self._right_gripper_pos = float(self.config.gripper_max_width)
            self._last_time = None

        left_ctrl = inputs["controller_left"]
        right_ctrl = inputs["controller_right"]
        left_ee_pose = outputs["left_ee_pose"]
        right_ee_pose = outputs["right_ee_pose"]
        left_gripper_out = outputs["left_gripper"]
        right_gripper_out = outputs["right_gripper"]
        active_out = outputs["active"]
        left_active_out = outputs.get("left_active")
        right_active_out = outputs.get("right_active")

        now = time.monotonic()
        if self._last_time is None:
            dt = 1.0 / 60.0
        else:
            dt = max(0.0, min(now - self._last_time, 0.2))
        self._last_time = now

        left_valid = _pose_valid(left_ctrl, self.config.pose_source)
        right_valid = _pose_valid(right_ctrl, self.config.pose_source)
        left_deadman = _deadman_pressed(
            left_ctrl, self.config.deadman_source, self.config.deadman_threshold
        )
        right_deadman = _deadman_pressed(
            right_ctrl, self.config.deadman_source, self.config.deadman_threshold
        )

        # Gripper buttons
        left_close = _button_pressed(left_ctrl, ControllerInputIndex.PRIMARY_CLICK)
        left_open = _button_pressed(left_ctrl, ControllerInputIndex.SECONDARY_CLICK)
        right_close = _button_pressed(right_ctrl, ControllerInputIndex.PRIMARY_CLICK)
        right_open = _button_pressed(right_ctrl, ControllerInputIndex.SECONDARY_CLICK)

        left_allowed = (not self.config.require_clutch_for_gripper) or left_deadman
        right_allowed = (not self.config.require_clutch_for_gripper) or right_deadman

        if left_allowed:
            if left_close and not left_open:
                self._left_gripper_pos = max(
                    self.config.gripper_min_width,
                    self._left_gripper_pos - self.config.gripper_speed_mps * dt,
                )
            elif left_open and not left_close:
                self._left_gripper_pos = min(
                    self.config.gripper_max_width,
                    self._left_gripper_pos + self.config.gripper_speed_mps * dt,
                )

        if right_allowed:
            if right_close and not right_open:
                self._right_gripper_pos = max(
                    self.config.gripper_min_width,
                    self._right_gripper_pos - self.config.gripper_speed_mps * dt,
                )
            elif right_open and not right_close:
                self._right_gripper_pos = min(
                    self.config.gripper_max_width,
                    self._right_gripper_pos + self.config.gripper_speed_mps * dt,
                )

        left_gripper_out[0] = np.array([self._left_gripper_pos], dtype=np.float32)
        right_gripper_out[0] = np.array([self._right_gripper_pos], dtype=np.float32)

        # Independent Arm Activation Logic
        if self.config.require_both_deadman:
            both_want_active = bool(
                left_valid and right_valid and left_deadman and right_deadman
            )
            left_wants_active = both_want_active
            right_wants_active = both_want_active
        else:
            left_wants_active = bool(left_valid and left_deadman)
            right_wants_active = bool(right_valid and right_deadman)

        left_source = (
            _apply_basis_change(
                _controller_pose(left_ctrl, self.config.pose_source),
                self._openxr_to_base,
            )
            if left_valid
            else None
        )
        right_source = (
            _apply_basis_change(
                _controller_pose(right_ctrl, self.config.pose_source),
                self._openxr_to_base,
            )
            if right_valid
            else None
        )

        needs_left_latch = (
            left_wants_active
            and not self._left.active
            and left_source is not None
        )
        needs_right_latch = (
            right_wants_active
            and not self._right.active
            and right_source is not None
        )

        if needs_left_latch or needs_right_latch:
            left_align, right_align, success, reason = self._get_align_poses()
            if not success:
                self.last_reject_reason = reason or "no_tcp_feedback"
            else:
                self.last_reject_reason = ""
                latched_any = False
                if needs_left_latch and left_align is not None:
                    self._left.target_initial = left_align
                    self._left.source_initial = left_source
                    self._left.previous_target = left_align
                    if self._left.filter is not None:
                        self._left.filter.reset()
                    self._left.active = True
                    latched_any = True

                if needs_right_latch and right_align is not None:
                    self._right.target_initial = right_align
                    self._right.source_initial = right_source
                    self._right.previous_target = right_align
                    if self._right.filter is not None:
                        self._right.filter.reset()
                    self._right.active = True
                    latched_any = True

                if latched_any:
                    self.snapshot_seq += 1

        # Deactivate if released
        if not left_wants_active:
            self._left.active = False
            self._left.source_initial = None

        if not right_wants_active:
            self._right.active = False
            self._right.source_initial = None

        if not left_wants_active and not right_wants_active:
            self.last_reject_reason = ""

        # Compute Left Arm Target
        if self._left.active and left_source is not None:
            left_target = self._relative_target(self._left, left_source, dt)
            left_ee_pose[0] = np.concatenate(
                [left_target.position, left_target.rotation.as_quat()]
            ).astype(np.float32)
            if left_active_out is not None:
                left_active_out[0] = np.array([1], dtype=np.int32)
        else:
            left_ee_pose[0] = np.zeros(7, dtype=np.float32)
            if left_active_out is not None:
                left_active_out[0] = np.array([0], dtype=np.int32)

        # Compute Right Arm Target
        if self._right.active and right_source is not None:
            right_target = self._relative_target(self._right, right_source, dt)
            right_ee_pose[0] = np.concatenate(
                [right_target.position, right_target.rotation.as_quat()]
            ).astype(np.float32)
            if right_active_out is not None:
                right_active_out[0] = np.array([1], dtype=np.int32)
        else:
            right_ee_pose[0] = np.zeros(7, dtype=np.float32)
            if right_active_out is not None:
                right_active_out[0] = np.array([0], dtype=np.int32)

        self._active = bool(self._left.active or self._right.active)
        active_out[0] = np.array([int(self._active)], dtype=np.int32)

    def _relative_target(
        self, side: SideState, current: ControllerPose, dt: float
    ) -> ControllerPose:
        if side.source_initial is None or side.target_initial is None:
            raise RuntimeError("relative target requested before snapshot latch")

        position_delta = (
            current.position - side.source_initial.position
        ) * self.config.linear_scale

        # Apply the controller's base/world-frame rotation delta to the latched
        # robot pose. Controller and gripper local axes are not interchangeable.
        rot_delta_world = current.rotation * side.source_initial.rotation.inv()
        if self.config.angular_scale != 1.0:
            rotvec = rot_delta_world.as_rotvec() * self.config.angular_scale
            rot_delta_world = Rotation.from_rotvec(rotvec)

        target_rotation = rot_delta_world * side.target_initial.rotation
        target_position = side.target_initial.position + position_delta

        # Adaptive Filtering (One Euro Filter on SE(3))
        if self.config.filter_type == "one_euro" and side.filter is not None:
            target_position, target_rotation = side.filter.filter(
                target_position, target_rotation, dt=dt
            )
        elif self.config.filter_type == "lowpass":
            previous = side.previous_target
            alpha = float(np.clip(self.config.lowpass_alpha, 0.0, 1.0))
            if previous is not None and alpha < 1.0:
                target_position = (alpha * target_position) + (
                    (1.0 - alpha) * previous.position
                )
                target_rotation = _slerp(previous.rotation, target_rotation, alpha)

        target = ControllerPose(
            position=target_position,
            rotation=target_rotation,
        )

        previous = side.previous_target
        if previous is not None:
            step = target.position - previous.position
            step_norm = float(np.linalg.norm(step))
            if (
                self.config.max_linear_step_m > 0.0
                and step_norm > self.config.max_linear_step_m
            ):
                target.position = previous.position + step * (
                    self.config.max_linear_step_m / step_norm
                )
            target.rotation = _clamp_rotation_step(
                previous.rotation,
                target.rotation,
                self.config.max_angular_step_rad,
            )

        side.previous_target = target
        return target
