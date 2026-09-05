"""Absolute bimanual target retargeting with One Euro Filter and discrete dynamic scaling."""

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
from scipy.spatial.transform import Rotation

from isaacteleop_toolbox.filters import OneEuroFilterSE3


@dataclass
class ControllerPose:
    position: np.ndarray
    rotation: Rotation


@dataclass
class SideAbsoluteState:
    calibrated_source: ControllerPose | None = None
    calibrated_robot_home: ControllerPose | None = None
    offset_position: np.ndarray | None = None
    align_rotation: Rotation | None = None
    previous_target: ControllerPose | None = None
    filter: OneEuroFilterSE3 | None = None
    active: bool = False


@dataclass(frozen=True)
class AbsoluteSnapshot:
    """Calibration snapshot latched during the most recent calibration event."""

    seq: int
    left_source: ControllerPose | None
    left_robot_home: ControllerPose | None
    right_source: ControllerPose | None
    right_robot_home: ControllerPose | None
    current_linear_scale: float = 1.0


@dataclass(frozen=True)
class BimanualAbsoluteConfig:
    pose_source: str
    calibrate_source: str = "menu"
    calibrate_threshold: float = 0.5
    enable_deadman: bool = False
    deadman_source: str = "squeeze"
    deadman_threshold: float = 0.5
    require_both_deadman: bool = False
    linear_scale: float = 1.0
    angular_scale: float = 1.0
    # Dynamic scale adjustment via thumbstick
    enable_dynamic_scale: bool = True
    scale_step: float = 0.1
    min_scale: float = 0.1
    max_scale: float = 3.0
    # One Euro Filter parameters
    filter_type: str = "one_euro"  # "one_euro", "lowpass", "none"
    one_euro_min_cutoff: float = 1.0
    one_euro_beta: float = 0.007
    one_euro_d_cutoff: float = 1.0
    lowpass_alpha: float = 0.85
    # Safety rate limits
    max_linear_step_m: float = 0.05
    max_angular_step_rad: float = 0.25
    openxr_to_base_rotation_xyzw: list[float] = None
    # Default standby home poses
    left_home_xyz: list[float] = None
    left_home_xyzw: list[float] = None
    right_home_xyz: list[float] = None
    right_home_xyzw: list[float] = None
    use_live_ee_as_home: bool = True
    # Gripper parameters (Release-to-Hold continuous servo)
    gripper_min_width: float = 0.0
    gripper_max_width: float = 0.08
    gripper_speed_mps: float = 0.05
    require_active_for_gripper: bool = False

    def __post_init__(self):
        if self.openxr_to_base_rotation_xyzw is None:
            object.__setattr__(
                self, "openxr_to_base_rotation_xyzw", [0.5, -0.5, -0.5, 0.5]
            )
        if self.left_home_xyz is None:
            object.__setattr__(self, "left_home_xyz", [0.3, 0.2, 0.2])
        if self.left_home_xyzw is None:
            object.__setattr__(self, "left_home_xyzw", [0.0, 0.0, 0.0, 1.0])
        if self.right_home_xyz is None:
            object.__setattr__(self, "right_home_xyz", [0.3, -0.2, 0.2])
        if self.right_home_xyzw is None:
            object.__setattr__(self, "right_home_xyzw", [0.0, 0.0, 0.0, 1.0])


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


def _button_pressed(
    ctrl: OptionalTensorGroup, source: str, threshold: float = 0.5
) -> bool:
    if ctrl.is_none:
        return False
    if source == "none":
        return False
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
    if source == "menu":
        if hasattr(ControllerInputIndex, "MENU_CLICK"):
            return (
                _scalar_value(ctrl[getattr(ControllerInputIndex, "MENU_CLICK")])
                >= threshold
            )
        return _scalar_value(ctrl[ControllerInputIndex.SECONDARY_CLICK]) >= threshold
    raise ValueError(f"Unknown button source: {source}")


def _deadman_pressed(ctrl: OptionalTensorGroup, source: str, threshold: float) -> bool:
    if ctrl.is_none:
        return False
    if source == "none":
        return True
    return _button_pressed(ctrl, source, threshold)


class BimanualAbsoluteRetargeter(BaseRetargeter):
    """Calibrated absolute spatial and rotational mapping with One Euro filter & dynamic scaling."""

    def __init__(
        self,
        config: BimanualAbsoluteConfig,
        on_calibrate_fn=None,
        name: str = "bimanual_absolute",
    ) -> None:
        self.config = config
        self.on_calibrate_fn = on_calibrate_fn
        self._left = SideAbsoluteState(
            filter=OneEuroFilterSE3(
                min_cutoff_pos=config.one_euro_min_cutoff,
                beta_pos=config.one_euro_beta,
                min_cutoff_rot=config.one_euro_min_cutoff,
                beta_rot=config.one_euro_beta,
                d_cutoff=config.one_euro_d_cutoff,
            )
        )
        self._right = SideAbsoluteState(
            filter=OneEuroFilterSE3(
                min_cutoff_pos=config.one_euro_min_cutoff,
                beta_pos=config.one_euro_beta,
                min_cutoff_rot=config.one_euro_min_cutoff,
                beta_rot=config.one_euro_beta,
                d_cutoff=config.one_euro_d_cutoff,
            )
        )
        self._is_calibrated = False
        self.calibration_seq = 0
        self.left_linear_scale = float(config.linear_scale)
        self.right_linear_scale = float(config.linear_scale)
        self._openxr_to_base = Rotation.from_quat(
            np.asarray(config.openxr_to_base_rotation_xyzw, dtype=float)
        )
        self._left_gripper_pos = float(config.gripper_max_width)
        self._right_gripper_pos = float(config.gripper_max_width)
        self._last_time: float | None = None
        self.last_status_reason: str = ""

        # Thumbstick debouncing states for scale adjustment
        self._left_stick_latched = False
        self._right_stick_latched = False

        super().__init__(name=name)

    @property
    def snapshot(self) -> AbsoluteSnapshot:
        return AbsoluteSnapshot(
            seq=self.calibration_seq,
            left_source=self._left.calibrated_source,
            left_robot_home=self._left.calibrated_robot_home,
            right_source=self._right.calibrated_source,
            right_robot_home=self._right.calibrated_robot_home,
            current_linear_scale=self.left_linear_scale,
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

    def _update_side_scale(self, ctrl, scale: float, latched: bool) -> tuple[float, bool]:
        """Update one arm's scale exclusively from its matching controller."""
        if ctrl.is_none:
            return scale, False
        stick_y = _scalar_value(ctrl[ControllerInputIndex.THUMBSTICK_Y])
        clicked = _scalar_value(ctrl[ControllerInputIndex.THUMBSTICK_CLICK]) >= 0.5
        if clicked:
            return float(self.config.linear_scale), True
        if abs(stick_y) < 0.25:
            return scale, False
        if latched:
            return scale, True
        if stick_y > 0.65:
            return round(min(self.config.max_scale, scale + self.config.scale_step), 2), True
        if stick_y < -0.65:
            return round(max(self.config.min_scale, scale - self.config.scale_step), 2), True
        return scale, False

    def _update_dynamic_scale(self, right_ctrl: OptionalTensorGroup, left_ctrl: OptionalTensorGroup) -> None:
        """Adjust the left and right arm scales independently."""
        if not self.config.enable_dynamic_scale:
            return
        self.left_linear_scale, self._left_stick_latched = self._update_side_scale(
            left_ctrl, self.left_linear_scale, self._left_stick_latched
        )
        self.right_linear_scale, self._right_stick_latched = self._update_side_scale(
            right_ctrl, self.right_linear_scale, self._right_stick_latched
        )

    def _trigger_calibration(
        self,
        left_source: ControllerPose | None,
        right_source: ControllerPose | None,
    ) -> bool:
        left_home, right_home = None, None
        if self.config.use_live_ee_as_home and self.on_calibrate_fn is not None:
            res = self.on_calibrate_fn()
            if isinstance(res, tuple) and len(res) == 4:
                left_home, right_home, success, reason = res
                if not success:
                    self.last_status_reason = reason
                    return False
            elif isinstance(res, tuple) and len(res) == 2:
                left_home, right_home = res

        if left_home is None:
            left_home = ControllerPose(
                position=np.array(self.config.left_home_xyz, dtype=float),
                rotation=Rotation.from_quat(self.config.left_home_xyzw),
            )
        if right_home is None:
            right_home = ControllerPose(
                position=np.array(self.config.right_home_xyz, dtype=float),
                rotation=Rotation.from_quat(self.config.right_home_xyzw),
            )

        if left_source is not None:
            self._left.calibrated_source = left_source
            self._left.calibrated_robot_home = left_home
            self._left.offset_position = (
                left_home.position - left_source.position * self.left_linear_scale
            )
            self._left.align_rotation = (
                left_home.rotation * left_source.rotation.inv()
            )
            self._left.previous_target = left_home
            if self._left.filter is not None:
                self._left.filter.reset()

        if right_source is not None:
            self._right.calibrated_source = right_source
            self._right.calibrated_robot_home = right_home
            self._right.offset_position = (
                right_home.position - right_source.position * self.right_linear_scale
            )
            self._right.align_rotation = (
                right_home.rotation * right_source.rotation.inv()
            )
            self._right.previous_target = right_home
            if self._right.filter is not None:
                self._right.filter.reset()

        self.calibration_seq += 1
        self._is_calibrated = True
        self.last_status_reason = (
            f"calibrated (left_scale={self.left_linear_scale:.1f}x, "
            f"right_scale={self.right_linear_scale:.1f}x)"
        )
        return True

    def _compute_fn(self, inputs: RetargeterIO, outputs: RetargeterIO, context) -> None:
        if context.execution_events.reset:
            self._left = SideAbsoluteState(
                filter=OneEuroFilterSE3(
                    min_cutoff_pos=self.config.one_euro_min_cutoff,
                    beta_pos=self.config.one_euro_beta,
                    min_cutoff_rot=self.config.one_euro_min_cutoff,
                    beta_rot=self.config.one_euro_beta,
                    d_cutoff=self.config.one_euro_d_cutoff,
                )
            )
            self._right = SideAbsoluteState(
                filter=OneEuroFilterSE3(
                    min_cutoff_pos=self.config.one_euro_min_cutoff,
                    beta_pos=self.config.one_euro_beta,
                    min_cutoff_rot=self.config.one_euro_min_cutoff,
                    beta_rot=self.config.one_euro_beta,
                    d_cutoff=self.config.one_euro_d_cutoff,
                )
            )
            self._is_calibrated = False
            self.left_linear_scale = float(self.config.linear_scale)
            self.right_linear_scale = float(self.config.linear_scale)
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

        # Update dynamic scale if enabled
        self._update_dynamic_scale(right_ctrl, left_ctrl)

        left_valid = _pose_valid(left_ctrl, self.config.pose_source)
        right_valid = _pose_valid(right_ctrl, self.config.pose_source)

        left_deadman = _deadman_pressed(
            left_ctrl, self.config.deadman_source, self.config.deadman_threshold
        )
        right_deadman = _deadman_pressed(
            right_ctrl, self.config.deadman_source, self.config.deadman_threshold
        )

        # Gripper buttons (Release-to-Hold continuous servo)
        left_close = _button_pressed(left_ctrl, "primary")
        left_open = _button_pressed(left_ctrl, "secondary")
        right_close = _button_pressed(right_ctrl, "primary")
        right_open = _button_pressed(right_ctrl, "secondary")

        left_allowed = (not self.config.require_active_for_gripper) or left_deadman
        right_allowed = (not self.config.require_active_for_gripper) or right_deadman

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

        # Extract current controller poses in robot-aligned frame
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

        # Calibration check
        left_calib_btn = _button_pressed(
            left_ctrl, self.config.calibrate_source, self.config.calibrate_threshold
        )
        right_calib_btn = _button_pressed(
            right_ctrl, self.config.calibrate_source, self.config.calibrate_threshold
        )
        should_calibrate = (
            left_calib_btn
            or right_calib_btn
            or (not self._is_calibrated and (left_valid or right_valid))
        )

        if should_calibrate and (left_valid or right_valid):
            self._trigger_calibration(left_source, right_source)

        # Independent Arm Activation Logic
        if self.config.enable_deadman:
            if self.config.require_both_deadman:
                both_ok = (
                    left_valid and right_valid and left_deadman and right_deadman
                )
                left_active = both_ok
                right_active = both_ok
            else:
                left_active = left_valid and left_deadman
                right_active = right_valid and right_deadman
        else:
            left_active = left_valid and (self._left.align_rotation is not None)
            right_active = right_valid and (self._right.align_rotation is not None)

        self._left.active = left_active
        self._right.active = right_active

        # Compute Left Arm Absolute Target
        if left_active and left_source is not None:
            left_target = self._compute_absolute_target(self._left, left_source, dt)
            left_ee_pose[0] = np.concatenate(
                [left_target.position, left_target.rotation.as_quat()]
            ).astype(np.float32)
            if left_active_out is not None:
                left_active_out[0] = np.array([1], dtype=np.int32)
        else:
            left_ee_pose[0] = np.zeros(7, dtype=np.float32)
            if left_active_out is not None:
                left_active_out[0] = np.array([0], dtype=np.int32)

        # Compute Right Arm Absolute Target
        if right_active and right_source is not None:
            right_target = self._compute_absolute_target(self._right, right_source, dt)
            right_ee_pose[0] = np.concatenate(
                [right_target.position, right_target.rotation.as_quat()]
            ).astype(np.float32)
            if right_active_out is not None:
                right_active_out[0] = np.array([1], dtype=np.int32)
        else:
            right_ee_pose[0] = np.zeros(7, dtype=np.float32)
            if right_active_out is not None:
                right_active_out[0] = np.array([0], dtype=np.int32)

        overall_active = int(left_active or right_active)
        active_out[0] = np.array([overall_active], dtype=np.int32)

    def _compute_absolute_target(
        self, side: SideAbsoluteState, current: ControllerPose, dt: float
    ) -> ControllerPose:
        if (
            side.offset_position is None
            or side.align_rotation is None
            or side.calibrated_robot_home is None
            or side.calibrated_source is None
        ):
            raise RuntimeError(
                "Absolute target requested before side state calibration."
            )

        # Absolute linear position: Home + (Current - Initial) * dynamic_scale
        target_pos = side.calibrated_robot_home.position + (
            current.position - side.calibrated_source.position
        ) * float(
            self.left_linear_scale if side is self._left else self.right_linear_scale
        )

        # Absolute rotation: apply the controller's base/world-frame delta to
        # the calibrated robot pose.  Controller and gripper local axes are not
        # interchangeable, so a controller-local delta would invert or swap
        # pitch/roll whenever their calibrated frames are not axis-aligned.
        world_delta = current.rotation * side.calibrated_source.rotation.inv()
        if self.config.angular_scale != 1.0:
            world_delta = Rotation.from_rotvec(
                world_delta.as_rotvec() * self.config.angular_scale
            )
        target_rot = world_delta * side.calibrated_robot_home.rotation

        # Adaptive Filtering (One Euro Filter on SE(3))
        if self.config.filter_type == "one_euro" and side.filter is not None:
            target_pos, target_rot = side.filter.filter(target_pos, target_rot, dt=dt)
        elif self.config.filter_type == "lowpass":
            previous = side.previous_target
            alpha = float(np.clip(self.config.lowpass_alpha, 0.0, 1.0))
            if previous is not None and alpha < 1.0:
                target_pos = (alpha * target_pos) + ((1.0 - alpha) * previous.position)
                target_rot = _slerp(previous.rotation, target_rot, alpha)

        target = ControllerPose(position=target_pos, rotation=target_rot)

        # Rate Clamping Watchdog
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
