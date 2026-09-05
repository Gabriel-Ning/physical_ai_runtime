"""Unit tests for BimanualAbsoluteRetargeter (calibration, 1:1 reaching, rotation, decoupling, gripper, one euro filter, dynamic scaling)."""

import time
import unittest
from types import SimpleNamespace

import numpy as np
from isaacteleop.retargeting_engine.interface import OptionalTensorGroup
from isaacteleop.retargeting_engine.tensor_types import (
    ControllerInput,
    ControllerInputIndex,
)
from isaacteleop_toolbox.filters import OneEuroFilterSE3
from isaacteleop_toolbox.retargeters import (
    BimanualAbsoluteConfig,
    BimanualAbsoluteRetargeter,
    ControllerPose,
)
from scipy.spatial.transform import Rotation


def _make_controller_group(
    pos=(0.0, 0.0, 0.0),
    rot=(0.0, 0.0, 0.0, 1.0),
    primary=0.0,
    secondary=0.0,
    squeeze=0.0,
    trigger=0.0,
    thumbstick_y=0.0,
    thumbstick_click=0.0,
    menu_click=0.0,
    valid=True,
):
    otg = OptionalTensorGroup(ControllerInput())
    otg[ControllerInputIndex.GRIP_POSITION] = np.array(pos, dtype=np.float32)
    otg[ControllerInputIndex.GRIP_ORIENTATION] = np.array(rot, dtype=np.float32)
    otg[ControllerInputIndex.GRIP_IS_VALID] = bool(valid)
    otg[ControllerInputIndex.AIM_POSITION] = np.array(pos, dtype=np.float32)
    otg[ControllerInputIndex.AIM_ORIENTATION] = np.array(rot, dtype=np.float32)
    otg[ControllerInputIndex.AIM_IS_VALID] = bool(valid)
    otg[ControllerInputIndex.PRIMARY_CLICK] = float(primary)
    otg[ControllerInputIndex.SECONDARY_CLICK] = float(secondary)
    otg[ControllerInputIndex.THUMBSTICK_X] = 0.0
    otg[ControllerInputIndex.THUMBSTICK_Y] = float(thumbstick_y)
    otg[ControllerInputIndex.THUMBSTICK_CLICK] = float(thumbstick_click)
    otg[ControllerInputIndex.MENU_CLICK] = float(menu_click)
    otg[ControllerInputIndex.SQUEEZE_VALUE] = float(squeeze)
    otg[ControllerInputIndex.TRIGGER_VALUE] = float(trigger)
    return otg


def _make_outputs():
    return {
        "left_ee_pose": [np.zeros(7, dtype=np.float32)],
        "right_ee_pose": [np.zeros(7, dtype=np.float32)],
        "left_gripper": [np.zeros(1, dtype=np.float32)],
        "right_gripper": [np.zeros(1, dtype=np.float32)],
        "active": [np.zeros(1, dtype=np.int32)],
        "left_active": [np.zeros(1, dtype=np.int32)],
        "right_active": [np.zeros(1, dtype=np.int32)],
    }


class BimanualAbsoluteRetargeterTest(unittest.TestCase):
    def test_calibration_and_1to1_translation(self):
        config = BimanualAbsoluteConfig(
            pose_source="grip",
            calibrate_source="menu",
            enable_deadman=False,
            linear_scale=1.0,
            angular_scale=1.0,
            filter_type="none",
            max_linear_step_m=0.0,
            max_angular_step_rad=0.0,
            openxr_to_base_rotation_xyzw=[0.0, 0.0, 0.0, 1.0],  # Identity
            left_home_xyz=[0.4, 0.2, 0.3],
            left_home_xyzw=[0.0, 0.0, 0.0, 1.0],
            right_home_xyz=[0.4, -0.2, 0.3],
            right_home_xyzw=[0.0, 0.0, 0.0, 1.0],
            use_live_ee_as_home=False,
        )
        retargeter = BimanualAbsoluteRetargeter(config)
        context = SimpleNamespace(execution_events=SimpleNamespace(reset=False))

        # Initial calibration frame: left hand at (0.1, 0.2, 0.3), right at (0.1, -0.2, 0.3)
        inputs_calib = {
            "controller_left": _make_controller_group(pos=(0.1, 0.2, 0.3), menu_click=1.0),
            "controller_right": _make_controller_group(pos=(0.1, -0.2, 0.3), menu_click=1.0),
        }
        outputs = _make_outputs()
        retargeter._compute_fn(inputs_calib, outputs, context)

        self.assertTrue(retargeter._is_calibrated)
        self.assertEqual(int(outputs["active"][0][0]), 1)
        self.assertEqual(int(outputs["left_active"][0][0]), 1)
        self.assertEqual(int(outputs["right_active"][0][0]), 1)

        # Output at calibration pose must match home pose exactly
        np.testing.assert_allclose(outputs["left_ee_pose"][0][:3], [0.4, 0.2, 0.3], atol=1e-4)
        np.testing.assert_allclose(outputs["right_ee_pose"][0][:3], [0.4, -0.2, 0.3], atol=1e-4)

        # Step 2: Left hand moves +0.05m in X, +0.02m in Z; Right hand remains stationary
        inputs_step = {
            "controller_left": _make_controller_group(pos=(0.15, 0.2, 0.32)),
            "controller_right": _make_controller_group(pos=(0.1, -0.2, 0.3)),
        }
        outputs = _make_outputs()
        retargeter._compute_fn(inputs_step, outputs, context)

        np.testing.assert_allclose(outputs["left_ee_pose"][0][:3], [0.45, 0.2, 0.32], atol=1e-4)
        np.testing.assert_allclose(outputs["right_ee_pose"][0][:3], [0.40, -0.2, 0.30], atol=1e-4)

    def test_rotational_alignment_uses_world_frame_delta(self):
        config = BimanualAbsoluteConfig(
            pose_source="grip",
            openxr_to_base_rotation_xyzw=[0.0, 0.0, 0.0, 1.0],
            left_home_xyz=[0.0, 0.0, 0.0],
            left_home_xyzw=Rotation.from_euler("z", 90, degrees=True).as_quat().tolist(),
            right_home_xyz=[0.0, 0.0, 0.0],
            right_home_xyzw=[0.0, 0.0, 0.0, 1.0],
            filter_type="none",
            max_linear_step_m=0.0,
            max_angular_step_rad=0.0,
            use_live_ee_as_home=False,
        )
        retargeter = BimanualAbsoluteRetargeter(config)
        context = SimpleNamespace(execution_events=SimpleNamespace(reset=False))

        # Initial calibration: hand tilted 30 deg about Y
        initial_rot = Rotation.from_euler("y", 30, degrees=True).as_quat()
        inputs_calib = {
            "controller_left": _make_controller_group(rot=initial_rot, menu_click=1.0),
            "controller_right": _make_controller_group(rot=initial_rot, menu_click=1.0),
        }
        outputs = _make_outputs()
        retargeter._compute_fn(inputs_calib, outputs, context)

        robot_home = Rotation.from_euler("z", 90, degrees=True)
        q_calibrated = outputs["left_ee_pose"][0][3:7]
        self.assertAlmostEqual(abs(np.dot(q_calibrated, robot_home.as_quat())), 1.0, places=3)

        # Now hand rotates an additional 45 deg about X (wrist roll in tool frame)
        rotated_rot = (Rotation.from_euler("y", 30, degrees=True) * Rotation.from_euler("x", 45, degrees=True)).as_quat()
        inputs_rot = {
            "controller_left": _make_controller_group(rot=rotated_rot),
            "controller_right": _make_controller_group(rot=initial_rot),
        }
        outputs = _make_outputs()
        retargeter._compute_fn(inputs_rot, outputs, context)

        controller_initial = Rotation.from_quat(initial_rot)
        controller_current = Rotation.from_quat(rotated_rot)
        expected_rot = (
            controller_current * controller_initial.inv() * robot_home
        ).as_quat()
        q_out = outputs["left_ee_pose"][0][3:7]
        dot = np.abs(np.dot(q_out, expected_rot))
        self.assertAlmostEqual(dot, 1.0, places=3)

        old_local_delta = (
            robot_home * controller_initial.inv() * controller_current
        ).as_quat()
        self.assertLess(abs(np.dot(q_out, old_local_delta)), 0.99)

    def test_decoupled_asymmetric_arm_control(self):
        config = BimanualAbsoluteConfig(
            pose_source="grip",
            enable_deadman=True,
            deadman_source="squeeze",
            require_both_deadman=False,  # Decoupled!
            openxr_to_base_rotation_xyzw=[0.0, 0.0, 0.0, 1.0],
            use_live_ee_as_home=False,
        )
        retargeter = BimanualAbsoluteRetargeter(config)
        context = SimpleNamespace(execution_events=SimpleNamespace(reset=False))

        # Calibrate both hands first
        inputs_calib = {
            "controller_left": _make_controller_group(pos=(0.1, 0.2, 0.3), squeeze=1.0, menu_click=1.0),
            "controller_right": _make_controller_group(pos=(0.1, -0.2, 0.3), squeeze=1.0, menu_click=1.0),
        }
        outputs = _make_outputs()
        retargeter._compute_fn(inputs_calib, outputs, context)

        self.assertEqual(int(outputs["left_active"][0][0]), 1)
        self.assertEqual(int(outputs["right_active"][0][0]), 1)
        self.assertEqual(int(outputs["active"][0][0]), 1)

        # Now Left hand releases squeeze (becomes inactive), Right hand keeps holding squeeze
        inputs_async = {
            "controller_left": _make_controller_group(pos=(0.1, 0.2, 0.3), squeeze=0.0),
            "controller_right": _make_controller_group(pos=(0.1, -0.2, 0.3), squeeze=1.0),
        }
        outputs = _make_outputs()
        retargeter._compute_fn(inputs_async, outputs, context)

        # Left should be inactive (zeros), Right should remain active!
        self.assertEqual(int(outputs["left_active"][0][0]), 0)
        self.assertEqual(int(outputs["right_active"][0][0]), 1)
        self.assertEqual(int(outputs["active"][0][0]), 1)
        np.testing.assert_allclose(outputs["left_ee_pose"][0], np.zeros(7), atol=1e-5)
        self.assertFalse(np.all(outputs["right_ee_pose"][0] == 0))

    def test_gripper_button_servo_release_to_hold(self):
        config = BimanualAbsoluteConfig(
            pose_source="grip",
            openxr_to_base_rotation_xyzw=[0.0, 0.0, 0.0, 1.0],
            gripper_min_width=0.0,
            gripper_max_width=0.08,
            gripper_speed_mps=0.05,
            use_live_ee_as_home=False,
        )
        retargeter = BimanualAbsoluteRetargeter(config)
        context = SimpleNamespace(execution_events=SimpleNamespace(reset=False))

        # Initial cycle: gripper max open (0.08m)
        inputs_0 = {
            "controller_left": _make_controller_group(),
            "controller_right": _make_controller_group(),
        }
        outputs = _make_outputs()
        retargeter._compute_fn(inputs_0, outputs, context)
        self.assertAlmostEqual(float(outputs["left_gripper"][0][0]), 0.08, places=3)
        self.assertAlmostEqual(float(outputs["right_gripper"][0][0]), 0.08, places=3)

        # Press primary button (X on left, A on right) to close
        inputs_close = {
            "controller_left": _make_controller_group(primary=1.0),
            "controller_right": _make_controller_group(primary=1.0),
        }
        time.sleep(0.05)
        retargeter._compute_fn(inputs_close, outputs, context)
        pos_after_close = float(outputs["left_gripper"][0][0])
        self.assertLess(pos_after_close, 0.08)

        # Release buttons (primary=0, secondary=0): Position MUST be held!
        inputs_hold = {
            "controller_left": _make_controller_group(primary=0.0, secondary=0.0),
            "controller_right": _make_controller_group(primary=0.0, secondary=0.0),
        }
        time.sleep(0.05)
        retargeter._compute_fn(inputs_hold, outputs, context)
        pos_after_hold = float(outputs["left_gripper"][0][0])
        self.assertAlmostEqual(pos_after_hold, pos_after_close, places=4)

    def test_dynamic_scale_thumbstick_discrete_and_reset(self):
        config = BimanualAbsoluteConfig(
            pose_source="grip",
            linear_scale=1.0,
            enable_dynamic_scale=True,
            scale_step=0.1,
            use_live_ee_as_home=False,
        )
        retargeter = BimanualAbsoluteRetargeter(config)
        context = SimpleNamespace(execution_events=SimpleNamespace(reset=False))

        # Initial scale is 1.0
        self.assertEqual(retargeter.left_linear_scale, 1.0)
        self.assertEqual(retargeter.right_linear_scale, 1.0)

        # 1. Flick Thumbstick Y Up (+0.8)
        inputs_up = {
            "controller_left": _make_controller_group(),
            "controller_right": _make_controller_group(thumbstick_y=0.8),
        }
        outputs = _make_outputs()
        retargeter._compute_fn(inputs_up, outputs, context)
        self.assertAlmostEqual(retargeter.right_linear_scale, 1.1, places=2)
        self.assertAlmostEqual(retargeter.left_linear_scale, 1.0, places=2)

        # 2. Return stick to deadzone (0.0) -> Debounce clears
        inputs_neutral = {
            "controller_left": _make_controller_group(),
            "controller_right": _make_controller_group(thumbstick_y=0.0),
        }
        retargeter._compute_fn(inputs_neutral, outputs, context)
        self.assertAlmostEqual(retargeter.right_linear_scale, 1.1, places=2)

        # 3. Flick Up again -> Increments to 1.2
        retargeter._compute_fn(inputs_up, outputs, context)
        self.assertAlmostEqual(retargeter.right_linear_scale, 1.2, places=2)

        # 4. Instant Reset: Press Thumbstick Click -> Snaps back to exactly 1.0!
        inputs_click = {
            "controller_left": _make_controller_group(),
            "controller_right": _make_controller_group(thumbstick_click=1.0),
        }
        retargeter._compute_fn(inputs_click, outputs, context)
        self.assertAlmostEqual(retargeter.right_linear_scale, 1.0, places=4)

        # Left input changes only the left arm scale.
        inputs_left_up = {
            "controller_left": _make_controller_group(thumbstick_y=0.8),
            "controller_right": _make_controller_group(),
        }
        retargeter._compute_fn(inputs_left_up, outputs, context)
        self.assertAlmostEqual(retargeter.left_linear_scale, 1.1, places=2)
        self.assertAlmostEqual(retargeter.right_linear_scale, 1.0, places=2)

    def test_one_euro_filter_se3_adaptive_smoothing(self):
        filter_se3 = OneEuroFilterSE3(min_cutoff_pos=1.0, beta_pos=0.01)

        # 1. Stationary with high-frequency human tremor noise
        p_base = np.array([0.5, 0.0, 0.3])
        r_base = Rotation.identity()

        noisy_positions = []
        filtered_positions = []

        for t_idx in range(60):
            # 10Hz noise jitter (+-2mm)
            noise = 0.002 * np.sin(2.0 * np.pi * 10.0 * (t_idx / 60.0))
            p_meas = p_base + np.array([noise, -noise, noise])
            p_hat, r_hat = filter_se3.filter(p_meas, r_base, dt=1.0 / 60.0)
            noisy_positions.append(p_meas)
            filtered_positions.append(p_hat)

        # Variance of filtered trajectory should be significantly smaller than raw jitter
        raw_std = np.std(noisy_positions, axis=0)
        filtered_std = np.std(filtered_positions, axis=0)
        self.assertLess(filtered_std[0], raw_std[0] * 0.4)


if __name__ == "__main__":
    unittest.main()
