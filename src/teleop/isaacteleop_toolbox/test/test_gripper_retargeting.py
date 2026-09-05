"""Deterministic unit tests for button-servo gripper retargeting."""

import time
import unittest
from types import SimpleNamespace

import numpy as np
from isaacteleop.retargeting_engine.interface import OptionalTensorGroup
from isaacteleop.retargeting_engine.tensor_types import (
    ControllerInput,
    ControllerInputIndex,
)
from isaacteleop_toolbox.retargeters import (
    BimanualRelativeConfig,
    BimanualRelativeRetargeter,
    ControllerPose,
)
from isaacteleop_toolbox.retargeters.bimanual_relative import SideState
from scipy.spatial.transform import Rotation


def _make_config(**kwargs):
    defaults = {
        "pose_source": "aim",
        "deadman_source": "squeeze",
        "deadman_threshold": 0.5,
        "require_both_deadman": True,
        "linear_scale": 1.0,
        "angular_scale": 1.0,
        "lowpass_alpha": 1.0,
        "max_linear_step_m": 0.0,
        "max_angular_step_rad": 0.0,
        "openxr_to_base_rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "gripper_min_width": 0.0,
        "gripper_max_width": 0.04,
        "gripper_speed_mps": 0.04,  # 40mm/s for fast testing
        "require_clutch_for_gripper": False,
    }
    defaults.update(kwargs)
    return BimanualRelativeConfig(**defaults)


def _make_controller_group(primary=0.0, secondary=0.0, squeeze=1.0):
    otg = OptionalTensorGroup(ControllerInput())
    otg[ControllerInputIndex.GRIP_POSITION] = np.zeros((3,), dtype=np.float32)
    otg[ControllerInputIndex.GRIP_ORIENTATION] = np.array([0, 0, 0, 1], dtype=np.float32)
    otg[ControllerInputIndex.GRIP_IS_VALID] = True
    otg[ControllerInputIndex.AIM_POSITION] = np.zeros((3,), dtype=np.float32)
    otg[ControllerInputIndex.AIM_ORIENTATION] = np.array([0, 0, 0, 1], dtype=np.float32)
    otg[ControllerInputIndex.AIM_IS_VALID] = True
    otg[ControllerInputIndex.PRIMARY_CLICK] = float(primary)
    otg[ControllerInputIndex.SECONDARY_CLICK] = float(secondary)
    otg[ControllerInputIndex.THUMBSTICK_X] = 0.0
    otg[ControllerInputIndex.THUMBSTICK_Y] = 0.0
    otg[ControllerInputIndex.THUMBSTICK_CLICK] = 0.0
    otg[ControllerInputIndex.MENU_CLICK] = 0.0
    otg[ControllerInputIndex.SQUEEZE_VALUE] = float(squeeze)
    otg[ControllerInputIndex.TRIGGER_VALUE] = 0.0
    return otg


class GripperRetargetingTest(unittest.TestCase):
    def test_relative_rotation_uses_world_frame_delta(self):
        retargeter = BimanualRelativeRetargeter(_make_config(filter_type="none"))
        controller_initial = Rotation.from_euler("y", 30, degrees=True)
        controller_current = controller_initial * Rotation.from_euler(
            "x", 45, degrees=True
        )
        robot_initial = Rotation.from_euler("z", 90, degrees=True)
        side = SideState(
            source_initial=ControllerPose(np.zeros(3), controller_initial),
            target_initial=ControllerPose(np.zeros(3), robot_initial),
        )

        target = retargeter._relative_target(
            side,
            ControllerPose(np.zeros(3), controller_current),
            dt=1.0 / 60.0,
        )
        expected = controller_current * controller_initial.inv() * robot_initial
        self.assertAlmostEqual(
            abs(np.dot(target.rotation.as_quat(), expected.as_quat())), 1.0, places=3
        )
        old_local_delta = robot_initial * controller_initial.inv() * controller_current
        self.assertLess(
            abs(np.dot(target.rotation.as_quat(), old_local_delta.as_quat())), 0.99
        )

    def test_gripper_defaults_to_max_open(self):
        config = _make_config(gripper_max_width=0.04)
        retargeter = BimanualRelativeRetargeter(config)

        inputs = {
            "controller_left": _make_controller_group(),
            "controller_right": _make_controller_group(),
        }
        outputs = {
            "left_ee_pose": [np.zeros(7, dtype=np.float32)],
            "right_ee_pose": [np.zeros(7, dtype=np.float32)],
            "left_gripper": [np.zeros(1, dtype=np.float32)],
            "right_gripper": [np.zeros(1, dtype=np.float32)],
            "active": [np.zeros(1, dtype=np.int32)],
        }
        context = SimpleNamespace(
            execution_events=SimpleNamespace(reset=False)
        )

        retargeter._compute_fn(inputs, outputs, context)
        self.assertAlmostEqual(float(outputs["left_gripper"][0][0]), 0.04, places=4)
        self.assertAlmostEqual(float(outputs["right_gripper"][0][0]), 0.04, places=4)

    def test_primary_button_closes_gripper_continuously(self):
        config = _make_config(
            gripper_min_width=0.0,
            gripper_max_width=0.04,
            gripper_speed_mps=0.04,
        )
        retargeter = BimanualRelativeRetargeter(config)

        outputs = {
            "left_ee_pose": [np.zeros(7, dtype=np.float32)],
            "right_ee_pose": [np.zeros(7, dtype=np.float32)],
            "left_gripper": [np.zeros(1, dtype=np.float32)],
            "right_gripper": [np.zeros(1, dtype=np.float32)],
            "active": [np.zeros(1, dtype=np.int32)],
        }
        context = SimpleNamespace(
            execution_events=SimpleNamespace(reset=False)
        )

        inputs = {
            "controller_left": _make_controller_group(primary=1.0),
            "controller_right": _make_controller_group(primary=1.0),
        }
        retargeter._last_time = time.monotonic() - 0.5

        retargeter._compute_fn(inputs, outputs, context)
        self.assertLess(float(outputs["left_gripper"][0][0]), 0.035)
        self.assertGreater(float(outputs["left_gripper"][0][0]), 0.005)

    def test_release_to_hold(self):
        config = _make_config(gripper_max_width=0.04, gripper_speed_mps=0.04)
        retargeter = BimanualRelativeRetargeter(config)

        outputs = {
            "left_ee_pose": [np.zeros(7, dtype=np.float32)],
            "right_ee_pose": [np.zeros(7, dtype=np.float32)],
            "left_gripper": [np.zeros(1, dtype=np.float32)],
            "right_gripper": [np.zeros(1, dtype=np.float32)],
            "active": [np.zeros(1, dtype=np.int32)],
        }
        context = SimpleNamespace(
            execution_events=SimpleNamespace(reset=False)
        )

        inputs_close = {
            "controller_left": _make_controller_group(primary=1.0),
            "controller_right": _make_controller_group(primary=1.0),
        }
        retargeter._last_time = time.monotonic() - 0.25
        retargeter._compute_fn(inputs_close, outputs, context)
        held_pos = float(outputs["left_gripper"][0][0])
        self.assertLess(held_pos, 0.04)

        inputs_hold = {
            "controller_left": _make_controller_group(primary=0.0, secondary=0.0),
            "controller_right": _make_controller_group(primary=0.0, secondary=0.0),
        }
        retargeter._last_time = time.monotonic() - 0.5
        retargeter._compute_fn(inputs_hold, outputs, context)
        self.assertAlmostEqual(float(outputs["left_gripper"][0][0]), held_pos, places=4)

    def test_limits_clamping(self):
        config = _make_config(gripper_min_width=0.0, gripper_max_width=0.04, gripper_speed_mps=1.0)
        retargeter = BimanualRelativeRetargeter(config)

        outputs = {
            "left_ee_pose": [np.zeros(7, dtype=np.float32)],
            "right_ee_pose": [np.zeros(7, dtype=np.float32)],
            "left_gripper": [np.zeros(1, dtype=np.float32)],
            "right_gripper": [np.zeros(1, dtype=np.float32)],
            "active": [np.zeros(1, dtype=np.int32)],
        }
        context = SimpleNamespace(
            execution_events=SimpleNamespace(reset=False)
        )

        inputs_close = {
            "controller_left": _make_controller_group(primary=1.0),
            "controller_right": _make_controller_group(primary=1.0),
        }
        retargeter._last_time = time.monotonic() - 0.2
        retargeter._compute_fn(inputs_close, outputs, context)
        self.assertAlmostEqual(float(outputs["left_gripper"][0][0]), 0.0, places=4)

        inputs_open = {
            "controller_left": _make_controller_group(secondary=1.0),
            "controller_right": _make_controller_group(secondary=1.0),
        }
        retargeter._last_time = time.monotonic() - 0.2
        retargeter._compute_fn(inputs_open, outputs, context)
        self.assertAlmostEqual(float(outputs["left_gripper"][0][0]), 0.04, places=4)

    def test_activation_refused_without_tcp_feedback(self):
        config = _make_config()
        # on_activate_fn returns failure when TF lookup fails
        fail_fn = lambda: (None, None, False, "no_tcp_feedback")
        retargeter = BimanualRelativeRetargeter(config, on_activate_fn=fail_fn)

        outputs = {
            "left_ee_pose": [np.zeros(7, dtype=np.float32)],
            "right_ee_pose": [np.zeros(7, dtype=np.float32)],
            "left_gripper": [np.zeros(1, dtype=np.float32)],
            "right_gripper": [np.zeros(1, dtype=np.float32)],
            "active": [np.zeros(1, dtype=np.int32)],
        }
        context = SimpleNamespace(execution_events=SimpleNamespace(reset=False))
        inputs = {
            "controller_left": _make_controller_group(squeeze=1.0),
            "controller_right": _make_controller_group(squeeze=1.0),
        }

        retargeter._compute_fn(inputs, outputs, context)
        # Activation should be rejected!
        self.assertEqual(int(outputs["active"][0][0]), 0)
        self.assertEqual(retargeter.last_reject_reason, "no_tcp_feedback")
        self.assertEqual(retargeter.snapshot_seq, 0)
        np.testing.assert_array_equal(outputs["left_ee_pose"][0], np.zeros(7))
        np.testing.assert_array_equal(outputs["right_ee_pose"][0], np.zeros(7))

    def test_activation_succeeds_with_fk(self):
        from isaacteleop_toolbox.retargeters import ControllerPose
        from scipy.spatial.transform import Rotation

        config = _make_config()
        left_fk = ControllerPose(np.array([0.5, 0.1, 0.2]), Rotation.identity())
        right_fk = ControllerPose(np.array([0.5, -0.1, 0.2]), Rotation.identity())
        success_fn = lambda: (left_fk, right_fk, True, "")
        retargeter = BimanualRelativeRetargeter(config, on_activate_fn=success_fn)

        outputs = {
            "left_ee_pose": [np.zeros(7, dtype=np.float32)],
            "right_ee_pose": [np.zeros(7, dtype=np.float32)],
            "left_gripper": [np.zeros(1, dtype=np.float32)],
            "right_gripper": [np.zeros(1, dtype=np.float32)],
            "active": [np.zeros(1, dtype=np.int32)],
        }
        context = SimpleNamespace(execution_events=SimpleNamespace(reset=False))
        inputs = {
            "controller_left": _make_controller_group(squeeze=1.0),
            "controller_right": _make_controller_group(squeeze=1.0),
        }

        retargeter._compute_fn(inputs, outputs, context)
        # Activation succeeds with robot FK as initial target anchor!
        self.assertEqual(int(outputs["active"][0][0]), 1)
        self.assertEqual(retargeter.last_reject_reason, "")
        self.assertEqual(retargeter.snapshot_seq, 1)
        np.testing.assert_almost_equal(outputs["left_ee_pose"][0][:3], np.array([0.5, 0.1, 0.2]))
        np.testing.assert_almost_equal(outputs["right_ee_pose"][0][:3], np.array([0.5, -0.1, 0.2]))

    def test_reject_reason_clears_on_squeeze_release(self):
        config = _make_config()
        fail_fn = lambda: (None, None, False, "no_tcp_feedback")
        retargeter = BimanualRelativeRetargeter(config, on_activate_fn=fail_fn)

        outputs = {
            "left_ee_pose": [np.zeros(7, dtype=np.float32)],
            "right_ee_pose": [np.zeros(7, dtype=np.float32)],
            "left_gripper": [np.zeros(1, dtype=np.float32)],
            "right_gripper": [np.zeros(1, dtype=np.float32)],
            "active": [np.zeros(1, dtype=np.int32)],
        }
        context = SimpleNamespace(execution_events=SimpleNamespace(reset=False))

        # 1. Pressed with TF failure -> feedback rejection is reported.
        inputs_pressed = {
            "controller_left": _make_controller_group(squeeze=1.0),
            "controller_right": _make_controller_group(squeeze=1.0),
        }
        retargeter._compute_fn(inputs_pressed, outputs, context)
        self.assertEqual(retargeter.last_reject_reason, "no_tcp_feedback")

        # 2. Release squeeze -> inactive branch executes, clearing last_reject_reason
        inputs_released = {
            "controller_left": _make_controller_group(squeeze=0.0),
            "controller_right": _make_controller_group(squeeze=0.0),
        }
        retargeter._compute_fn(inputs_released, outputs, context)
        self.assertEqual(retargeter.last_reject_reason, "")
        self.assertEqual(int(outputs["active"][0][0]), 0)


if __name__ == "__main__":
    unittest.main()
