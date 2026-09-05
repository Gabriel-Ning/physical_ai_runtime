"""Deterministic tests for the public bimanual retarget parameter contract."""

import math
import unittest

from isaacteleop_toolbox.node_parameters import _validate_retarget_parameters


def _valid_parameters():
    return {
        "pose_source": "aim",
        "deadman_source": "squeeze",
        "deadman_threshold": 0.5,
        "linear_scale": 1.0,
        "angular_scale": 1.0,
        "lowpass_alpha": 0.35,
        "max_linear_step_m": 0.03,
        "max_angular_step_rad": 0.15,
        "openxr_to_base_quat": [0.5, -0.5, -0.5, 0.5],
        "gripper_min_width": 0.0,
        "gripper_max_width": 0.04,
        "gripper_speed_mps": 0.025,
    }


class ParameterValidationTest(unittest.TestCase):
    def test_accepts_reference_profile(self):
        _validate_retarget_parameters(**_valid_parameters())

    def test_rejects_invalid_scalar_contracts(self):
        cases = {
            "pose_source": "palm",
            "deadman_source": "button_99",
            "deadman_threshold": 1.1,
            "linear_scale": -1.0,
            "angular_scale": math.inf,
            "lowpass_alpha": 1.01,
            "max_linear_step_m": -0.01,
            "max_angular_step_rad": math.nan,
            "gripper_min_width": -0.01,
            "gripper_max_width": -0.05,
            "gripper_speed_mps": 0.0,
        }
        for name, value in cases.items():
            with self.subTest(name=name), self.assertRaises(ValueError):
                params = _valid_parameters()
                params[name] = value
                _validate_retarget_parameters(**params)

    def test_rejects_invalid_basis_rotation(self):
        for name, value in (
            ("openxr_to_base_quat", [0.0, 0.0]),
            ("openxr_to_base_quat", [0.0, math.nan, 0.0, 1.0]),
            ("openxr_to_base_quat", [0.0, 0.0, 0.0, 2.0]),
        ):
            with self.subTest(name=name), self.assertRaises(ValueError):
                params = _valid_parameters()
                params[name] = value
                _validate_retarget_parameters(**params)


if __name__ == "__main__":
    unittest.main()
