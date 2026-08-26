from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType, SimpleNamespace
from typing import ClassVar

import numpy as np
import pytest
from rmi import Observation
from rmi.sensing import TimestampedSample

from policy_inference.common.contract import PolicyIOContract
from policy_inference.lerobot.bridge import (
    LeRobotToRmiActionBridge,
    RmiToLeRobotObservationBridge,
    ros_image_to_numpy,
)


@dataclass
class _Part:
    joint_names: tuple[str, ...]


@dataclass
class _Agent:
    resources: dict[str, str]
    frequency: float = 30.0


@dataclass
class _Camera:
    ros_topic: str


class _Profile:
    name = "test_bimanual"
    parts: ClassVar[dict[str, _Part]] = {
        "left_arm": _Part(("left_1", "left_2")),
        "left_gripper": _Part(("left_gripper",)),
        "right_arm": _Part(("right_1", "right_2")),
    }
    agents: ClassVar[dict[str, _Agent]] = {
        "Policy": _Agent(
            {
                "left_arm": "joint_reference",
                "left_gripper": "joint_reference",
                "right_arm": "joint_reference",
            }
        )
    }
    cameras: ClassVar[dict[str, _Camera]] = {
        "wrist": _Camera("/wrist/image_raw")
    }
    features: ClassVar[dict[str, dict]] = {
        "observation": {
            "observation.images.wrist": {
                "type": "image",
                "shape": [3, 8, 8],
                "source": "sensors.cameras.wrist",
            }
        },
        "action": {"action": {"shape": [5]}},
    }

    @staticmethod
    def profile_hash() -> str:
        return "profile-digest"


def test_contract_resolves_order_once_and_splits_action() -> None:
    contract = PolicyIOContract.from_profile(_Profile())

    assert contract.action_feature_names == (
        "left_1.pos",
        "left_2.pos",
        "left_gripper.pos",
        "right_1.pos",
        "right_2.pos",
    )
    assert contract.camera_shapes == {"observation.images.wrist": (8, 8, 3)}
    assert contract.camera_sources == {"observation.images.wrist": "wrist"}
    assert [values for _, values in contract.split_action(np.arange(5.0))] == [
        [0.0, 1.0],
        [2.0],
        [3.0, 4.0],
    ]


def test_contract_rejects_profile_shape_mismatch() -> None:
    profile = _Profile()
    profile.features = {"action": {"action": {"shape": [4]}}}
    with pytest.raises(ValueError, match="resolved action dimension 5"):
        PolicyIOContract.from_profile(profile)


def _observation(image: np.ndarray, *, camera_receive_time: float = 1.1) -> Observation:
    sample = TimestampedSample(
        value=image,
        source_time_s=1.0,
        receive_time_s=camera_receive_time,
        sequence=1,
    )
    return Observation(
        data={
            "joint_names": (
                "right_2",
                "left_gripper",
                "left_1",
                "right_1",
                "left_2",
            ),
            "joint_positions": (4.0, 2.0, 0.0, 3.0, 1.0),
        },
        source_time_s=1.0,
        receive_time_s=1.0,
        sensors=MappingProxyType({"wrist": sample}),
    )


def test_observation_bridge_orders_joints_and_reuses_rmi_camera_payload() -> None:
    from lerobot.utils.constants import OBS_STR
    from lerobot.utils.feature_utils import build_dataset_frame

    from policy_inference.lerobot.utils import make_dataset_features

    contract = PolicyIOContract.from_profile(_Profile())
    bridge = RmiToLeRobotObservationBridge(contract)
    image = np.zeros((8, 8, 3), dtype=np.uint8)

    values = bridge.encode(_observation(image))
    frame = build_dataset_frame(
        make_dataset_features(contract), values, prefix=OBS_STR
    )

    assert [values[name] for name in contract.state_feature_names] == list(range(5))
    assert values["wrist"] is image
    assert frame["observation.state"].tolist() == list(range(5))
    assert frame["observation.images.wrist"] is image


def test_observation_bridge_rejects_missing_sensor_and_stream_skew() -> None:
    contract = PolicyIOContract.from_profile(_Profile())
    bridge = RmiToLeRobotObservationBridge(contract, max_stream_skew_s=0.5)
    image = np.zeros((8, 8, 3), dtype=np.uint8)
    missing = _observation(image)
    missing = Observation(
        data=missing.data,
        source_time_s=missing.source_time_s,
        receive_time_s=missing.receive_time_s,
    )

    with pytest.raises(RuntimeError, match="missing sensor 'wrist'"):
        bridge.encode(missing)
    with pytest.raises(RuntimeError, match="freshness window"):
        bridge.encode(_observation(image, camera_receive_time=2.0))


def test_action_bridge_emits_one_native_rmi_action_per_profile_resource() -> None:
    contract = PolicyIOContract.from_profile(_Profile())

    actions = LeRobotToRmiActionBridge(contract).decode(np.arange(5.0))

    assert [(action.part, action.value) for action in actions] == [
        ("left_arm", [0.0, 1.0]),
        ("left_gripper", [2.0]),
        ("right_arm", [3.0, 4.0]),
    ]


def test_action_bridge_rejects_bad_shape_and_nonfinite_values() -> None:
    contract = PolicyIOContract.from_profile(_Profile())
    bridge = LeRobotToRmiActionBridge(contract)

    with pytest.raises(ValueError, match="action shape"):
        bridge.decode(np.zeros(4))
    with pytest.raises(ValueError, match="NaN or Inf"):
        bridge.decode(np.array([0.0, 1.0, 2.0, 3.0, np.nan]))


def test_ros_image_bridge_outputs_contiguous_rgb() -> None:
    message = SimpleNamespace(
        encoding="bgr8",
        height=1,
        width=2,
        step=8,
        data=bytes([1, 2, 3, 4, 5, 6, 99, 99]),
    )

    image = ros_image_to_numpy(message)

    assert image.flags.c_contiguous
    assert image.tolist() == [[[3, 2, 1], [6, 5, 4]]]
