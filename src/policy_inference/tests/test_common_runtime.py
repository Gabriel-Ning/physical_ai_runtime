from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType, SimpleNamespace
from typing import ClassVar

import numpy as np
import pytest
from rmi import (
    JointGroup,
    JointLayout,
    Observation,
    PolicyLayout,
    ros_image_to_numpy,
)
from rmi.sensing import TimestampedSample

from policy_inference.lerobot.bridges import (
    JointActionDecoder,
    ObservationEncoder,
    make_action_decoder,
)


@dataclass
class _Part:
    joint_names: tuple[str, ...]


@dataclass
class _Node:
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
    nodes: ClassVar[dict[str, _Node]] = {
        "Policy": _Node(
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

    def joint_layout(self, node_name: str) -> JointLayout:
        node = self.nodes[node_name]
        groups = []
        names = []
        offset = 0
        for part_name, command in node.resources.items():
            joint_names = self.parts[part_name].joint_names
            stop = offset + len(joint_names)
            groups.append(
                JointGroup(part_name, command, joint_names, offset, stop)
            )
            names.extend(joint_names)
            offset = stop
        return JointLayout(tuple(groups), tuple(names))


def _layout() -> PolicyLayout:
    profile = _Profile()
    return PolicyLayout(
        profile_name=profile.name,
        profile_hash=profile.profile_hash(),
        joints=profile.joint_layout("Policy"),
        state_topic="/joint_states",
        action_topics={
            "left_arm": "/execution/left_arm/joint_reference",
            "left_gripper": "/execution/left_gripper/joint_reference",
            "right_arm": "/execution/right_arm/joint_reference",
        },
        camera_sources={"observation.images.wrist": "wrist"},
        camera_topics={"observation.images.wrist": "/wrist/image_raw"},
        camera_shapes={"observation.images.wrist": (8, 8, 3)},
        frequency=30.0,
    )


def test_policy_features_follow_rmi_joint_layout() -> None:
    layout = _layout()

    assert layout.state_feature_names == (
        "left_1.pos",
        "left_2.pos",
        "left_gripper.pos",
        "right_1.pos",
        "right_2.pos",
    )
    assert layout.camera_shapes == {"observation.images.wrist": (8, 8, 3)}
    assert layout.camera_sources == {"observation.images.wrist": "wrist"}
    assert [values for _, values in layout.joints.split_values(np.arange(5.0))] == [
        [0.0, 1.0],
        [2.0],
        [3.0, 4.0],
    ]


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


def test_observation_encoder_orders_joints_and_reuses_rmi_camera_payload() -> None:
    from lerobot.utils.constants import OBS_STR
    from lerobot.utils.feature_utils import build_dataset_frame

    from policy_inference.lerobot.features import make_dataset_features

    layout = _layout()
    encoder = ObservationEncoder(layout)
    image = np.zeros((8, 8, 3), dtype=np.uint8)

    values = encoder.encode(_observation(image))
    frame = build_dataset_frame(
        make_dataset_features(layout), values, prefix=OBS_STR
    )

    assert [values[name] for name in layout.state_feature_names] == list(range(5))
    assert values["wrist"] is image
    assert frame["observation.state"].tolist() == list(range(5))
    assert frame["observation.images.wrist"] is image


def test_observation_encoder_rejects_missing_sensor_and_stream_skew() -> None:
    layout = _layout()
    encoder = ObservationEncoder(layout, max_stream_skew_s=0.5)
    image = np.zeros((8, 8, 3), dtype=np.uint8)
    missing = _observation(image)
    missing = Observation(
        data=missing.data,
        source_time_s=missing.source_time_s,
        receive_time_s=missing.receive_time_s,
    )

    with pytest.raises(RuntimeError, match="missing sensor 'wrist'"):
        encoder.encode(missing)
    with pytest.raises(RuntimeError, match="freshness window"):
        encoder.encode(_observation(image, camera_receive_time=2.0))


def test_observation_encoder_cartesian_packs_tcp_and_gripper() -> None:
    from rmi.sensing import PoseSample

    layout = _cartesian_layout()
    # Expand state slots to LIBERO 8D (fake joint names matching packed length).
    layout = PolicyLayout(
        profile_name=layout.profile_name,
        profile_hash=layout.profile_hash,
        joints=JointLayout(
            (
                JointGroup(
                    "arm",
                    "pose_reference",
                    ("s0", "s1", "s2", "s3", "s4", "s5"),
                    0,
                    6,
                ),
                JointGroup(
                    "end_effector",
                    "joint_reference",
                    ("grip_l", "grip_r"),
                    6,
                    8,
                ),
            ),
            ("s0", "s1", "s2", "s3", "s4", "s5", "grip_l", "grip_r"),
        ),
        state_topic=layout.state_topic,
        action_topics=layout.action_topics,
        camera_sources={},
        camera_topics={},
        camera_shapes={},
        frequency=30.0,
        control_mode="cartesian",
        action_space="rel",
        pose_part="arm",
        gripper_parts=("end_effector",),
    )
    pose = PoseSample(
        position_xyz=(0.1, 0.2, 0.3),
        orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
        frame_id="base_link",
        child_frame_id="tcp",
    )
    observation = Observation(
        data={
            "joint_names": ("s0", "s1", "s2", "s3", "s4", "s5", "grip_l", "grip_r"),
            "joint_positions": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, -0.04),
        },
        source_time_s=1.0,
        receive_time_s=1.0,
        sensors=MappingProxyType(
            {
                "arm": TimestampedSample(
                    value=pose,
                    source_time_s=1.0,
                    receive_time_s=1.05,
                    sequence=1,
                )
            }
        ),
    )
    encoder = ObservationEncoder(layout)
    values = encoder.encode(observation)

    assert [values[name] for name in layout.state_feature_names] == pytest.approx(
        [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.04, -0.04]
    )


def test_observation_encoder_cartesian_includes_tcp_in_skew_gate() -> None:
    from rmi.sensing import PoseSample

    layout = PolicyLayout(
        profile_name="cart",
        profile_hash="h",
        joints=JointLayout(
            (
                JointGroup(
                    "arm",
                    "pose_reference",
                    ("s0", "s1", "s2", "s3", "s4", "s5"),
                    0,
                    6,
                ),
                JointGroup(
                    "end_effector",
                    "joint_reference",
                    ("grip_l", "grip_r"),
                    6,
                    8,
                ),
            ),
            ("s0", "s1", "s2", "s3", "s4", "s5", "grip_l", "grip_r"),
        ),
        state_topic="/joint_states",
        action_topics={
            "arm": "/execution/arm/pose_reference",
            "end_effector": "/execution/end_effector/joint_reference",
        },
        camera_sources={},
        camera_topics={},
        camera_shapes={},
        frequency=30.0,
        control_mode="cartesian",
        action_space="rel",
        pose_part="arm",
        gripper_parts=("end_effector",),
    )
    pose = PoseSample(
        position_xyz=(0.0, 0.0, 0.0),
        orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
        frame_id="base",
        child_frame_id="tcp",
    )
    observation = Observation(
        data={
            "joint_names": ("s0", "s1", "s2", "s3", "s4", "s5", "grip_l", "grip_r"),
            "joint_positions": (0.0,) * 8,
        },
        source_time_s=1.0,
        receive_time_s=1.0,
        sensors=MappingProxyType(
            {
                "arm": TimestampedSample(
                    value=pose,
                    source_time_s=1.0,
                    receive_time_s=2.0,
                    sequence=1,
                )
            }
        ),
    )
    encoder = ObservationEncoder(layout, max_stream_skew_s=0.5)
    with pytest.raises(RuntimeError, match="freshness window"):
        encoder.encode(observation)

def test_joint_decoder_emits_one_native_rmi_action_per_profile_resource() -> None:
    layout = _layout()

    decoder = make_action_decoder(layout)
    actions = decoder.decode(np.arange(5.0))

    assert isinstance(decoder, JointActionDecoder)
    assert [(action.part, action.value) for action in actions] == [
        ("left_arm", [0.0, 1.0]),
        ("left_gripper", [2.0]),
        ("right_arm", [3.0, 4.0]),
    ]


def test_joint_decoder_rejects_bad_shape_and_nonfinite_values() -> None:
    decoder = JointActionDecoder(_layout())

    with pytest.raises(ValueError, match="action shape"):
        decoder.decode(np.zeros(4))
    with pytest.raises(ValueError, match="NaN or Inf"):
        decoder.decode(np.array([0.0, 1.0, 2.0, 3.0, np.nan]))


def test_joint_gripper_unit_scale_roundtrip() -> None:
    layout = _layout()
    decoder = JointActionDecoder(
        layout, normalize_gripper=True, gripper_max_width=0.04
    )
    actions = decoder.decode(np.array([0.1, 0.2, 1.0, 0.3, 0.5]))
    by_part = {action.part: action.value for action in actions}
    assert by_part["left_arm"] == [pytest.approx(0.1), pytest.approx(0.2)]
    assert by_part["left_gripper"] == [pytest.approx(0.04)]
    assert by_part["right_arm"] == [pytest.approx(0.3), pytest.approx(0.5)]

    encoder = ObservationEncoder(
        layout, normalize_gripper=True, gripper_max_width=0.04
    )
    image = np.zeros((8, 8, 3), dtype=np.uint8)
    observation = _observation(image)
    # Override joint meters: left_gripper = 0.02 m → 0.5 after normalize.
    observation = Observation(
        data={
            "joint_names": (
                "right_2",
                "left_gripper",
                "left_1",
                "right_1",
                "left_2",
            ),
            "joint_positions": (4.0, 0.02, 0.0, 3.0, 1.0),
        },
        source_time_s=1.0,
        receive_time_s=1.0,
        sensors=observation.sensors,
    )
    encoded = encoder.encode(observation)
    assert encoded["left_1.pos"] == pytest.approx(0.0)
    assert encoded["left_gripper.pos"] == pytest.approx(0.5)
    assert encoded["right_2.pos"] == pytest.approx(4.0)


def test_ros_image_converter_outputs_contiguous_rgb() -> None:
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


def _cartesian_layout() -> PolicyLayout:
    joints = JointLayout(
        (
            JointGroup("arm", "pose_reference", ("j1", "j2"), 0, 2),
            JointGroup("end_effector", "joint_reference", ("grip",), 2, 3),
        ),
        ("j1", "j2", "grip"),
    )
    return PolicyLayout(
        profile_name="cart",
        profile_hash="h",
        joints=joints,
        state_topic="/joint_states",
        action_topics={
            "arm": "/execution/arm/pose_reference",
            "end_effector": "/execution/end_effector/joint_reference",
        },
        camera_sources={},
        camera_topics={},
        camera_shapes={},
        frequency=30.0,
        control_mode="cartesian",
        action_space="rel",
        pose_part="arm",
        gripper_parts=("end_effector",),
    )


def test_cartesian_decoder_integrates_rel_to_abs_pose() -> None:
    from policy_inference.lerobot.bridges import CartesianActionDecoder

    decoder = make_action_decoder(
        _cartesian_layout(),
        action_space="rel",
        gripper_max_width=0.04,
        position_scale=1.0,
    )
    assert isinstance(decoder, CartesianActionDecoder)
    decoder.set_current_pose([1.0, 2.0, 3.0], [1.0, 0.0, 0.0, 0.0])
    actions = decoder.decode(np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0]))

    assert actions[0].part == "arm"
    assert actions[0].command == "pose_reference"
    assert actions[0].value["position"] == pytest.approx([1.1, 2.0, 3.0])
    assert actions[1].part == "end_effector"
    assert actions[1].value == pytest.approx([0.04])


def test_cartesian_decoder_applies_position_scale() -> None:
    from policy_inference.lerobot.bridges import CartesianActionDecoder

    decoder = CartesianActionDecoder(
        _cartesian_layout(), action_space="rel", position_scale=0.05
    )
    decoder.set_current_pose([1.0, 2.0, 3.0], [1.0, 0.0, 0.0, 0.0])
    actions = decoder.decode(np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0]))
    assert actions[0].value["position"] == pytest.approx([1.05, 2.0, 3.0])


def test_cartesian_rel_requires_current_pose() -> None:
    from policy_inference.lerobot.bridges import CartesianActionDecoder

    decoder = CartesianActionDecoder(_cartesian_layout(), action_space="rel")
    with pytest.raises(RuntimeError, match="set_current_pose"):
        decoder.decode(np.zeros(7))


def test_joint_decoder_ignores_the_optional_pose_hook() -> None:
    decoder = JointActionDecoder(_layout())

    decoder.set_current_pose([1.0, 2.0, 3.0], [1.0, 0.0, 0.0, 0.0])

    assert [action.part for action in decoder.decode(np.arange(5.0))] == [
        "left_arm",
        "left_gripper",
        "right_arm",
    ]


def test_pack_libero_ee_state_identity_quat_and_single_gripper() -> None:
    from policy_inference.lerobot.geometry import (
        pack_libero_ee_state,
        quat_wxyz_to_axis_angle,
    )

    assert quat_wxyz_to_axis_angle([1.0, 0.0, 0.0, 0.0]).tolist() == pytest.approx(
        [0.0, 0.0, 0.0]
    )
    state = pack_libero_ee_state([0.1, 0.2, 0.3], [1.0, 0.0, 0.0, 0.0], [0.04])
    assert state.shape == (8,)
    assert state[:3].tolist() == pytest.approx([0.1, 0.2, 0.3])
    assert state[3:6].tolist() == pytest.approx([0.0, 0.0, 0.0])
    assert state[6:].tolist() == pytest.approx([0.04, -0.04])
