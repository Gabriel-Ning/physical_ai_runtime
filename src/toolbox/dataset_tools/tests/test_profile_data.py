from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace

import pytest
from rmi.config import EmbodimentConfig

from toolbox.dataset_tools.contract import DatasetContract
from toolbox.dataset_tools.episode import ProfileEpisodeReader, ros_image_to_numpy


def _profile() -> EmbodimentConfig:
    return EmbodimentConfig.from_yaml("apps/profiles/piper_bimanual.yaml")


def _camera_shape_contract(shape: tuple[int, int, int]) -> DatasetContract:
    contract = DatasetContract.from_profile(_profile())
    policy = replace(
        contract.policy,
        camera_shapes={name: shape for name in contract.policy.camera_shapes},
    )
    return replace(contract, policy=policy)


def test_dataset_contract_uses_profile_execution_and_policy_names() -> None:
    contract = DatasetContract.from_profile(_profile())

    assert contract.state_topic == "/joint_states"
    assert contract.action_topics["left_arm"] == "/execution/left_arm/joint_reference"
    assert tuple(contract.policy.camera_topics) == (
        "observation.images.top",
        "observation.images.left_wrist",
        "observation.images.right_wrist",
    )


def test_raw_ros_image_handles_row_padding_and_bgr() -> None:
    message = SimpleNamespace(
        encoding="bgr8",
        height=1,
        width=2,
        step=8,
        data=bytes([1, 2, 3, 4, 5, 6, 99, 99]),
    )

    image = ros_image_to_numpy(message)

    assert image.shape == (1, 2, 3)
    assert image.tolist() == [[[3, 2, 1], [6, 5, 4]]]


class _Reader:
    def __init__(self, messages):
        self._messages = messages
        self.image_deserializations = 0

    def topic_types(self):
        return {topic: "test/Message" for topic, _, _ in self._messages}

    def raw_messages(self, topics):
        selected = set(topics)
        yield from (item for item in self._messages if item[0] in selected)

    def deserialize(self, topic, serialized, topic_types):
        assert topic_types[topic] == "test/Message"
        if hasattr(serialized, "encoding"):
            self.image_deserializations += 1
        return serialized


def test_episode_reader_causally_emits_complete_profile_frame() -> None:
    contract = _camera_shape_contract((1, 1, 3))
    names = [name.removesuffix(".pos") for name in contract.policy.state_feature_names]
    joint_state = SimpleNamespace(name=names, position=list(range(14)))
    image_message = SimpleNamespace(
        encoding="rgb8",
        height=1,
        width=1,
        step=3,
        data=bytes([1, 2, 3]),
    )
    messages = [(contract.state_topic, joint_state, 0)]
    for group in contract.policy.action_groups:
        point = SimpleNamespace(positions=[0.1] * len(group.joint_names))
        command = SimpleNamespace(points=[point], joint_names=group.joint_names)
        messages.append((contract.action_topics[group.part], command, 50_000_000))
    for topic in contract.policy.camera_topics.values():
        messages.extend(
            [(topic, image_message, 0), (topic, image_message, 100_000_000)]
        )
    messages.append((contract.state_topic, joint_state, 100_000_000))
    messages.sort(key=lambda item: item[2])

    reader = ProfileEpisodeReader(_Reader(messages), contract)
    frames = list(reader.frames())
    frame = frames[0]

    assert frame.values["observation.state"].shape == (14,)
    assert frame.values["action"].shape == (14,)
    assert frame.values["observation.images.top"].tolist() == [[[1, 2, 3]]]
    # Before the first command, action is a hold initialized from measured state.
    assert frame.values["action"].tolist() == list(map(float, range(14)))
    assert frames[-1].values["action"].tolist() == pytest.approx([0.1] * 14)


def test_episode_reader_decodes_only_images_needed_by_the_next_frame() -> None:
    contract = _camera_shape_contract((1, 1, 3))
    names = [name.removesuffix(".pos") for name in contract.policy.state_feature_names]
    joint_state = SimpleNamespace(name=names, position=list(range(14)))
    image = SimpleNamespace(
        encoding="rgb8", height=1, width=1, step=3, data=bytes([1, 2, 3])
    )
    messages = [
        (contract.state_topic, joint_state, 0),
        (contract.state_topic, joint_state, 100_000_000),
    ]
    for group in contract.policy.action_groups:
        point = SimpleNamespace(positions=[0.1] * len(group.joint_names))
        command = SimpleNamespace(points=[point], joint_names=group.joint_names)
        messages.append((contract.action_topics[group.part], command, 0))
    for topic in contract.policy.camera_topics.values():
        messages.extend([(topic, image, 0), (topic, image, 100_000_000)])
    messages.sort(key=lambda item: item[2])
    reader = _Reader(messages)
    frames = ProfileEpisodeReader(reader, contract).frames()

    next(frames)
    assert reader.image_deserializations == 3
    next(frames)
    assert reader.image_deserializations == 3


def test_episode_reader_rejects_camera_shape_mismatch_at_decode_boundary() -> None:
    contract = _camera_shape_contract((2, 2, 3))
    names = [name.removesuffix(".pos") for name in contract.policy.state_feature_names]
    joint_state = SimpleNamespace(name=names, position=list(range(14)))
    image = SimpleNamespace(
        encoding="rgb8", height=1, width=1, step=3, data=bytes([1, 2, 3])
    )
    messages = [
        (contract.state_topic, joint_state, 0),
        (contract.state_topic, joint_state, 100_000_000),
    ]
    for group in contract.policy.action_groups:
        point = SimpleNamespace(positions=[0.1] * len(group.joint_names))
        command = SimpleNamespace(points=[point], joint_names=group.joint_names)
        messages.append((contract.action_topics[group.part], command, 0))
    for topic in contract.policy.camera_topics.values():
        messages.extend([(topic, image, 0), (topic, image, 100_000_000)])
    messages.sort(key=lambda item: item[2])

    with pytest.raises(ValueError, match="does not match Profile"):
        next(ProfileEpisodeReader(_Reader(messages), contract).frames())
