from collections import deque
from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest
from rmi import Action

from policy_inference.lerobot.bridges.action import CartesianActionDecoder
from policy_inference.lerobot.engines.local import LocalAsyncBackend, LocalSyncBackend
from policy_inference.lerobot.engines.remote import RemoteBackend
from policy_inference.lerobot.handover import ChunkHandover
from policy_inference.lerobot.policy import LeRobotPolicy


def layout(cartesian=False):
    return NS(
        frequency=10.0,
        pose_part='tcp' if cartesian else None,
        gripper_parts=(),
        joints=NS(groups=[NS(part='arm', joint_names=['j'])]),
    )


def obs(q=0.0, v=0.0, quaternion=(1.0, 0.0, 0.0, 0.0)):
    return NS(
        joint_names=['j'],
        joint_positions=[q],
        joint_velocities=[v],
        sensors={'tcp': NS(value=NS(position_xyz=[q, 0.0, 0.0], orientation_wxyz=quaternion))},
    )


def frame(q):
    return (Action('arm', 'joint_reference', [q]),)


def test_forward_direction_disambiguates_nearby_points():
    plan = ChunkHandover(layout())
    assert plan.start(tuple(frame(x) for x in (0.01, -0.1, 0.011, 0.1)), obs(0.0, 0.1), 0.0)
    assert plan.join_index == 2  # Index 0 is spatially closer but moves backwards.


def test_quintic_bridge_matches_endpoint_positions_and_velocities():
    plan = ChunkHandover(layout())
    assert plan.start((frame(0.1), frame(0.12)), obs(0.0, 0.05), 0.0)
    assert plan.join_index == 0
    duration = plan.bridge_duration
    eps = 1e-5
    start = plan.select_action(0.0)[0].value[0]
    near_start = plan.select_action(eps)[0].value[0]
    near_end = plan.select_action(duration - eps)[0].value[0]
    end = plan.select_action(duration)[0].value[0]
    assert start == 0.0
    assert end == pytest.approx(0.1)
    assert (near_start - start) / eps == pytest.approx(0.05, abs=1e-4)
    assert (end - near_end) / eps == pytest.approx(0.2, abs=1e-4)
    assert plan.select_action(duration + 0.1)[0].value == [0.12]
    assert plan.select_action(duration + 0.2) is None


def test_cartesian_join_keeps_shortest_orientation_and_tail():
    plan = ChunkHandover(layout(True))
    frames = tuple(
        (
            Action(
                'tcp',
                'pose_reference',
                {
                    'position': [x, 0.0, 0.0],
                    'orientation': [-1.0, 0.0, 0.0, 0.0],
                },
            ),
        )
        for x in (0.1, 0.12)
    )
    assert plan.start(frames, obs(), 0.0)
    middle = plan.select_action(plan.bridge_duration / 2)[0]
    assert abs(middle.value['orientation'][0]) == pytest.approx(1.0)
    endpoint = plan.select_action(plan.bridge_duration)[0]
    np.testing.assert_allclose(endpoint.value['position'], [0.1, 0.0, 0.0])
    assert plan.select_action(plan.bridge_duration + 0.1) == frames[1]


def test_pose_join_uses_orientation_not_just_tcp_distance():
    plan = ChunkHandover(layout(True))
    frames = (
        (
            Action(
                'tcp',
                'pose_reference',
                {'position': [0.0, 0.0, 0.0], 'orientation': [0.0, 1.0, 0.0, 0.0]},
            ),
        ),
        (
            Action(
                'tcp',
                'pose_reference',
                {'position': [0.02, 0.0, 0.0], 'orientation': [1.0, 0.0, 0.0, 0.0]},
            ),
        ),
    )
    assert plan.start(frames, obs(), 0.0)
    assert plan.join_index == 1


def test_frozen_chunk_is_not_mutated_by_producer_updates():
    source = [frame(0.1), frame(0.12)]
    plan = ChunkHandover(layout())
    assert plan.start(source, obs(), 0.0)
    source[0][0].value[0] = 0.5
    assert plan.select_action(plan.bridge_duration)[0].value == pytest.approx([0.1])


def test_old_leased_commands_cannot_be_used_as_chunk_intents():
    plan = ChunkHandover(layout())
    with pytest.raises(ValueError, match='leased'):
        plan.start(((replace(frame(0.1)[0], _lease_id='revoked'),),), obs(), 0.0)


def test_no_feasible_point_is_rejected():
    plan = ChunkHandover(layout())
    assert not plan.start((frame(2.0),), obs(), 0.0)
    assert plan.reason == 'no_feasible_join'
    assert plan.select_action(0.5) is None


def test_sync_transfer_postprocesses_and_reorders_without_inference():
    import torch

    queue = deque([torch.tensor([[0.1, 0.2]]), torch.tensor([[0.3, 0.4]])])
    native = NS(
        _postprocessor=lambda a: a * 10,
        _dataset_features={'action': {'names': ['b', 'a']}},
        _ordered_action_keys=['a', 'b'],
    )
    backend = LocalSyncBackend(
        native,
        dataset_features={},
        action_key='action',
        policy=NS(_action_queue_attrs=('_action_queue',), _action_queue=queue),
    )
    np.testing.assert_allclose(backend.take_pending_actions(), [[2.0, 1.0], [4.0, 3.0]])
    assert not queue
    assert backend.take_pending_actions() == []


def test_rtc_transfer_consumes_tail_without_resetting_generation():
    import torch
    from lerobot.policies.rtc.action_queue import ActionQueue
    from lerobot.policies.rtc.configuration_rtc import RTCConfig

    queue = ActionQueue(RTCConfig())
    queue.queue = torch.tensor([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]])
    queue.last_index = 1
    native = NS(action_queue=queue, _reset_epoch=7)
    backend = LocalAsyncBackend(native)
    result = backend.take_pending_actions()
    np.testing.assert_allclose(np.stack(result), [[3.0, 4.0], [5.0, 6.0]])
    assert queue.empty() and queue.last_index == 3
    assert native._reset_epoch == 7


def test_remote_transfer_preserves_transport():
    backend = RemoteBackend.__new__(RemoteBackend)
    backend._queue = deque([[1.0, 2.0], [3.0, 4.0]])
    assert backend.take_pending_actions() == [[1.0, 2.0], [3.0, 4.0]]
    assert not backend._queue


def test_relative_cartesian_tail_keeps_original_chunk_anchor():
    cfg = NS(
        control_mode='cartesian',
        action_space='rel',
        action_dimension=6,
        pose_part='tcp',
        gripper_parts=(),
        joints=NS(groups=[]),
    )
    decoder = CartesianActionDecoder(cfg, position_scale=1.0, orientation_scale=1.0)
    decoder.set_current_pose([1.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])
    policy = LeRobotPolicy.__new__(LeRobotPolicy)
    policy._decoder = decoder
    policy._last_actions = decoder.decode([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    policy._backend = NS(take_pending_actions=lambda: [[0.2, 0.0, 0.0, 0.0, 0.0, 0.0]])
    frames = policy.take_action_chunk()
    assert frames[0][0].value['position'][0] == pytest.approx(1.1)
    assert frames[1][0].value['position'][0] == pytest.approx(1.3)
    assert policy.take_action_chunk() == ()
