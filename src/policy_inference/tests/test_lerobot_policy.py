from __future__ import annotations

import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest
from test_common_runtime import _layout

from policy_inference.lerobot.engines.remote import RemoteBackend, RemoteBackendConfig
from policy_inference.lerobot.policy import LeRobotPolicy
from policy_inference.lerobot.telemetry import ChunkTelemetry


def test_lerobot_policy_loader_import_is_lazy() -> None:
    code = """
import sys
import policy_inference.lerobot.policy
assert 'torch' not in sys.modules
assert 'lerobot.policies' not in sys.modules
"""
    subprocess.run([sys.executable, "-c", code], check=True)


class _Encoder:
    def encode(self, observation):
        assert observation is _OBSERVATION or getattr(
            observation, "sensors", None
        ) is not None
        return {"encoded": observation, "left_1.pos": 0.0}


class _Decoder:
    def __init__(self):
        self.poses = []

    def decode(self, action):
        return ("decoded", action)

    def set_current_pose(self, position_xyz, orientation_wxyz):
        self.poses.append((position_xyz, orientation_wxyz))


class _SyncBackend:
    mode = "sync"

    def __init__(self, action):
        self.action = action
        self.raws = []
        self.reset_count = 0
        self.stop_count = 0
        self.failed = False
        self.failure_traceback = None
        self.task = None
        self.paused = False
        self.weights = None

    def start(self):
        return None

    def stop(self):
        self.stop_count += 1

    def reset(self):
        self.reset_count += 1

    def detect_replan(self):
        return True

    def remaining_actions(self):
        return []

    def set_task(self, task):
        self.task = task

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False

    def update_weights(self, state_dict):
        self.weights = state_dict

    def step(self, raw):
        self.raws.append(raw)
        return self.action


class _AsyncBackend(_SyncBackend):
    mode = "async"

    def __init__(self, action):
        super().__init__(action)
        self.notified = []

    def step(self, raw):
        self.notified.append(raw)
        self.raws.append(None)
        return self.action


_OBSERVATION = object()


def _policy(action, *, inference: str = "sync"):
    policy = LeRobotPolicy.__new__(LeRobotPolicy)
    policy.layout = _layout()
    policy.task = ""
    policy.inference = inference
    policy.bundle = None
    policy.compatibility = None
    policy._encoder = _Encoder()
    policy._decoder = _Decoder()
    policy._dataset_features = {}
    policy._backend = (
        _AsyncBackend(action) if inference == "async" else _SyncBackend(action)
    )
    policy._telemetry = ChunkTelemetry()
    policy._closed = False
    return policy


def test_policy_exposes_one_select_action_boundary() -> None:
    action = np.arange(5.0)
    policy = _policy(action)

    result = policy.select_action(_OBSERVATION)

    assert result == ("decoded", action)
    assert policy._backend.raws[0]["encoded"] is _OBSERVATION
    assert policy.last_raw_action.tolist() == list(range(5))
    assert policy.last_chunk_index == 0


def test_policy_preserves_no_action_and_owns_only_engine_lifecycle() -> None:
    policy = _policy(None)

    assert policy.select_action(_OBSERVATION) is None
    policy.reset()
    policy.close()
    policy.close()

    assert policy._backend.reset_count == 1
    assert policy._backend.stop_count == 1
    assert policy.last_raw_action is None
    assert policy.last_chunk_index == -1


def test_policy_anchors_the_decoder_from_observation_tcp_at_chunk_boundaries() -> None:
    from rmi.sensing import PoseSample, TimestampedSample

    policy = _policy(np.arange(5.0))
    policy.layout = SimpleNamespace(control_mode="cartesian", pose_part="arm")
    policy._backend.detect_replan = lambda: False
    pose = PoseSample(
        position_xyz=(1.0, 2.0, 3.0),
        orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
        frame_id="base",
        child_frame_id="tcp",
    )
    observation = SimpleNamespace(
        sensors={
            "arm": TimestampedSample(
                value=pose,
                source_time_s=1.0,
                receive_time_s=1.0,
                sequence=1,
            )
        }
    )

    policy.select_action(observation)
    # Second tick: not a replan and chunk already started → no re-anchor.
    pose2 = PoseSample(
        position_xyz=(9.0, 9.0, 9.0),
        orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
        frame_id="base",
        child_frame_id="tcp",
    )
    observation.sensors["arm"] = TimestampedSample(
        value=pose2,
        source_time_s=1.0,
        receive_time_s=1.0,
        sequence=2,
    )
    policy.select_action(observation)

    assert [pose[0] for pose in policy._decoder.poses] == [(1.0, 2.0, 3.0)]


def test_async_policy_steps_backend() -> None:
    policy = _policy(np.arange(5.0), inference="async")

    result = policy.select_action(_OBSERVATION)

    assert result == ("decoded", policy._backend.action)
    assert len(policy._backend.notified) == 1
    assert policy._backend.raws == [None]


def _sync_backend(policy):
    from policy_inference.lerobot.engines.local import LocalSyncBackend

    return LocalSyncBackend(
        object(), dataset_features={}, action_key="action", policy=policy
    )


def test_sync_backend_reads_both_lerobot_queue_conventions() -> None:
    """LeRobot declares two chunk-queue layouts; both must be detected.

    A dict queue covers diffusion/smolvla/pi0/vla_jepa; a bare deque covers
    act/groot/eo1. Reading only ``_queues`` silently reports "never replanned"
    for the deque family.
    """
    from collections import deque

    dict_style = SimpleNamespace(
        _action_queue_attrs=("_queues", "_action_queue"),
        _queues={"action": deque([np.zeros(5)])},
    )
    deque_style = SimpleNamespace(
        _action_queue_attrs=("_queues", "_action_queue"),
        _action_queue=deque([np.zeros(5), np.ones(5)]),
    )

    assert _sync_backend(dict_style).detect_replan() is False
    assert len(_sync_backend(dict_style).remaining_actions()) == 1
    assert _sync_backend(deque_style).detect_replan() is False
    assert len(_sync_backend(deque_style).remaining_actions()) == 2

    deque_style._action_queue.clear()
    assert _sync_backend(deque_style).detect_replan() is True


def test_sync_backend_treats_a_queueless_policy_as_always_replanning() -> None:
    """ACT with temporal ensembling keeps no queue: every tick runs the model."""
    backend = _sync_backend(SimpleNamespace(_action_queue_attrs=("_action_queue",)))

    assert backend.detect_replan() is True
    assert backend.remaining_actions() is None


def test_normalize_inference_mode_aliases() -> None:
    from policy_inference.lerobot.engines import normalize_inference_mode

    assert normalize_inference_mode("sync") == "sync"
    assert normalize_inference_mode("async") == "async"
    assert normalize_inference_mode("rtc") == "async"
    assert normalize_inference_mode("remote") == "remote"


class _FakeTransport:
    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.connected = False
        self.closed = False

    def connect(self):
        self.connected = True

    def close(self):
        self.closed = True

    def request_action_chunk(self, raw_observation):
        assert "left_1.pos" in raw_observation or "encoded" in raw_observation
        if not self.chunks:
            return None
        return self.chunks.pop(0)


def test_remote_backend_pops_chunk_across_ticks() -> None:
    transport = _FakeTransport(
        [[np.arange(5.0), np.arange(5.0) + 1.0]],
    )
    backend = RemoteBackend(
        transport, config=RemoteBackendConfig(actions_per_chunk=2, chunk_size_threshold=0.5)
    )
    backend.start()
    assert backend.detect_replan() is True
    first = backend.step({"left_1.pos": 0.0})
    assert first.tolist() == list(range(5))
    assert backend.detect_replan() is False
    second = backend.step({"left_1.pos": 0.0})
    assert second.tolist() == [1, 2, 3, 4, 5]
    assert backend.step({"left_1.pos": 0.0}) is None
    backend.stop()
    assert transport.closed


def test_policy_remote_select_action() -> None:
    transport = _FakeTransport([[np.arange(5.0)]])
    backend = RemoteBackend(
        transport, config=RemoteBackendConfig(actions_per_chunk=1)
    )
    policy = _policy(None, inference="remote")
    policy.inference = "remote"
    policy._backend = backend
    backend.start()
    try:
        result = policy.select_action(_OBSERVATION)
        assert result[0] == "decoded"
        assert np.allclose(result[1], np.arange(5.0))
        assert policy.last_raw_action.tolist() == list(range(5))
    finally:
        policy.close()
        assert transport.closed


def test_policy_lifecycle_and_weight_update() -> None:
    backend = _SyncBackend(np.zeros(5))
    policy = _policy(None, inference="sync")
    policy._backend = backend
    policy.task = "initial_task"

    policy.set_task("pick_cube")
    assert policy.task == "pick_cube"
    assert backend.task == "pick_cube"

    with pytest.raises(ValueError):
        policy.set_task("   ")

    policy.pause()
    assert backend.paused is True
    policy.resume()
    assert backend.paused is False

    weights = {"weight1": np.ones((3, 3))}
    policy.update_weights(weights)
    assert backend.weights is weights

    policy.close()
    with pytest.raises(RuntimeError):
        policy.update_weights(weights)


def test_hil_transition_collector() -> None:
    from policy_inference.lerobot.bridges.hil_collector import (
        HILTransition,
        HILTransitionCollector,
    )

    collector = HILTransitionCollector(max_buffer_size=10)
    s0 = {"left_1.pos": 0.0}
    s1 = {"left_1.pos": 0.1}
    s2 = {"left_1.pos": 0.2}

    collector.start_episode(s0)
    t1 = collector.record_step(
        executed_action=[0.1, 0.2],
        reward=1.0,
        next_state=s1,
        done=False,
        is_intervened=False,
    )
    assert t1 is not None
    assert t1.intervened is False
    assert collector.pending_count == 1

    # Human intervention step
    t2 = collector.record_step(
        executed_action=[0.5, 0.6],
        reward=2.0,
        next_state=s2,
        done=True,
        is_intervened=True,
        info={"source": "teleop"},
    )
    assert t2 is not None
    assert t2.intervened is True
    assert collector.pending_count == 2

    flushed = collector.flush()
    assert len(flushed) == 2
    assert flushed[0]["reward"] == 1.0
    assert flushed[1]["intervened"] is True
    assert collector.pending_count == 0


def test_lerobot_remote_transport_grpc() -> None:
    import pickle
    import torch
    from unittest.mock import MagicMock
    from lerobot.async_inference.helpers import TimedAction
    from lerobot.transport import services_pb2
    from policy_inference.lerobot.engines.remote import LeRobotRemoteTransport

    mock_stub = MagicMock()
    mock_stub.Ready.return_value = services_pb2.Empty()
    mock_stub.SendPolicyInstructions.return_value = services_pb2.Empty()
    mock_stub.SendObservations.return_value = services_pb2.Empty()

    action_tensor = torch.tensor([1.0, 2.0, 3.0], dtype=torch.float32)
    timed_action = TimedAction(timestamp=0.0, timestep=1, action=action_tensor)
    actions_payload = pickle.dumps([timed_action])
    mock_stub.GetActions.return_value = services_pb2.Actions(data=actions_payload)

    transport = LeRobotRemoteTransport("localhost:50051", pretrained_name_or_path="mock_model")
    transport._stub = mock_stub
    transport._connected = True

    chunk = transport.request_action_chunk({"left_1.pos": 0.0})
    assert chunk is not None
    assert len(chunk) == 1
    assert np.allclose(chunk[0], [1.0, 2.0, 3.0])
    assert mock_stub.SendObservations.called
    assert mock_stub.GetActions.called


def test_lerobot_learner_client_grpc() -> None:
    import torch
    from unittest.mock import MagicMock
    from lerobot.transport import services_pb2
    from lerobot.transport.utils import state_to_bytes
    from policy_inference.lerobot.engines.learner_client import LeRobotLearnerClient

    mock_stub = MagicMock()
    mock_stub.Ready.return_value = services_pb2.Empty()
    mock_stub.SendTransitions.return_value = services_pb2.Empty()
    mock_stub.SendInteractions.return_value = services_pb2.Empty()

    test_weights = {"fc.weight": torch.ones(2, 2)}
    payload = state_to_bytes(test_weights)
    from lerobot.transport.utils import TransferState
    stream = MagicMock()
    stream.__iter__.return_value = iter([
        services_pb2.Parameters(transfer_state=TransferState.TRANSFER_BEGIN, data=payload[:10]),
        services_pb2.Parameters(transfer_state=TransferState.TRANSFER_MIDDLE, data=payload[10:20]),
        services_pb2.Parameters(transfer_state=TransferState.TRANSFER_END, data=payload[20:]),
    ])
    mock_stub.StreamParameters.return_value = stream

    client = LeRobotLearnerClient("localhost:50052")
    client._stub = mock_stub
    client._connected = True

    fetched = client.fetch_policy_parameters()
    stream.cancel.assert_called_once()
    assert fetched is not None
    assert "fc.weight" in fetched
    assert torch.equal(fetched["fc.weight"], torch.ones(2, 2))

    client.send_transitions([{"obs": [0.1], "action": [0.2], "reward": 1.0, "done": False}])
    assert mock_stub.SendTransitions.called

    client.send_interactions({"event": "human_intervention_start"})
    assert mock_stub.SendInteractions.called


def test_remote_backend_refill_replaces_stale_queue() -> None:
    # First chunk has 3 actions [0, 0], [1, 1], [2, 2]
    # Second chunk has 2 actions [10, 10], [11, 11]
    transport = _FakeTransport([
        [np.array([0.0, 0.0]), np.array([1.0, 1.0]), np.array([2.0, 2.0])],
        [np.array([10.0, 10.0]), np.array([11.0, 11.0])],
    ])
    # threshold=0.7: after 1 pop (len=2, ratio=2/3=0.67 <= 0.7), refill triggers!
    backend = RemoteBackend(
        transport, config=RemoteBackendConfig(actions_per_chunk=3, chunk_size_threshold=0.7)
    )
    backend.start()
    first = backend.step({"left_1.pos": 0.0})
    assert first.tolist() == [0.0, 0.0]

    # At second step, ratio=2/3 <= 0.7, so refill arrives. Old [1, 1] and [2, 2] are replaced by [10, 10]
    second = backend.step({"left_1.pos": 0.0})
    assert second.tolist() == [10.0, 10.0]
    third = backend.step({"left_1.pos": 0.0})
    assert third.tolist() == [11.0, 11.0]
    backend.stop()


def test_policy_encode_observation() -> None:
    backend = _SyncBackend(np.zeros(5))
    policy = _policy(None, inference="sync")
    policy._backend = backend
    encoded = policy.encode_observation(_OBSERVATION)
    assert "left_1.pos" in encoded
    assert encoded["left_1.pos"] == 0.0







def test_remote_refill_reanchors_only_when_a_new_chunk_arrives():
    transport = _FakeTransport([[np.zeros(7)] * 4, None, [np.ones(7)] * 4])
    backend = RemoteBackend(transport, config=RemoteBackendConfig(actions_per_chunk=4))
    policy = _policy(None, inference="remote")
    policy.layout = SimpleNamespace(control_mode="cartesian", pose_part="arm")
    policy._backend = backend
    backend.start()
    for x in (1.0, 2.0, 3.0, 4.0):
        obs = SimpleNamespace(sensors={"arm": SimpleNamespace(value=SimpleNamespace(
            position_xyz=(x, 0., 0.), orientation_wxyz=(1., 0., 0., 0.),
        ))})
        policy.select_action(obs)
        assert policy.last_was_replan is (x in (1.0, 4.0))
    assert [pose[0][0] for pose in policy._decoder.poses] == [1.0, 4.0]
    assert policy.last_chunk_index == 0
    policy.close()


@pytest.mark.parametrize("status", ["empty", "timeout", "unavailable"])
def test_learner_parameter_stream_cleanup(status):
    import grpc
    from unittest.mock import MagicMock
    from policy_inference.lerobot.engines.learner_client import LeRobotLearnerClient

    stream = MagicMock()
    if status == "empty":
        stream.__iter__.return_value = iter([])
    else:
        error = grpc.RpcError()
        error.code = lambda: (grpc.StatusCode.DEADLINE_EXCEEDED if status == "timeout"
                              else grpc.StatusCode.UNAVAILABLE)
        stream.__iter__.side_effect = error
    client = LeRobotLearnerClient("unused")
    client._connected = True
    client._stub = MagicMock()
    client._stub.StreamParameters.return_value = stream
    if status == "unavailable":
        with pytest.raises(grpc.RpcError):
            client.fetch_policy_parameters()
    else:
        assert client.fetch_policy_parameters() is None
    stream.cancel.assert_called_once()
