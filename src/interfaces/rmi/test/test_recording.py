from types import SimpleNamespace

import pytest
from rmi import Recorder


class FakeRecorder:
    def __init__(self):
        self.calls = []

    async def activate(self):
        self.calls.append(("activate",))

    async def start_recording(self, *, task, manifest_context):
        self.calls.append(("start", task, dict(manifest_context)))
        return SimpleNamespace(state="recording")

    async def stop_recording(self, *, timeout_s):
        self.calls.append(("stop", timeout_s))
        return SimpleNamespace(state="idle", episode_path="/episodes/episode_1")

    async def get_status(self):
        self.calls.append(("status",))
        return SimpleNamespace(state="idle")

    async def discard(self):
        self.calls.append(("discard",))
        return SimpleNamespace(state="idle")


def test_episode_scope_is_synchronous_and_forwards_per_episode_metadata():
    sdk = FakeRecorder()
    recorder = Recorder(sdk)

    with recorder.episode(
        task="pick",
        metadata={"policy": "diffusion"},
        stop_timeout=2.0,
    ) as episode:
        assert episode.started_status.state == "recording"

    assert episode.final_status.episode_path == "/episodes/episode_1"
    assert sdk.calls == [
        ("activate",),
        ("start", "pick", {"policy": "diffusion"}),
        ("stop", 2.0),
    ]


def test_episode_discards_when_application_body_raises():
    sdk = FakeRecorder()
    recorder = Recorder(sdk)

    with (
        pytest.raises(RuntimeError, match="policy failed"),
        recorder.episode(task="pick", stop_timeout=3.0) as episode,
    ):
        raise RuntimeError("policy failed")

    assert episode.discarded
    assert sdk.calls[-1] == ("discard",)


def test_episode_can_be_discarded_explicitly_without_finalizing():
    sdk = FakeRecorder()
    recorder = Recorder(sdk)

    with recorder.episode(task="pick", stop_timeout=3.0) as episode:
        status = episode.discard()
        assert status.state == "idle"

    assert episode.discarded
    assert sdk.calls == [
        ("activate",),
        ("start", "pick", {}),
        ("discard",),
    ]


def test_episode_cannot_be_discarded_outside_its_context():
    episode = Recorder(FakeRecorder()).episode(task="pick")

    with pytest.raises(RuntimeError, match="not active"):
        episode.discard()


def test_discard_failure_does_not_mask_application_error():
    class FailingDiscardRecorder(FakeRecorder):
        async def discard(self):
            raise RuntimeError("discard failed")

    recorder = Recorder(FailingDiscardRecorder())

    with (
        pytest.raises(RuntimeError, match="policy failed"),
        recorder.episode(task="pick", stop_timeout=3.0) as episode,
    ):
        raise RuntimeError("policy failed")

    assert not episode.discarded


def test_memory_replay_buffer_step_and_sample():
    from rmi.recording import MemoryReplayBuffer

    buffer = MemoryReplayBuffer(capacity=100)
    with buffer.episode(task="rl_task", metadata={"env": "fr3"}):
        for i in range(10):
            buffer.step(
                observation={"joint": [i * 0.1]},
                action={"joint": [i * 0.1 + 0.05]},
                reward=1.0 if i == 9 else 0.0,
                done=(i == 9),
            )

    assert len(buffer) == 10
    sample = buffer.sample(batch_size=4)
    assert len(sample) == 4
    assert all("observation" in item and "action" in item for item in sample)



def test_episode_validates_task_and_timeout_before_starting():
    recorder = Recorder(FakeRecorder())

    with pytest.raises(ValueError, match="task"):
        recorder.episode(task="")
    with pytest.raises(ValueError, match="stop_timeout"):
        recorder.episode(task="pick", stop_timeout=0.0)
