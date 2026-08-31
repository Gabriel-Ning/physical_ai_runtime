from copy import deepcopy

import pytest
from rmi import (
    ActionTimestampRebaser,
    McapActionSource,
    RecordedAction,
    ReplayClockJumpError,
    ReplayPacer,
    ReplayPlayer,
)
from trajectory_msgs.msg import JointTrajectory


def _trajectory(stamp_ns=123):
    message = JointTrajectory()
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.header.frame_id = "base"
    return message


def test_rebaser_copies_payload_and_preserves_relative_timeline():
    original = _trajectory()
    rebaser = ActionTimestampRebaser()
    rebaser.start(source_origin_ns=10, ros_origin_ns=1_000, steady_origin_ns=2_000)

    rewritten, deadline = rebaser.rewrite(original, 30)

    assert rewritten is not original
    assert rewritten.header.frame_id == "base"
    assert rewritten.header.stamp.nanosec == 1_020
    assert deadline == 2_020
    assert original == _trajectory()


class FakeSource:
    def __init__(self, actions):
        self.items = actions
        self.opened = 0
        self.closed = 0
        self.resets = 0

    def open(self):
        self.opened += 1

    def reset(self):
        self.resets += 1

    def close(self):
        self.closed += 1

    def actions(self):
        for action in self.items:
            yield deepcopy(action)


def test_player_replays_at_one_x():
    emitted = []
    sleeps = []
    source = FakeSource(
        [RecordedAction(100, _trajectory()), RecordedAction(200, _trajectory())]
    )
    player = ReplayPlayer(
        source,
        ros_clock_ns=lambda: 1_000,
        steady_clock_ns=lambda: 2_000,
        sleep=sleeps.append,
    )
    player.open()
    emitted.extend(player)
    player.close()

    assert source.opened == source.resets == source.closed == 1
    assert len(emitted) == 2
    assert emitted[0].header.stamp.nanosec == 1_000
    assert emitted[1].header.stamp.nanosec == 1_100
    assert sleeps == [0.0000001]


def test_sim_replay_waits_for_clock_start_and_pause_then_resumes():
    clock_values = iter([0, 0, 1_000, 1_000, 1_000, 1_100])
    sleeps = []
    pacer = ReplayPacer(
        ros_clock_ns=lambda: next(clock_values),
        use_sim_time=True,
        sleep=sleeps.append,
        poll_interval_s=0.01,
    )

    assert pacer.start() == 1_000
    pacer.wait_until(0.0000001)

    assert sleeps == [0.01, 0.01, 0.01, 0.01]


def test_sim_replay_rejects_backward_clock_jump():
    clock_values = iter([1_000, 900])
    pacer = ReplayPacer(
        ros_clock_ns=lambda: next(clock_values),
        use_sim_time=True,
        sleep=lambda _: None,
    )
    pacer.start()

    with pytest.raises(ReplayClockJumpError, match="moved backwards"):
        pacer.wait_until(0.0000001)


def test_player_follows_sim_clock_instead_of_steady_clock():
    clock_values = iter([1_000, 1_000, 1_000, 1_100])
    sleeps = []
    source = FakeSource(
        [RecordedAction(100, _trajectory()), RecordedAction(200, _trajectory())]
    )
    player = ReplayPlayer(
        source,
        ros_clock_ns=lambda: next(clock_values),
        steady_clock_ns=lambda: 2_000,
        sleep=sleeps.append,
        use_sim_time=True,
        poll_interval_s=0.02,
    )
    player.open()

    emitted = list(player)

    assert [message.header.stamp.nanosec for message in emitted] == [1_000, 1_100]
    assert sleeps == [0.02]


class FakeReader:
    def __init__(self):
        self.messages = [("/actions", b"a", 10), ("/actions", b"b", 20)]

    def open(self, storage, converter):
        self.storage = storage
        self.converter = converter

    def get_all_topics_and_types(self):
        return [type("Topic", (), {"name": "/actions", "type": "test/Action"})()]

    def set_filter(self, value):
        self.filter = value

    def has_next(self):
        return bool(self.messages)

    def read_next(self):
        return self.messages.pop(0)


def test_mcap_source_exposes_explicit_record_time_and_native_payload():
    source = McapActionSource(
        "/episodes/one",
        "/actions",
        reader_factory=FakeReader,
        message_type_resolver=lambda _name: bytes,
        deserializer=lambda value, _type: value.decode(),
    )
    source.open()
    result = list(source.actions())
    source.close()

    assert [(item.source_time_ns, item.payload) for item in result] == [
        (10, "a"),
        (20, "b"),
    ]
