from fakes import FakeRecorder
from orchestrator.blackboard import Blackboard
from orchestrator.nodes.recorder import RecordEpisode
from orchestrator.nodes.task import Fail, Succeed
from orchestrator.tree import Node, NodeContext, Status


def make_context(recorder):
    return NodeContext(robot=None, blackboard=Blackboard(), recorder=recorder)


class RunOnceThenSucceed(Node):
    def __init__(self, name=None, **config):
        super().__init__(name, **config)
        self.ticks = 0

    def update(self):
        self.ticks += 1
        return Status.RUNNING if self.ticks < 2 else Status.SUCCESS


def test_record_episode_wraps_child_and_finalizes_on_success():
    recorder = FakeRecorder()
    child = RunOnceThenSucceed()
    node = RecordEpisode(child=child, task="pick")
    node.bind(make_context(recorder))

    assert node.tick() is Status.RUNNING
    scope = recorder.episodes[0]
    assert scope.entered and not scope.exited

    assert node.tick() is Status.SUCCESS
    assert scope.exited
    assert scope.kwargs["task"] == "pick"


def test_record_episode_still_finalizes_on_child_failure():
    recorder = FakeRecorder()
    node = RecordEpisode(child=Fail(), task="pick")
    node.bind(make_context(recorder))

    assert node.tick() is Status.FAILURE
    scope = recorder.episodes[0]
    assert scope.entered and scope.exited


def test_record_episode_requires_recorder():
    node = RecordEpisode(child=Succeed(), task="pick")
    node.bind(make_context(None))
    try:
        node.tick()
    except RuntimeError:
        pass
    else:
        raise AssertionError("expected RuntimeError when no recorder is configured")
