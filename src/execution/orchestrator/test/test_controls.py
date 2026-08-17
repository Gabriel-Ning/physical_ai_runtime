from orchestrator.blackboard import Blackboard
from orchestrator.controls import Fallback, Parallel, Retry, Sequence
from orchestrator.nodes.task import Fail, Succeed
from orchestrator.tree import Node, NodeContext, Status


def make_context() -> NodeContext:
    return NodeContext(robot=None, blackboard=Blackboard())


class Scripted(Node):
    """Returns a scripted sequence of statuses, one per tick (index persists
    across separate runs so Retry-style re-initialise still advances)."""

    def __init__(self, name=None, *, statuses, **config):
        super().__init__(name, **config)
        self._statuses = list(statuses)
        self._i = 0
        self.ticks = 0

    def update(self):
        self.ticks += 1
        status = self._statuses[min(self._i, len(self._statuses) - 1)]
        self._i += 1
        return status


def test_sequence_all_success():
    root = Sequence(children=[Succeed(), Succeed(), Succeed()])
    root.bind(make_context())
    assert root.tick() is Status.SUCCESS


def test_sequence_fails_fast_and_does_not_tick_later_children():
    later = Succeed()
    root = Sequence(children=[Fail(), later])
    root.bind(make_context())
    assert root.tick() is Status.FAILURE
    assert later.status is Status.INVALID


def test_sequence_resumes_running_child_without_retick_earlier_success():
    first = Scripted(statuses=[Status.SUCCESS])
    second = Scripted(statuses=[Status.RUNNING, Status.SUCCESS])
    root = Sequence(children=[first, second])
    root.bind(make_context())

    assert root.tick() is Status.RUNNING
    assert first.ticks == 1
    assert root.tick() is Status.SUCCESS
    # first child was not re-ticked on the second pass
    assert first.ticks == 1
    assert second.ticks == 2


def test_fallback_first_success_wins():
    second = Succeed()
    root = Fallback(children=[Fail(), second])
    root.bind(make_context())
    assert root.tick() is Status.SUCCESS


def test_fallback_all_fail():
    root = Fallback(children=[Fail(), Fail()])
    root.bind(make_context())
    assert root.tick() is Status.FAILURE


def test_parallel_success_threshold():
    root = Parallel(children=[Succeed(), Succeed(), Fail()], success_threshold=2)
    root.bind(make_context())
    assert root.tick() is Status.SUCCESS


def test_parallel_fails_when_success_becomes_unreachable():
    root = Parallel(children=[Fail(), Fail(), Succeed()], success_threshold=2)
    root.bind(make_context())
    assert root.tick() is Status.FAILURE


def test_parallel_does_not_restart_terminal_child_while_sibling_runs():
    terminal = Scripted(statuses=[Status.SUCCESS])
    running = Scripted(statuses=[Status.RUNNING, Status.SUCCESS])
    root = Parallel(children=[terminal, running])
    root.bind(make_context())

    assert root.tick() is Status.RUNNING
    assert root.tick() is Status.SUCCESS
    assert terminal.ticks == 1


def test_retry_succeeds_within_attempts():
    child = Scripted(statuses=[Status.FAILURE, Status.SUCCESS])
    root = Retry(child=child, num_attempts=3)
    root.bind(make_context())
    assert root.tick() is Status.RUNNING
    assert root.tick() is Status.SUCCESS


def test_retry_fails_after_exhausting_attempts():
    child = Fail()
    root = Retry(child=child, num_attempts=2)
    root.bind(make_context())
    assert root.tick() is Status.RUNNING
    assert root.tick() is Status.FAILURE
