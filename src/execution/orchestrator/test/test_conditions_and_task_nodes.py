from orchestrator.blackboard import Blackboard
from orchestrator.nodes.conditions import CheckBlackboard
from orchestrator.nodes.task import Fail, SetBlackboard, Succeed, Wait
from orchestrator.tree import NodeContext, Status


def make_context(**initial):
    return NodeContext(robot=None, blackboard=Blackboard(initial))


def test_set_blackboard_writes_and_succeeds():
    node = SetBlackboard(key="foo", value="bar")
    context = make_context()
    node.bind(context)
    assert node.tick() is Status.SUCCESS
    assert context.blackboard.get("foo") == "bar"


def test_check_blackboard_truthy():
    node = CheckBlackboard(key="ready")
    node.bind(make_context(ready=True))
    assert node.tick() is Status.SUCCESS


def test_check_blackboard_missing_key_fails():
    node = CheckBlackboard(key="missing")
    node.bind(make_context())
    assert node.tick() is Status.FAILURE


def test_check_blackboard_equals():
    node = CheckBlackboard(key="state", equals="armed")
    node.bind(make_context(state="disarmed"))
    assert node.tick() is Status.FAILURE


def test_wait_runs_then_succeeds():
    clock = {"t": 0.0}
    node = Wait(seconds=1.0, clock=lambda: clock["t"])
    node.bind(make_context())
    assert node.tick() is Status.RUNNING
    clock["t"] = 2.0
    assert node.tick() is Status.SUCCESS


def test_succeed_and_fail_constants():
    succeed = Succeed()
    succeed.bind(make_context())
    assert succeed.tick() is Status.SUCCESS

    fail = Fail()
    fail.bind(make_context())
    assert fail.tick() is Status.FAILURE
