from fakes import (
    FakeControlSession,
    FakePolicy,
    FakePolicyResult,
    FakeRobot,
    FakeSource,
)
from orchestrator.blackboard import Blackboard
from orchestrator.nodes.policy import RunPolicy
from orchestrator.tree import NodeContext, Status


def make_context(robot, policies=None, planners=None, recorder=None, sources=None):
    return NodeContext(
        robot=robot,
        blackboard=Blackboard(),
        recorder=recorder,
        policies=policies or {},
        planners=planners or {},
        sources=sources or {name: FakeSource(name) for name in (policies or {})},
    )


def test_run_policy_sends_actions_until_done():
    robot = FakeRobot()
    policy = FakePolicy(
        "Pick_v1",
        [
            FakePolicyResult(action="a1"),
            FakePolicyResult(action="a2"),
            FakePolicyResult(done=True),
        ],
    )
    node = RunPolicy(policy="Pick_v1")
    node.bind(make_context(robot, policies={"Pick_v1": policy}))

    assert node.tick() is Status.RUNNING
    assert node.tick() is Status.RUNNING
    assert node.tick() is Status.SUCCESS

    session: FakeControlSession = robot.control_sessions[0]
    assert session.entered and session.exited
    assert isinstance(session.source, FakeSource)
    assert [action for action, _ in session.sent] == ["a1", "a2"]


def test_run_policy_uncertain_writes_recovery_target_and_fails():
    robot = FakeRobot()
    policy = FakePolicy(
        "Align_v2",
        [FakePolicyResult(uncertain=True, recovery_target="ood_pose")],
    )
    node = RunPolicy(policy="Align_v2")
    context = make_context(robot, policies={"Align_v2": policy})
    node.bind(context)

    assert node.tick() is Status.FAILURE
    assert context.blackboard.get("recovery_target") == "ood_pose"
    assert robot.control_sessions[0].exited


def test_run_policy_exits_control_scope_on_terminate_reset():
    robot = FakeRobot()
    policy = FakePolicy("Pick_v1", [FakePolicyResult(action="a1")])
    node = RunPolicy(policy="Pick_v1")
    node.bind(make_context(robot, policies={"Pick_v1": policy}))

    assert node.tick() is Status.RUNNING
    session = robot.control_sessions[0]
    assert not session.exited
    node.reset()
    assert session.exited
