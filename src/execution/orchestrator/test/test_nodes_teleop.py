from fakes import FakeObservation, FakePolicy, FakeRobot
from orchestrator.blackboard import Blackboard
from orchestrator.nodes.teleop import PolicySkill
from orchestrator.tree import NodeContext, Status


def make_context(robot, policy):
    return NodeContext(
        robot=robot, blackboard=Blackboard(), policies={"Pick_v1": policy}
    )


def test_policy_skill_running_while_policy_owns_arm():
    robot = FakeRobot(FakeObservation({"arm": {"provider": "Pick_v1"}}))
    policy = FakePolicy("Pick_v1", [])
    node = PolicySkill(policy="Pick_v1")
    context = make_context(robot, policy)
    node.bind(context)

    assert node.tick() is Status.RUNNING
    assert context.blackboard.get("intervened") is False


def test_policy_skill_running_and_intervened_while_teleop_owns_arm():
    robot = FakeRobot(FakeObservation({"arm": {"provider": "Teleop"}}))
    policy = FakePolicy("Pick_v1", [])
    node = PolicySkill(policy="Pick_v1", teleop="Teleop")
    context = make_context(robot, policy)
    node.bind(context)

    assert node.tick() is Status.RUNNING
    assert context.blackboard.get("intervened") is True


def test_policy_skill_fails_when_neither_owns_arm():
    robot = FakeRobot(FakeObservation({"arm": {"provider": "SomeoneElse"}}))
    policy = FakePolicy("Pick_v1", [])
    node = PolicySkill(policy="Pick_v1")
    node.bind(make_context(robot, policy))

    assert node.tick() is Status.FAILURE


def test_policy_skill_never_calls_handover():
    """The tree must only observe ownership, never drive EM allocation itself."""
    robot = FakeRobot(FakeObservation({"arm": {"provider": "Teleop"}}))
    assert not hasattr(robot, "handover")
    policy = FakePolicy("Pick_v1", [])
    node = PolicySkill(policy="Pick_v1")
    node.bind(make_context(robot, policy))
    node.tick()
    # FakeRobot intentionally has no handover()/release() methods at all;
    # ticking must not attempt to call anything beyond get_observation().
