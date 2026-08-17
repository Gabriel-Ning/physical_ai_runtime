from fakes import FakePlan, FakePlanner, FakeRobot, FakeSource
from orchestrator.blackboard import Blackboard
from orchestrator.nodes.planner import ExecuteRecoveryPlan
from orchestrator.tree import NodeContext, Status


def make_context(robot, planners, sources):
    return NodeContext(
        robot=robot, blackboard=Blackboard(), planners=planners, sources=sources
    )


def test_execute_recovery_plan_success():
    robot = FakeRobot()
    robot.execute_result = True
    planner = FakePlanner(FakePlan(valid=True))
    planner_source = FakeSource("Planner")
    node = ExecuteRecoveryPlan(planner="default", planner_source="Planner")
    context = make_context(robot, {"default": planner}, {"Planner": planner_source})
    context.blackboard.set("recovery_target", "ood_pose")
    node.bind(context)

    assert node.tick() is Status.SUCCESS
    assert planner.calls == [(robot, "ood_pose")]
    assert robot.executed == [("arm", planner.plan_result)]


def test_execute_recovery_plan_invalid_plan_fails():
    robot = FakeRobot()
    planner = FakePlanner(FakePlan(valid=False))
    planner_source = FakeSource("Planner")
    node = ExecuteRecoveryPlan(planner="default", planner_source="Planner")
    context = make_context(robot, {"default": planner}, {"Planner": planner_source})
    context.blackboard.set("recovery_target", "ood_pose")
    node.bind(context)

    assert node.tick() is Status.FAILURE
    assert robot.executed == []


def test_execute_recovery_plan_missing_target_fails():
    robot = FakeRobot()
    planner = FakePlanner()
    planner_source = FakeSource("Planner")
    node = ExecuteRecoveryPlan(planner="default", planner_source="Planner")
    context = make_context(robot, {"default": planner}, {"Planner": planner_source})
    node.bind(context)

    assert node.tick() is Status.FAILURE
    assert planner.calls == []


def test_execute_recovery_plan_execution_failure():
    robot = FakeRobot()
    robot.execute_result = False
    planner = FakePlanner(FakePlan(valid=True))
    planner_source = FakeSource("Planner")
    node = ExecuteRecoveryPlan(planner="default", planner_source="Planner")
    context = make_context(robot, {"default": planner}, {"Planner": planner_source})
    context.blackboard.set("recovery_target", "ood_pose")
    node.bind(context)

    assert node.tick() is Status.FAILURE
