import pytest
from fakes import FakePolicy, FakePolicyResult, FakeRobot, FakeSource
from orchestrator.blackboard import Blackboard
from orchestrator.loaders.xml import load_tree_from_string
from orchestrator.registry import default_registry
from orchestrator.tree import NodeContext, Status


def test_load_tree_builds_sequence_of_leaves_and_ticks():
    xml_text = """
    <BehaviorTree name="demo">
      <Sequence>
        <SetBlackboard key="foo" value="bar" />
        <CheckBlackboard key="foo" equals="bar" />
      </Sequence>
    </BehaviorTree>
    """
    root = load_tree_from_string(xml_text, default_registry())
    root.bind(NodeContext(robot=None, blackboard=Blackboard()))
    assert root.tick() is Status.SUCCESS


def test_load_tree_resolves_named_policy_attribute():
    xml_text = """
    <BehaviorTree name="demo">
      <RunPolicy name="PickPolicy" policy="Pick_v1" />
    </BehaviorTree>
    """
    root = load_tree_from_string(xml_text, default_registry())
    robot = FakeRobot()
    policy = FakePolicy("Pick_v1", [FakePolicyResult(done=True)])
    root.bind(
        NodeContext(
            robot=robot,
            blackboard=Blackboard(),
            policies={"Pick_v1": policy},
            sources={"Pick_v1": FakeSource("Pick_v1")},
        )
    )

    assert root.name == "PickPolicy"
    assert root.tick() is Status.SUCCESS


def test_load_tree_fallback_and_recovery_scenario():
    xml_text = """
    <BehaviorTree name="assembly">
      <Sequence>
        <RunPolicy name="Pick" policy="Pick_v1" />
        <Fallback>
          <RunPolicy name="Align" policy="Align_v2" />
          <ExecuteRecoveryPlan planner="default" planner_source="Planner" />
        </Fallback>
      </Sequence>
    </BehaviorTree>
    """
    root = load_tree_from_string(xml_text, default_registry())
    robot = FakeRobot()
    pick = FakePolicy("Pick_v1", [FakePolicyResult(done=True)])
    align = FakePolicy(
        "Align_v2", [FakePolicyResult(uncertain=True, recovery_target="ood_pose")]
    )
    planner_source = FakeSource("Planner")
    from fakes import FakePlan, FakePlanner

    planner = FakePlanner(FakePlan(valid=True))
    context = NodeContext(
        robot=robot,
        blackboard=Blackboard(),
        policies={"Pick_v1": pick, "Align_v2": align},
        planners={"default": planner},
        sources={
            "Pick_v1": FakeSource("Pick_v1"),
            "Align_v2": FakeSource("Align_v2"),
            "Planner": planner_source,
        },
    )
    root.bind(context)

    assert root.tick() is Status.SUCCESS


def test_root_must_be_behaviortree_tag():
    with pytest.raises(ValueError):
        load_tree_from_string("<NotATree><Succeed/></NotATree>", default_registry())


def test_root_must_have_exactly_one_child():
    with pytest.raises(ValueError):
        load_tree_from_string(
            "<BehaviorTree><Succeed/><Fail/></BehaviorTree>", default_registry()
        )


def test_leaf_node_rejects_children():
    with pytest.raises(ValueError):
        load_tree_from_string(
            "<BehaviorTree><Succeed><Fail/></Succeed></BehaviorTree>", default_registry()
        )


def test_decorator_requires_exactly_one_child():
    with pytest.raises(ValueError):
        load_tree_from_string(
            '<BehaviorTree><RecordEpisode task="t"/></BehaviorTree>', default_registry()
        )
