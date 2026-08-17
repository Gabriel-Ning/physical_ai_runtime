from fakes import FakeRobot
from orchestrator.blackboard import Blackboard
from orchestrator.nodes.task import Succeed
from orchestrator.registry import default_registry
from orchestrator.runtime import BehaviorTreeRuntime
from orchestrator.server import OrchestratorServer
from orchestrator.tree import NodeContext


def make_server():
    context = NodeContext(robot=FakeRobot(), blackboard=Blackboard())
    runtime = BehaviorTreeRuntime(Succeed(name="root"), context)
    return OrchestratorServer(runtime, default_registry())


def test_get_node_catalog_lists_registered_tags():
    server = make_server()
    catalog = server.get_node_catalog()
    tags = {entry["tag"] for entry in catalog["nodes"]}
    assert "RunPolicy" in tags


def test_task_lifecycle_endpoints():
    server = make_server()
    started = server.start_task(background=False)
    # start() only resets + arms the tree; nothing has been ticked yet.
    assert started["task_phase"] == "running"


def test_pause_and_resume_and_abort_endpoints():
    server = make_server()
    server.runtime.start()
    paused = server.pause_task()
    assert paused["task_phase"] == "paused"
    resumed = server.resume_task()
    assert resumed["task_phase"] == "running"
    aborted = server.abort_task()
    assert aborted["task_phase"] == "aborted"


def test_get_tree_status_shape():
    server = make_server()
    server.runtime.start()
    status = server.get_tree_status()
    assert set(status) == {"task_phase", "root", "running_path", "failure_reason", "blackboard"}
