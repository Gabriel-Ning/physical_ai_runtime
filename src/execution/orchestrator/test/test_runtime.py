import time

import pytest
from fakes import FakePolicy, FakePolicyResult, FakeRecorder, FakeRobot, FakeSource
from orchestrator.blackboard import Blackboard
from orchestrator.controls import Sequence
from orchestrator.nodes.policy import RunPolicy
from orchestrator.nodes.recorder import RecordEpisode
from orchestrator.nodes.task import Fail, Succeed, Wait
from orchestrator.runtime import BehaviorTreeRuntime, TaskPhase
from orchestrator.tree import Node, NodeContext, Status


class Explode(Node):
    def update(self):
        raise RuntimeError("leaf failed")


def make_runtime(root, **kwargs):
    context = NodeContext(robot=FakeRobot(), blackboard=Blackboard())
    return BehaviorTreeRuntime(root, context, **kwargs)


def test_start_and_tick_until_terminal_success():
    runtime = make_runtime(Sequence(children=[Succeed(), Succeed()]))
    runtime.start()
    assert runtime.tick_until_terminal() is Status.SUCCESS
    assert runtime.phase is TaskPhase.SUCCEEDED


def test_start_and_tick_until_terminal_failure():
    runtime = make_runtime(Sequence(children=[Succeed(), Fail()]))
    runtime.start()
    assert runtime.tick_until_terminal() is Status.FAILURE
    assert runtime.phase is TaskPhase.FAILED


def test_cannot_tick_before_start():
    runtime = make_runtime(Succeed())
    with pytest.raises(RuntimeError):
        runtime.tick_once()


def test_pause_blocks_ticking_and_resume_continues():
    runtime = make_runtime(Wait(seconds=1000.0, clock=lambda: 0.0))
    runtime.start()
    runtime.tick_once()
    runtime.pause()
    assert runtime.tick_once() is Status.RUNNING  # paused: no-op, still RUNNING
    assert runtime.phase is TaskPhase.PAUSED
    runtime.resume()
    assert runtime.phase is TaskPhase.RUNNING


def test_abort_closes_open_control_scope():
    robot = FakeRobot()
    policy = FakePolicy("Pick_v1", [FakePolicyResult(action="a1")])
    context = NodeContext(
        robot=robot,
        blackboard=Blackboard(),
        policies={"Pick_v1": policy},
        sources={"Pick_v1": FakeSource("Pick_v1")},
    )
    node = RunPolicy(policy="Pick_v1")
    runtime = BehaviorTreeRuntime(node, context)

    runtime.start()
    assert runtime.tick_once() is Status.RUNNING
    assert not robot.control_sessions[0].exited

    runtime.abort()
    assert robot.control_sessions[0].exited
    assert runtime.phase is TaskPhase.ABORTED


def test_restart_after_terminal_resets_tree():
    root = Sequence(children=[Succeed(), Succeed()])
    runtime = make_runtime(root)
    runtime.start()
    runtime.tick_until_terminal()
    assert runtime.phase is TaskPhase.SUCCEEDED

    runtime.start()
    assert runtime.phase is TaskPhase.RUNNING
    assert runtime.tick_until_terminal() is Status.SUCCESS


def test_status_snapshot_reports_running_path_and_blackboard():
    root = Sequence(name="root", children=[Wait(name="wait", seconds=1000.0, clock=lambda: 0.0)])
    context = NodeContext(robot=FakeRobot(), blackboard=Blackboard({"foo": "bar"}))
    runtime = BehaviorTreeRuntime(root, context)
    runtime.start()
    runtime.tick_once()

    status = runtime.status
    assert status.task_phase == "running"
    assert "wait" in status.running_path
    assert status.blackboard == {"foo": "bar"}
    assert status.root.to_dict()["name"] == "root"


def test_background_tick_loop_runs_to_completion():
    runtime = make_runtime(Sequence(children=[Succeed(), Succeed()]), tick_hz=50.0)
    runtime.start(background=True)
    deadline = time.monotonic() + 2.0
    while runtime.phase is TaskPhase.RUNNING and time.monotonic() < deadline:
        time.sleep(0.01)
    assert runtime.phase is TaskPhase.SUCCEEDED


def test_leaf_exception_fails_task_and_closes_episode_scope():
    recorder = FakeRecorder()
    context = NodeContext(
        robot=FakeRobot(),
        blackboard=Blackboard(),
        recorder=recorder,
    )
    runtime = BehaviorTreeRuntime(
        RecordEpisode(child=Explode(), task="failure"),
        context,
    )
    runtime.start()

    with pytest.raises(RuntimeError, match="leaf failed"):
        runtime.tick_once()

    assert runtime.phase is TaskPhase.FAILED
    assert runtime.status.failure_reason == "RuntimeError: leaf failed"
    assert recorder.episodes[0].exited
