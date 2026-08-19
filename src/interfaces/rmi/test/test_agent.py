from types import SimpleNamespace

import pytest
from rmi import (
    Action,
    Agent,
    Context,
    EmbodimentConfig,
    Observation,
    PlanExecutionState,
    Robot,
)
from rmi.planning import (
    JointHorizonPoint,
    JointHorizonResult,
    PlanPoint,
    PlanResult,
    PoseHorizonPoint,
    PoseHorizonResult,
    ResolveResult,
)


def _profile():
    return EmbodimentConfig.from_dict(
        {
            "metadata": {"name": "test_robot"},
            "groups": {
                "arm": {
                    "type": "arm",
                    "joint_names": ["j1"],
                    "base_frame": "base",
                    "controller_manager": "/controller_manager",
                    "default_controller": "joint_space_reference",
                    "controllers": {
                        "joint_space_reference": {
                            "name": "arm_controller",
                            "implementation": "test/Controller",
                            "command_interface": "position",
                            "ros_topics": {"joint_reference": "/controller/input"},
                        }
                    },
                }
            },
            "execution_manager": {
                "providers": {
                    "Policy": {
                        "priority": 10,
                        "controllers": {"arm": "joint_space_reference"},
                    },
                    "Teleop": {
                        "priority": 20,
                        "controllers": {"arm": "joint_space_reference"},
                    },
                },
                "sources": [
                    {
                        "provider": "Policy",
                        "part": "arm",
                        "command": "joint_reference",
                        "topic": "/sources/policy/arm",
                    },
                    {
                        "provider": "Teleop",
                        "part": "arm",
                        "command": "joint_reference",
                        "topic": "/sources/teleop/arm",
                    },
                ],
            },
            "agents": {
                "PolicyLoop": {"provider": "Policy", "frequency": 30.0},
            },
            "sensors": {
                "cameras": {
                    "head": {
                        "ros_topic": "/head/image",
                        "encoding": "rgb8",
                        "fps": 30,
                        "resolution": [480, 640],
                    }
                }
            },
        }
    )


class FakeExecution:
    def __init__(self):
        self.allocations = {}
        self.next_generation = 0
        self.calls = []
        self.events = []

    def prepare(self, provider):
        self.calls.append(("prepare", provider))

    def acquire(self, provider):
        self.next_generation += 1
        self.allocations["arm"] = {
            "provider": provider,
            "generation": self.next_generation,
        }
        self.calls.append(("acquire", provider))
        return self.next_generation

    def release(self, provider, *, reason="", next_provider=""):
        del next_provider
        self.calls.append(("release", provider, reason))
        if self.allocations.get("arm", {}).get("provider") == provider:
            self.allocations.pop("arm")
        return self.next_generation

    def get_allocations(self):
        return self.allocations

    def get_events(self, *, correlation_id=None):
        if correlation_id is None:
            return list(self.events)
        return [
            event
            for event in self.events
            if event.get("correlation_id") == correlation_id
        ]

    def wait_for_execution_event(self, correlation_id, kinds, *, timeout_sec):
        del timeout_sec
        return next(
            (
                event
                for event in self.events
                if event.get("correlation_id") == correlation_id
                and event.get("kind") in kinds
            ),
            None,
        )


class ResumingFakeExecution(FakeExecution):
    def __init__(self):
        super().__init__()
        self.displaced = []

    def acquire(self, provider):
        current = self.allocations.get("arm")
        if current is not None:
            self.displaced.append(current["provider"])
        return super().acquire(provider)

    def release(self, provider, *, reason="", next_provider=""):
        super().release(provider, reason=reason, next_provider=next_provider)
        if self.displaced:
            resumed = self.displaced.pop()
            self.next_generation += 1
            self.allocations["arm"] = {
                "provider": resumed,
                "generation": self.next_generation,
            }
        return self.next_generation


class FakeProviderClient:
    def __init__(self, name, parts=("arm",)):
        self.name = name
        self.controllers = {part: object() for part in parts}
        self.sent = []
        self.goal_handle = SimpleNamespace(
            goal_id=SimpleNamespace(uuid=bytes(range(16)))
        )
        self.wait_error = None
        self.feedback_callback = None

    def send(self, part, command, value):
        self.sent.append((part, command, value))

    def send_joint_reference(self, part, joint_names, positions, times):
        self.sent.append((part, "joint_reference", joint_names, positions, times))

    def send_pose_reference(self, part, positions, orientations, times, frame_id):
        self.sent.append(
            (part, "pose_reference", positions, orientations, times, frame_id)
        )

    async def start_joint_trajectory(
        self, part, trajectory, joint_names, feedback_callback=None
    ):
        self.sent.append((part, "joint_trajectory", trajectory, joint_names))
        self.feedback_callback = feedback_callback
        return self.goal_handle

    def emit_feedback(self, elapsed_s):
        seconds = int(elapsed_s)
        nanoseconds = round((elapsed_s - seconds) * 1e9)
        feedback = SimpleNamespace(
            desired=SimpleNamespace(
                time_from_start=SimpleNamespace(sec=seconds, nanosec=nanoseconds)
            )
        )
        self.feedback_callback(SimpleNamespace(feedback=feedback))
        return feedback

    async def wait_joint_trajectory(self, part, handle, timeout):
        self.sent.append((part, "wait", handle, timeout))
        if self.wait_error is not None:
            raise self.wait_error
        return "complete"

    async def cancel_joint_trajectory(self, part):
        self.sent.append((part, "cancel"))


class FakeNode:
    def __init__(self):
        self.subscriptions = []
        self.publishers = []

    def create_client(self, service_type, endpoint):
        return SimpleNamespace(service_type=service_type, endpoint=endpoint)

    def create_subscription(self, message_type, topic, callback, qos):
        subscription = SimpleNamespace(
            message_type=message_type,
            topic=topic,
            callback=callback,
            qos=qos,
        )
        self.subscriptions.append(subscription)
        return subscription

    def create_publisher(self, message_type, endpoint, qos):
        publisher = SimpleNamespace(
            message_type=message_type,
            endpoint=endpoint,
            qos=qos,
            publish=lambda message: None,
        )
        self.publishers.append(publisher)
        return publisher

    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(
                to_msg=lambda: SimpleNamespace(sec=1, nanosec=0)
            )
        )


def _joint_state(position=0.0, stamp_s=1):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp_s, nanosec=0)),
        name=["j1"],
        position=[position],
        velocity=[0.0],
        effort=[],
    )


def test_control_scope_acquires_sends_and_releases_synchronously():
    execution = FakeExecution()
    robot = Robot(_profile(), execution)
    policy_client = FakeProviderClient("Policy")
    policy = Agent("Policy", policy_client, robot=robot)

    with policy.run() as session:
        robot.update_joint_state(_joint_state(), receive_time_s=1.01)
        observation = session.observe()
        session.act(
            Action("arm", "joint_reference", "command"),
            observation=observation,
        )

    assert policy_client.sent == [("arm", "joint_reference", "command")]
    assert session.diagnostics.sent == 1
    assert execution.calls == [
        ("prepare", "Policy"),
        ("acquire", "Policy"),
        ("release", "Policy", "control_scope_exit"),
    ]


def test_agent_session_observes_configured_sensors():
    execution = FakeExecution()
    robot = Robot(_profile(), execution)
    robot.update_joint_state(_joint_state(), receive_time_s=1.01)
    sample = SimpleNamespace(value="force", source_time_s=1.0)
    sensor = SimpleNamespace(name="wrench", latest=sample)
    agent = Agent(
        "Policy",
        FakeProviderClient("Policy"),
        robot=robot,
        sensors=(sensor,),
    )

    with agent.run() as session:
        observation = session.observe()

    assert observation.data["joint_positions"] == (0.0,)
    assert observation.sensors == {"wrench": sample}


def test_teleop_takeover_closes_policy_output_gate_without_receipt_handling():
    execution = FakeExecution()
    robot = Robot(_profile(), execution)
    policy_client = FakeProviderClient("Policy")
    teleop_client = FakeProviderClient("Teleop")
    policy = Agent("Policy", policy_client)
    teleop = Agent("Teleop", teleop_client)

    with robot.control(policy, resume=True) as policy_control:
        robot.update_joint_state(_joint_state(position=0.0, stamp_s=1))
        old_observation = robot.get_observation()

        with robot.control(teleop):
            robot.send_action(Action("arm", "joint_reference", "teleop"))
            policy_control.send(
                Action("arm", "joint_reference", "shadow"),
                observation=old_observation,
            )

        execution.next_generation += 1
        execution.allocations["arm"] = {
            "provider": "Policy",
            "generation": execution.next_generation,
        }

        policy_control.send(
            Action("arm", "joint_reference", "old-inference"),
            observation=old_observation,
        )

        robot.update_joint_state(_joint_state(position=0.2, stamp_s=3))
        fresh_observation = robot.get_observation()
        policy_control.send(
            Action("arm", "joint_reference", "fresh-inference"),
            observation=fresh_observation,
        )

    assert teleop_client.sent == [("arm", "joint_reference", "teleop")]
    assert policy_client.sent == [("arm", "joint_reference", "fresh-inference")]
    assert policy_control.diagnostics.inactive_drops == 1
    assert policy_control.diagnostics.stale_observation_drops == 1
    assert policy_control.diagnostics.resumes == 1


def test_planner_recovery_resumes_policy_and_fences_pre_takeover_inference():
    execution = ResumingFakeExecution()
    robot = Robot(_profile(), execution)
    policy_client = FakeProviderClient("Policy")
    planner_client = FakeProviderClient("Planner")
    policy = Agent("Policy", policy_client)
    planner = Agent("Planner", planner_client, _profile())
    plan = PlanResult(
        valid=True,
        joint_names=["j1"],
        points=[PlanPoint(positions=[0.2], time_from_start_s=0.1)],
    )

    with robot.control(policy, resume=True) as policy_control:
        robot.update_joint_state(_joint_state(position=0.0, stamp_s=1))
        pre_takeover = robot.get_observation()

        with robot.control(planner):
            assert robot.execute("arm", plan).wait(timeout=1.0) == "complete"

        policy_control.send(
            Action("arm", "joint_reference", "stale-policy-action"),
            observation=pre_takeover,
        )
        robot.update_joint_state(_joint_state(position=0.2, stamp_s=2))
        policy_control.send(
            Action("arm", "joint_reference", "fresh-policy-action"),
            observation=robot.get_observation(),
        )

    assert policy_client.sent == [("arm", "joint_reference", "fresh-policy-action")]
    assert planner_client.sent[0][1] == "joint_trajectory"
    assert policy_control.generation_for("arm") == 3
    assert policy_control.diagnostics.resumes == 1
    assert policy_control.diagnostics.stale_observation_drops == 1


def test_partial_teleop_takeover_keeps_untouched_policy_part_active():
    class PartialExecution(FakeExecution):
        def __init__(self):
            super().__init__()
            self.provider_parts = {
                "Policy": ("arm", "gripper"),
                "Teleop": ("arm",),
            }
            self.displaced = {}

        def acquire(self, provider):
            self.next_generation += 1
            for part in self.provider_parts[provider]:
                current = self.allocations.get(part)
                if current is not None and current["provider"] != provider:
                    self.displaced[part] = current["provider"]
                self.allocations[part] = {
                    "provider": provider,
                    "generation": self.next_generation,
                }
            self.calls.append(("acquire", provider))
            return self.next_generation

        def release(self, provider, *, reason="", next_provider=""):
            del next_provider
            self.calls.append(("release", provider, reason))
            for part in self.provider_parts[provider]:
                allocation = self.allocations.get(part)
                if allocation is None or allocation["provider"] != provider:
                    continue
                resumed = self.displaced.pop(part, None)
                if resumed is None:
                    self.allocations.pop(part)
                    continue
                self.next_generation += 1
                self.allocations[part] = {
                    "provider": resumed,
                    "generation": self.next_generation,
                }
            return self.next_generation

    execution = PartialExecution()
    robot = Robot(_profile(), execution)
    policy_client = FakeProviderClient("Policy", parts=("arm", "gripper"))
    teleop_client = FakeProviderClient("Teleop")
    policy = Agent("Policy", policy_client)
    teleop = Agent("Teleop", teleop_client)

    with robot.control(policy, resume=True) as policy_control:
        robot.update_joint_state(_joint_state(position=0.0, stamp_s=1))
        before_takeover = robot.get_observation()
        assert policy_control.generation_for("arm") == 1
        assert policy_control.generation_for("gripper") == 1

        with robot.control(teleop):
            policy_control.send(
                Action("arm", "joint_reference", "blocked-arm"),
                observation=before_takeover,
            )
            policy_control.send(
                Action("gripper", "joint_reference", "live-gripper"),
                observation=before_takeover,
            )

        assert policy_control.generation_for("arm") == 1
        assert policy_control.generation_for("gripper") == 1
        assert policy_control.active
        assert policy_control.generation_for("arm") == 3
        assert policy_control.generation_for("gripper") == 1

        policy_control.send(
            Action("arm", "joint_reference", "stale-arm"),
            observation=before_takeover,
        )
        robot.update_joint_state(_joint_state(position=0.1, stamp_s=2))
        policy_control.send(
            Action("arm", "joint_reference", "fresh-arm"),
            observation=robot.get_observation(),
        )

    assert policy_client.sent == [
        ("gripper", "joint_reference", "live-gripper"),
        ("arm", "joint_reference", "fresh-arm"),
    ]
    assert policy_control.diagnostics.inactive_drops == 1
    assert policy_control.diagnostics.stale_observation_drops == 1
    assert policy_control.diagnostics.resumes == 1


def test_observation_preserves_source_receive_and_allocation_snapshot():
    execution = FakeExecution()
    execution.allocations["arm"] = {"provider": "Policy", "generation": 7}
    robot = Robot(_profile(), execution)
    robot.update_joint_state(
        _joint_state(position=0.4, stamp_s=12), receive_time_s=12.1
    )

    observation = robot.state
    execution.allocations["arm"]["generation"] = 8

    assert isinstance(observation, Observation)
    assert observation.data["joint_positions"] == (0.4,)
    assert observation.source_time_s == 12.0
    assert observation.receive_time_s == 12.1
    assert observation.allocation_generation("arm", "Policy") == 7


def test_context_shares_node_and_lazily_caches_profile_agents():
    node = FakeNode()
    context = Context(_profile(), node)

    first = context.make_agent("Policy", robot=context.robot)
    second = context.make_agent("Policy", robot=context.robot)
    state_subscription = next(
        subscription
        for subscription in node.subscriptions
        if subscription.topic == "/joint_states"
    )
    state_subscription.callback(_joint_state(position=0.5, stamp_s=4))

    assert first is second
    assert context.robot.state.data["joint_positions"] == (0.5,)
    assert any(
        publisher.endpoint == "/sources/policy/arm" for publisher in node.publishers
    )


def test_context_agent_uses_configured_provider_and_frequency():
    context = Context(_profile(), FakeNode())

    agent = context.make_agent("PolicyLoop")

    assert agent.name == "PolicyLoop"
    assert agent.provider == "Policy"
    assert agent.frequency == 30.0
    assert agent.run().period == pytest.approx(1.0 / 30.0)


def test_session_wait_uses_monotonic_deadlines(monkeypatch):
    execution = FakeExecution()
    robot = Robot(_profile(), execution)
    agent = Agent(
        "Policy",
        FakeProviderClient("Policy"),
        robot=robot,
        frequency=20.0,
    )
    clock = [1.0]
    sleeps = []

    def sleep(delay):
        sleeps.append(delay)
        clock[0] += delay

    monkeypatch.setattr("rmi.agent.time.monotonic", lambda: clock[0])
    monkeypatch.setattr("rmi.agent.time.sleep", sleep)

    with agent.run() as session:
        session.wait()
        session.wait()

    assert session.frequency == 20.0
    assert session.period == 0.05
    assert sleeps == pytest.approx([0.05, 0.05])


def test_context_lazily_builds_and_caches_profile_camera():
    node = FakeNode()
    context = Context(_profile(), node)

    first = context.make_camera("head")
    second = context.make_camera("head")

    assert first is second
    assert first.topic == "/head/image"

    with pytest.raises(KeyError, match="unknown profile camera"):
        context.make_camera("missing")


def test_resolver_results_can_be_sent_periodically_directly_to_jspc():
    client = FakeProviderClient("Resolver")
    source = Agent("Resolver", client, _profile())

    for position in (0.2, 0.3):
        source.send(
            Action(
                "arm",
                "joint_reference",
                ResolveResult(
                    valid=True,
                    joint_names=["j1"],
                    positions=[position],
                ),
            )
        )

    assert client.sent == [
        ("arm", "joint_reference", ["j1"], [[0.2]], [0.0]),
        ("arm", "joint_reference", ["j1"], [[0.3]], [0.0]),
    ]


def test_joint_horizon_result_maps_to_multi_point_jspc_reference():
    client = FakeProviderClient("Streamer")
    source = Agent("Streamer", client, _profile())
    horizon = JointHorizonResult(
        valid=True,
        points=[
            JointHorizonPoint(positions=[0.2], time_from_start_s=0.02),
            JointHorizonPoint(positions=[0.3], time_from_start_s=0.04),
        ],
    )

    source.send(Action("arm", "joint_reference", horizon))

    assert client.sent == [
        ("arm", "joint_reference", ["j1"], [[0.2], [0.3]], [0.02, 0.04])
    ]


def test_pose_horizon_result_maps_wxyz_contract_to_xyzw_ros_reference():
    client = FakeProviderClient("CartesianStreamer")
    source = Agent("CartesianStreamer", client, _profile())
    horizon = PoseHorizonResult(
        valid=True,
        points=[
            PoseHorizonPoint(
                position_xyz=(0.1, 0.2, 0.3),
                orientation_wxyz=(0.5, 0.1, 0.2, 0.3),
                time_from_start_s=0.02,
            )
        ],
    )

    source.send(Action("arm", "pose_reference", horizon))

    assert client.sent == [
        (
            "arm",
            "pose_reference",
            [[0.1, 0.2, 0.3]],
            [[0.1, 0.2, 0.3, 0.5]],
            [0.02],
            "base",
        )
    ]


def test_invalid_planning_result_is_never_sent():
    client = FakeProviderClient("Resolver")
    source = Agent("Resolver", client, _profile())

    with pytest.raises(ValueError, match="cannot send invalid planning result"):
        source.send(
            Action(
                "arm",
                "joint_reference",
                ResolveResult(valid=False, reason="ik failed"),
            )
        )

    assert client.sent == []


def test_plan_executes_synchronously_through_active_control_source():
    execution_manager = FakeExecution()
    robot = Robot(_profile(), execution_manager)
    client = FakeProviderClient("Planner")
    source = Agent("Planner", client, _profile())
    plan = PlanResult(
        valid=True,
        joint_names=["j1"],
        points=[
            PlanPoint(positions=[0.2], time_from_start_s=0.1),
            PlanPoint(positions=[0.4], time_from_start_s=0.2),
        ],
    )

    with robot.control(source) as control:
        handle = robot.execute("arm", plan)
        result = handle.wait(timeout=2.0)

    assert result == "complete"
    assert handle.done
    assert handle.state is PlanExecutionState.COMPLETED
    assert handle.correlation_id == bytes(range(16)).hex()
    assert control.diagnostics.sent == 1
    assert client.sent == [
        (
            "arm",
            "joint_trajectory",
            {
                "joint_names": ["j1"],
                "points": [
                    {
                        "positions": [0.2],
                        "velocities": [],
                        "accelerations": [],
                        "time_from_start_s": 0.1,
                    },
                    {
                        "positions": [0.4],
                        "velocities": [],
                        "accelerations": [],
                        "time_from_start_s": 0.2,
                    },
                ],
            },
            ["j1"],
        ),
        ("arm", "wait", client.goal_handle, 2.0),
    ]


def test_plan_execution_supports_synchronous_cancel():
    from rmi.controllers import TrajectoryCanceledError

    execution_manager = FakeExecution()
    robot = Robot(_profile(), execution_manager)
    client = FakeProviderClient("Planner")
    client.wait_error = TrajectoryCanceledError("trajectory goal was canceled")
    source = Agent("Planner", client, _profile())
    plan = PlanResult(
        valid=True,
        joint_names=["j1"],
        points=[PlanPoint(positions=[0.2], time_from_start_s=0.1)],
    )

    with robot.control(source):
        handle = robot.execute("arm", plan)
        handle.cancel()
        assert handle.wait(timeout=1.0) is None

    assert handle.canceled
    assert handle.done
    assert handle.state is PlanExecutionState.CANCELED
    assert client.sent[-2] == ("arm", "cancel")


def test_externally_canceled_plan_raises_but_records_canceled_state():
    from rmi.controllers import TrajectoryCanceledError

    execution_manager = FakeExecution()
    robot = Robot(_profile(), execution_manager)
    client = FakeProviderClient("Planner")
    client.wait_error = TrajectoryCanceledError("trajectory goal was canceled")
    source = Agent("Planner", client, _profile())
    plan = PlanResult(
        valid=True,
        joint_names=["j1"],
        points=[PlanPoint(positions=[0.2], time_from_start_s=0.1)],
    )

    with robot.control(source):
        handle = robot.execute("arm", plan)
        with pytest.raises(TrajectoryCanceledError):
            handle.wait(timeout=1.0)

    assert handle.canceled
    assert handle.done
    assert handle.state is PlanExecutionState.CANCELED


def test_displaced_control_scope_exit_skips_release():
    class StrictExecution(FakeExecution):
        def release(self, provider, *, reason="", next_provider=""):
            del next_provider
            if self.allocations.get("arm", {}).get("provider") != provider:
                raise RuntimeError(f"active provider {provider!r} has no allocation")
            return super().release(provider, reason=reason)

    execution = StrictExecution()
    robot = Robot(_profile(), execution)
    policy = Agent("Policy", FakeProviderClient("Policy"))

    with robot.control(policy) as control:
        execution.allocations["arm"] = {"provider": "Teleop", "generation": 99}

    assert control.diagnostics.displaced_exits == 1
    assert ("release", "Policy", "control_scope_exit") not in execution.calls


def test_enter_timeout_raises_timeout_error_without_release_masking():
    class NeverAllocatingExecution(FakeExecution):
        def acquire(self, provider):
            self.calls.append(("acquire", provider))
            self.next_generation += 1
            return self.next_generation  # allocations never confirm the provider

        def release(self, provider, *, reason="", next_provider=""):
            raise RuntimeError("active provider has no allocation")

    execution = NeverAllocatingExecution()
    execution.allocations = {}
    robot = Robot(_profile(), execution)
    policy = Agent("Policy", FakeProviderClient("Policy"))

    with (
        pytest.raises(TimeoutError, match="authoritative allocation"),
        robot.control(policy, acquire_timeout=0.05),
    ):
        pass

    assert robot._control_stack == []


def test_nested_send_action_routes_by_owned_part():
    class DualExecution(FakeExecution):
        def __init__(self):
            super().__init__()
            self.provider_parts = {
                "Policy": ("arm", "gripper"),
                "Planner": ("arm",),
            }

        def acquire(self, provider):
            self.next_generation += 1
            for part in self.provider_parts[provider]:
                self.allocations[part] = {
                    "provider": provider,
                    "generation": self.next_generation,
                }
            self.calls.append(("acquire", provider))
            return self.next_generation

        def release(self, provider, *, reason="", next_provider=""):
            del next_provider
            self.calls.append(("release", provider, reason))
            for part in self.provider_parts[provider]:
                allocation = self.allocations.get(part)
                if allocation is not None and allocation["provider"] == provider:
                    self.allocations.pop(part)
            return self.next_generation

    execution = DualExecution()
    robot = Robot(_profile(), execution)
    policy_client = FakeProviderClient("Policy", parts=("arm", "gripper"))
    planner_client = FakeProviderClient("Planner")
    policy = Agent("Policy", policy_client)
    planner = Agent("Planner", planner_client)

    with (
        robot.control(policy, parts=("arm", "gripper"), resume=True),
        robot.control(planner, parts=("arm",)),
    ):
        robot.send_action(Action("gripper", "joint_reference", "grip"))
        robot.send_action(Action("arm", "joint_reference", "plan"))

    assert policy_client.sent == [("gripper", "joint_reference", "grip")]
    assert planner_client.sent == [("arm", "joint_reference", "plan")]


def test_plan_execution_exposes_correlated_em_events_as_advanced_diagnostics():
    execution_manager = FakeExecution()
    robot = Robot(_profile(), execution_manager)
    client = FakeProviderClient("Planner")
    source = Agent("Planner", client, _profile(), execution_manager)
    plan = PlanResult(
        valid=True,
        joint_names=["j1"],
        points=[PlanPoint(positions=[0.2], time_from_start_s=0.1)],
    )

    with robot.control(source):
        handle = robot.execute("arm", plan)

    event = {
        "kind": "trajectory_accepted",
        "correlation_id": handle.correlation_id,
    }
    execution_manager.events.append(event)

    assert handle.events == [event]
    assert handle.wait_event("trajectory_accepted", timeout=0.0) == event


def test_plan_execution_timeout_is_explicit_state() -> None:
    execution_manager = FakeExecution()
    robot = Robot(_profile(), execution_manager)
    client = FakeProviderClient("Planner")
    client.wait_error = TimeoutError("trajectory timed out")
    source = Agent("Planner", client, _profile())
    plan = PlanResult(
        valid=True,
        joint_names=["j1"],
        points=[PlanPoint(positions=[0.2], time_from_start_s=0.1)],
    )

    with robot.control(source):
        handle = robot.execute("arm", plan)
        with pytest.raises(TimeoutError, match="trajectory timed out"):
            handle.wait(timeout=0.1)

    assert handle.state is PlanExecutionState.TIMED_OUT
    assert not handle.done


def test_plan_execution_exposes_latest_feedback_and_bounded_progress() -> None:
    execution_manager = FakeExecution()
    robot = Robot(_profile(), execution_manager)
    client = FakeProviderClient("Planner")
    source = Agent("Planner", client, _profile())
    plan = PlanResult(
        valid=True,
        joint_names=["j1"],
        points=[PlanPoint(positions=[0.2], time_from_start_s=2.0)],
    )

    with robot.control(source):
        handle = robot.execute("arm", plan)
        assert handle.progress is None
        feedback = client.emit_feedback(0.5)

    assert handle.feedback is feedback
    assert handle.progress == pytest.approx(0.25)
