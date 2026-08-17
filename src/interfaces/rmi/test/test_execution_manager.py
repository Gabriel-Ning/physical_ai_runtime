import asyncio

import pytest
from rmi import (
    ActionChunk,
    ArbitrationRejected,
    EmbodimentConfig,
    ExecutionManager,
    HandoverError,
    LifecycleTransitionError,
    ProviderRegistration,
    ProviderState,
    RobotTopology,
)


class FakeController:
    def __init__(self, name):
        self.controller_name = name
        self.messages = []

    async def cancel(self):
        self.cancel_count = getattr(self, "cancel_count", 0) + 1

    async def send(self, message):
        self.messages.append(message)


class FakeSwitcher:
    def __init__(self, error=None):
        self.error = error
        self.requests = []

    async def switch_controller(self, *, activate, deactivate):
        self.requests.append((activate, deactivate))
        if self.error:
            raise self.error


class FakeProvider:
    def __init__(self, *, start_error=None, reset_error=None):
        self.start_error = start_error
        self.reset_error = reset_error
        self.started = 0
        self.stopped = 0
        self.deactivated = 0
        self.observations = []

    async def start(self):
        self.started += 1
        if self.start_error:
            raise self.start_error

    async def stop(self):
        self.stopped += 1

    async def deactivate(self):
        self.deactivated += 1

    async def reset(self, observation):
        self.observations.append(observation)
        if self.reset_error:
            raise self.reset_error


def _robot(*, switch_error=None):
    profile = {
        "metadata": {"name": "test_robot"},
        "groups": {
            "arm": {
                "type": "arm",
                "joint_names": ["joint_1"],
                "controller_manager": "/controller_manager",
                "default_controller": "task_space_reference",
                "controllers": {
                    "task_space_reference": {
                        "name": "arm_task",
                        "implementation": "example/TaskController",
                        "command_interface": "position",
                        "ros_topics": {"pose_reference": "/arm/pose"},
                    },
                    "joint_trajectory": {
                        "name": "arm_jtc",
                        "implementation": "example/JTC",
                        "command_interface": "position",
                        "ros_actions": {"follow_joint_trajectory": "/arm/follow"},
                    },
                },
            },
            "gripper": {
                "type": "parallel_gripper",
                "joint_names": ["finger_joint"],
                "parent_group": "arm",
                "controller_manager": "/controller_manager",
                "default_controller": "joint_space_reference",
                "controllers": {
                    "joint_space_reference": {
                        "name": "gripper_controller",
                        "implementation": "example/GripperController",
                        "command_interface": "position",
                        "ros_topics": {"joint_reference": "/gripper/reference"},
                    }
                },
            },
        },
    }
    return RobotTopology(
        EmbodimentConfig.from_dict(profile),
        lambda _part, _contract, cfg: FakeController(cfg.name),
        lambda _manager: FakeSwitcher(switch_error),
    )


def _registration(name, provider, contract="task_space_reference", priority=0):
    return ProviderRegistration(
        name=name,
        provider=provider,
        controllers={"arm": contract},
        priority=priority,
    )


def test_provider_lifecycle_reaches_ready_and_rejects_invalid_transition():
    provider = FakeProvider()
    manager = ExecutionManager(_robot())
    manager.register(_registration("teleop", provider))

    asyncio.run(manager.prepare("teleop"))

    assert manager.provider_states["teleop"] is ProviderState.READY
    assert provider.started == 1
    with pytest.raises(LifecycleTransitionError, match="READY"):
        asyncio.run(manager.prepare("teleop"))


def test_start_failure_transitions_provider_to_failed():
    manager = ExecutionManager(_robot())
    manager.register(
        _registration("policy", FakeProvider(start_error=ValueError("bad")))
    )

    with pytest.raises(ValueError, match="bad"):
        asyncio.run(manager.prepare("policy"))

    assert manager.provider_states["policy"] is ProviderState.FAILED


def test_same_controller_handover_fences_old_generation_without_switching():
    events = []
    manager = ExecutionManager(_robot(), events.append, clock_ns=lambda: 42)
    policy = FakeProvider()
    teleop = FakeProvider()
    manager.register(_registration("policy", policy))
    manager.register(_registration("teleop", teleop))
    asyncio.run(manager.prepare("policy"))
    asyncio.run(manager.prepare("teleop"))

    policy_generation = asyncio.run(manager.handover("policy", {"q": [0.0]}))
    old_chunk = ActionChunk("policy", policy_generation, ("arm",), 0, object())
    assert manager.admit(old_chunk)

    teleop_generation = asyncio.run(manager.handover("teleop", {"q": [0.1]}))

    assert teleop_generation > policy_generation
    assert not manager.admit(old_chunk)
    assert manager.admit(
        ActionChunk("teleop", teleop_generation, ("arm",), 0, object())
    )
    assert manager.provider_states["policy"] is ProviderState.READY
    assert policy.deactivated == 1
    assert manager.provider_states["teleop"] is ProviderState.ACTIVE
    assert teleop.observations == [{"q": [0.1]}]
    assert events[-1].kind == "chunk_rejected" or any(
        event.kind == "chunk_rejected" for event in events
    )
    assert all(event.timestamp_ns == 42 for event in events)
    takeover_events = [
        event for event in events if event.kind == "provider_takeover_requested"
    ]
    assert [event.provider for event in takeover_events] == ["policy", "teleop"]


def test_acquire_is_wire_alias_for_handover():
    manager = ExecutionManager(_robot())
    manager.register(_registration("policy", FakeProvider()))
    asyncio.run(manager.prepare("policy"))

    generation = asyncio.run(manager.acquire("policy", {"q": [0.0]}))

    assert generation == 1
    assert manager.provider_states["policy"] is ProviderState.ACTIVE
    assert manager.allocations["arm"].generation == 1


def test_rejected_takeover_retains_request_and_rejection_as_raw_facts():
    events = []
    manager = ExecutionManager(_robot(), events.append)
    policy = FakeProvider()
    teleop = FakeProvider()
    manager.register(_registration("policy", policy, priority=10))
    manager.register(_registration("teleop", teleop, priority=20))
    asyncio.run(manager.prepare("policy"))
    asyncio.run(manager.prepare("teleop"))
    asyncio.run(manager.handover("teleop"))

    with pytest.raises(ArbitrationRejected):
        asyncio.run(manager.handover("policy"))

    assert [event.kind for event in events[-2:]] == [
        "provider_takeover_requested",
        "arbitration_rejected",
    ]


def test_cross_controller_handover_switches_and_resets_before_admission():
    manager = ExecutionManager(_robot())
    planner = FakeProvider()
    manager.register(_registration("planner", planner, "joint_trajectory"))
    asyncio.run(manager.prepare("planner"))

    generation = asyncio.run(manager.handover("planner", {"q": [0.2]}))

    assert manager.provider_states["planner"] is ProviderState.ACTIVE
    assert manager.allocations["arm"].controller == "joint_trajectory"
    assert manager.allocations["arm"].generation == generation
    assert planner.observations == [{"q": [0.2]}]


def test_cross_controller_failure_rolls_back_but_keeps_old_generation_fenced():
    robot = _robot(switch_error=RuntimeError("switch rejected"))
    manager = ExecutionManager(robot)
    teleop = FakeProvider()
    planner = FakeProvider()
    manager.register(_registration("teleop", teleop))
    manager.register(_registration("planner", planner, "joint_trajectory"))
    asyncio.run(manager.prepare("teleop"))
    asyncio.run(manager.prepare("planner"))
    old_generation = asyncio.run(manager.handover("teleop"))
    old_chunk = ActionChunk("teleop", old_generation, ("arm",), 0, object())

    with pytest.raises(HandoverError, match="switch rejected"):
        asyncio.run(manager.handover("planner", {"q": [0.2]}))

    assert not manager.admit(old_chunk)
    assert not manager.allocations
    assert manager.provider_states["teleop"] is ProviderState.READY
    assert manager.provider_states["planner"] is ProviderState.READY


def test_reset_failure_rolls_controller_back_and_marks_new_provider_failed():
    robot = _robot()
    manager = ExecutionManager(robot)
    planner = FakeProvider(reset_error=RuntimeError("reset failed"))
    manager.register(_registration("planner", planner, "joint_trajectory"))
    asyncio.run(manager.prepare("planner"))

    with pytest.raises(HandoverError, match="reset failed"):
        asyncio.run(manager.handover("planner", {"q": [0.2]}))

    assert robot.parts["arm"].active_controller.controller_name == "arm_task"
    assert manager.provider_states["planner"] is ProviderState.FAILED
    assert not manager.allocations


def test_cross_controller_handover_requires_current_observation():
    manager = ExecutionManager(_robot())
    planner = FakeProvider()
    manager.register(_registration("planner", planner, "joint_trajectory"))
    asyncio.run(manager.prepare("planner"))

    with pytest.raises(ValueError, match="current observation"):
        asyncio.run(manager.handover("planner"))


def test_fault_releases_allocation_and_rejects_previous_chunk():
    events = []
    manager = ExecutionManager(_robot(), events.append)
    provider = FakeProvider()
    manager.register(_registration("teleop", provider))
    asyncio.run(manager.prepare("teleop"))
    generation = asyncio.run(manager.handover("teleop"))
    chunk = ActionChunk("teleop", generation, ("arm",), 0, object())

    asyncio.run(manager.fail("teleop", "controller_fault"))

    assert manager.provider_states["teleop"] is ProviderState.FAILED
    assert not manager.allocations
    assert not manager.admit(chunk)
    assert any(
        event.kind == "provider_failed" and event.reason == "controller_fault"
        for event in events
    )


def test_lower_priority_provider_cannot_displace_active_owner():
    events = []
    manager = ExecutionManager(_robot(), events.append)
    teleop = FakeProvider()
    policy = FakeProvider()
    manager.register(_registration("teleop", teleop, priority=100))
    manager.register(_registration("policy", policy, priority=10))
    asyncio.run(manager.prepare("teleop"))
    asyncio.run(manager.prepare("policy"))
    generation = asyncio.run(manager.handover("teleop"))

    with pytest.raises(ArbitrationRejected, match="teleop"):
        asyncio.run(manager.handover("policy"))

    assert manager.allocations["arm"].provider == "teleop"
    assert manager.allocations["arm"].generation == generation
    assert manager.provider_states["policy"] is ProviderState.READY
    assert events[-1].kind == "arbitration_rejected"


def test_release_stops_teleop_and_atomically_restores_displaced_policy():
    events = []
    manager = ExecutionManager(_robot(), events.append)
    policy = FakeProvider()
    teleop = FakeProvider()
    manager.register(_registration("policy", policy, priority=10))
    manager.register(_registration("teleop", teleop, priority=100))
    asyncio.run(manager.prepare("policy"))
    asyncio.run(manager.prepare("teleop"))
    policy_generation = asyncio.run(manager.handover("policy"))
    teleop_generation = asyncio.run(manager.handover("teleop"))

    resumed_generation = asyncio.run(
        manager.release("teleop", observation={"q": [0.3]}, reason="operator_done")
    )

    assert resumed_generation > teleop_generation > policy_generation
    assert manager.provider_states["teleop"] is ProviderState.STOPPED
    assert manager.provider_states["policy"] is ProviderState.ACTIVE
    assert manager.allocations["arm"].provider == "policy"
    assert manager.allocations["arm"].generation == resumed_generation
    assert teleop.stopped == 1
    assert policy.started == 1
    assert policy.observations[-1] == {"q": [0.3]}
    assert not manager.admit(
        ActionChunk("teleop", teleop_generation, ("arm",), 0, object())
    )
    assert manager.admit(
        ActionChunk("policy", resumed_generation, ("arm",), 0, object())
    )
    kinds = [event.kind for event in events]
    assert "provider_release_started" in kinds
    assert "provider_released" in kinds
    assert any(
        event.kind == "provider_state"
        and event.provider == "teleop"
        and event.reason == "STOPPED"
        for event in events
    )
    handback = [
        event
        for event in events
        if event.kind == "provider_handover" and event.provider == "policy"
    ][-1]
    assert handback.reason == "resumed_after:teleop"


def test_partial_arm_takeover_preserves_eef_and_restores_active_policy():
    manager = ExecutionManager(_robot())
    policy = FakeProvider()
    teleop = FakeProvider()
    manager.register(
        ProviderRegistration(
            name="policy",
            provider=policy,
            controllers={
                "arm": "task_space_reference",
                "gripper": "joint_space_reference",
            },
            priority=10,
        )
    )
    manager.register(_registration("teleop", teleop, priority=100))
    asyncio.run(manager.prepare("policy"))
    asyncio.run(manager.prepare("teleop"))

    policy_generation = asyncio.run(manager.handover("policy"))
    teleop_generation = asyncio.run(manager.handover("teleop"))

    assert manager.provider_states["policy"] is ProviderState.ACTIVE
    assert manager.allocations["arm"].provider == "teleop"
    assert manager.allocations["arm"].generation == teleop_generation
    assert manager.allocations["gripper"].provider == "policy"
    assert manager.allocations["gripper"].generation == policy_generation

    resumed_generation = asyncio.run(
        manager.release("teleop", observation={"q": [0.3]})
    )

    assert manager.provider_states["policy"] is ProviderState.ACTIVE
    assert manager.allocations["arm"].provider == "policy"
    assert manager.allocations["arm"].generation == resumed_generation
    assert manager.allocations["gripper"].provider == "policy"
    assert manager.allocations["gripper"].generation == policy_generation
    assert manager.admit(
        ActionChunk("policy", policy_generation, ("gripper",), 0, object())
    )
    assert not manager.admit(
        ActionChunk("policy", policy_generation, ("arm",), 0, object())
    )


def test_release_can_explicitly_select_ready_provider():
    manager = ExecutionManager(_robot())
    policy_a = FakeProvider()
    policy_b = FakeProvider()
    teleop = FakeProvider()
    manager.register(_registration("policy_a", policy_a, priority=10))
    manager.register(_registration("policy_b", policy_b, priority=20))
    manager.register(_registration("teleop", teleop, priority=100))
    for name in ("policy_a", "policy_b", "teleop"):
        asyncio.run(manager.prepare(name))
    asyncio.run(manager.handover("policy_a"))
    asyncio.run(manager.handover("teleop"))

    generation = asyncio.run(manager.release("teleop", next_provider="policy_b"))

    assert manager.allocations["arm"].provider == "policy_b"
    assert manager.allocations["arm"].generation == generation
    assert manager.provider_states["policy_a"] is ProviderState.READY
    assert manager.provider_states["policy_b"] is ProviderState.ACTIVE


def test_release_without_eligible_provider_stops_without_allocation():
    manager = ExecutionManager(_robot())
    teleop = FakeProvider()
    manager.register(_registration("teleop", teleop, priority=100))
    asyncio.run(manager.prepare("teleop"))
    asyncio.run(manager.handover("teleop"))

    generation = asyncio.run(manager.release("teleop"))

    assert generation is None
    assert manager.provider_states["teleop"] is ProviderState.STOPPED
    assert not manager.allocations


def test_composite_tracks_share_epoch_but_advance_independently():
    manager = ExecutionManager(_robot())
    provider = FakeProvider()
    manager.register(
        ProviderRegistration(
            name="policy",
            provider=provider,
            controllers={
                "arm": "task_space_reference",
                "gripper": "joint_space_reference",
            },
        )
    )
    asyncio.run(manager.prepare("policy"))
    generation = asyncio.run(manager.handover("policy"))

    assert manager.allocations["arm"].generation == generation
    assert manager.allocations["gripper"].generation == generation
    assert manager.admit(ActionChunk("policy", generation, ("arm",), 0, "a0"))
    assert manager.admit(ActionChunk("policy", generation, ("arm",), 1, "a1"))
    assert manager.admit(ActionChunk("policy", generation, ("gripper",), 0, "g0"))
    assert not manager.admit(
        ActionChunk("policy", generation, ("arm",), 1, "duplicate")
    )
    assert manager.admit(ActionChunk("policy", generation, ("gripper",), 3, "g3"))


def test_multi_part_chunk_admission_is_atomic_across_tracks():
    manager = ExecutionManager(_robot())
    provider = FakeProvider()
    manager.register(
        ProviderRegistration(
            name="policy",
            provider=provider,
            controllers={
                "arm": "task_space_reference",
                "gripper": "joint_space_reference",
            },
        )
    )
    asyncio.run(manager.prepare("policy"))
    generation = asyncio.run(manager.handover("policy"))
    assert manager.admit(ActionChunk("policy", generation, ("arm",), 2, "a2"))

    assert not manager.admit(
        ActionChunk("policy", generation, ("arm", "gripper"), 1, "out_of_order")
    )
    assert manager.admit(ActionChunk("policy", generation, ("gripper",), 1, "g1"))


def test_chunk_rejects_duplicate_parts_and_negative_sequence():
    events = []
    manager = ExecutionManager(_robot(), events.append)
    provider = FakeProvider()
    manager.register(_registration("policy", provider))
    asyncio.run(manager.prepare("policy"))
    generation = asyncio.run(manager.handover("policy"))

    assert not manager.admit(
        ActionChunk("policy", generation, ("arm", "arm"), 0, object())
    )
    assert not manager.admit(ActionChunk("policy", generation, ("arm",), -1, object()))
    assert [event.reason for event in events if event.kind == "chunk_rejected"] == [
        "invalid_parts",
        "non_monotonic_sequence",
    ]


def test_dispatch_fences_then_awaits_native_controller_client():
    manager = ExecutionManager(_robot())
    provider = FakeProvider()
    manager.register(
        _registration("trajectory", provider, "joint_trajectory", priority=10)
    )
    asyncio.run(manager.prepare("trajectory"))
    generation = asyncio.run(manager.handover("trajectory", observation={"q": [0.0]}))
    payload = object()

    accepted = asyncio.run(
        manager.dispatch(
            ActionChunk("trajectory", generation, ("arm",), 0, payload),
            "joint_trajectory",
        )
    )
    stale = asyncio.run(
        manager.dispatch(
            ActionChunk("trajectory", generation - 1, ("arm",), 1, object()),
            "joint_trajectory",
        )
    )

    client = manager.robot.parts["arm"].active_controller
    assert accepted is True
    assert stale is False
    assert client.messages == [payload]


def test_dispatch_rejects_multi_part_payload_without_per_part_tracks():
    manager = ExecutionManager(_robot())

    with pytest.raises(ValueError, match="exactly one Part"):
        asyncio.run(
            manager.dispatch(
                ActionChunk("policy", 1, ("arm", "gripper"), 0, object()),
                "joint_reference",
            )
        )


def _activated_trajectory_manager(events=None):
    manager = ExecutionManager(_robot(), events.append if events is not None else None)
    provider = FakeProvider()
    manager.register(_registration("planner", provider, "joint_trajectory"))
    asyncio.run(manager.prepare("planner"))
    asyncio.run(manager.handover("planner", {"q": [0.0]}))
    client = manager.robot.parts["arm"].controllers["joint_trajectory"]
    return manager, provider, client


def test_fail_cancels_in_flight_goal_and_fences_allocation():
    manager, _provider, client = _activated_trajectory_manager()

    asyncio.run(manager.fail("planner", "controller_fault"))

    assert client.cancel_count == 1
    assert not manager.allocations
    assert manager.provider_states["planner"] is ProviderState.FAILED


def test_stop_cancels_in_flight_goal():
    manager, provider, client = _activated_trajectory_manager()

    asyncio.run(manager.stop("planner"))

    assert client.cancel_count == 1
    assert provider.stopped == 1
    assert manager.provider_states["planner"] is ProviderState.STOPPED


def test_release_without_resume_cancels_goal():
    manager, _provider, client = _activated_trajectory_manager()

    generation = asyncio.run(manager.release("planner"))

    assert generation is None
    assert client.cancel_count == 1
    assert not manager.allocations


def test_same_controller_takeover_cancels_displaced_goal():
    manager, _provider, client = _activated_trajectory_manager()
    replay = FakeProvider()
    manager.register(_registration("replay", replay, "joint_trajectory", priority=5))
    asyncio.run(manager.prepare("replay"))

    asyncio.run(manager.handover("replay", {"q": [0.1]}))

    assert client.cancel_count == 1
    assert manager.allocations["arm"].provider == "replay"


def test_failing_event_sink_does_not_break_operations():
    def broken_sink(_event):
        raise RuntimeError("sink exploded")

    manager = ExecutionManager(_robot(), broken_sink)
    provider = FakeProvider()
    manager.register(_registration("teleop", provider))
    asyncio.run(manager.prepare("teleop"))

    generation = asyncio.run(manager.handover("teleop"))

    assert generation == 1
    assert manager.provider_states["teleop"] is ProviderState.ACTIVE
