from dataclasses import replace
from types import SimpleNamespace

from execution_manager_interfaces.msg import AuthorityEvent, ResourceAuthority
import pytest
from rmi import Action, Agent, Robot
from rmi.selection import EndpointBinding, LeaseGrant
from sensor_msgs.msg import JointState


class FakeCommandClient:
    def __init__(self):
        self.bound = ""
        self.sent = []

    def bind(self, grant):
        self.bound = grant.lease_id

    def unbind(self):
        self.bound = ""

    def send(self, part, command, value):
        self.sent.append((self.bound, part, command, value))


class FakeAuthority:
    def __init__(self):
        self.allocations = {}
        self.counter = 0
        self.releases = []
        self.events = []

    def claim(self, role, instance, resources, *, preempt=False, metadata=None):
        del metadata
        if not preempt and any(part in self.allocations for part in resources):
            raise RuntimeError("resources_busy_preempt_required")
        displaced = {
            value["lease_id"]
            for part, value in self.allocations.items()
            if part in resources
        }
        self.events.extend(
            SimpleNamespace(type=AuthorityEvent.PREEMPTED, lease_id=lease_id)
            for lease_id in displaced
        )
        self.allocations = {
            part: value
            for part, value in self.allocations.items()
            if value["lease_id"] not in displaced
        }
        self.counter += 1
        lease_id = f"lease-{self.counter}"
        for part, command in resources.items():
            self.allocations[part] = {
                "authority_state": ResourceAuthority.OWNED,
                "lease_id": lease_id,
                "source_instance": instance,
                "source_role": role,
                "command_contract": command,
            }
        endpoints = {
            (part, command): EndpointBinding(
                part,
                command,
                f"/action_sources/{str(role).lower()}/{part}/{command}",
                command == "joint_trajectory",
            )
            for part, command in resources.items()
        }
        return LeaseGrant(lease_id, endpoints)

    def release(self, lease_id):
        self.releases.append(lease_id)
        self.allocations = {
            part: value
            for part, value in self.allocations.items()
            if value["lease_id"] != lease_id
        }

    def get_allocations(self):
        return {part: dict(value) for part, value in self.allocations.items()}

    def get_events(self, *, lease_id=None):
        if lease_id is None:
            return list(self.events)
        return [event for event in self.events if event.lease_id == lease_id]


def _fixture():
    authority = FakeAuthority()
    robot = Robot(SimpleNamespace(name="test"), authority)
    state = JointState()
    state.name = ["joint"]
    state.position = [0.0]
    robot.update_joint_state(state)
    policy_client = FakeCommandClient()
    teleop_client = FakeCommandClient()
    policy = Agent(
        "policy-v2",
        policy_client,
        source_role="POLICY",
        resources={"arm": "joint_reference"},
        robot=robot,
    )
    teleop = Agent(
        "human-left",
        teleop_client,
        source_role="TELEOP",
        resources={"arm": "joint_reference"},
        robot=robot,
    )
    return robot, authority, policy, policy_client, teleop, teleop_client


def test_session_claim_binds_commands_and_releases_exact_lease():
    robot, authority, policy, client, _, _ = _fixture()
    with policy.run(robot) as session:
        assert session.active
        assert session.lease_id == "lease-1"
        session.act(Action("arm", "joint_reference", [0.1]))
        assert client.sent[0][0] == "lease-1"
    assert authority.releases == ["lease-1"]
    assert client.bound == ""


def test_takeover_fences_old_session_and_requires_explicit_reacquire():
    robot, authority, policy, policy_client, teleop, _ = _fixture()
    with policy.run(robot) as old_policy:
        with teleop.run(robot, preempt=True) as human:
            assert human.active
            assert not old_policy.active
            old_policy.act(Action("arm", "joint_reference", [0.2]))
            assert old_policy.diagnostics.inactive_drops == 1
        assert not old_policy.active
    assert "lease-1" not in authority.releases

    with policy.run(robot) as new_policy:
        assert new_policy.lease_id == "lease-3"
        assert new_policy.active
    assert policy_client.bound == ""


def test_partial_takeover_invalidates_entire_multi_resource_lease():
    authority = FakeAuthority()
    old = authority.claim(
        "POLICY",
        "policy",
        {"left_arm": "joint_reference", "right_arm": "joint_reference"},
    )
    authority.claim(
        "TELEOP",
        "human",
        {"left_arm": "joint_reference"},
        preempt=True,
    )

    allocations = authority.get_allocations()
    assert old.lease_id not in {value["lease_id"] for value in allocations.values()}
    assert "right_arm" not in allocations


def test_observation_lease_prevents_stale_action():
    robot, _, policy, client, _, _ = _fixture()
    with policy.run(robot) as session:
        observation = session.observe()
        stale = dict(observation.allocations["arm"])
        stale["lease_id"] = "old-lease"
        observation = replace(observation, allocations={"arm": stale})
        session.act(Action("arm", "joint_reference", [0.3]), observation=observation)
        assert client.sent == []
        assert session.diagnostics.stale_observation_drops == 1


def test_observation_lease_matches_custom_source_instance():
    authority = FakeAuthority()
    robot = Robot(SimpleNamespace(name="test"), authority)
    state = JointState()
    state.name = ["joint"]
    state.position = [0.0]
    robot.update_joint_state(state)
    client = FakeCommandClient()
    agent = Agent(
        "Replay",
        client,
        source_role="POLICY",
        source_instance="replay:episode_042",
        resources={"arm": "joint_reference"},
        robot=robot,
    )
    assert agent.name != agent.source_instance
    with agent.run(robot) as session:
        observation = session.observe()
        session.act(Action("arm", "joint_reference", [0.5]), observation=observation)
        assert client.sent == [("lease-1", "arm", "joint_reference", [0.5])]
        assert session.diagnostics.stale_observation_drops == 0
        assert session.diagnostics.sent == 1


def test_confirmation_timeout_releases_claim_even_without_status():
    robot, authority, policy, client, _, _ = _fixture()
    authority.get_allocations = lambda: {}
    with pytest.raises(TimeoutError):
        with policy.run(robot, acquire_timeout=0.02):
            pass
    assert authority.releases == ["lease-1"]
    assert client.bound == ""


def test_session_exit_releases_claim_when_status_becomes_unknown():
    robot, authority, policy, client, _, _ = _fixture()
    with policy.run(robot) as session:
        assert session.lease_id == "lease-1"
        authority.get_allocations = lambda: {}

    assert authority.releases == ["lease-1"]
    assert session.diagnostics.displaced_exits == 0
    assert client.bound == ""


@pytest.mark.parametrize("reported_lease_id", ["lease-1", ""])
def test_session_exit_releases_non_owned_without_replacement(reported_lease_id):
    robot, authority, policy, client, _, _ = _fixture()
    with policy.run(robot) as session:
        assert session.lease_id == "lease-1"
        authority.allocations["arm"]["authority_state"] = ResourceAuthority.UNOWNED
        authority.allocations["arm"]["lease_id"] = reported_lease_id

    assert authority.releases == ["lease-1"]
    assert session.diagnostics.displaced_exits == 0
    assert client.bound == ""


def test_explicit_hardware_check_fails_when_authority_has_no_diagnostics():
    robot, _, _, _, _, _ = _fixture()

    with pytest.raises(RuntimeError, match="hardware diagnostics are not supported"):
        robot.wait_until_ready(check_hardware=True)
