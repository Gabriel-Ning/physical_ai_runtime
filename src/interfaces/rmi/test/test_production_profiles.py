from pathlib import Path

import pytest
import yaml
from rmi import EmbodimentConfig
from rmi.context import _resolve_profile_path

ROOT = Path(__file__).parents[4]
PROFILES = ROOT / "apps" / "profiles"


def test_profile_resolver_preserves_relative_site_directory():
    assert _resolve_profile_path("site/marvin_bimanual_no_cam.yaml") == (
        PROFILES / "site" / "marvin_bimanual_no_cam.yaml"
    )


@pytest.mark.parametrize(
    "profile_name",
    ["fr3_pika_single_arm.yaml", "marvin_bimanual.yaml", "piper_bimanual.yaml"],
)
def test_every_node_resource_is_an_em_capability(profile_name):
    config = EmbodimentConfig.from_yaml(PROFILES / profile_name)
    assert config.nodes
    for node in config.nodes.values():
        for resource, command in node.resources.items():
            part = config.parts[resource]
            topic_commands = {
                name
                for controller in part.controllers.values()
                for name in controller.ros_topics
            }
            action_commands = {
                command
                for controller in part.controllers.values()
                for action, command in {
                    "follow_joint_trajectory": "joint_trajectory",
                    "gripper_command": "gripper_command",
                }.items()
                if action in controller.ros_actions
            }
            assert command in topic_commands | action_commands


@pytest.mark.parametrize(
    "path",
    [
        "apps/recording/franka_manipulation.yaml",
        "apps/recording/franka_manipulation_no_cam.yaml",
        "apps/recording/marvin_manipulation.yaml",
        "apps/recording/marvin_manipulation_no_cam.yaml",
        "apps/recording/piper_bimanual.yaml",
    ],
)
def test_recording_contracts_use_typed_authority_and_commands(path):
    contract = yaml.safe_load((ROOT / path).read_text(encoding="utf-8"))
    streams = contract["streams"]
    by_topic = {stream["topic"]: stream for stream in streams}

    # Authority status must be required with start_gate: true
    auth_status = by_topic["/execution_manager/authority_status"]
    assert auth_status["expected_type"] == (
        "execution_manager_interfaces/msg/AuthorityStatus"
    )
    assert auth_status["required"] is True
    assert auth_status["start_gate"] is True

    # Authority events must be required
    auth_events = by_topic["/execution_manager/authority_events"]
    assert auth_events["expected_type"] == (
        "execution_manager_interfaces/msg/AuthorityEvent"
    )
    assert auth_events["required"] is True

    for stream in streams:
        # Application-facing action sources use ordinary ROS messages. Lease
        # envelopes exist only inside EM and may appear on execution traces.
        if stream["topic"].startswith("/action_sources/"):
            assert not stream["expected_type"].startswith(
                "execution_manager_interfaces/msg/Leased"
            )
        assert "CandidateDecision" not in stream["expected_type"]
        assert "SelectionState" not in stream["expected_type"]


def test_no_priority_or_static_provider_tables_in_production_profiles():
    for profile_path in PROFILES.glob("*.yaml"):
        raw = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
        assert "provider_selection" not in raw
        nodes = raw.get("nodes") or raw.get("agents", {})
        for node in nodes.values():
            assert "priority" not in node
            assert "provider" not in node
            assert set(node) <= {"source_role", "resources", "frequency"}
