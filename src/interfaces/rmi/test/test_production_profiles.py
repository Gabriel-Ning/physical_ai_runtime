from pathlib import Path

import pytest
import yaml
from rmi import EmbodimentConfig

ROOT = Path(__file__).parents[4]
PROFILES = ROOT / "apps" / "profiles"


@pytest.mark.parametrize(
    "profile_name",
    ["fr3_pika_single_arm.yaml", "marvin_bimanual.yaml", "piper_bimanual.yaml"],
)
def test_every_agent_resource_is_an_em_capability(profile_name):
    config = EmbodimentConfig.from_yaml(PROFILES / profile_name)
    for agent in config.agents.values():
        for resource, command in agent.resources.items():
            part = config.parts[resource]
            topic_commands = {
                name
                for controller in part.controllers.values()
                for name in controller.ros_topics
            }
            action_commands = {
                "joint_trajectory"
                for controller in part.controllers.values()
                if "follow_joint_trajectory" in controller.ros_actions
            }
            assert command in topic_commands | action_commands


@pytest.mark.parametrize(
    "path",
    [
        "src/bringup/franka_manipulation/workstation_launch/config/recording/rmi_fr3_policy.yaml",
        "src/bringup/marvin_manipulation/workstation_launch/config/recording/marvin_manipulation.yaml",
        "src/bringup/piper_manipulation/workstation_launch/config/recording/rmi_piper_bimanual.yaml",
    ],
)
def test_recording_contracts_use_typed_authority_and_leased_envelopes(path):
    contract = yaml.safe_load((ROOT / path).read_text(encoding="utf-8"))
    streams = contract["streams"]
    by_topic = {stream["topic"]: stream for stream in streams}
    assert by_topic["/execution_manager/authority_status"]["expected_type"] == (
        "execution_manager_interfaces/msg/AuthorityStatus"
    )
    assert by_topic["/execution_manager/authority_events"]["expected_type"] == (
        "execution_manager_interfaces/msg/AuthorityEvent"
    )
    for stream in streams:
        if stream["topic"].startswith("/action_sources/"):
            assert stream["expected_type"].startswith(
                "execution_manager_interfaces/msg/Leased"
            )
        assert "CandidateDecision" not in stream["expected_type"]
        assert "SelectionState" not in stream["expected_type"]


def test_no_priority_or_static_provider_tables_in_production_profiles():
    for profile_path in PROFILES.glob("*.yaml"):
        raw = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
        assert "provider_selection" not in raw
        for agent in raw["agents"].values():
            assert "priority" not in agent
            assert "provider" not in agent
            assert set(agent) <= {"source_role", "resources", "frequency"}
