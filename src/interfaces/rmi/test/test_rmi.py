from copy import deepcopy
from pathlib import Path

import pytest
from rmi import EmbodimentConfig

PROFILES = Path(__file__).parents[4] / "apps" / "profiles"


@pytest.mark.parametrize(
    "name,part_count",
    [
        ("fr3_pika_single_arm.yaml", 2),
        ("piper_bimanual.yaml", 4),
        ("marvin_bimanual.yaml", 4),
    ],
)
def test_production_profiles_use_dynamic_node_contracts(name, part_count):
    config = EmbodimentConfig.from_yaml(PROFILES / name)
    assert len(config.parts) == part_count
    assert config.nodes
    assert all(
        node.source_role in {"POLICY", "TELEOP", "PLANNER", "MEMORY"}
        for node in config.nodes.values()
    )
    assert all(node.resources for node in config.nodes.values())
    assert "provider_selection" not in config.raw_data


def _minimal_profile():
    return {
        "metadata": {"name": "test"},
        "groups": {
            "arm": {
                "type": "arm",
                "joint_names": ["j1"],
                "controller_manager": "/controller_manager",
                "default_controller": "joint_trajectory",
                "controllers": {
                    "joint_trajectory": {
                        "name": "arm_jtc",
                        "ros_actions": {
                            "follow_joint_trajectory": "/execution/arm/follow_joint_trajectory"
                        },
                    },
                    "joint_space_reference": {
                        "name": "arm_jspc",
                        "ros_topics": {
                            "joint_reference": "/execution/arm/joint_reference"
                        },
                    },
                },
            }
        },
        "nodes": {
            "Policy": {
                "source_role": "POLICY",
                "resources": {"arm": "joint_reference"},
            }
        },
    }


def test_profile_rejects_removed_static_provider_table():
    profile = _minimal_profile()
    profile["provider_selection"] = {}
    with pytest.raises(ValueError, match="provider_selection is removed"):
        EmbodimentConfig.from_dict(profile)


@pytest.mark.parametrize("role", ["AUTO", "", 1])
def test_profile_rejects_unknown_source_role(role):
    profile = deepcopy(_minimal_profile())
    profile["nodes"]["Policy"]["source_role"] = role
    with pytest.raises((TypeError, ValueError), match="source_role"):
        EmbodimentConfig.from_dict(profile)


def test_profile_rejects_unknown_resource_or_command():
    profile = deepcopy(_minimal_profile())
    profile["nodes"]["Policy"]["resources"] = {"base": "joint_reference"}
    with pytest.raises(ValueError, match="unknown resource"):
        EmbodimentConfig.from_dict(profile)

    profile = deepcopy(_minimal_profile())
    profile["nodes"]["Policy"]["resources"] = {"arm": "force_reference"}
    with pytest.raises(ValueError, match="unsupported command"):
        EmbodimentConfig.from_dict(profile)


def test_marvin_no_camera_profile_only_selects_no_camera_recording():
    config = EmbodimentConfig.from_yaml(
        PROFILES / "site" / "marvin_bimanual_no_cam.yaml"
    )
    assert "right_gripper" in config.nodes["Policy"].resources
    assert "right_gripper" in config.nodes["TeleopJoint"].resources
    assert set(config.cameras) == {
        "left_pika_d405",
        "left_pika_fisheye",
        "right_pika_d405",
        "right_pika_fisheye",
        "head_d435",
        "third_person_d435",
    }
    assert config.recording["config"].endswith(
        "recording/marvin_manipulation_no_cam.yaml"
    )
