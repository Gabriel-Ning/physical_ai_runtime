from copy import deepcopy
from pathlib import Path

import pytest
from rmi import EmbodimentConfig, JointLayout, PolicyLayout

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


def test_profile_rejects_duplicate_camera_topics():
    profile = deepcopy(_minimal_profile())
    profile["sensors"] = {
        "cameras": {
            "policy_view": {"ros_topic": "/camera/image"},
            "readiness_view": {"ros_topic": "/camera/image"},
        }
    }

    with pytest.raises(ValueError, match="camera topic.*assigned to both"):
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


def test_joint_layout_resolves_node_order_and_part_slices():
    profile = _minimal_profile()
    arm = profile["groups"]["arm"]
    arm["joint_names"] = ["j1", "j2"]
    profile["groups"]["gripper"] = {
        **deepcopy(arm),
        "type": "gripper",
        "joint_names": ["gripper_joint"],
    }
    profile["nodes"]["Policy"]["resources"]["gripper"] = "joint_reference"

    layout = EmbodimentConfig.from_dict(profile).joint_layout("Policy")

    assert isinstance(layout, JointLayout)
    assert layout.joint_names == ("j1", "j2", "gripper_joint")
    assert layout.dimension == 3
    assert [(group.part, group.start, group.stop) for group in layout.groups] == [
        ("arm", 0, 2),
        ("gripper", 2, 3),
    ]
    assert layout.order_values(
        ["gripper_joint", "j2", "j1"], [3, 2, 1]
    ) == [1.0, 2.0, 3.0]
    assert [values for _, values in layout.split_values([1, 2, 3])] == [
        [1.0, 2.0],
        [3.0],
    ]


def test_joint_layout_rejects_incomplete_named_values():
    layout = EmbodimentConfig.from_dict(_minimal_profile()).joint_layout("Policy")

    with pytest.raises(ValueError, match="missing required names"):
        layout.order_values([], [])
    with pytest.raises(ValueError, match="must be unique"):
        layout.order_values(["j1", "j1"], [1, 2])
    with pytest.raises(ValueError, match="dimension"):
        layout.split_values([])


def test_policy_layout_resolves_shared_frequency_and_action_dimension():
    profile = _minimal_profile()
    profile["nodes"]["Policy"]["frequency"] = 30.0
    profile["features"] = {"action": {"action": {"shape": [1]}}}

    layout = EmbodimentConfig.from_dict(profile).policy_layout()

    assert isinstance(layout, PolicyLayout)
    assert layout.joints.joint_names == ("j1",)
    assert layout.state_topic == "/joint_states"
    assert layout.action_topics == {
        "arm": "/execution/arm/joint_reference"
    }
    assert layout.state_feature_names == ("j1.pos",)
    assert layout.action_feature_names == ("j1.pos",)
    assert layout.frequency == 30.0

    profile["features"]["action"]["action"]["shape"] = [2]
    with pytest.raises(ValueError, match="resolved action dimension 1"):
        EmbodimentConfig.from_dict(profile).policy_layout()
