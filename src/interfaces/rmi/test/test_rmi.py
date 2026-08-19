import asyncio
from pathlib import Path
from types import SimpleNamespace

import pytest
from rmi import EmbodimentConfig, RobotTopology


def _profile() -> dict:
    return {
        "metadata": {"name": "dual_arm"},
        "groups": {
            "left_arm": {
                "type": "arm",
                "joint_names": ["left_joint_1"],
                "controller_manager": "/controller_manager",
                "default_controller": "joint_trajectory",
                "controllers": {
                    "joint_trajectory": {
                        "name": "left_arm_jtc",
                        "implementation": "joint_trajectory_controller/JointTrajectoryController",
                        "command_interface": "position",
                        "ros_actions": {
                            "follow_joint_trajectory": "/left_arm_jtc/follow_joint_trajectory"
                        },
                    }
                },
            },
            "left_gripper": {
                "type": "end_effector",
                "joint_names": ["left_finger_joint"],
                "parent_group": "left_arm",
                "controller_manager": "/controller_manager",
                "default_controller": "joint_space_reference",
                "controllers": {
                    "joint_space_reference": {
                        "name": "left_gripper_controller",
                        "implementation": "position_controllers/GripperActionController",
                        "command_interface": "position",
                        "ros_actions": {
                            "gripper_command": "/left_gripper_controller/gripper_cmd"
                        },
                    }
                },
            },
        },
        "compound_groups": {
            "left_manipulator": {"included_groups": ["left_arm", "left_gripper"]}
        },
    }


class FakeController:
    def __init__(self, name: str) -> None:
        self.controller_name = name


class FakeSwitcher:
    def __init__(self, manager: str) -> None:
        self.manager = manager
        self.active = None

        self.requests = []

    async def switch_controller(self, *, activate, deactivate) -> None:
        self.requests.append((activate, deactivate))
        self.active = activate[0]


class DiscoveringSwitcher(FakeSwitcher):
    async def active_controllers(self, candidates):
        if self.active in candidates:
            return (self.active,)
        return ()

    async def switch_controller(self, *, activate, deactivate) -> None:
        self.requests.append((activate, deactivate))
        self.active = activate[0] if activate else None


class FakeRosNode:
    def create_client(self, service_type, endpoint):
        return SimpleNamespace(service_type=service_type, endpoint=endpoint)

    def create_publisher(self, message_type, endpoint, qos):
        return SimpleNamespace(
            message_type=message_type,
            endpoint=endpoint,
            qos=qos,
            publish=lambda _: None,
        )

    def create_timer(self, period_sec, callback):
        return SimpleNamespace(period_sec=period_sec, callback=callback)


class FakeActionClient:
    def __init__(self, node, action_type, endpoint):
        self.node = node
        self.action_type = action_type
        self.endpoint = endpoint


def _robot() -> RobotTopology:
    config = EmbodimentConfig.from_dict(_profile())
    return RobotTopology(
        config,
        lambda _part, _contract, cfg: FakeController(cfg.name),
        FakeSwitcher,
    )


def test_robot_exposes_parts_groups_and_direct_controller_clients():
    robot = _robot()

    assert (
        robot.parts["left_arm"].controllers["joint_trajectory"].controller_name
        == "left_arm_jtc"
    )
    assert [part.name for part in robot.groups["left_manipulator"]] == [
        "left_arm",
        "left_gripper",
    ]
    assert not hasattr(robot, "execution_manager")
    assert not hasattr(robot, "recorder")


def test_parts_share_one_switcher_for_one_remote_controller_manager():
    robot = _robot()

    assert (
        robot.parts["left_arm"].controller_switcher
        is robot.parts["left_gripper"].controller_switcher
    )


def test_switching_to_active_contract_is_a_noop_and_returns_direct_client():
    robot = _robot()

    controller = asyncio.run(
        robot.parts["left_arm"].switch_controller("joint_trajectory")
    )

    assert controller is robot.parts["left_arm"].controllers["joint_trajectory"]
    assert robot.parts["left_arm"].controller_switcher.active is None
    assert robot.parts["left_arm"].controller_switcher.requests == []


def test_switch_controller_deactivates_only_the_parts_active_controller():
    robot = _robot()
    arm = robot.parts["left_arm"]
    alternate = FakeController("left_arm_servo")
    arm.controllers = {**arm.controllers, "joint_space_reference": alternate}

    controller = asyncio.run(arm.switch_controller("joint_space_reference"))

    assert controller is alternate
    assert arm.active_controller is alternate
    assert arm.controller_switcher.requests == [
        (("left_arm_servo",), ("left_arm_jtc",))
    ]


def test_discovery_overrides_profile_default_when_no_controller_is_active():
    robot = _robot()
    arm = robot.parts["left_arm"]
    arm.controller_switcher = DiscoveringSwitcher("/controller_manager")

    controller = asyncio.run(arm.switch_controller("joint_trajectory"))

    assert controller is arm.active_controller
    assert arm.controller_switcher.requests == [(("left_arm_jtc",), ())]

    asyncio.run(arm.deactivate_active_controller())
    assert arm.active_controller is None
    assert arm.controller_switcher.requests[-1] == ((), ("left_arm_jtc",))


def test_profile_rejects_legacy_controller_shape():
    profile = _profile()
    profile["groups"]["left_arm"]["controllers"] = {
        "trajectory_execution": {
            "name": "left_arm_jtc",
            "type": "joint_trajectory_controller/JointTrajectoryController",
            "ros_actions": {
                "follow_joint_trajectory": "/left_arm_jtc/follow_joint_trajectory"
            },
        }
    }

    with pytest.raises(TypeError, match="implementation"):
        EmbodimentConfig.from_dict(profile)


def test_profile_uses_groups_as_parts_and_rejects_parts_alias():
    profile = _profile()
    profile["parts"] = profile.pop("groups")

    with pytest.raises(TypeError, match="groups must be a mapping"):
        EmbodimentConfig.from_dict(profile)


def test_profile_validates_parent_group():
    profile = _profile()
    profile["groups"]["left_gripper"]["parent_group"] = "missing_arm"

    with pytest.raises(ValueError, match="unknown parent"):
        EmbodimentConfig.from_dict(profile)


@pytest.mark.parametrize(
    "profile_name",
    [
        "fr3_pika_single_arm.yaml",
        "piper_bimanual.yaml",
        "marvin_bimanual.yaml",
    ],
)
def test_current_profiles_parse(profile_name):
    profile_path = (
        Path(__file__).parents[4]
        / "apps"
        / "profiles"
        / profile_name
    )

    config = EmbodimentConfig.from_yaml(profile_path)

    assert config.parts
    if profile_name in {"marvin_bimanual.yaml", "piper_bimanual.yaml"}:
        assert config.groups["dual_arm"] == ("left_arm", "right_arm")
    else:
        assert set(config.parts) == {"arm", "end_effector"}
        assert config.parts["end_effector"].parent == "arm"


@pytest.mark.parametrize(
    "profile_name, expected_parts",
    [
        ("fr3_pika_single_arm.yaml", 2),
        ("piper_bimanual.yaml", 4),
        ("marvin_bimanual.yaml", 4),
    ],
)
def test_robot_from_profile_builds_all_declared_clients(profile_name, expected_parts):
    profile_path = (
        Path(__file__).parents[4]
        / "apps"
        / "profiles"
        / profile_name
    )

    robot = RobotTopology.from_profile(
        profile_path,
        FakeRosNode(),
        action_client_factory=FakeActionClient,
    )

    assert len(robot.parts) == expected_parts
    assert all(part.controllers for part in robot.parts.values())


@pytest.mark.parametrize(
    "profile_name",
    ["marvin_bimanual.yaml", "piper_bimanual.yaml"],
)
def test_bimanual_profile_contains_complete_execution_manager_deployment(profile_name):
    profile_path = (
        Path(__file__).parents[4]
        / "apps"
        / "profiles"
        / profile_name
    )
    config = EmbodimentConfig.from_yaml(profile_path)

    providers = set(config.execution["providers"])
    assert {"Policy", "Planner"} <= providers
    for side in ("Left", "Right"):
        for kind in ("TeleopJoint", "TeleopCartesian", "TeleopTwist"):
            assert f"{kind}_{side}" in providers
    assert config.get_topic_endpoint(
        "left_arm", "joint_reference", provider="Policy"
    ) == ("/execution/left_arm/joint_reference")
    assert config.get_action_endpoint("right_arm", "joint_trajectory") == (
        "/execution/right_arm/follow_joint_trajectory"
    )
    assert set(config.parts) == {
        "left_arm",
        "right_arm",
        "left_gripper",
        "right_gripper",
    }
    assert config.agents["Policy"].provider == "Policy"
    assert config.agents["Policy"].frequency == 30.0


def test_profile_rejects_invalid_agent_provider_and_frequency():
    profile = _profile()
    profile["execution_manager"] = {
        "providers": {
            "Policy": {
                "priority": 10,
                "controllers": {"left_arm": "joint_trajectory"},
            }
        },
        "sources": [
            {
                "provider": "Policy",
                "part": "left_arm",
                "command": "joint_trajectory",
                "action": "/policy/trajectory",
            }
        ],
    }
    profile["agents"] = {"BadProvider": {"provider": "Missing", "frequency": 30.0}}
    with pytest.raises(ValueError, match="unknown provider"):
        EmbodimentConfig.from_dict(profile)

    profile["agents"] = {"BadFrequency": {"provider": "Policy", "frequency": 0.0}}
    with pytest.raises(ValueError, match="frequency must be positive"):
        EmbodimentConfig.from_dict(profile)


def test_profile_rejects_removed_ingress_key():
    with pytest.raises(ValueError, match="ingress is removed"):
        EmbodimentConfig.from_dict(
            {
                "metadata": {"name": "legacy"},
                "groups": {
                    "arm": {
                        "type": "arm",
                        "joint_names": ["j1"],
                        "controller_manager": "/controller_manager",
                        "default_controller": "joint_trajectory",
                        "controllers": {
                            "joint_trajectory": {
                                "name": "jtc",
                                "implementation": "joint_trajectory_controller/JointTrajectoryController",
                                "command_interface": "position",
                                "ros_actions": {
                                    "follow_joint_trajectory": "/jtc/follow_joint_trajectory"
                                },
                            }
                        },
                    }
                },
                "execution_manager": {"ingress": {}},
            }
        )
