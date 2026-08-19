from pathlib import Path

import yaml
from rmi import EmbodimentConfig

ROOT = Path(__file__).resolve().parents[4]
PROFILES = ROOT / "apps" / "profiles"
TEMPLATES = Path(__file__).resolve().parents[1] / "config" / "templates"
PRODUCTION_PROFILE_NAMES = {
    "fr3_pika_single_arm.yaml",
    "marvin_bimanual.yaml",
    "piper_bimanual.yaml",
}


def _controllers(app: str) -> dict:
    path = ROOT / "src" / "rt_launch" / app / "config" / "controller" / "controllers.yaml"
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_only_three_production_profiles_use_controller_local_endpoints():
    assert {path.name for path in PROFILES.glob("*.yaml")} == (PRODUCTION_PROFILE_NAMES)
    for profile_name in PRODUCTION_PROFILE_NAMES:
        config = EmbodimentConfig.from_yaml(PROFILES / profile_name)
        for part in config.parts.values():
            for controller in part.controllers.values():
                endpoints = (
                    *controller.ros_topics.values(),
                    *controller.ros_actions.values(),
                )
                assert endpoints
                assert all(
                    not endpoint.startswith("/execution_manager/")
                    for endpoint in endpoints
                )


def test_production_profiles_route_jtc_liveness_to_rt_guards():
    for profile_name in PRODUCTION_PROFILE_NAMES:
        config = EmbodimentConfig.from_yaml(PROFILES / profile_name)
        assert config.host_role("rt_host")["owns"] == [
            "ros2_control",
            "local_safety",
        ]
        assert "execution_manager" in config.host_role("policy_host")["owns"]
        for part_name, part in config.parts.items():
            trajectory = part.controllers.get("joint_trajectory")
            if trajectory is None:
                continue
            assert trajectory.ros_topics["trajectory_guard_heartbeat"] == (
                f"/execution/{part_name}/trajectory_guard_heartbeat"
            )


def test_production_profiles_expose_complete_manipulator_views():
    fr3 = EmbodimentConfig.from_yaml(PROFILES / "fr3_pika_single_arm.yaml")
    assert fr3.groups == {"manipulator": ("arm", "end_effector")}
    assert fr3.get_part_joints("manipulator") == (
        fr3.get_part_joints("arm") + fr3.get_part_joints("end_effector")
    )

    for profile_name in ("marvin_bimanual.yaml", "piper_bimanual.yaml"):
        config = EmbodimentConfig.from_yaml(PROFILES / profile_name)
        assert config.groups == {
            "left_manipulator": ("left_arm", "left_gripper"),
            "right_manipulator": ("right_arm", "right_gripper"),
            "dual_arm": ("left_arm", "right_arm"),
            "dual_manipulator": (
                "left_arm",
                "left_gripper",
                "right_arm",
                "right_gripper",
            ),
        }
        assert config.get_part_joints("dual_manipulator") == [
            joint
            for part in config.groups["dual_manipulator"]
            for joint in config.get_part_joints(part)
        ]


def test_fr3_pika_profile_matches_controller_and_description_contracts():
    config = EmbodimentConfig.from_yaml(PROFILES / "fr3_pika_single_arm.yaml")

    assert config.recording["profile"] == "rmi_fr3_policy"
    controllers = _controllers("franka_manipulation_controller_bringup")
    manager = controllers["controller_manager"]["ros__parameters"]

    assert config.parts["arm"].joint_names == tuple(
        f"fr3_joint{i}" for i in range(1, 8)
    )
    assert config.parts["arm"].tcp_frame == "pika_gripper_tcp"
    assert config.parts["end_effector"].joint_names == ("gripper_left_joint",)
    for route in config.parts["arm"].controllers.values():
        assert route.name in manager
    gripper = config.parts["end_effector"].controllers["joint_space_reference"]
    assert gripper.name == "pika_gripper_fwd"
    assert controllers[gripper.name]["ros__parameters"]["joints"] == [
        "gripper_left_joint"
    ]
    assert (
        config.parts["arm"]
        .controllers["joint_space_reference"]
        .ros_topics["joint_reference"]
        == controllers["franka_arm_jsic"]["ros__parameters"]["input_topic"]
    )
    task_topics = config.parts["arm"].controllers["task_space_reference"].ros_topics
    assert (
        task_topics["pose_reference"]
        == controllers["franka_arm_tsjic"]["ros__parameters"]["input_topic"]
    )
    assert (
        task_topics["twist_reference"]
        == controllers["franka_arm_tsjic"]["ros__parameters"]["twist_topic"]
    )


def test_marvin_pika_profile_matches_controller_contracts():
    config = EmbodimentConfig.from_yaml(PROFILES / "marvin_bimanual.yaml")
    controllers = _controllers("marvin_manipulation_controller_bringup")
    manager = controllers["controller_manager"]["ros__parameters"]

    for part in config.parts.values():
        for route in part.controllers.values():
            assert route.name in manager
    assert config.parts["left_gripper"].joint_names == ("left_gripper_left_joint",)
    assert config.parts["right_gripper"].joint_names == ("right_gripper_left_joint",)
    for side in ("left", "right"):
        arm_part = config.parts[f"{side}_arm"]
        arm = arm_part.controllers
        assert arm_part.tcp_frame == f"{side}_pika_gripper_tcp"
        assert (
            arm["joint_space_reference"].ros_topics["joint_reference"]
            == (controllers[f"{side}_arm_jspc"]["ros__parameters"]["input_topic"])
        )
        assert (
            arm["task_space_reference"].ros_topics["pose_reference"]
            == (controllers[f"{side}_arm_tskpc"]["ros__parameters"]["input_topic"])
        )
        assert (
            arm["task_space_reference"].ros_topics["twist_reference"]
            == (controllers[f"{side}_arm_tskpc"]["ros__parameters"]["twist_topic"])
        )


def test_dual_piper_native_gripper_profile_matches_controller_contracts():
    config = EmbodimentConfig.from_yaml(PROFILES / "piper_bimanual.yaml")
    controllers = _controllers("piper_manipulation_controller_bringup")
    manager = controllers["controller_manager"]["ros__parameters"]

    bringup_args = config.host_role("rt_host")["bringup"]["arguments"]
    assert bringup_args["left_end_effector"] == "piper_gripper"
    assert bringup_args["right_end_effector"] == "piper_gripper"
    for part in config.parts.values():
        for route in part.controllers.values():
            assert route.name in manager
    assert config.parts["left_gripper"].joint_names == ("left_gripper_joint1",)
    assert config.parts["right_gripper"].joint_names == ("right_gripper_joint1",)

    for side in ("left", "right"):
        arm_part = config.parts[f"{side}_arm"]
        arm = arm_part.controllers
        assert arm_part.tcp_frame == f"{side}_gripper_tcp"
        assert (
            arm["joint_space_reference"].ros_topics["joint_reference"]
            == (controllers[f"{side}_arm_jspc"]["ros__parameters"]["input_topic"])
        )
        assert (
            arm["task_space_reference"].ros_topics["pose_reference"]
            == (controllers[f"{side}_arm_tskpc"]["ros__parameters"]["input_topic"])
        )
        assert (
            arm["task_space_reference"].ros_topics["twist_reference"]
            == (controllers[f"{side}_arm_tskpc"]["ros__parameters"]["twist_topic"])
        )

    for side, provider in (
        ("left", "TeleopJoint_Left"),
        ("right", "TeleopJoint_Right"),
    ):
        provider_config = config.execution["providers"][provider]
        assert provider_config["controllers"] == {
            f"{side}_arm": "joint_space_reference",
            f"{side}_gripper": "joint_space_reference",
        }
        endpoints = {
            source["part"]: source["topic"]
            for source in config.execution["sources"]
            if source["provider"] == provider
        }
        assert endpoints == {
            f"{side}_arm": f"/execution/{side}_arm/joint_reference",
            f"{side}_gripper": f"/execution/{side}_gripper/joint_reference",
        }

    for side, provider in (
        ("left", "TeleopCartesian_Left"),
        ("right", "TeleopCartesian_Right"),
    ):
        assert config.execution["providers"][provider]["controllers"] == {
            f"{side}_arm": "task_space_reference"
        }
        pose_sources = [
            source
            for source in config.execution["sources"]
            if source["provider"] == provider
        ]
        assert {source["command"] for source in pose_sources} == {"pose_reference"}
        assert all(source["part"] == f"{side}_arm" for source in pose_sources)

    for side, provider in (
        ("left", "TeleopTwist_Left"),
        ("right", "TeleopTwist_Right"),
    ):
        assert config.execution["providers"][provider]["controllers"] == {
            f"{side}_arm": "task_space_reference"
        }
        twist_sources = [
            source
            for source in config.execution["sources"]
            if source["provider"] == provider
        ]
        assert {source["command"] for source in twist_sources} == {"twist_reference"}


def test_piper_leader_configs_match_teleop_provider_ingress():
    config = EmbodimentConfig.from_yaml(PROFILES / "piper_bimanual.yaml")
    teleop_config_dir = ROOT / "src" / "teleop" / "piper_leader_teleop" / "config"

    for side, provider in (
        ("left", "TeleopJoint_Left"),
        ("right", "TeleopJoint_Right"),
    ):
        loaded = yaml.safe_load(
            (teleop_config_dir / f"piper_leader_{side}.yaml").read_text(
                encoding="utf-8"
            )
        )
        leader = loaded.get("piper_leader", loaded.get("/**", {}))["ros__parameters"]
        sources = {
            source["part"]: source["topic"]
            for source in config.execution["sources"]
            if source["provider"] == provider
        }
        # Leader publishes on action_sources; LocalEM TeleopJoint ingress is
        # the RT /execution topic. examples/14_piper_leader_teleop.py relays.
        assert leader["joint_reference_topic"] == (
            f"/action_sources/piper_leader_{side}/arm/joint_reference"
        )
        assert leader["gripper_reference_topic"] == (
            f"/action_sources/piper_leader_{side}/end_effector/joint_reference"
        )
        assert sources[f"{side}_arm"] == f"/execution/{side}_arm/joint_reference"
        assert sources[f"{side}_gripper"] == (
            f"/execution/{side}_gripper/joint_reference"
        )
        assert leader["joint_names"] == list(config.parts[f"{side}_arm"].joint_names)
        assert (
            leader["gripper_joint_name"]
            == config.parts[f"{side}_gripper"].joint_names[0]
        )


def test_motion_demo_contracts_exist_on_all_production_profiles():
    """04/08/09/11 need Planner+Policy joint sources; 10 needs pose Teleop."""
    for profile_name in PRODUCTION_PROFILE_NAMES:
        config = EmbodimentConfig.from_yaml(PROFILES / profile_name)
        arm = next(
            name for name, part in config.parts.items() if part.part_type == "arm"
        )
        providers = config.execution["providers"]
        sources = config.execution["sources"]
        assert "Planner" in providers
        assert "Policy" in providers
        assert any(
            source["provider"] == "Planner"
            and source["part"] == arm
            and source["command"] == "joint_trajectory"
            for source in sources
        )
        assert any(
            source["provider"] == "Policy"
            and source["part"] == arm
            and source["command"] == "joint_reference"
            for source in sources
        )
        pose_teleops = [
            source["provider"]
            for source in sources
            if source["command"] == "pose_reference"
            and source["part"] in config.parts
            and config.parts[source["part"]].part_type == "arm"
        ]
        assert pose_teleops, f"{profile_name} missing Cartesian teleop pose_reference"
        assert all(
            name == "TeleopCartesian" or name.startswith("TeleopCartesian_")
            for name in pose_teleops
        )


def test_production_profiles_declare_joint_cartesian_and_twist_teleop():
    for profile_name in PRODUCTION_PROFILE_NAMES:
        config = EmbodimentConfig.from_yaml(PROFILES / profile_name)
        providers = config.execution["providers"]
        for kind in ("TeleopJoint", "TeleopCartesian", "TeleopTwist"):
            matches = [
                name
                for name in providers
                if name == kind or name.startswith(f"{kind}_")
            ]
            assert matches, f"{profile_name} missing {kind}*"


def test_template_profile_is_valid():
    template_path = TEMPLATES / "embodiment_profile.template.yaml"
    assert template_path.is_file(), "Template YAML file missing"
    config = EmbodimentConfig.from_yaml(template_path)
    assert config.name == "template_embodiment"
    assert "arm" in config.parts
    assert "end_effector" in config.parts
    assert "manipulator" in config.groups
    assert set(config.execution["providers"]) == {
        "Policy",
        "Planner",
        "TeleopJoint",
        "TeleopCartesian",
        "TeleopTwist",
    }
