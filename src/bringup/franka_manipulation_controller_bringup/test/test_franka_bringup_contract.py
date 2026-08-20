from pathlib import Path

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / "config"


def _load_config(*parts: str) -> dict:
    return yaml.safe_load((CONFIG_DIR.joinpath(*parts)).read_text(encoding="utf-8"))


def test_server_configures_three_arm_routes_and_pika_gripper() -> None:
    config = _load_config("controller", "controllers.yaml")
    manager = config["controller_manager"]["ros__parameters"]

    assert manager["franka_arm_jsic"]["type"] == (
        "manipulation_position_controllers/JointSpaceImpedanceController"
    )
    assert manager["franka_arm_tsjic"]["type"] == (
        "manipulation_position_controllers/TaskSpaceJointImpedanceController"
    )
    assert manager["franka_arm_jtc"]["type"] == (
        "joint_trajectory_controller/JointTrajectoryController"
    )
    assert config["franka_arm_jtc"]["ros__parameters"]["command_interfaces"] == [
        "effort"
    ]
    assert manager["pika_gripper_fwd"]["type"] == (
        "forward_command_controller/ForwardCommandController"
    )
    assert config["pika_gripper_fwd"]["ros__parameters"] == {
        "joints": ["gripper_left_joint"],
        "interface_name": "position",
    }


def test_fake_hardware_uses_position_command_controllers() -> None:
    config = _load_config("controller", "controllers_fake.yaml")
    manager = config["controller_manager"]["ros__parameters"]

    assert manager["franka_arm_jsic"]["type"] == (
        "manipulation_position_controllers/JointSpacePositionController"
    )
    assert manager["franka_arm_tsjic"]["type"] == (
        "manipulation_position_controllers/TaskSpaceKinematicPositionController"
    )
    assert manager["franka_arm_jtc"]["type"] == (
        "joint_trajectory_controller/JointTrajectoryController"
    )
    assert config["franka_arm_jtc"]["ros__parameters"]["command_interfaces"] == [
        "position"
    ]


def _collect_topic_params(config: dict) -> dict[str, str]:
    found: dict[str, str] = {}

    def walk(obj: object, prefix: str) -> None:
        if not isinstance(obj, dict):
            return
        for key, value in obj.items():
            path = f"{prefix}.{key}" if prefix else key
            if key.endswith("_topic") or key == "pose_topic":
                found[path] = value
            else:
                walk(value, path)

    walk(config, "")
    return found


def test_real_and_fake_share_controller_names() -> None:
    skip = {"update_rate", "thread_priority", "overruns"}
    real = set(_load_config("controller", "controllers.yaml")["controller_manager"]["ros__parameters"])
    fake = set(
        _load_config("controller", "controllers_fake.yaml")["controller_manager"]["ros__parameters"]
    )
    assert real - skip == fake - skip


def test_real_and_fake_share_execution_command_topics() -> None:
    real = _load_config("controller", "controllers.yaml")
    fake = _load_config("controller", "controllers_fake.yaml")
    assert _collect_topic_params(real) == _collect_topic_params(fake)
    topics = _collect_topic_params(real)
    assert topics == {
        "franka_arm_jsic.ros__parameters.input_topic": "/execution/arm/joint_reference",
        "franka_arm_tsjic.ros__parameters.input_topic": "/execution/arm/pose_reference",
        "franka_arm_tsjic.ros__parameters.twist_topic": "/execution/arm/twist_reference",
    }
    for topic in topics.values():
        assert topic.startswith("/execution/")
        assert not topic.startswith("/execution_manager/")
    for config in (real, fake):
        jsic = config["franka_arm_jsic"]["ros__parameters"]
        tsjic = config["franka_arm_tsjic"]["ros__parameters"]
        assert "pose_topic" not in tsjic
        assert "robot_time_interface" not in jsic
        assert "robot_time_interface" not in tsjic


def test_all_reference_routes_reject_zero_stamps() -> None:
    for config_name in ("controllers.yaml", "controllers_fake.yaml"):
        config = _load_config("controller", config_name)
        for controller in ("franka_arm_jsic", "franka_arm_tsjic"):
            assert (
                config[controller]["ros__parameters"]["reject_zero_stamped_references"]
                is True
            )


def test_manipulation_routes_do_not_switch_to_jtc_on_error() -> None:
    for config_name in ("controllers.yaml", "controllers_fake.yaml"):
        manager = _load_config("controller", config_name)["controller_manager"]["ros__parameters"]
        for controller in ("franka_arm_jsic", "franka_arm_tsjic"):
            assert "fallback_controllers" not in manager[controller]


def test_jtc_cancel_uses_profile_specific_deceleration() -> None:
    expected = [3.75, 1.875, 2.5, 3.125, 3.75, 5.0, 5.0]
    for config_name in ("controllers.yaml", "controllers_fake.yaml"):
        params = _load_config("controller", config_name)["franka_arm_jtc"]["ros__parameters"]
        assert params["state_interfaces"] == ["position", "velocity"]
        constraints = params["constraints"]
        assert constraints["decelerate_on_cancel"] is True
        assert [
            constraints[f"fr3_joint{index}"]["max_deceleration_on_cancel"]
            for index in range(1, 8)
        ] == expected


def test_bringup_leaves_execution_to_rmi_deployment() -> None:
    source = (PACKAGE_ROOT / "launch" / "controller_bringup.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'FindPackageShare("franka_manipulation_controller_bringup")' in source
    assert '"urdf"' in source
    assert '"fr3_manipulation.urdf.xacro"' in source
    assert (PACKAGE_ROOT / "urdf" / "fr3_manipulation.urdf.xacro").is_file()
    assert 'LaunchConfiguration("controllers_yaml")' in source
    assert "controllers_fake.yaml" in source
    for controller in (
        "franka_arm_tsjic",
        "franka_arm_jsic",
        "franka_arm_jtc",
        "pika_gripper_fwd",
    ):
        assert f'"{controller}"' in source
    assert '"--inactive"' in source
    assert '"visualize_franka.rviz"' in source
    assert "manipulation_execution_manager" not in source
    assert 'package="joint_trajectory_controller_guard"' in source
    assert 'name="franka_arm_jtc_guard"' in source
    assert '"action_name": "/execution/arm/follow_joint_trajectory"' in source
    assert (
        "--remap franka_arm_jtc/follow_joint_trajectory:=/execution/arm/follow_joint_trajectory"
        in source
    )
    assert (
        "--remap pika_gripper_fwd/commands:=/execution/end_effector/joint_reference"
        in source
    )


def test_visualize_launch_loads_manipulation_xacro() -> None:
    source = (
        PACKAGE_ROOT / "launch" / "visualize_fr3_manipulation.launch.py"
    ).read_text(encoding="utf-8")
    assert "fr3_manipulation.urdf.xacro" in source
    assert '"ros2_control": "false"' in source
    assert 'default_value=""' in source
    assert "joint_state_publisher_gui" in source


def test_package_xml_declares_runtime_plugins() -> None:
    text = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    for dep in (
        "forward_command_controller",
        "franka_hardware",
        "manipulation_position_controllers",
        "pika_gripper_hardware_interface",
        "joint_trajectory_controller_guard",
    ):
        assert f"<exec_depend>{dep}</exec_depend>" in text


def test_docs_do_not_point_at_removed_new_apps() -> None:
    for relative in ("docs/BRINGUP.md", "README.md"):
        text = (PACKAGE_ROOT / relative).read_text(encoding="utf-8")
        assert "src/new_apps" not in text


def test_rt_stack_contains_only_rt_runtime_components() -> None:
    source = (PACKAGE_ROOT / "launch" / "rt_stack.launch.py").read_text(
        encoding="utf-8"
    )
    assert "controller_bringup.launch.py" in source
    assert "execution_manager.launch.py" not in source
    assert 'get_package_share_directory("rmi")' not in source
    assert '"load_pika_hardware"' in source
