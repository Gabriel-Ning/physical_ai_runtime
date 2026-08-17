from pathlib import Path

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / "config"


def _load_config(*parts: str) -> dict:
    return yaml.safe_load((CONFIG_DIR.joinpath(*parts)).read_text(encoding="utf-8"))


def test_three_position_routes_are_configured_per_arm() -> None:
    params = _load_config("controller", "controllers.yaml")["controller_manager"][
        "ros__parameters"
    ]

    for side in ("left", "right"):
        assert params[f"{side}_arm_jspc"]["type"] == (
            "manipulation_position_controllers/JointSpacePositionController"
        )
        assert params[f"{side}_arm_tskpc"]["type"] == (
            "manipulation_position_controllers/TaskSpaceKinematicPositionController"
        )
        assert params[f"{side}_arm_jtc"]["type"] == (
            "joint_trajectory_controller/JointTrajectoryController"
        )
        assert params[f"{side}_pika_gripper_fwd"]["type"] == (
            "forward_command_controller/ForwardCommandController"
        )


def test_execution_command_topics_use_group_namespace() -> None:
    controllers = _load_config("controller", "controllers.yaml")
    for side in ("left", "right"):
        group = f"/execution/{side}_arm"
        jspc = controllers[f"{side}_arm_jspc"]["ros__parameters"]
        tskpc = controllers[f"{side}_arm_tskpc"]["ros__parameters"]
        assert jspc["input_topic"] == f"{group}/joint_reference"
        assert tskpc["input_topic"] == f"{group}/pose_reference"
        assert tskpc["twist_topic"] == f"{group}/twist_reference"
        assert "pose_topic" not in tskpc
        assert "robot_time_interface" not in jspc
        assert "robot_time_interface" not in tskpc
        for topic in (jspc["input_topic"], tskpc["input_topic"], tskpc["twist_topic"]):
            assert not topic.startswith("/execution_manager/")


def test_all_reference_routes_reject_zero_stamps() -> None:
    controllers = _load_config("controller", "controllers.yaml")
    for side in ("left", "right"):
        for route in ("jspc", "tskpc"):
            assert (
                controllers[f"{side}_arm_{route}"]["ros__parameters"][
                    "reject_zero_stamped_references"
                ]
                is True
            )


def test_manipulation_routes_do_not_switch_to_jtc_on_error() -> None:
    manager = _load_config("controller", "controllers.yaml")["controller_manager"][
        "ros__parameters"
    ]
    for side in ("left", "right"):
        for route in ("jspc", "tskpc"):
            assert "fallback_controllers" not in manager[f"{side}_arm_{route}"]


def test_pika_gripper_forward_controller_contract() -> None:
    controllers = _load_config("controller", "controllers.yaml")

    for side in ("left", "right"):
        params = controllers[f"{side}_pika_gripper_fwd"]["ros__parameters"]
        assert params["joints"] == [f"{side}_gripper_left_joint"]
        assert params["interface_name"] == "position"


def test_jtc_uses_validated_marvin_goal_constraints() -> None:
    controllers = _load_config("controller", "controllers.yaml")

    for side in ("left", "right"):
        params = controllers[f"{side}_arm_jtc"]["ros__parameters"]
        constraints = params["constraints"]
        assert constraints["stopped_velocity_tolerance"] == 0.05
        assert constraints["goal_time"] == 0.5
        assert constraints["decelerate_on_cancel"] is True
        for index in range(1, 8):
            assert constraints[f"Joint{index}_{side[0].upper()}"] == {
                "max_deceleration_on_cancel": 8.0
            }
        assert (
            params["set_last_command_interface_value_as_state_on_activation"] is False
        )


def test_server_defaults_safe_and_leaves_execution_to_rmi_deployment() -> None:
    launch_source = (
        PACKAGE_ROOT / "launch" / "controller_bringup.launch.py"
    ).read_text(encoding="utf-8")

    assert '"--inactive"' in launch_source
    assert "target_action=route_controller_spawner" in launch_source
    assert "manipulation_execution_manager" not in launch_source
    assert "Shutdown(reason=reason)" in launch_source
    assert '"cpu_affinity"' in launch_source
    assert '"RT_CM_CPU_AFFINITY"' in launch_source
    assert (
        'prefix=f"taskset -c {cpu_affinity}" if cpu_affinity else None' in launch_source
    )
    assert '"marvin_manipulation.urdf.xacro"' in launch_source
    assert "bringup_share" in launch_source
    assert launch_source.count('package="joint_trajectory_controller_guard"') == 1
    assert 'name=f"{side}_arm_jtc_guard"' in launch_source
    assert 'f"/execution/{side}_arm/follow_joint_trajectory"' in launch_source
    assert (
        "--remap left_arm_jtc/follow_joint_trajectory:=/execution/left_arm/follow_joint_trajectory"
        in launch_source
    )
    assert (
        "--remap right_arm_jtc/follow_joint_trajectory:=/execution/right_arm/follow_joint_trajectory"
        in launch_source
    )
    assert "/execution/left_gripper/joint_reference" in launch_source
    assert "/execution/right_gripper/joint_reference" in launch_source
    for argument in (
        "connected_to",
        "xyz",
        "rpy",
        "mounts_file",
        "robot_ip",
        "stale_warn_ms",
        "stale_error_ms",
        "max_joint_velocity",
    ):
        assert f'LaunchConfiguration("{argument}")' in launch_source
        assert f'"{argument}"' in launch_source


def test_manipulation_xacro_is_owned_by_bringup() -> None:
    xacro = PACKAGE_ROOT / "urdf" / "marvin_manipulation.urdf.xacro"
    assert xacro.is_file()
    text = xacro.read_text(encoding="utf-8")
    assert "marvin_manipulation_controller_bringup" in text
    assert "pika_gripper_description" in text
    assert "marvin_description" in text


def test_bimanual_manipulation_dual_pika_control_contract() -> None:
    import subprocess
    import xml.etree.ElementTree as ET

    xacro_path = PACKAGE_ROOT / "urdf" / "marvin_manipulation.urdf.xacro"
    result = subprocess.run(
        [
            "xacro",
            str(xacro_path),
            "ros2_control:=true",
            "use_fake_hardware:=true",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    root = ET.fromstring(result.stdout)
    links = {link.get("name") for link in root.findall("link")}
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    revolute = [
        joint for joint in root.findall("joint") if joint.get("type") == "revolute"
    ]
    controls = root.findall("ros2_control")
    arm_control = controls[0]

    assert len(revolute) == 14
    assert {"flange_L", "flange_R"} <= links
    assert {"left_pika_adaptor_link", "right_pika_adaptor_link"} <= links
    assert {"left_pika_gripper_tcp", "right_pika_gripper_tcp"} <= links
    assert joints["left_pika_adaptor_joint"].find("parent").get("link") == "flange_L"
    assert joints["right_pika_adaptor_joint"].find("parent").get("link") == "flange_R"
    assert len(controls) == 3
    assert len(arm_control.findall("joint")) == 14
    assert len(root.findall(".//command_interface[@name='position']")) == 16
    assert {control.get("name") for control in controls[1:]} == {
        "LeftPikaGripperHardware",
        "RightPikaGripperHardware",
    }


def test_controller_manager_profile_targets_rt_host() -> None:
    manager = _load_config("controller", "controllers.yaml")["controller_manager"][
        "ros__parameters"
    ]

    assert manager["update_rate"] == 500
    assert 1 <= manager["thread_priority"] <= 98
    assert manager["overruns"]["manage"] is False
    assert (
        manager["joint_state_broadcaster"]["type"]
        == "joint_state_broadcaster/JointStateBroadcaster"
    )


def test_package_xml_declares_runtime_plugins() -> None:
    text = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    for dep in (
        "forward_command_controller",
        "marvin_hardware_interface",
        "manipulation_position_controllers",
        "pika_gripper_hardware_interface",
        "joint_trajectory_controller_guard",
    ):
        assert f"<exec_depend>{dep}</exec_depend>" in text


def test_docs_do_not_point_at_removed_new_apps() -> None:
    bringup = (PACKAGE_ROOT / "docs" / "BRINGUP.md").read_text(encoding="utf-8")
    assert "src/new_apps" not in bringup


def test_rt_stack_contains_only_rt_runtime_components() -> None:
    source = (PACKAGE_ROOT / "launch" / "rt_stack.launch.py").read_text(
        encoding="utf-8"
    )
    assert "controller_bringup.launch.py" in source
    assert "execution_manager.launch.py" not in source
    assert 'get_package_share_directory("rmi")' not in source
