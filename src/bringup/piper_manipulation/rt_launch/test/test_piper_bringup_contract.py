import re
from pathlib import Path

import yaml

ROOT = Path(__file__).parents[1]


def test_controller_config_is_present():
    assert (ROOT / "config" / "controller" / "controllers.yaml").is_file()


def test_controller_manager_uses_fifo_priority_98():
    manager = yaml.safe_load((ROOT / "config" / "controller" / "controllers.yaml").read_text())[
        "controller_manager"
    ]["ros__parameters"]
    assert manager["update_rate"] == 500
    assert manager["thread_priority"] == 98


def test_rt_profile_is_scoped_to_native_piper_grippers():
    controllers = yaml.safe_load((ROOT / "config" / "controller" / "controllers.yaml").read_text())
    launch = (ROOT / "launch" / "controller_bringup.launch.py").read_text()

    assert "pika_gripper_fwd" not in controllers
    assert 'VALID_END_EFFECTORS = {"none", "piper_gripper"}' in launch
    assert "pika_gripper_joint" not in launch


def test_controller_routes_have_six_joints_per_arm():
    controllers = yaml.safe_load((ROOT / "config" / "controller" / "controllers.yaml").read_text())
    for side in ("left", "right"):
        assert len(controllers[f"{side}_arm_jtc"]["ros__parameters"]["joints"]) == 6
        tskpc = controllers[f"{side}_arm_tskpc"]["ros__parameters"]
        assert len(tskpc["joints"]) == 6
        assert tskpc["base_frame"] == f"{side}_base_link"
        assert tskpc["tip_frame"] == f"{side}_gripper_tcp"
        assert tskpc["reject_zero_stamped_references"] is True


def test_execution_command_topics_use_group_namespace():
    controllers = yaml.safe_load((ROOT / "config" / "controller" / "controllers.yaml").read_text())
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


def test_joint_reference_buffering_remains_controller_owned():
    controllers = yaml.safe_load((ROOT / "config" / "controller" / "controllers.yaml").read_text())
    for side in ("left", "right"):
        behavior = controllers[f"{side}_arm_jspc"]["ros__parameters"][
            "trajectory_behavior"
        ]
        assert behavior["max_points"] == 1
        assert (
            controllers[f"{side}_arm_jspc"]["ros__parameters"][
                "reject_zero_stamped_references"
            ]
            is True
        )


def test_manipulation_routes_do_not_switch_to_jtc_on_error():
    manager = yaml.safe_load((ROOT / "config" / "controller" / "controllers.yaml").read_text())[
        "controller_manager"
    ]["ros__parameters"]
    for side in ("left", "right"):
        for route in ("jspc", "tskpc"):
            assert "fallback_controllers" not in manager[f"{side}_arm_{route}"]


def test_jtc_cancel_deceleration_is_configured_per_joint():
    controllers = yaml.safe_load((ROOT / "config" / "controller" / "controllers.yaml").read_text())
    for side in ("left", "right"):
        params = controllers[f"{side}_arm_jtc"]["ros__parameters"]
        assert params["state_interfaces"] == ["position", "velocity"]
        constraints = params["constraints"]
        assert constraints["decelerate_on_cancel"] is True
        for index in range(1, 7):
            assert constraints[f"{side}_joint{index}"] == {
                "max_deceleration_on_cancel": 40.0
            }


def test_launch_owns_no_robot_model():
    assert not (ROOT / "urdf").exists()
    launch = (ROOT / "launch" / "controller_bringup.launch.py").read_text()
    assert 'get_package_share_directory("piper_description")' in launch
    assert '"urdf", "piper_bimanual_manipulation.urdf.xacro"' in launch
    assert "piper_with_teach.urdf.xacro" not in launch
    assert '"enable_left": str("left" in active).lower()' in launch
    assert '"enable_right": str("right" in active).lower()' in launch
    assert "manipulation_execution_manager" not in launch
    assert 'package="joint_trajectory_controller_guard"' in launch
    assert 'name=f"{side}_arm_jtc_guard"' in launch
    assert 'f"/execution/{side}_arm/follow_joint_trajectory"' in launch
    assert "{side}_arm_jtc/follow_joint_trajectory:=" in launch


def test_deployment_choices_are_launch_arguments():
    launch = (ROOT / "launch" / "controller_bringup.launch.py").read_text()
    for argument in (
        "arms",
        "use_fake_hardware",
        "load_gripper_hardware",
        "connected_to",
        "enable_table",
        "table_xyz",
        "table_rpy",
        "left_can_interface",
        "right_can_interface",
        "left_mit_kd_effort_damping",
        "right_mit_kd_effort_damping",
        "left_xyz",
        "right_xyz",
        "left_rpy",
        "right_rpy",
        "left_end_effector",
        "right_end_effector",
        "left_gripper_home_on_activate",
        "right_gripper_home_on_activate",
        "cpu_affinity",
    ):
        assert re.search(rf'DeclareLaunchArgument\(\s*"{argument}"', launch)
    assert 'default_value="piper0"' in launch
    assert 'default_value="piper1"' in launch
    assert "init_can" not in launch
    assert 'prefix=f"taskset -c {cpu_affinity}" if cpu_affinity else None' in launch
    assert "RT_CM_CPU_AFFINITY" in launch


def test_workcell_poses_defer_to_description_xacro():
    launch = (ROOT / "launch" / "controller_bringup.launch.py").read_text()
    docs = (ROOT / "docs" / "BRINGUP.md").read_text()
    for argument in (
        "table_xyz",
        "table_rpy",
        "left_xyz",
        "right_xyz",
        "left_rpy",
        "right_rpy",
    ):
        assert f'DeclareLaunchArgument("{argument}", default_value="")' in launch
    assert "_optional_xacro_args" in launch
    assert "-0.38 0.32 0.71" not in launch
    assert "-0.38 0.32 0.71" not in docs


def test_rviz_is_opt_in_and_reuses_description_package_config():
    launch = (ROOT / "launch" / "controller_bringup.launch.py").read_text()
    assert re.search(
        r'DeclareLaunchArgument\(\s*"use_rviz",\s*default_value="false"', launch
    )
    assert 'package="rviz2"' in launch
    assert 'os.path.join(description_share, "rviz", "visualize_piper.rviz")' in launch


def test_route_spawner_keeps_arm_and_gripper_routes_inactive_at_bringup():
    launch = (ROOT / "launch" / "controller_bringup.launch.py").read_text()
    assert 'for route in ("jspc", "tskpc", "jtc")' in launch
    assert "*gripper_controllers" in launch
    assert "/execution/{side}_gripper/joint_reference" in launch
    assert "/execution/{side}_gripper/gripper_command" in launch
    assert 'f"{side}_gripper_action"' in launch
    for argument in ("--inactive", "--controller-manager", "/controller_manager"):
        assert f'"{argument}"' in launch


def test_rt_stack_contains_only_rt_runtime_components():
    combo = (ROOT / "launch" / "rt_stack.launch.py").read_text()
    assert "controller_bringup.launch.py" in combo
    assert "execution_manager.launch.py" not in combo
    assert 'get_package_share_directory("rmi")' not in combo
    assert 'DeclareLaunchArgument(\n        "left_can_interface",\n        default_value="piper0"' in combo or 'DeclareLaunchArgument(\n                "left_can_interface"' in combo
    assert 'DeclareLaunchArgument(\n        "right_can_interface",\n        default_value="piper1"' in combo or 'DeclareLaunchArgument(\n                "right_can_interface"' in combo
    assert re.search(
        r'DeclareLaunchArgument\(\s*"cpu_affinity",\s*default_value=""', combo
    )
    assert "cpu_affinity:=none" not in combo


def test_controller_bringup_still_leaves_em_to_rmi_deployment():
    launch = (ROOT / "launch" / "controller_bringup.launch.py").read_text()
    assert "execution_manager.launch.py" not in launch
    assert "manipulation_execution_manager" not in launch


def test_package_xml_declares_runtime_plugins():
    text = (ROOT / "package.xml").read_text(encoding="utf-8")
    for dep in (
        "forward_command_controller",
        "piper_hardware_interface",
        "manipulation_position_controllers",
        "joint_trajectory_controller_guard",
        "parallel_gripper_action_controller",
        "launch",
        "launch_ros",
    ):
        assert f"<exec_depend>{dep}</exec_depend>" in text


def test_docs_do_not_point_at_removed_new_apps():
    assert "src/new_apps" not in (ROOT / "docs" / "BRINGUP.md").read_text(
        encoding="utf-8"
    )
    assert "src/new_apps" not in (ROOT / "README.md").read_text(encoding="utf-8")
