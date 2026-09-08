from pathlib import Path

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / "config"


def _load_config(*parts: str) -> dict:
    return yaml.safe_load((CONFIG_DIR.joinpath(*parts)).read_text(encoding="utf-8"))


def test_server_configures_three_arm_routes_and_pika_gripper() -> None:
    config = _load_config("controller", "controllers.yaml")
    manager = config["controller_manager"]["ros__parameters"]

    assert manager["franka_arm_jspc"]["type"] == (
        "manipulation_position_controllers/JointSpaceImpedanceController"
    )
    assert manager["franka_arm_tskpc"]["type"] == (
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
    assert manager["pika_gripper_action"]["type"] == (
        "parallel_gripper_action_controller/GripperActionController"
    )
    assert config["pika_gripper_fwd"]["ros__parameters"] == {
        "joints": ["gripper_left_joint"],
        "interface_name": "position",
    }


def test_fake_hardware_uses_position_command_controllers() -> None:
    config = _load_config("controller", "controllers_fake.yaml")
    manager = config["controller_manager"]["ros__parameters"]

    assert manager["franka_arm_jspc"]["type"] == (
        "manipulation_position_controllers/JointSpacePositionController"
    )
    assert manager["franka_arm_tskpc"]["type"] == (
        "manipulation_position_controllers/TaskSpaceKinematicPositionController"
    )
    assert manager["franka_arm_jtc"]["type"] == (
        "joint_trajectory_controller/JointTrajectoryController"
    )
    assert config["franka_arm_jtc"]["ros__parameters"]["command_interfaces"] == [
        "position"
    ]


def test_mujoco_streams_only_profile_observation_cameras() -> None:
    config = _load_config("mujoco_plugins.yaml")
    camera = config["/**"]["ros__parameters"]["mujoco_plugins"][
        "mujoco_camera_plugin"
    ]

    assert camera["camera_publish_rate"] == 30.0
    policies = {
        name: values["policy"]
        for name, values in camera.items()
        if isinstance(values, dict)
    }
    assert {name for name, policy in policies.items() if policy == "streaming"} == {
        "agentview",
        "pika_d405",
    }
    assert {name for name, policy in policies.items() if policy == "disabled"} == {
        "frontview",
        "birdview",
        "sideview",
        "head_camera",
        "wrist_camera",
        "robot0_eye_in_hand",
    }


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
    real = set(
        _load_config("controller", "controllers.yaml")["controller_manager"][
            "ros__parameters"
        ]
    )
    fake = set(
        _load_config("controller", "controllers_fake.yaml")["controller_manager"][
            "ros__parameters"
        ]
    )
    assert real - skip == fake - skip


def test_real_and_fake_share_execution_command_topics() -> None:
    real = _load_config("controller", "controllers.yaml")
    fake = _load_config("controller", "controllers_fake.yaml")
    assert _collect_topic_params(real) == _collect_topic_params(fake)
    topics = _collect_topic_params(real)
    assert topics == {
        "franka_arm_jspc.ros__parameters.input_topic": "/execution/arm/joint_reference",
        "franka_arm_tskpc.ros__parameters.input_topic": "/execution/arm/pose_reference",
        "franka_arm_tskpc.ros__parameters.twist_topic": "/execution/arm/twist_reference",
    }
    for topic in topics.values():
        assert topic.startswith("/execution/")
        assert not topic.startswith("/execution_manager/")
    for config in (real, fake):
        jspc = config["franka_arm_jspc"]["ros__parameters"]
        tskpc = config["franka_arm_tskpc"]["ros__parameters"]
        assert "pose_topic" not in tskpc
        assert "robot_time_interface" not in jspc
        assert "robot_time_interface" not in tskpc


def test_all_reference_routes_reject_zero_stamps() -> None:
    for config_name in ("controllers.yaml", "controllers_fake.yaml"):
        config = _load_config("controller", config_name)
        for controller in ("franka_arm_jspc", "franka_arm_tskpc"):
            assert (
                config[controller]["ros__parameters"]["reject_zero_stamped_references"]
                is True
            )


def test_manipulation_routes_do_not_switch_to_jtc_on_error() -> None:
    for config_name in ("controllers.yaml", "controllers_fake.yaml"):
        manager = _load_config("controller", config_name)["controller_manager"][
            "ros__parameters"
        ]
        for controller in ("franka_arm_jspc", "franka_arm_tskpc"):
            assert "fallback_controllers" not in manager[controller]


def test_jtc_cancel_uses_profile_specific_deceleration() -> None:
    expected = [3.75, 1.875, 2.5, 3.125, 3.75, 5.0, 5.0]
    for config_name in ("controllers.yaml", "controllers_fake.yaml"):
        params = _load_config("controller", config_name)["franka_arm_jtc"][
            "ros__parameters"
        ]
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

    assert 'FindPackageShare("franka_manipulation_rt_launch")' in source
    assert '"urdf"' in source
    assert '"fr3_manipulation.urdf.xacro"' in source
    assert (PACKAGE_ROOT / "urdf" / "fr3_manipulation.urdf.xacro").is_file()
    assert 'LaunchConfiguration("controllers_yaml")' in source
    assert "controllers_fake.yaml" in source
    for controller in (
        "franka_arm_tskpc",
        "franka_arm_jspc",
        "franka_arm_jtc",
        "pika_gripper_fwd",
        "pika_gripper_action",
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
    assert (
        "--remap pika_gripper_action/gripper_cmd:=/execution/end_effector/gripper_command"
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
        "libero_tasks",
        "manipulation_position_controllers",
        "mujoco_ros2_control",
        "pika_gripper_hardware_interface",
        "parallel_gripper_action_controller",
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
    assert "mujoco_bringup.launch.py" in source
    assert "execution_manager.launch.py" not in source
    assert 'get_package_share_directory("rmi")' not in source
    assert '"load_pika_hardware"' in source
    assert "jtc_guard_cancel_response_timeout_s" in source


def test_mujoco_task_resolves_absolute_path(tmp_path) -> None:
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "mujoco_bringup",
        PACKAGE_ROOT / "launch" / "mujoco_bringup.launch.py",
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)

    model = tmp_path / "scene.xml"
    model.write_text("<mujoco/>")
    assert module._resolve_mujoco_model(str(model)) == str(model)


def test_controller_bringup_has_no_mujoco_path() -> None:
    source = (PACKAGE_ROOT / "launch" / "controller_bringup.launch.py").read_text(
        encoding="utf-8"
    )
    assert "mujoco_model" not in source
    assert "libero_tasks" not in source
    assert "mujoco_image_bridge" not in source
    assert "use_sim_mujoco" in source  # xacro mapping forced false
    assert "mujoco_bringup.launch.py" in source


def test_mujoco_bringup_owns_task_and_bridge() -> None:
    source = (PACKAGE_ROOT / "launch" / "mujoco_bringup.launch.py").read_text(
        encoding="utf-8"
    )
    assert "_resolve_mujoco_model" in source
    assert 'package="mujoco_ros2_control"' in source
    assert 'executable="mujoco_image_bridge"' in source
    assert "_camera_bridge_parameters" in source
    assert "mujoco_task" not in source


def test_fisheye_launch_uses_mjpeg_cam_original_jpeg() -> None:
    source = (PACKAGE_ROOT / "launch" / "camera_bringup.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'package="mjpeg_cam"' in source
    assert 'executable="mjpeg_cam_node"' in source
    assert "usb_cam" not in source

    cameras = _load_config("camera", "pika_cameras.yaml")
    fisheye = cameras["pika_fisheye"]["camera"]["ros__parameters"]
    assert fisheye["video_device"] == "/dev/pika_left_fisheye"
    assert fisheye["image_width"] == 1280
    assert fisheye["image_height"] == 720
    assert fisheye["framerate"] == 30.0
    assert fisheye["compressed_topic"] == "image/compressed"
    assert fisheye["format"] == "jpeg"
    assert "pixel_format" not in fisheye

    d405 = cameras["pika_d405"]["camera"]["ros__parameters"]
    assert d405["serial_no"] == "_323622270897"
    assert d405["enable_sync"] is True
    assert d405["align_depth"]["enable"] is True
    assert d405["rgb_camera"]["color_profile"] == "848x480x30"
    assert "color_profile" not in d405["depth_module"]


def test_readme_matches_launch_files_and_udev_names() -> None:
    readme = (PACKAGE_ROOT / "README.md").read_text(encoding="utf-8")
    bringup = (PACKAGE_ROOT / "docs" / "BRINGUP.md").read_text(encoding="utf-8")
    stack = (PACKAGE_ROOT / "launch" / "rt_stack.launch.py").read_text(encoding="utf-8")
    launch_names = {path.name for path in (PACKAGE_ROOT / "launch").glob("*.py")}
    assert "/dev/pika_left_gripper" in bringup
    assert "/dev/pika_left_fisheye" in bringup
    assert ":=/dev/ttyUSB" not in bringup
    for text in (readme, bringup):
        assert "pixi run" not in text.lower()
    assert "rt_stack.launch.py" in readme
    assert "controller_bringup.launch.py" in readme
    assert "mujoco_bringup.launch.py" in readme
    assert "camera_bringup.launch.py" in readme
    assert "controller_bringup.launch.py" in launch_names
    assert "mujoco_bringup.launch.py" in launch_names
    assert "camera_bringup.launch.py" in launch_names
    assert "mujoco_task.py" not in launch_names
    assert "mujoco_camera_bridge.py" not in launch_names
    assert "_CONTROL_MODULES" in stack
    assert "use_fake_hardware:=false" in readme
    assert '"use_fake_hardware"' in stack
    assert 'DeclareLaunchArgument("use_fake_hardware"' in stack


def test_camera_bridge_uses_plugin_configuration(tmp_path) -> None:
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "mujoco_bringup",
        PACKAGE_ROOT / "launch" / "mujoco_bringup.launch.py",
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    parameters = module._camera_bridge_parameters

    config = _load_config("mujoco_plugins.yaml")
    camera = config["/**"]["ros__parameters"]["mujoco_plugins"]["mujoco_camera_plugin"]
    assert camera["output"] == "shm"
    assert camera["default_policy"] == "disabled"
    result = parameters(str(CONFIG_DIR / "mujoco_plugins.yaml"))
    assert set(result["camera_names"]) == {"agentview", "pika_d405"}
    assert result["use_sim_time"] is False
    assert result["shm_prefix"] == "/pai_mj_cam_"
    camera["new_camera"] = {"policy": "streaming"}
    camera["pika_d405"]["policy"] = "disabled"
    camera["shm_prefix"] = "/custom_"
    path = tmp_path / "plugins.yaml"
    path.write_text(yaml.safe_dump(config))
    result = parameters(str(path))
    assert set(result["camera_names"]) == {"agentview", "new_camera"}
    assert result["shm_prefix"] == "/custom_"
    camera["output"] = "ros"
    path.write_text(yaml.safe_dump(config))
    assert parameters(str(path)) is None

    bringup = (PACKAGE_ROOT / "launch" / "mujoco_bringup.launch.py").read_text(
        encoding="utf-8"
    )
    assert "mujoco_image_relay" not in bringup
    assert "/mujoco/raw" not in bringup
    assert 'executable="mujoco_image_bridge"' in bringup
    assert "if bridge_parameters is not None:" in bringup


def test_bridge_dds_override_preserves_parent_configuration(monkeypatch) -> None:
    import importlib.util
    import os

    spec = importlib.util.spec_from_file_location(
        "mujoco_bringup",
        PACKAGE_ROOT / "launch" / "mujoco_bringup.launch.py",
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)

    monkeypatch.setenv("CYCLONEDDS_URI", "file:///custom/nic-and-peers.xml")
    monkeypatch.delenv("MUJOCO_IMAGE_BRIDGE_CYCLONEDDS_URI", raising=False)
    result = module._camera_bridge_environment()["CYCLONEDDS_URI"]
    assert result.startswith("file:///custom/nic-and-peers.xml,")
    assert "<AllowMulticast>spdp</AllowMulticast>" in result
    assert os.environ["CYCLONEDDS_URI"] == "file:///custom/nic-and-peers.xml"
    monkeypatch.setenv("MUJOCO_IMAGE_BRIDGE_CYCLONEDDS_URI", "file:///explicit.xml")
    assert module._camera_bridge_environment() == {
        "CYCLONEDDS_URI": "file:///explicit.xml"
    }
