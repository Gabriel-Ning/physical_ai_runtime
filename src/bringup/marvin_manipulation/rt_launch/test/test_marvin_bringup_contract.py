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
        assert params[f"{side}_pika_gripper_action"]["type"] == (
            "parallel_gripper_action_controller/GripperActionController"
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


def test_pika_gripper_action_controller_contract() -> None:
    controllers = _load_config("controller", "controllers.yaml")

    for side in ("left", "right"):
        params = controllers[f"{side}_pika_gripper_action"]["ros__parameters"]
        assert params["joint"] == f"{side}_gripper_left_joint"
        assert params["state_interfaces"] == ["position", "velocity"]
        assert params["allow_stalling"] is True


def test_pika_finger_travel_matches_vendor_default() -> None:
    limits = _load_config("model", "joint_limits.yaml")["gripper_left_joint"]["limit"]
    assert limits["lower"] == 0.0
    assert limits["upper"] == 0.045


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
    assert "/execution/left_gripper/gripper_command" in launch_source
    assert "/execution/right_gripper/gripper_command" in launch_source
    assert '"load_pika_hardware"' in launch_source
    for argument in (
        "connected_to",
        "xyz",
        "rpy",
        "mounts_file",
        "robot_ip",
        "stale_warn_ms",
        "stale_error_ms",
        "max_joint_velocity",
        "load_pika_hardware",
        "use_rviz",
        "left_gripper_serial_port",
        "right_gripper_serial_port",
    ):
        assert f'"{argument}"' in launch_source
    assert 'default_value="/dev/pika_left_gripper"' in launch_source
    assert 'default_value="/dev/pika_right_gripper"' in launch_source


def test_manipulation_xacro_is_owned_by_bringup() -> None:
    xacro = PACKAGE_ROOT / "urdf" / "marvin_manipulation.urdf.xacro"
    assert xacro.is_file()
    text = xacro.read_text(encoding="utf-8")
    assert "marvin_manipulation_rt_launch" in text
    assert "pika_gripper_description" in text
    assert "marvin_description" in text
    assert 'name="left_gripper_serial_port"' in text
    assert 'name="right_gripper_serial_port"' in text
    assert 'default="/dev/pika_left_gripper"' in text
    assert 'default="/dev/pika_right_gripper"' in text


def _run_xacro(*args: str) -> str:
    import os
    import subprocess

    env = dict(os.environ)
    install_root = next(
        (
            parent / "install"
            for parent in PACKAGE_ROOT.parents
            if (parent / "apps" / "profiles").is_dir()
        ),
        PACKAGE_ROOT.parents[4] / "install",
    )
    installed_prefixes = (
        [str(p) for p in install_root.iterdir() if p.is_dir()]
        if install_root.is_dir()
        else []
    )
    existing_prefix = env.get("AMENT_PREFIX_PATH", "")
    prefix_list = [p for p in installed_prefixes if p not in existing_prefix]
    if existing_prefix:
        prefix_list.append(existing_prefix)
    env["AMENT_PREFIX_PATH"] = ":".join(prefix_list)

    xacro_path = PACKAGE_ROOT / "urdf" / "marvin_manipulation.urdf.xacro"
    result = subprocess.run(
        ["xacro", str(xacro_path), *args],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )
    return result.stdout


def test_bimanual_manipulation_dual_pika_control_contract() -> None:
    import xml.etree.ElementTree as ET

    stdout = _run_xacro("ros2_control:=true", "use_fake_hardware:=true")
    root = ET.fromstring(stdout)
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
    for name in ("left_gripper_left_joint", "right_gripper_left_joint"):
        limit = joints[name].find("limit")
        assert float(limit.get("lower")) == 0.0
        assert float(limit.get("upper")) == 0.045


def test_load_pika_hardware_false_omits_both_grippers() -> None:
    import xml.etree.ElementTree as ET

    stdout = _run_xacro(
        "ros2_control:=true",
        "use_fake_hardware:=true",
        "load_pika_hardware:=false",
    )
    root = ET.fromstring(stdout)
    links = {link.get("name") for link in root.findall("link")}
    controls = root.findall("ros2_control")
    revolute = [
        joint for joint in root.findall("joint") if joint.get("type") == "revolute"
    ]

    assert len(revolute) == 14
    assert "left_pika_gripper_tcp" not in links
    assert "right_pika_gripper_tcp" not in links
    assert "left_pika_adaptor_link" not in links
    assert "right_pika_adaptor_link" not in links
    assert len(controls) == 1
    names = {control.get("name") for control in controls}
    assert "LeftPikaGripperHardware" not in names
    assert "RightPikaGripperHardware" not in names


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
    jsb = _load_config("controller", "controllers.yaml")["joint_state_broadcaster"][
        "ros__parameters"
    ]
    assert jsb["publish_rate"] == 200.0


def test_package_xml_declares_runtime_plugins() -> None:
    text = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    for dep in (
        "forward_command_controller",
        "marvin_hardware_interface",
        "manipulation_position_controllers",
        "pika_gripper_hardware_interface",
        "joint_trajectory_controller_guard",
        "realsense2_camera",
        "mjpeg_cam",
    ):
        assert f"<exec_depend>{dep}</exec_depend>" in text


def test_docs_do_not_point_at_removed_new_apps() -> None:
    bringup = (PACKAGE_ROOT / "docs" / "BRINGUP.md").read_text(encoding="utf-8")
    assert "src/new_apps" not in bringup


def test_camera_yaml_is_unique_stream_source_for_app_profile() -> None:
    d405_file = _load_config("camera", "pika_d405.yaml")
    fisheye_file = _load_config("camera", "pika_fisheye.yaml")
    assert not (CONFIG_DIR / "camera" / "marvin_cameras.yaml").exists()
    assert all("fisheye" not in key for key in d405_file)
    assert all("d405" not in key for key in fisheye_file)
    repo_root = next(
        parent
        for parent in PACKAGE_ROOT.parents
        if (parent / "apps" / "profiles" / "marvin_bimanual.yaml").is_file()
    )
    profile = yaml.safe_load(
        (repo_root / "apps" / "profiles" / "marvin_bimanual.yaml").read_text(
            encoding="utf-8"
        )
    )
    features = profile["features"]["observation"]
    template = yaml.safe_load(
        (
            repo_root / "src" / "toolbox" / "mjpeg_cam" / "config" / "mjpeg_cam.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]

    def _wh_fps(profile_str: str) -> tuple[int, int, int]:
        width_s, height_s, fps_s = profile_str.split("x")
        return int(width_s), int(height_s), int(fps_s)

    for side in ("left", "right"):
        d405 = d405_file[f"{side}_pika_d405"]["camera"]["ros__parameters"]
        color_w, color_h, color_fps = _wh_fps(d405["rgb_camera"]["color_profile"])
        depth_w, depth_h, depth_fps = _wh_fps(d405["depth_module"]["depth_profile"])
        assert (color_w, color_h, color_fps) == (848, 480, 30)
        assert (depth_w, depth_h, depth_fps) == (848, 480, 30)
        assert d405["enable_depth"] is True
        assert d405["enable_sync"] is True
        assert d405["align_depth"]["enable"] is True

        fisheye = fisheye_file[f"{side}_pika_fisheye"]["camera"]["ros__parameters"]
        assert fisheye["image_width"] == 1280
        assert fisheye["image_height"] == 720
        assert fisheye["framerate"] == 30.0
        assert set(fisheye) <= set(template)

        color_shape = features[f"observation.images.{side}_pika_d405"]["shape"]
        fish_shape = features[f"observation.images.{side}_pika_fisheye"]["shape"]
        assert color_shape == [3, color_h, color_w]
        assert fish_shape == [3, fisheye["image_height"], fisheye["image_width"]]
        assert (
            profile["sensors"]["cameras"][f"{side}_pika_d405"]["ros_topic"]
            == f"/{side}_pika_d405/camera/color/image_raw"
        )
        assert (
            profile["sensors"]["cameras"][f"{side}_pika_fisheye"]["ros_topic"]
            == f"/{side}_pika_fisheye/image/compressed"
        )
        assert (
            profile["sensors"]["cameras"][f"{side}_pika_fisheye"]["encoding"] == "jpeg"
        )
        assert fisheye["video_device"] == f"/dev/pika_{side}_fisheye"
        assert fisheye["compressed_topic"] == "image/compressed"
        assert fisheye["format"] == "jpeg"
        assert "pixel_format" not in fisheye


def test_rt_stack_contains_only_rt_runtime_components() -> None:
    source = (PACKAGE_ROOT / "launch" / "rt_stack.launch.py").read_text(
        encoding="utf-8"
    )
    assert "controller_bringup.launch.py" in source
    assert "prime_arm_position.launch.py" in source
    assert '"prime_arm_position"' in source
    assert "execution_manager.launch.py" not in source
    assert 'get_package_share_directory("rmi")' not in source
    assert '"load_pika_hardware"' in source


def test_prime_arm_position_is_real_hardware_only() -> None:
    prime = (PACKAGE_ROOT / "launch" / "prime_arm_position.launch.py").read_text(
        encoding="utf-8"
    )
    stack = (PACKAGE_ROOT / "launch" / "rt_stack.launch.py").read_text(encoding="utf-8")
    controller = (PACKAGE_ROOT / "launch" / "controller_bringup.launch.py").read_text(
        encoding="utf-8"
    )
    assert "left_arm_jtc" in prime and "right_arm_jtc" in prime
    assert "switch_controllers" in prime
    assert "libmarvin" in prime or "CCS" in prime or "vendor" in prime
    assert "use_fake_hardware" in stack
    assert "_prime_actions" in stack
    assert "prime_arm_position" not in controller
    assert "switch_controllers" not in controller


def test_fisheye_launch_uses_mjpeg_cam_original_jpeg() -> None:
    source = (PACKAGE_ROOT / "launch" / "pika_camera_bringup.launch.py").read_text(
        encoding="utf-8"
    )
    stack = (PACKAGE_ROOT / "launch" / "rt_stack.launch.py").read_text(encoding="utf-8")
    assert 'package="mjpeg_cam"' in source
    assert 'executable="mjpeg_cam_node"' in source
    assert "yaml.safe_load" in source
    assert "_configured_cameras" in source
    assert "for namespace in _configured_cameras(fisheye_config)" in source
    assert "enumerate(_configured_cameras(d405_config))" in source
    assert "parameters=[fisheye_config]" in source
    assert "usb_cam" not in source
    assert "pika_d405.yaml" in source
    assert "pika_fisheye.yaml" in source
    assert '"d405_config"' in source
    assert '"fisheye_config"' in source
    assert "marvin_cameras.yaml" not in source
    assert "pika_d405.yaml" in stack
    assert "pika_fisheye.yaml" in stack
    assert '"camera_config"' not in stack
    assert "TimerAction" in source
    assert '"right_d405_delay"' in source
    assert '"right_d405_delay"' in stack


def test_rt_bringup_has_optional_rviz_surface() -> None:
    controller = (PACKAGE_ROOT / "launch" / "controller_bringup.launch.py").read_text(
        encoding="utf-8"
    )
    stack = (PACKAGE_ROOT / "launch" / "rt_stack.launch.py").read_text(encoding="utf-8")
    package_xml = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    assert '"use_rviz"' in controller
    assert '"use_rviz"' in stack
    assert 'package="rviz2"' in controller
    assert "<exec_depend>rviz2</exec_depend>" in package_xml
    assert 'default_value="false"' in controller
    assert not (
        PACKAGE_ROOT / "launch" / "visualize_marvin_manipulation.launch.py"
    ).exists()
    assert not (CONFIG_DIR / "rviz").exists()
