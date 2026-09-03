import importlib.util
from pathlib import Path

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE_ROOT / "launch"
CONFIG_DIR = PACKAGE_ROOT / "config"
REPO_ROOT = next(
    parent
    for parent in PACKAGE_ROOT.parents
    if (parent / "apps" / "profiles" / "piper_bimanual.yaml").is_file()
)


def _load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _profile() -> dict:
    return _load_yaml(REPO_ROOT / "apps" / "profiles" / "piper_bimanual.yaml")


def _five_camera_profile() -> dict:
    return _load_yaml(
        REPO_ROOT / "apps" / "profiles" / "piper_bimanual_five_camera.yaml"
    )


def _execution_manager() -> dict:
    return _load_yaml(CONFIG_DIR / "execution_manager.yaml")


def test_profile_points_at_canonical_workstation_stack() -> None:
    profile = _profile()
    bringup = profile["host_roles"]["workstation_host"]["bringup"]
    assert bringup == {
        "package": "piper_manipulation_workstation_launch",
        "launch_file": "workstation_stack.launch.py",
    }
    assert "groups" not in profile
    assert profile["execution_manager_config"] == {
        "package": "piper_manipulation_workstation_launch",
        "file": "config/execution_manager.yaml",
    }
    assert profile["recorder"]["config"] == {
        "package": "piper_manipulation_workstation_launch",
        "file": "config/recording/rmi_piper_bimanual.yaml",
    }
    assert profile["host_roles"]["rt_host"]["owns"] == [
        "ros2_control",
        "local_safety",
    ]
    assert len(profile["sensors"]["cameras"]) == 3
    assert "profile" not in profile["recorder"]


def test_execution_manager_config_is_the_only_routing_table() -> None:
    config = _execution_manager()
    assert set(config) == {"metadata", "max_command_age_s", "groups"}
    assert config["max_command_age_s"] == 0.25
    assert set(config["groups"]) == {
        "left_arm",
        "left_gripper",
        "right_arm",
        "right_gripper",
    }
    assert config["groups"]["left_arm"]["joint_names"] == [
        "left_joint1",
        "left_joint2",
        "left_joint3",
        "left_joint4",
        "left_joint5",
        "left_joint6",
    ]
    assert (
        config["groups"]["right_arm"]["controllers"]["joint_space_reference"][
            "ros_topics"
        ]["joint_reference"]
        == "/execution/right_arm/joint_reference"
    )


def test_execution_manager_launch_uses_package_config() -> None:
    path = LAUNCH_DIR / "execution_manager.launch.py"
    launch = path.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("config"' in launch
    assert "execution_manager.yaml" in launch
    assert 'get_package_share_directory("execution_manager")' in launch
    assert 'os.path.join(em_share, "launch", "execution_manager.launch.py")' in launch
    assert "workstation_defaults" not in launch
    assert "IfCondition" not in launch

    spec = importlib.util.spec_from_file_location("piper_em_launch", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module._max_command_age_s(str(CONFIG_DIR / "execution_manager.yaml")) == (
        "0.25"
    )


def test_recorder_launch_owns_only_the_daemon_queue() -> None:
    launch = (LAUNCH_DIR / "recorder.launch.py").read_text(encoding="utf-8")
    assert 'get_package_share_directory("episode_recorder")' in launch
    assert 'os.path.join(recorder_share, "launch", "recorder.launch.py")' in launch
    assert '"queue_capacity_bytes": str(4 * 1024 * 1024 * 1024)' in launch
    assert '"queue_capacity_messages": "16384"' in launch
    for legacy_argument in (
        "stream_config_uri",
        "recording_stream_config",
        "experiment_name",
        "task",
    ):
        assert legacy_argument not in launch


def test_workstation_stack_starts_one_selectable_static_camera() -> None:
    launch = (LAUNCH_DIR / "workstation_stack.launch.py").read_text(encoding="utf-8")
    for child in (
        "execution_manager.launch.py",
        "piper_leaders.launch.py",
        "piper_orbbec.launch.py",
        "piper_static_realsense.launch.py",
        "piper_realsense.launch.py",
        "recorder.launch.py",
    ):
        assert child in launch
    assert "execution_manager.yaml" in launch
    assert "GroupAction" in launch
    assert "forwarding=True" in launch
    assert '"static_camera"' in launch
    assert 'default_value="orbbec"' in launch
    assert '"static_realsense_serial_no"' in launch
    assert "IfCondition" in launch
    assert "with_execution_manager" not in launch
    assert "with_recorder" not in launch
    assert "workstation_defaults" not in launch


def test_camera_launches_use_piper_configs_and_three_names() -> None:
    orbbec = (LAUNCH_DIR / "piper_orbbec.launch.py").read_text(encoding="utf-8")
    realsense = (LAUNCH_DIR / "piper_realsense.launch.py").read_text(encoding="utf-8")
    static_realsense = (
        LAUNCH_DIR / "piper_static_realsense.launch.py"
    ).read_text(encoding="utf-8")
    assert "femto_bolt.yaml" in orbbec
    assert 'default_value="orbbec"' in orbbec
    for argument in (
        '"enable_depth": "false"',
        '"enable_ir": "false"',
        '"enable_accel": "false"',
        '"enable_gyro": "false"',
        '"enable_point_cloud": "false"',
        '"enable_colored_point_cloud": "false"',
        '"enable_frame_sync": "false"',
        '"publish_tf": "false"',
        '"use_hardware_time": "false"',
    ):
        assert argument in orbbec
    assert "d435i_dual.yaml" in realsense
    assert 'default_value="left_hand_realsense"' in realsense
    assert 'default_value="right_hand_realsense"' in realsense
    assert "_332522075913" in realsense
    assert "_332322073584" in realsense
    assert "OpaqueFunction(function=_resolved_cameras)" in realsense
    assert "forwarding=False" in realsense
    assert "resolved=True" not in realsense
    assert 'DeclareLaunchArgument("left_camera_delay", default_value="5.0")' in realsense
    assert 'DeclareLaunchArgument("right_camera_delay", default_value="15.0")' in realsense
    assert "d435i_single.yaml" in static_realsense
    assert 'default_value="static_d435i"' in static_realsense
    assert 'default_value="_310222078614"' in static_realsense
    d435i = _load_yaml(CONFIG_DIR / "camera" / "d435i_dual.yaml")
    assert d435i["wait_for_device_timeout"] == -1.0
    assert d435i["reconnect_timeout"] > 0.0
    static_d435i = _load_yaml(CONFIG_DIR / "camera" / "d435i_single.yaml")
    assert static_d435i["rgb_camera.color_profile"] == "640x480x30"
    assert static_d435i["enable_color"] is True
    assert static_d435i["enable_depth"] is False

    orbbec = _load_yaml(CONFIG_DIR / "camera" / "femto_bolt.yaml")["/**"][
        "ros__parameters"
    ]
    assert orbbec["enable_color"] is True
    for disabled in (
        "enable_depth",
        "enable_ir",
        "enable_accel",
        "enable_gyro",
        "enable_point_cloud",
        "enable_colored_point_cloud",
        "enable_frame_sync",
        "publish_tf",
    ):
        assert orbbec[disabled] is False


def test_five_camera_experiment_topology_and_recording_contract() -> None:
    launch = (LAUNCH_DIR / "piper_external_cameras.launch.py").read_text(
        encoding="utf-8"
    )
    assert "_405622076349" in launch
    assert "_310222078614" in launch
    for camera_name in ("d435i1", "d435i2"):
        assert camera_name in launch
    assert "OpaqueFunction(function=_resolved_cameras)" in launch
    assert "forwarding=False" in launch

    workstation = (LAUNCH_DIR / "five_camera_workstation.launch.py").read_text(
        encoding="utf-8"
    )
    assert "piper_orbbec.launch.py" in workstation
    assert "CL8384201CG" in workstation
    assert 'include("piper_realsense.launch.py")' in workstation
    assert 'include("piper_external_cameras.launch.py")' not in workstation

    recording = _load_yaml(
        CONFIG_DIR / "recording" / "rmi_piper_bimanual_five_camera.yaml"
    )
    required_cameras = {
        stream["topic"]
        for stream in recording["streams"]
        if stream["required"] and "image_raw" in stream["topic"]
    }
    assert required_cameras == {
        "/observation/orbbec/color/image_raw",
        "/observation/d435i1/color/image_raw",
        "/observation/d435i2/color/image_raw",
        "/observation/left_hand_realsense/color/image_raw",
        "/observation/right_hand_realsense/color/image_raw",
    }
    assert all(
        stream["start_gate"]
        for stream in recording["streams"]
        if stream["topic"] in required_cameras
    )

    profile = _five_camera_profile()
    assert profile["host_roles"]["rt_host"]["owns"][-1] == (
        "external_realsense_cameras"
    )
    assert profile["host_roles"]["workstation_host"]["bringup"]["launch_file"] == (
        "five_camera_workstation.launch.py"
    )
    assert len(profile["sensors"]["cameras"]) == 5
    assert profile["recorder"]["config"]["file"].endswith(
        "rmi_piper_bimanual_five_camera.yaml"
    )
    launch = (LAUNCH_DIR / "five_camera_workstation.launch.py").read_text(
        encoding="utf-8"
    )
    assert "forwarding=False" in launch
    assert '{"autostart": "true"}' in launch
    assert 'DeclareLaunchArgument("autostart"' not in launch


def test_leader_site_defaults_stay_in_yaml() -> None:
    leaders = _load_yaml(CONFIG_DIR / "teleop" / "piper_leaders.yaml")
    assert leaders["piper_leader_left"]["ros__parameters"]["can_interface"] == "can1"
    assert leaders["piper_leader_right"]["ros__parameters"]["can_interface"] == "can0"
    launch = (LAUNCH_DIR / "piper_leaders.launch.py").read_text(encoding="utf-8")
    assert 'default_value=""' in launch
    assert '"autostart": LaunchConfiguration("autostart")' in launch
    assert "left_joint1,left_joint2" not in launch


def test_recording_gripper_streams_use_float64_multiarray() -> None:
    recording = _load_yaml(CONFIG_DIR / "recording" / "rmi_piper_bimanual.yaml")
    by_id = {stream["id"]: stream for stream in recording["streams"]}
    for stream_id in (
        "execution_left_gripper_joint_reference",
        "execution_right_gripper_joint_reference",
    ):
        assert by_id[stream_id]["expected_type"] == "std_msgs/msg/Float64MultiArray"


def test_recording_requires_all_three_training_cameras_before_start() -> None:
    recording = _load_yaml(CONFIG_DIR / "recording" / "rmi_piper_bimanual.yaml")
    by_id = {stream["id"]: stream for stream in recording["streams"]}

    for stream_id in (
        "orbbec_color",
        "left_wrist_realsense_color",
        "right_wrist_realsense_color",
    ):
        assert by_id[stream_id]["required"] is True
        assert by_id[stream_id]["start_gate"] is True
