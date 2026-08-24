from pathlib import Path
import re

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE_ROOT / "launch"
CONFIG_DIR = PACKAGE_ROOT / "config"
REPO_ROOT = next(
    parent
    for parent in PACKAGE_ROOT.parents
    if (parent / "apps" / "profiles" / "marvin_bimanual.yaml").is_file()
)


def _load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _profile() -> dict:
    return _load_yaml(REPO_ROOT / "apps" / "profiles" / "marvin_bimanual.yaml")


def _em() -> dict:
    return _load_yaml(CONFIG_DIR / "execution_manager.yaml")


def _recording() -> dict:
    return _load_yaml(CONFIG_DIR / "recording" / "marvin_manipulation.yaml")


def _streams_by_id() -> dict:
    return {stream["id"]: stream for stream in _recording()["streams"]}


def _provider_token(provider: str) -> str:
    snake = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", "_", provider.strip())
    return re.sub(r"[^a-z0-9]+", "_", snake.lower()).strip("_")


def test_profile_points_at_this_workstation_package() -> None:
    host = _profile()["host_roles"]["workstation_host"]
    bringup = host["bringup"]
    assert bringup["package"] == "marvin_manipulation_workstation_launch"
    assert bringup["launch_file"] == "workstation_stack.launch.py"
    assert "arguments" not in bringup
    assert "execution_manager" in host["owns"]
    assert "ros2_control" not in host["owns"]


def test_workstation_stack_always_starts_em_recorder_and_cameras() -> None:
    launch = (LAUNCH_DIR / "workstation_stack.launch.py").read_text(encoding="utf-8")
    camera = (LAUNCH_DIR / "realsense_camera.launch.py").read_text(encoding="utf-8")
    assert "execution_manager.launch.py" in launch
    assert "realsense_camera.launch.py" in launch
    assert "recorder.launch.py" in launch
    assert "em_config" in launch
    assert "execution_manager.yaml" in launch
    assert "IfCondition" not in launch
    assert "with_execution_manager" not in launch
    assert "with_recorder" not in launch
    assert "with_cameras" not in launch
    assert "workstation_launch_defaults" not in launch
    assert not (LAUNCH_DIR / "workstation_defaults.py").is_file()
    assert "apps/profiles" not in launch
    assert 'get_package_share_directory("episode_recorder")' not in launch
    assert "pika_camera_bringup.launch.py" not in launch
    assert "rt_stack.launch.py" not in launch
    assert "controller_bringup.launch.py" not in launch
    assert '_realsense("head_d435")' in camera
    assert '_realsense("third_person_d435")' in camera
    assert 'package="realsense2_camera"' in camera
    assert "workstation_realsense.yaml" in camera
    assert "TimerAction" in camera
    assert '"second_camera_delay"' in camera
    assert "pika_d405.yaml" not in camera
    assert "pika_fisheye.yaml" not in camera


def test_execution_manager_launch_uses_package_config() -> None:
    launch = (LAUNCH_DIR / "execution_manager.launch.py").read_text(encoding="utf-8")
    workstation = (LAUNCH_DIR / "workstation_stack.launch.py").read_text(
        encoding="utf-8"
    )
    assert (LAUNCH_DIR / "execution_manager.launch.py").is_file()
    assert (CONFIG_DIR / "execution_manager.yaml").is_file()
    assert "workstation_defaults" not in launch
    assert "marvin_bimanual.yaml" not in launch
    assert "apps/profiles" not in launch
    assert 'DeclareLaunchArgument("config"' in launch
    assert "execution_manager.yaml" in launch
    assert 'get_package_share_directory("execution_manager")' in launch
    assert 'os.path.join(em_share, "launch", "execution_manager.launch.py")' in launch
    assert 'DeclareLaunchArgument("max_command_age_s"' not in launch
    assert "embodiment_profile" not in launch
    assert '"config": LaunchConfiguration("embodiment_profile")' not in workstation
    assert "with_execution_manager" not in launch
    assert "IfCondition" not in launch
    assert "rt_stack.launch.py" not in launch
    assert "controller_bringup" not in launch
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "marvin_em_launch", LAUNCH_DIR / "execution_manager.launch.py"
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module._max_command_age_s(str(CONFIG_DIR / "execution_manager.yaml")) == (
        "0.25"
    )


def test_execution_manager_config_is_the_only_routing_table() -> None:
    em = _em()
    profile = _profile()
    assert set(em) == {
        "metadata",
        "max_command_age_s",
        "groups",
    }
    assert em["max_command_age_s"] == 0.25
    assert em["metadata"]["name"] == "marvin_bimanual"
    for forbidden in (
        "host_roles",
        "compound_groups",
        "agents",
        "sensors",
        "recorder",
        "features",
        "teleoperators",
    ):
        assert forbidden not in em
    assert "groups" not in profile
    assert "provider_selection" not in profile
    assert profile["execution_manager_config"] == {
        "package": "marvin_manipulation_workstation_launch",
        "file": "config/execution_manager.yaml",
    }
    assert set(em["groups"]) == {
        "left_arm",
        "left_gripper",
        "right_arm",
        "right_gripper",
    }
    assert em["groups"]["right_arm"]["joint_names"][5] == "Joint6_R"


def test_no_right_gripper_overlay_removes_execution_capability() -> None:
    overlay = _load_yaml(CONFIG_DIR / "execution_manager_no_right_gripper.yaml")
    default = _em()
    assert set(overlay["groups"]) == set(default["groups"]) - {"right_gripper"}
    assert "right_gripper" not in overlay["groups"]
    site = _load_yaml(
        REPO_ROOT
        / "apps"
        / "profiles"
        / "site"
        / "marvin_bimanual_no_right_gripper.yaml"
    )
    assert site["execution_manager_config"]["file"] == (
        "config/execution_manager_no_right_gripper.yaml"
    )


def test_recorder_launch_uses_package_config() -> None:
    recorder = (LAUNCH_DIR / "recorder.launch.py").read_text(encoding="utf-8")
    workstation = (LAUNCH_DIR / "workstation_stack.launch.py").read_text(
        encoding="utf-8"
    )
    assert (LAUNCH_DIR / "recorder.launch.py").is_file()
    assert "workstation_defaults" not in recorder
    assert "apps/profiles" not in recorder
    assert "apps/profiles/marvin_bimanual.yaml" not in recorder
    assert "DeclareLaunchArgument" not in recorder
    assert "marvin_manipulation.yaml" not in recorder
    assert "rmi_marvin_bimanual.yaml" not in recorder
    assert not (CONFIG_DIR / "recording" / "rmi_marvin_bimanual.yaml").exists()
    assert 'get_package_share_directory("episode_recorder")' in recorder
    assert 'os.path.join(recorder_share, "launch", "recorder.launch.py")' in recorder
    assert "stream_config_uri" not in recorder
    assert '"queue_capacity_bytes": str(4 * 1024 * 1024 * 1024)' in recorder
    assert '"queue_capacity_messages": "16384"' in recorder
    assert "recording_stream_config" not in workstation
    assert '"root_dir"' not in workstation
    assert '"experiment_name"' not in workstation
    assert '"task"' not in workstation
    assert '"root_dir"' not in recorder
    assert '"experiment_name"' not in recorder
    assert '"task"' not in recorder


def test_recording_config_is_the_stream_contract() -> None:
    recording = _recording()
    profile = _profile()
    assert set(recording) == {"schema_version", "streams"}
    assert recording["schema_version"] == 1
    for forbidden in (
        "root_dir",
        "experiment_name",
        "task",
        "host_roles",
        "groups",
        "provider_selection",
        "agents",
        "sensors",
        "features",
        "teleoperators",
    ):
        assert forbidden not in recording
    assert profile["recorder"]["config"] == {
        "package": "marvin_manipulation_workstation_launch",
        "file": "config/recording/marvin_manipulation.yaml",
    }
    assert "profile" not in profile["recorder"]
    assert profile["recorder"]["root_dir"] == "data/episodes"
    assert profile["recorder"]["experiment_name"] == "marvin_bimanual"
    assert profile["recorder"]["task"] == "bimanual_manipulation"


def test_recording_gripper_streams_use_float64_multiarray() -> None:
    by_id = _streams_by_id()
    for stream_id in (
        "execution_left_gripper_joint_reference",
        "execution_right_gripper_joint_reference",
    ):
        assert by_id[stream_id]["expected_type"] == "std_msgs/msg/Float64MultiArray"
        assert by_id[stream_id]["required"] is False
        assert by_id[stream_id]["start_gate"] is False


def test_recording_keeps_right_gripper_even_if_rt_omits_hardware() -> None:
    by_id = _streams_by_id()
    assert "right_gripper" in _em()["groups"]
    assert by_id["source_policy_right_gripper"]["topic"] == (
        "/action_sources/policy/right_gripper/joint_reference"
    )
    assert by_id["execution_right_gripper_joint_reference"]["topic"] == (
        "/execution/right_gripper/joint_reference"
    )
    assert by_id["trace_policy_right_gripper"]["topic"] == (
        "/execution_trace/policy/right_gripper/joint_reference"
    )


def test_recording_start_gate_is_only_joint_states() -> None:
    streams = _recording()["streams"]
    start_gates = [stream["id"] for stream in streams if stream.get("start_gate")]
    required = [stream["id"] for stream in streams if stream.get("required")]
    assert start_gates == ["robot_joint_states"]
    assert required == ["robot_joint_states"]


def test_recording_covers_profile_provider_topics_and_traces() -> None:
    topics = {stream["topic"] for stream in _recording()["streams"]}
    assert {"/execution_manager/authority_status", "/execution_manager/authority_events"} <= topics
    assert "/action_sources/policy/left_arm/joint_reference" in topics
    assert "/action_sources/teleop/left_arm/joint_reference" in topics
    assert "/execution_trace/policy/left_arm/joint_reference" in topics
    assert "/execution_trace/teleop/left_arm/joint_reference" in topics
    for side in ("left", "right"):
        assert f"/execution/{side}_arm/joint_reference" in topics
        assert f"/execution/{side}_gripper/joint_reference" in topics


def test_recording_camera_topics_match_profile_and_rt_wrist_streams() -> None:
    profile = _profile()
    topics = {stream["topic"] for stream in _recording()["streams"]}
    cameras = profile["sensors"]["cameras"]
    assert set(cameras) == {
        "left_pika_d405",
        "left_pika_fisheye",
        "right_pika_d405",
        "right_pika_fisheye",
        "head_d435",
        "third_person_d435",
    }
    for name, camera in cameras.items():
        assert camera["ros_topic"] in topics
        if "fisheye" in name:
            assert camera["encoding"] == "jpeg"
    for side in ("left", "right"):
        assert f"/{side}_pika_d405/camera/color/image_raw" in topics
        assert f"/{side}_pika_d405/camera/aligned_depth_to_color/image_raw" in topics
        assert f"/{side}_pika_d405/camera/depth/image_rect_raw" not in topics
        assert f"/{side}_pika_fisheye/image/compressed" in topics
    assert "/head_d435/camera/color/image_raw" in topics
    assert "/head_d435/camera/aligned_depth_to_color/image_raw" in topics
    assert "/third_person_d435/camera/color/image_raw" in topics
    assert "/third_person_d435/camera/aligned_depth_to_color/image_raw" in topics
    assert "/workstation_realsense/camera/depth/image_rect_raw" not in topics
    assert not any(topic.endswith("/depth/image_rect_raw") for topic in topics)
    assert profile["features"]["observation"]["observation.images.head_d435"][
        "shape"
    ] == [3, 720, 1280]
    assert profile["features"]["observation"]["observation.images.third_person_d435"][
        "shape"
    ] == [3, 720, 1280]


def test_workstation_camera_config_has_head_and_third_person_d435() -> None:
    camera = _load_yaml(CONFIG_DIR / "camera" / "workstation_realsense.yaml")
    assert set(camera) == {"head_d435", "third_person_d435"}
    head = camera["head_d435"]["camera"]["ros__parameters"]
    assert head["serial_no"] == "_243222071293"
    assert head["camera_name"] == "head_d435"
    assert head["rgb_camera"]["color_profile"] == "1280x720x30"
    side = camera["third_person_d435"]["camera"]["ros__parameters"]
    assert side["serial_no"] == "_405622076349"
    assert side["camera_name"] == "third_person_d435"
    assert side["enable_color"] is True
    assert side["enable_depth"] is True
    assert side["rgb_camera"]["color_profile"] == "1280x720x30"
    assert side["depth_module"]["depth_profile"] == "1280x720x30"
    assert side["align_depth"]["enable"] is True
    assert side["depth_module"]["frames_queue_size"] == 8


def test_package_xml_declares_workstation_runtime_only() -> None:
    text = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    for dep in (
        "episode_recorder",
        "execution_manager",
        "execution_manager_interfaces",
        "python3-yaml",
        "realsense2_camera",
    ):
        assert f"<exec_depend>{dep}</exec_depend>" in text
    assert "marvin_manipulation_rt_launch" not in text
    assert "marvin_hardware_interface" not in text
    assert "controller_manager" not in text


def test_docs_do_not_point_at_removed_packages() -> None:
    readme = (PACKAGE_ROOT / "README.md").read_text(encoding="utf-8")
    assert "marvin_manipulation_controller_bringup" not in readme
    assert "physical_ai_workstation_bringup" not in readme
    assert "src/new_apps" not in readme
    assert "workstation_stack.launch.py" in readme
    assert "execution_manager.launch.py" in readme
    assert "execution_manager.yaml" in readme
    assert "recorder.launch.py" in readme
    assert "marvin_manipulation.yaml" in readme
    assert "rmi_marvin_bimanual.yaml" not in readme
    assert "双 Pika" in readme
    assert "D405" in readme
    assert "鱼眼" in readme
    assert "D435" in readme
    assert "D430" not in readme
