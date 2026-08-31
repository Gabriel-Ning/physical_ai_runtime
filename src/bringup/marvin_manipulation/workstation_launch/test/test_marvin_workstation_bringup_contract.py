import re
from pathlib import Path

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
    return _load_yaml(REPO_ROOT / "apps" / "recording" / "marvin_manipulation.yaml")


def _no_camera_recording() -> dict:
    return _load_yaml(
        REPO_ROOT / "apps" / "recording" / "marvin_manipulation_no_cam.yaml"
    )


def _streams_by_id() -> dict:
    return {stream["id"]: stream for stream in _recording()["streams"]}


def _provider_token(provider: str) -> str:
    snake = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", "_", provider.strip())
    return re.sub(r"[^a-z0-9]+", "_", snake.lower()).strip("_")


def test_application_profile_has_no_deployment_roles() -> None:
    assert "host_roles" not in _profile()


def test_workstation_stack_conditionally_starts_cameras() -> None:
    launch = (LAUNCH_DIR / "workstation_stack.launch.py").read_text(encoding="utf-8")
    camera = (LAUNCH_DIR / "realsense_camera.launch.py").read_text(encoding="utf-8")
    assert "execution_manager.launch.py" in launch
    assert "realsense_camera.launch.py" in launch
    assert "recorder.launch.py" in launch
    assert "quest3_teleop.launch.py" in launch
    assert "em_config" in launch
    assert "execution_manager.yaml" in launch
    assert "IfCondition" in launch
    assert "with_execution_manager" not in launch
    assert "with_recorder" not in launch
    assert 'DeclareLaunchArgument(\n                "with_cameras"' in launch
    assert 'IfCondition(LaunchConfiguration("with_cameras"))' in launch
    assert 'DeclareLaunchArgument(\n                "with_teleop"' in launch
    assert 'IfCondition(LaunchConfiguration("with_teleop"))' in launch
    assert "workstation_launch_defaults" not in launch
    assert not (LAUNCH_DIR / "workstation_defaults.py").is_file()
    assert "apps/profiles" not in launch
    assert 'get_package_share_directory("episode_recorder")' not in launch
    assert "pika_camera_bringup.launch.py" not in launch
    assert "rt_stack.launch.py" not in launch
    assert "controller_bringup.launch.py" not in launch
    assert "configured_cameras = yaml.safe_load(stream)" in camera
    assert "for index, namespace in enumerate(configured_cameras)" in camera
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
        "sources",
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
    assert em["sources"]["Replay"]["source_role"] == "MEMORY"
    for side in ("left", "right"):
        gripper = em["groups"][f"{side}_gripper"]
        assert gripper["controllers"]["gripper_command"]["ros_actions"] == {
            "gripper_command": f"/execution/{side}_gripper/gripper_command"
        }
        assert em["sources"]["Planner"]["inputs"][f"{side}_gripper"] == {
            "command_contract": "gripper_command",
            "action": f"/action_sources/planner/{side}_gripper/gripper_command",
        }


def test_no_camera_profile_keeps_full_execution_capability() -> None:
    site = _load_yaml(
        REPO_ROOT / "apps" / "profiles" / "site" / "marvin_bimanual_no_cam.yaml"
    )
    expected = _profile()
    expected["recorder"]["config"] = "../../recording/marvin_manipulation_no_cam.yaml"
    assert site == expected
    assert site["nodes"]["DummyPolicy"]["resources"] == {
        "left_arm": "joint_reference",
        "left_gripper": "joint_reference",
        "right_arm": "joint_reference",
        "right_gripper": "joint_reference",
    }
    launch = (LAUNCH_DIR / "workstation_stack.launch.py").read_text(encoding="utf-8")
    assert "execution_manager.launch.py" in launch
    assert "recorder.launch.py" in launch
    assert "realsense_camera.launch.py" in launch
    assert not (LAUNCH_DIR / "workstation_stack_no_cam.launch.py").exists()


def test_marvin_profiles_register_supported_teleop_nodes() -> None:
    profiles = [
        _profile(),
        _load_yaml(
            REPO_ROOT / "apps" / "profiles" / "site" / "marvin_bimanual_no_cam.yaml"
        ),
    ]
    for profile in profiles:
        teleop_nodes = {
            name
            for name, node in profile["nodes"].items()
            if node["source_role"] == "TELEOP"
        }
        assert teleop_nodes == {"Quest3Teleop", "TeleopJoint"}
        assert profile["nodes"]["Quest3Teleop"]["resources"] == {
            "left_arm": "pose_reference",
            "left_gripper": "joint_reference",
            "right_arm": "pose_reference",
            "right_gripper": "joint_reference",
        }
        assert profile["nodes"]["TeleopJoint"]["resources"] == {
            "left_arm": "joint_reference",
            "left_gripper": "joint_reference",
            "right_arm": "joint_reference",
            "right_gripper": "joint_reference",
        }
        assert profile["nodes"]["Replay"]["source_role"] == "MEMORY"
        assert profile["nodes"]["Planner"]["resources"] == {
            "left_arm": "joint_trajectory",
            "left_gripper": "gripper_command",
            "right_arm": "joint_trajectory",
            "right_gripper": "gripper_command",
        }


def test_quest3_launch_uses_marvin_specific_parameter_overrides() -> None:
    config = _load_yaml(CONFIG_DIR / "teleop" / "quest3_bimanual_relative.yaml")
    params = config["quest3_bimanual_target"]["ros__parameters"]
    assert params["left_output_topic"] == (
        "/action_sources/quest3/left_arm/pose_reference"
    )
    assert params["right_output_topic"] == (
        "/action_sources/quest3/right_arm/pose_reference"
    )
    assert params["left_tcp_frame"] == "left_pika_gripper_tcp"
    assert params["right_tcp_frame"] == "right_pika_gripper_tcp"
    assert params["left_gripper_joint_name"] == "left_gripper_left_joint"
    assert params["right_gripper_joint_name"] == "right_gripper_left_joint"

    launch = (LAUNCH_DIR / "quest3_teleop.launch.py").read_text(encoding="utf-8")
    assert "quest3_bimanual_relative.yaml" in launch
    assert "bimanual_target_live.launch.py" in launch
    assert '"profile_config": LaunchConfiguration("quest3_config")' in launch
    assert '"left_base_frame": "Base_L"' in launch
    assert '"right_base_frame": "Base_R"' in launch


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
    assert profile["recorder"]["config"] == "../recording/marvin_manipulation.yaml"
    assert "profile" not in profile["recorder"]
    assert profile["recorder"]["root_dir"] == "data/episodes"
    assert profile["recorder"]["experiment_name"] == "marvin_bimanual"
    assert profile["recorder"]["task"] == "bimanual_manipulation"
    assert "homing_duration_s" not in profile["recorder"]
    assert "home_pose" not in profile["recorder"]
    assert profile["homing"] == {
        "duration_s": 8.0,
        "joint_positions": {
            "left_arm": [
                -1.0503277641628497,
                1.2404911146950939,
                1.566118084866009,
                -2.086290437405812,
                0.48874412219966884,
                0.09129618319127784,
                0.13037110635746443,
            ],
            "left_gripper": [0.045],
            "right_arm": [
                -2.0890998060397363,
                -1.1433997182018572,
                1.526900108337393,
                -2.0866996321073024,
                -0.41910032916522466,
                0.13079942588453322,
                -0.21079963654229344,
            ],
            "right_gripper": [0.045],
        },
    }


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


def test_recording_start_gate_is_joint_states_and_authority() -> None:
    streams = _recording()["streams"]
    no_camera_streams = {
        stream["id"]: stream for stream in _no_camera_recording()["streams"]
    }
    start_gates = [stream["id"] for stream in streams if stream.get("start_gate")]
    required = [stream["id"] for stream in streams if stream.get("required")]
    camera_ids = {
        stream["id"]
        for stream in streams
        if stream["topic"].startswith(
            ("/left_pika_", "/right_pika_", "/head_d435/", "/third_person_d435/")
        )
    }
    assert set(start_gates) == {
        "robot_joint_states",
        "authority_status",
        *camera_ids,
    }
    assert set(required) >= {
        "robot_joint_states",
        "authority_status",
        "authority_events",
        *camera_ids,
    }
    authority = next(stream for stream in streams if stream["id"] == "authority_status")
    assert authority["start_gate_max_age_s"] == 2.0
    assert no_camera_streams["authority_status"]["start_gate_max_age_s"] == 2.0


def test_camera_and_no_camera_recording_contracts_only_change_camera_gates() -> None:
    with_cameras = {stream["id"]: stream for stream in _recording()["streams"]}
    without_cameras = {
        stream["id"]: stream for stream in _no_camera_recording()["streams"]
    }
    assert set(with_cameras) == set(without_cameras)

    camera_ids = {
        stream_id
        for stream_id, stream in with_cameras.items()
        if stream["topic"].startswith(
            ("/left_pika_", "/right_pika_", "/head_d435/", "/third_person_d435/")
        )
    }
    assert len(camera_ids) == 10
    for stream_id in camera_ids:
        assert with_cameras[stream_id]["required"] is True
        assert with_cameras[stream_id]["start_gate"] is True
        assert without_cameras[stream_id]["required"] is False
        assert without_cameras[stream_id]["start_gate"] is False

    for stream_id in set(with_cameras) - camera_ids:
        assert with_cameras[stream_id] == without_cameras[stream_id]


def test_recording_covers_profile_provider_topics_and_traces() -> None:
    topics = {stream["topic"] for stream in _recording()["streams"]}
    assert {
        "/execution_manager/authority_status",
        "/execution_manager/authority_events",
    } <= topics
    assert "/action_sources/policy/left_arm/joint_reference" in topics
    assert "/action_sources/quest3/left_arm/pose_reference" in topics
    assert "/execution_trace/policy/left_arm/joint_reference" in topics
    assert "/execution_trace/teleop/left_arm/pose_reference" in topics
    assert "/execution_trace/teleop/left_gripper/joint_reference" in topics
    assert "/action_sources/replay/left_arm/joint_reference" in topics
    for side in ("left", "right"):
        assert f"/execution/{side}_arm/joint_reference" in topics
        assert f"/execution/{side}_arm/pose_reference" in topics
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


def test_workstation_camera_config_has_connected_head_d435() -> None:
    camera = _load_yaml(CONFIG_DIR / "camera" / "workstation_realsense.yaml")
    assert set(camera) == {"head_d435"}
    head = camera["head_d435"]["camera"]["ros__parameters"]
    assert head["serial_no"] == "_243222071293"
    assert head["camera_name"] == "head_d435"
    assert head["rgb_camera"]["color_profile"] == "1280x720x30"


def test_package_xml_declares_workstation_runtime_only() -> None:
    text = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    for dep in (
        "episode_recorder",
        "execution_manager",
        "execution_manager_interfaces",
        "isaacteleop_toolbox",
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
