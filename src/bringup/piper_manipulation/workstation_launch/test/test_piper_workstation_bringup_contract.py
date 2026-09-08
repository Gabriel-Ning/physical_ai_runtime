from pathlib import Path

import yaml

ROOT = Path(__file__).parents[1]
LAUNCH_DIR = ROOT / "launch"


def test_workstation_camera_configs_exist():
    assert (ROOT / "config" / "camera" / "femto_bolt.yaml").is_file()
    assert (ROOT / "config" / "camera" / "realsense_d435.yaml").is_file()
    assert not (ROOT / "config" / "camera" / "piper_cameras.yaml").exists()


def test_orbbec_launch_uses_workstation_camera_config():
    launch = (LAUNCH_DIR / "piper_orbbec.launch.py").read_text(encoding="utf-8")
    config = yaml.safe_load(
        (ROOT / "config" / "camera" / "femto_bolt.yaml").read_text(encoding="utf-8")
    )
    assert "piper_manipulation_workstation_launch" in launch
    assert '"config", "camera", "femto_bolt.yaml"' in launch
    assert "orbbec_config" in launch
    assert 'DeclareLaunchArgument("config"' not in launch
    assert "femto_bolt.launch.py" in launch
    assert "launch.yaml" not in launch
    assert set(config) == {"static_orbbec"}
    assert config["static_orbbec"]["camera_namespace"] == "observation"
    assert config["static_orbbec"]["serial_number"] == ""
    assert config["static_orbbec"]["parameters"]["color_width"] == 1280


def test_realsense_launch_uses_workstation_camera_config():
    launch = (LAUNCH_DIR / "piper_realsense.launch.py").read_text(encoding="utf-8")
    config = yaml.safe_load(
        (ROOT / "config" / "camera" / "realsense_d435.yaml").read_text(encoding="utf-8")
    )
    assert "piper_manipulation_workstation_launch" in launch
    assert '"config", "camera", "realsense_d435.yaml"' in launch
    assert "realsense_config" in launch
    assert 'DeclareLaunchArgument("config"' not in launch
    assert "launch.yaml" not in launch
    assert "_332522075913" not in launch
    assert "OpaqueFunction(function=_setup)" in launch
    assert "left_serial_no" not in launch
    assert "right_serial_no" not in launch
    assert all(c["parameters"]["wait_for_device_timeout"] == 30.0 for c in config.values())
    assert all(c["parameters"]["reconnect_timeout"] == 2.0 for c in config.values())


def test_recording_gripper_streams_use_float64_multiarray():
    contract = ROOT.parents[3] / "apps" / "recording" / "piper_bimanual_mujoco.yaml"
    recording = yaml.safe_load(contract.read_text(encoding="utf-8"))
    by_id = {stream["id"]: stream for stream in recording["streams"]}
    assert (
        by_id["execution_left_gripper_joint_reference"]["expected_type"]
        == "std_msgs/msg/Float64MultiArray"
    )
    assert (
        by_id["execution_right_gripper_joint_reference"]["expected_type"]
        == "std_msgs/msg/Float64MultiArray"
    )


def test_leader_preempt_matches_apps_and_em_clutch():
    apps = ROOT.parents[3] / "apps"
    profile = yaml.safe_load(
        (apps / "profiles" / "piper_bimanual.yaml").read_text(encoding="utf-8")
    )
    real = yaml.safe_load(
        (apps / "profiles" / "site" / "piper_bimanual_real.yaml").read_text(encoding="utf-8")
    )
    em = yaml.safe_load((ROOT / "config" / "execution_manager.yaml").read_text(encoding="utf-8"))
    leaders = yaml.safe_load(
        (ROOT / "config" / "teleop" / "piper_leaders.yaml").read_text(encoding="utf-8")
    )
    recording = yaml.safe_load(
        (apps / "recording" / "piper_bimanual_real.yaml").read_text(encoding="utf-8")
    )
    left_clutch = "/teleop/piper_joint/left_clutch"
    right_clutch = "/teleop/piper_joint/right_clutch"
    for app_name in ("record.py", "teleop.py"):
        text = (apps / app_name).read_text(encoding="utf-8")
        assert "from std_msgs.msg import Bool" not in text
        assert "def set_teleop_preempt(" in text
        assert "preempt_service" in text
    teleop_app = (apps / "teleop.py").read_text(encoding="utf-8")
    assert "piper_leaders.yaml" not in teleop_app
    assert "JointTrajectory" not in teleop_app
    assert "teleoperators" not in profile
    assert "teleop" not in profile
    assert "teleoperators" not in real
    assert "teleop" not in real
    assert "teleoperators" not in leaders
    left_inputs = em["sources"]["TeleopJoint_Left"]["inputs"]
    right_inputs = em["sources"]["TeleopJoint_Right"]["inputs"]
    left_params = leaders["piper_leader_left"]["ros__parameters"]
    right_params = leaders["piper_leader_right"]["ros__parameters"]
    assert left_params["joint_reference_topic"] == left_inputs["left_arm"]["topic"]
    assert left_params["gripper_reference_topic"] == left_inputs["left_gripper"]["topic"]
    assert right_params["joint_reference_topic"] == right_inputs["right_arm"]["topic"]
    assert right_params["gripper_reference_topic"] == right_inputs["right_gripper"]["topic"]
    assert em["sources"]["TeleopJoint_Left"]["activation_topic"] == left_clutch
    assert em["sources"]["TeleopJoint_Right"]["activation_topic"] == right_clutch
    assert leaders["piper_leader_left"]["ros__parameters"]["preempt_service"] == (
        "/piper_leader_left/preempt"
    )
    assert leaders["piper_leader_right"]["ros__parameters"]["preempt_service"] == (
        "/piper_leader_right/preempt"
    )
    assert leaders["piper_leader_left"]["ros__parameters"]["clutch_topic"] == left_clutch
    assert leaders["piper_leader_right"]["ros__parameters"]["clutch_topic"] == right_clutch
    assert (
        leaders["piper_leader_left"]["ros__parameters"]["joint_names"]
        == (em["groups"]["left_arm"]["joint_names"])
    )
    assert (
        leaders["piper_leader_right"]["ros__parameters"]["joint_names"]
        == (em["groups"]["right_arm"]["joint_names"])
    )
    assert set(em["sources"]["TeleopJoint_Left"]["inputs"]) == {
        "left_arm",
        "left_gripper",
    }
    topics = {stream["topic"] for stream in recording["streams"]}
    assert left_clutch in topics
    assert right_clutch in topics


def test_no_camera_recording_contract_contains_no_image_streams():
    contract = ROOT.parents[3] / "apps" / "recording" / "piper_bimanual_no_cam.yaml"
    recording = yaml.safe_load(contract.read_text(encoding="utf-8"))
    assert all(
        not stream["expected_type"].startswith("sensor_msgs/msg/Image")
        for stream in recording["streams"]
    )

    profile = yaml.safe_load(
        (ROOT.parents[3] / "apps" / "profiles" / "piper_bimanual.yaml").read_text(encoding="utf-8")
    )
    assert profile["recorder"]["config"].endswith("piper_bimanual_mujoco.yaml")
    assert "teleoperators" not in profile
    assert "teleop" not in profile
    leaders = yaml.safe_load(
        (ROOT / "config" / "teleop" / "piper_leaders.yaml").read_text(encoding="utf-8")
    )
    assert set(leaders) == {"piper_leader_left", "piper_leader_right"}
    assert leaders["piper_leader_left"]["ros__parameters"]["clutch_topic"] == (
        "/teleop/piper_joint/left_clutch"
    )


def test_leader_defaults_live_in_teleop_config_not_launch():
    config = yaml.safe_load(
        (ROOT / "config" / "teleop" / "piper_leaders.yaml").read_text(encoding="utf-8")
    )
    for side, can_interface in (("left", "can0"), ("right", "can1")):
        params = config[f"piper_leader_{side}"]["ros__parameters"]
        assert params["can_interface"] == can_interface
        assert params["autostart"] is True
        assert params["joint_reference_topic"] == (
            f"/action_sources/teleop_joint_{side}/{side}_arm/joint_reference"
        )
        assert params["gripper_reference_topic"] == (
            f"/action_sources/teleop_joint_{side}/{side}_gripper/joint_reference"
        )
        assert params["clutch_topic"] == f"/teleop/piper_joint/{side}_clutch"
        assert params["preempt_service"] == f"/piper_leader_{side}/preempt"
        assert list(config[f"piper_leader_{side}"]) == ["ros__parameters"]

        launch = (LAUNCH_DIR / "piper_leaders.launch.py").read_text(encoding="utf-8")
        assert '"config", "teleop", "piper_leaders.yaml"' in launch
        assert "leader_config" in launch
        assert 'DeclareLaunchArgument("config"' not in launch
        assert '"node_name": node_name' in launch
        assert '"use_sim_time": use_sim_time' in launch
        assert "launch.yaml" not in launch


def test_workstation_stack_does_not_use_profile_defaults_helper():
    # Same contract as Marvin: launch args are declared in
    # workstation_stack.launch.py; no YAML/Python defaults bridge.
    assert not (LAUNCH_DIR / "workstation_defaults.py").is_file()
    assert not (ROOT / "config" / "launch.yaml").exists()
    assert not (ROOT / "config" / "recorder.yaml").exists()
    launch = (LAUNCH_DIR / "workstation_stack.launch.py").read_text(encoding="utf-8")
    assert "workstation_defaults" not in launch
    assert "workstation_launch_defaults" not in launch
    assert "launch.yaml" not in launch
    assert "yaml.safe_load" not in launch


def test_workstation_stack_launches_em_recorder_and_optional_peripherals():
    launch = (LAUNCH_DIR / "workstation_stack.launch.py").read_text(encoding="utf-8")
    assert "piper_manipulation_workstation_launch" in launch
    assert "execution_manager" in launch
    assert "recorder.launch.py" in launch
    assert "piper_orbbec.launch.py" in launch
    assert "piper_realsense.launch.py" in launch
    assert "piper_leaders.launch.py" in launch
    assert 'default_value="false"' in launch
    assert launch.count('default_value="true"') == 5


def test_workstation_stack_forwards_use_sim_time_to_em_recorder_and_leaders():
    workstation = (LAUNCH_DIR / "workstation_stack.launch.py").read_text(encoding="utf-8")
    assert workstation.count('"use_sim_time": LaunchConfiguration("use_sim_time")') == 3
    assert "piper_leaders.launch.py" in workstation
    assert "em_config" not in workstation
    assert "leader_config" not in workstation


def test_recorder_launch_includes_episode_recorder():
    recorder = (LAUNCH_DIR / "recorder.launch.py").read_text(encoding="utf-8")
    assert 'get_package_share_directory("episode_recorder")' in recorder
    assert 'os.path.join(recorder_share, "launch", "recorder.launch.py")' in recorder
    assert "recorder.yaml" not in recorder
    assert "launch.yaml" not in recorder
    assert "params_file" not in recorder
    assert '"root_dir"' not in recorder
