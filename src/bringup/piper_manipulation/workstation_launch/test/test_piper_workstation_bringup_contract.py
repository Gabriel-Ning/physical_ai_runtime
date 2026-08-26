from pathlib import Path

import yaml

ROOT = Path(__file__).parents[1]
LAUNCH_DIR = ROOT / "launch"


def test_workstation_camera_configs_exist():
    assert (ROOT / "config" / "camera" / "femto_bolt.yaml").is_file()
    assert (ROOT / "config" / "camera" / "d435i_dual.yaml").is_file()
    assert not (ROOT / "config" / "camera" / "piper_cameras.yaml").exists()


def test_orbbec_launch_uses_workstation_camera_config():
    launch = (LAUNCH_DIR / "piper_orbbec.launch.py").read_text(encoding="utf-8")
    assert "piper_manipulation_workstation_launch" in launch
    assert "femto_bolt.yaml" in launch
    assert "femto_bolt.launch.py" in launch


def test_realsense_launch_uses_workstation_camera_config():
    launch = (LAUNCH_DIR / "piper_realsense.launch.py").read_text(encoding="utf-8")
    config = yaml.safe_load(
        (ROOT / "config" / "camera" / "d435i_dual.yaml").read_text(encoding="utf-8")
    )
    assert "piper_manipulation_workstation_launch" in launch
    assert "d435i_dual.yaml" in launch
    assert "_332522075913" in launch
    assert "OpaqueFunction(function=_delayed_right_camera)" in launch
    assert config["wait_for_device_timeout"] == 30.0
    assert config["reconnect_timeout"] == 2.0


def test_recording_gripper_streams_use_float64_multiarray():
    recording = yaml.safe_load(
        (ROOT / "config" / "recording" / "rmi_piper_bimanual.yaml").read_text(
            encoding="utf-8"
        )
    )
    by_id = {stream["id"]: stream for stream in recording["streams"]}
    assert (
        by_id["execution_left_gripper_joint_reference"]["expected_type"]
        == "std_msgs/msg/Float64MultiArray"
    )
    assert (
        by_id["execution_right_gripper_joint_reference"]["expected_type"]
        == "std_msgs/msg/Float64MultiArray"
    )


def test_leader_defaults_live_in_teleop_config_not_launch():
    leaders = yaml.safe_load(
        (ROOT / "config" / "teleop" / "piper_leaders.yaml").read_text(encoding="utf-8")
    )
    assert leaders["piper_leader_left"]["ros__parameters"]["can_interface"] == "can1"
    assert leaders["piper_leader_right"]["ros__parameters"]["can_interface"] == "can0"
    bringup = (LAUNCH_DIR / "piper_leaders.launch.py").read_text(encoding="utf-8")
    assert 'default_value=""' in bringup
    assert "left_joint1,left_joint2" not in bringup
    assert "/action_sources/piper_leader_left/arm/joint_reference" not in bringup


def test_leader_autostart_override_is_forwarded_to_both_includes():
    bringup = (LAUNCH_DIR / "piper_leaders.launch.py").read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("autostart", default_value="")' in bringup
    assert '"autostart": LaunchConfiguration("autostart")' in bringup


def test_workstation_stack_launches_em_recorder_and_optional_peripherals():
    launch = (LAUNCH_DIR / "piper_workstation.launch.py").read_text(encoding="utf-8")
    assert "piper_manipulation_workstation_launch" in launch
    assert "execution_manager" in launch
    assert "recorder.launch.py" in launch
    assert "piper_orbbec.launch.py" in launch
    assert "piper_realsense.launch.py" in launch
    assert "piper_leaders.launch.py" in launch
