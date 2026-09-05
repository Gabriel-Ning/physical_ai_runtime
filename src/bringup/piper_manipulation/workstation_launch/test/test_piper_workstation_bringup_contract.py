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
    contract = ROOT.parents[3] / "apps" / "recording" / "piper_bimanual.yaml"
    recording = yaml.safe_load(
        contract.read_text(encoding="utf-8")
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


def test_no_camera_recording_contract_contains_no_image_streams():
    contract = ROOT.parents[3] / "apps" / "recording" / "piper_bimanual_no_cam.yaml"
    recording = yaml.safe_load(contract.read_text(encoding="utf-8"))
    assert all(
        not stream["expected_type"].startswith("sensor_msgs/msg/Image")
        for stream in recording["streams"]
    )

    profile = yaml.safe_load(
        (ROOT.parents[3] / "apps" / "profiles" / "piper_bimanual.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert profile["recorder"]["config"].endswith("piper_bimanual_no_cam.yaml")
    assert {
        config["preempt_service"] for config in profile["teleoperators"].values()
    } == {"/piper_leader_left/preempt", "/piper_leader_right/preempt"}


def test_leader_defaults_live_in_teleop_config_not_launch():
    config = yaml.safe_load(
        (ROOT / "config" / "teleop" / "piper_leaders.yaml").read_text(
            encoding="utf-8"
        )
    )
    for side, can_interface in (("left", "can1"), ("right", "can0")):
        params = config[f"piper_leader_{side}"]["ros__parameters"]
        assert params["can_interface"] == can_interface
        assert params["autostart"] is True
        assert params["joint_reference_topic"] == (
            f"/action_sources/piper_leader_{side}/arm/joint_reference"
        )

        launch = (LAUNCH_DIR / "piper_leaders.launch.py").read_text(encoding="utf-8")
        assert "piper_leaders.yaml" in launch
        assert '"node_name": f"piper_leader_{side}"' in launch


def test_workstation_stack_launches_em_recorder_and_optional_peripherals():
    launch = (LAUNCH_DIR / "workstation_stack.launch.py").read_text(encoding="utf-8")
    assert "piper_manipulation_workstation_launch" in launch
    assert "execution_manager" in launch
    assert "recorder.launch.py" in launch
    assert "piper_orbbec.launch.py" in launch
    assert "piper_realsense.launch.py" in launch
    assert "piper_leaders.launch.py" in launch


def test_workstation_stack_does_not_forward_child_configs():
    workstation = (LAUNCH_DIR / "workstation_stack.launch.py").read_text(
        encoding="utf-8"
    )

    assert "launch_arguments=" not in workstation
    assert "em_config" not in workstation
    assert "leader_config" not in workstation
