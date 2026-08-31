from pathlib import Path

import yaml

ROOT = Path(__file__).parents[1]
LAUNCH_DIR = ROOT / "launch"


def test_gamepad_defaults_live_in_franka_teleop_config():
    config = yaml.safe_load(
        (ROOT / "config" / "teleop" / "gamepad.yaml").read_text(encoding="utf-8")
    )
    parameters = config["gamepad_teleop"]["ros__parameters"]
    assert parameters["frame_id"] == "fr3_link0"
    assert parameters["gripper_open_button"] == 1
    assert parameters["gripper_close_button"] == 3
    assert parameters["gripper_close_axis"] == -1
    assert parameters["gripper_joint_name"] == "gripper_left_joint"
    assert parameters["require_clutch_for_gripper"] is True


def test_gamepad_launch_includes_generic_input_source():
    launch = (LAUNCH_DIR / "gamepad_teleop.launch.py").read_text(encoding="utf-8")
    assert 'get_package_share_directory("gamepad_teleop")' in launch
    assert '"config", "teleop", "gamepad.yaml"' in launch
    assert "gamepad_teleop.launch.py" in launch
    assert 'DeclareLaunchArgument(\n                "gamepad_config"' in launch
    assert 'LaunchConfiguration("gamepad_config")' in launch


def test_execution_manager_config_owns_routes():
    config = yaml.safe_load(
        (ROOT / "config" / "execution_manager.yaml").read_text(encoding="utf-8")
    )
    assert (
        config["groups"]["arm"]["controllers"]["task_space_reference"]["ros_topics"][
            "twist_reference"
        ]
        == "/execution/arm/twist_reference"
    )
    assert (
        config["groups"]["end_effector"]["controllers"]["joint_space_reference"]["name"]
        == "pika_gripper_fwd"
    )
    assert config["groups"]["end_effector"]["controllers"]["joint_space_reference"][
        "implementation"
    ] == ("forward_command_controller/ForwardCommandController")
    assert config["groups"]["end_effector"]["controllers"]["gripper_command"] == {
        "name": "pika_gripper_action",
        "implementation": (
            "parallel_gripper_action_controller/GripperActionController"
        ),
        "ros_actions": {"gripper_command": "/execution/end_effector/gripper_command"},
    }
    assert (
        config["groups"]["arm"]["controllers"]["joint_space_reference"]["name"]
        == "franka_arm_jspc"
    )
    assert (
        config["groups"]["arm"]["controllers"]["task_space_reference"]["name"]
        == "franka_arm_tskpc"
    )
    sources = config["sources"]
    assert sources["DummyPolicy"]["inputs"]["arm"] == {
        "command_contract": "joint_reference",
        "topic": "/action_sources/dummy_policy/arm/joint_reference",
    }
    assert sources["GamepadTeleop"]["activation_topic"] == ("/teleop/gamepad/clutch")
    assert sources["GamepadTeleop"]["preempt"] is True
    assert sources["Planner"]["inputs"]["end_effector"] == {
        "command_contract": "gripper_command",
        "action": "/action_sources/planner/end_effector/gripper_command",
    }


def test_workstation_stack_launches_gamepad_em_and_recorder():
    launch = (LAUNCH_DIR / "workstation_stack.launch.py").read_text(encoding="utf-8")
    assert "execution_manager.launch.py" in launch
    assert "gamepad_teleop.launch.py" in launch
    assert "recorder.launch.py" in launch


def test_recorder_captures_preemption_and_routed_actions():
    contract = ROOT.parents[3] / "apps" / "recording" / "franka_manipulation.yaml"
    config = yaml.safe_load(contract.read_text(encoding="utf-8"))
    topics = {stream["topic"] for stream in config["streams"]}
    assert {
        "/teleop/gamepad/clutch",
        "/execution_manager/authority_status",
        "/execution_manager/authority_events",
        "/action_sources/dummy_policy/arm/joint_reference",
        "/action_sources/gamepad/arm/twist",
        "/execution_trace/policy/arm/joint_reference",
        "/execution_trace/teleop/arm/twist_reference",
        "/execution/arm/joint_reference",
        "/execution/arm/twist_reference",
    } <= topics


def test_camera_and_no_camera_recording_contracts_share_topics_but_gate_differently():
    recording_dir = ROOT.parents[3] / "apps" / "recording"
    camera = yaml.safe_load(
        (recording_dir / "franka_manipulation.yaml").read_text(encoding="utf-8")
    )
    no_camera = yaml.safe_load(
        (recording_dir / "franka_manipulation_no_cam.yaml").read_text(encoding="utf-8")
    )
    camera_by_id = {stream["id"]: stream for stream in camera["streams"]}
    no_camera_by_id = {stream["id"]: stream for stream in no_camera["streams"]}
    assert camera_by_id.keys() == no_camera_by_id.keys()
    camera_ids = {"pika_d405_color", "pika_d405_depth", "pika_fisheye_color"}
    for stream_id in camera_ids:
        assert camera_by_id[stream_id]["required"] is True
        assert camera_by_id[stream_id]["start_gate"] is True
        assert no_camera_by_id[stream_id]["required"] is False
        assert no_camera_by_id[stream_id]["start_gate"] is False


def test_application_profile_references_em_config_without_groups():
    profile = yaml.safe_load(
        (ROOT.parents[3] / "apps" / "profiles" / "fr3_pika_single_arm.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert "groups" not in profile
    assert profile["execution_manager_config"] == {
        "package": "franka_manipulation_workstation_launch",
        "file": "config/execution_manager.yaml",
    }
    assert profile["homing"] == {
        "duration_s": 8.0,
        "joint_positions": {
            "arm": [
                0.0,
                -0.7853981633974483,
                0.0,
                -2.356194490192345,
                0.0,
                1.5707963267948966,
                0.7853981633974483,
            ],
            "end_effector": [0.0],
        },
    }
    assert profile["recorder"]["episodes"] == 5
    assert profile["recorder"]["rate_hz"] == 30.0
    assert profile["recorder"]["max_duration_s"] == 60.0


def test_examples_16_and_17_share_homing_preemption_recording_lifecycle():
    examples = ROOT.parents[3] / "examples"
    franka = (examples / "16_franka_gamepad_teleop.py").read_text(encoding="utf-8")
    marvin = (examples / "17_marvin_quest3_teleop.py").read_text(encoding="utf-8")
    for contract in (
        "SmoothHomingPlanner",
        'context.make_node("DummyPolicy", policy)',
        'context.make_node("Planner", homing_planner)',
        "context.wait_until_ready(",
        "planner_node.activate()",
        "policy_node.activate()",
        "recorder.wait_ready(",
        "pending_episode.__enter__()",
        "episode_scope.__exit__(None, None, None)",
        "policy_node.is_active",
    ):
        assert contract in franka
        assert contract in marvin
