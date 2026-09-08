"""Exercise upstream RealSense launch for arbitrary instances without hardware."""

import importlib.util
from pathlib import Path

import yaml
from launch import LaunchContext
from launch.actions import TimerAction
from launch_ros.actions import Node


def test_camera_entries_are_independent_and_extensible(monkeypatch, capsys, tmp_path):
    root = Path(__file__).parents[1]
    spec = importlib.util.spec_from_file_location(
        "piper_realsense_launch", root / "launch/piper_realsense.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    data = yaml.safe_load((root / "config/camera/realsense_d435.yaml").read_text())
    data["right_hand_realsense"]["parameters"]["rgb_camera.color_profile"] = "1280x720x15"
    data["extra_camera"] = {
        "startup_delay": 20.0,
        "sdk_log_level": "ERROR",
        "parameters": {
            "camera_namespace": "observation",
            "serial_no": "_789",
            "enable_depth": False,
        },
    }
    config = tmp_path / "cameras.yaml"
    config.write_text(yaml.safe_dump(data, sort_keys=False))
    context = LaunchContext()
    parent = {
        "with_leaders": "true",
        "use_sim_time": "false",
        "realsense_config": str(config),
    }
    context.launch_configurations.update(parent)
    cameras = []

    def capture_node(self, context):
        cameras.append(dict(context.launch_configurations))
        assert context.environment["LRS_LOG_LEVEL"] == "ERROR"
        return []

    monkeypatch.setattr(Node, "execute", capture_node)

    def visit(entity):
        for child in entity.visit(context) or []:
            visit(child)

    actions = module._setup(context)
    assert len(actions) == 3
    for index, action in enumerate(actions):
        if isinstance(action, TimerAction):
            assert float(action.period) == (10.0, 20.0)[index - 1]
            # Inspect delayed actions immediately without starting timers/processes.
            for child in action.actions:
                visit(child)
        else:
            visit(action)
        assert context.launch_configurations == parent

    assert [c["camera_name"] for c in cameras] == list(data)
    assert [c["serial_no"] for c in cameras] == ["_332522075913", "_332322073584", "_789"]
    assert cameras[0]["rgb_camera.color_profile"] == "640x480x30"
    assert cameras[1]["rgb_camera.color_profile"] == "1280x720x15"
    assert all(c["enable_depth"] == "false" for c in cameras)
    assert all(not c.keys() & parent.keys() for c in cameras)
    assert "not supported" not in capsys.readouterr().out
