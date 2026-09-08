import importlib.util
from pathlib import Path

import pytest
from launch.actions import DeclareLaunchArgument

LAUNCH_PATH = Path(__file__).parents[1] / "launch" / "piper_leader.launch.py"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("piper_leader_launch", LAUNCH_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_optional_autostart_override_parsing():
    module = _load_launch_module()
    assert module._optional_bool("autostart", "") is None
    assert module._optional_bool("autostart", "true") is True
    assert module._optional_bool("autostart", "FALSE") is False
    with pytest.raises(ValueError, match="autostart"):
        module._optional_bool("autostart", "enabled")


def test_launch_declares_optional_autostart_argument(monkeypatch, tmp_path):
    module = _load_launch_module()
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    monkeypatch.setattr(
        module, "get_package_share_directory", lambda _: str(LAUNCH_PATH.parents[1])
    )
    description = module.generate_launch_description()
    arguments = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert "autostart" in arguments
    assert "use_sim_time" in arguments
    default_value = arguments["autostart"].default_value
    assert len(default_value) == 1
    assert default_value[0].perform(None) == ""
    sim_default = arguments["use_sim_time"].default_value
    assert len(sim_default) == 1
    assert sim_default[0].perform(None) == "false"


def test_sim_clock_stamps_only_when_use_sim_time_true():
    launch = LAUNCH_PATH.read_text(encoding="utf-8")
    cpp = (
        Path(__file__).parents[1] / "src" / "piper_leader_node.cpp"
    ).read_text(encoding="utf-8")
    assert 'node_params["use_sim_time"]' not in launch
    assert 'if use_sim_time:' in launch
    assert 'node_params["stamp_from_sim_clock"] = True' in launch
    assert "if (stamp_from_sim_clock_)" in cpp
    assert 'create_subscription<rosgraph_msgs::msg::Clock>' in cpp
    assert "if (!stamp_from_sim_clock_)" in cpp
