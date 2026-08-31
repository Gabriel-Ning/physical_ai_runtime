from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
import yaml
from launch.actions import DeclareLaunchArgument

PKG_DIR = Path(__file__).parents[1]
LAUNCH_PATH = PKG_DIR / "launch" / "gamepad_teleop.launch.py"
CONFIG_DIR = PKG_DIR / "config"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("gamepad_teleop_launch", LAUNCH_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_optional_bool_parsing():
    module = _load_launch_module()
    assert module._optional_bool("autostart", "") is None
    assert module._optional_bool("autostart", "true") is True
    assert module._optional_bool("autostart", "TRUE") is True
    assert module._optional_bool("autostart", "false") is False
    assert module._optional_bool("autostart", "FALSE") is False
    with pytest.raises(ValueError, match="autostart"):
        module._optional_bool("autostart", "invalid")


def test_launch_declares_arguments(monkeypatch, tmp_path):
    module = _load_launch_module()
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(PKG_DIR))

    description = module.generate_launch_description()
    arguments = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }

    expected_args = [
        "config",
        "joy_driver",
        "device_id",
        "joy_dev",
        "deadzone",
        "autorepeat_rate",
        "publish_rate_hz",
        "frame_id",
        "gripper_joint_name",
        "twist_topic",
        "gripper_topic",
        "clutch_topic",
        "status_topic",
        "joy_topic",
        "autostart",
    ]
    for arg in expected_args:
        assert arg in arguments, f"Missing launch argument: {arg}"


@pytest.mark.parametrize(
    "config_name", ["ps5.yaml", "xbox.yaml", "game_controller.yaml"]
)
def test_config_yaml_validity(config_name: str):
    config_file = CONFIG_DIR / config_name
    assert config_file.exists(), f"Missing config file: {config_file}"
    data = yaml.safe_load(config_file.read_text(encoding="utf-8"))
    assert "gamepad_teleop" in data
    params = data["gamepad_teleop"]["ros__parameters"]

    assert "clutch_button" in params
    assert "axis_linear" in params
    assert "scale_linear" in params
    assert "axis_angular" in params
    assert "scale_angular" in params
    assert "gripper_open_button" in params
    assert "gripper_close_button" in params
    assert "gripper_speed_m_per_s" in params
    assert "twist_topic" in params
    assert "gripper_topic" in params
    assert "clutch_topic" in params
    assert "status_topic" in params
