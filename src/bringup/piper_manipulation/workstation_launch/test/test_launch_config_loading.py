"""Check each child launch reads its own config without starting hardware."""

import importlib.util
import shutil
from pathlib import Path

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

ROOT = Path(__file__).parents[1]


def load_launch(name, share, monkeypatch):
    spec = importlib.util.spec_from_file_location(name, ROOT / "launch" / f"{name}.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda package: (
            str(share)
            if package == "piper_manipulation_workstation_launch"
            else str(share / package)
        ),
    )
    return module


def describe(module):
    context = LaunchContext()
    actions = module.generate_launch_description().entities
    for action in actions:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context, actions


def included_args(action, context):
    return {
        key: perform_substitutions(context, normalize_to_list_of_substitutions(value))
        for key, value in action.launch_arguments
    }


def test_stack_defaults_are_declared_in_launch(tmp_path, monkeypatch):
    shutil.copytree(ROOT / "config", tmp_path / "config")
    module = load_launch("workstation_stack", tmp_path, monkeypatch)
    context, _ = describe(module)
    assert context.launch_configurations["use_sim_time"] == "false"
    assert context.launch_configurations["with_execution_manager"] == "true"
    assert context.launch_configurations["with_recorder"] == "true"
    assert context.launch_configurations["with_orbbec"] == "true"
    assert context.launch_configurations["with_realsense"] == "true"
    assert context.launch_configurations["with_leaders"] == "true"


def test_execution_manager_reads_package_config(tmp_path, monkeypatch):
    shutil.copytree(ROOT / "config", tmp_path / "config")
    module = load_launch("execution_manager", tmp_path, monkeypatch)
    context, actions = describe(module)
    assert context.launch_configurations["config"] == str(
        tmp_path / "config/execution_manager.yaml"
    )
    opaque = next(action for action in actions if isinstance(action, OpaqueFunction))
    include = opaque.execute(context)[0]
    args = included_args(include, context)
    assert args["profile"] == str(tmp_path / "config/execution_manager.yaml")
    assert args["use_sim_time"] == "false"


def test_recorder_includes_episode_recorder_without_local_yaml(tmp_path, monkeypatch):
    shutil.copytree(ROOT / "config", tmp_path / "config")
    module = load_launch("recorder", tmp_path, monkeypatch)
    context, actions = describe(module)
    include = next(action for action in actions if isinstance(action, IncludeLaunchDescription))
    assert included_args(include, context) == {"use_sim_time": "false"}


def test_orbbec_instances_come_from_config(tmp_path, monkeypatch):
    shutil.copytree(ROOT / "config", tmp_path / "config")
    path = tmp_path / "config/camera/femto_bolt.yaml"
    data = yaml.safe_load(path.read_text())
    data["test_orbbec"] = data.pop("static_orbbec")
    data["test_orbbec"]["serial_number"] = "abc"
    path.write_text(yaml.safe_dump(data))
    module = load_launch("piper_orbbec", tmp_path, monkeypatch)
    context, _ = describe(module)
    group = module._setup(context)[0]
    include = next(
        entity
        for entity in group.get_sub_entities()
        if isinstance(entity, IncludeLaunchDescription)
    )
    args = included_args(include, context)
    assert args["camera_name"] == "test_orbbec"
    assert args["serial_number"] == "abc"
    assert args["color_width"] == "1280"


def test_orbbec_ignores_parent_em_config_argument(tmp_path, monkeypatch):
    shutil.copytree(ROOT / "config", tmp_path / "config")
    em_config = str(tmp_path / "config/execution_manager.yaml")
    module = load_launch("piper_orbbec", tmp_path, monkeypatch)
    context = LaunchContext()
    context.launch_configurations["config"] = em_config
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations["config"] == em_config
    assert context.launch_configurations["orbbec_config"] == str(
        tmp_path / "config/camera/femto_bolt.yaml"
    )
    group = module._setup(context)[0]
    include = next(
        entity
        for entity in group.get_sub_entities()
        if isinstance(entity, IncludeLaunchDescription)
    )
    args = included_args(include, context)
    assert args["camera_name"] == "static_orbbec"
    assert args["serial_number"] == ""


def test_realsense_reads_model_config(tmp_path, monkeypatch):
    shutil.copytree(ROOT / "config", tmp_path / "config")
    module = load_launch("piper_realsense", tmp_path, monkeypatch)
    context, _ = describe(module)
    assert context.launch_configurations["realsense_config"] == str(
        tmp_path / "config/camera/realsense_d435.yaml"
    )


def test_leader_node_names_come_from_config(tmp_path, monkeypatch):
    shutil.copytree(ROOT / "config", tmp_path / "config")
    path = tmp_path / "config/teleop/piper_leaders.yaml"
    data = yaml.safe_load(path.read_text())
    data["custom_left"] = data.pop("piper_leader_left")
    path.write_text(yaml.safe_dump(data))
    module = load_launch("piper_leaders", tmp_path, monkeypatch)
    context, actions = describe(module)
    opaque = next(action for action in actions if isinstance(action, OpaqueFunction))
    names = [included_args(action, context)["node_name"] for action in opaque.execute(context)]
    assert set(names) == {"custom_left", "piper_leader_right"}
