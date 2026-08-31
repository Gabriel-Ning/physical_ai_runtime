"""Launch Piper Execution Manager from its routing config."""
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def _setup(context):
    config = LaunchConfiguration("config").perform(context).strip()
    data = yaml.safe_load(Path(config).read_text(encoding="utf-8")) or {}
    if "max_command_age_s" not in data:
        raise KeyError(f"{config}: max_command_age_s is required")
    share = get_package_share_directory("execution_manager")
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "execution_manager.launch.py")),
        launch_arguments={"profile": config, "max_command_age_s": str(data["max_command_age_s"])}.items())]

def generate_launch_description():
    share = get_package_share_directory("piper_manipulation_workstation_launch")
    default = os.path.join(share, "config", "execution_manager.yaml")
    return LaunchDescription([DeclareLaunchArgument("config", default_value=default), OpaqueFunction(function=_setup)])
