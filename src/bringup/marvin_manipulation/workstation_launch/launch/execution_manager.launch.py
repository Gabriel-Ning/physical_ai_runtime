"""Launch Marvin Execution Manager from this package's config.

Workstation-owned file: ``config/execution_manager.yaml``.
"""

from __future__ import annotations

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _max_command_age_s(config_path: str) -> str:
    with Path(config_path).open(encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    value = data.get("max_command_age_s")
    if value is None:
        raise KeyError(f"{config_path}: max_command_age_s is required")
    return str(value)


def _launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration("config").perform(context).strip()
    em_share = get_package_share_directory("execution_manager")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(em_share, "launch", "execution_manager.launch.py")
            ),
            launch_arguments={
                "profile": config_path,
                "max_command_age_s": _max_command_age_s(config_path),
            }.items(),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    workstation_share = get_package_share_directory(
        "marvin_manipulation_workstation_launch"
    )
    default_config = os.path.join(
        workstation_share, "config", "execution_manager.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            OpaqueFunction(function=_launch_setup),
        ]
    )
