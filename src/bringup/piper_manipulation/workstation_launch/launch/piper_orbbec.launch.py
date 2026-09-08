"""Launch every Femto Bolt instance listed in the camera configuration.

Driver and identity parameters come from
``piper_manipulation_workstation_launch/config/camera/femto_bolt.yaml``.
"""

from __future__ import annotations

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def _stringify(value):
    return str(value).lower() if isinstance(value, bool) else str(value)


def _camera_include(name, camera):
    arguments = {
        "camera_name": name,
        "serial_number": _stringify(camera["serial_number"]),
    }
    arguments.update(
        {key: _stringify(value) for key, value in camera["parameters"].items()}
    )
    return GroupAction(
        actions=[
            PushRosNamespace(camera["camera_namespace"]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("orbbec_camera"), "launch", "femto_bolt.launch.py"]
                    )
                ),
                launch_arguments=arguments.items(),
            ),
        ]
    )


def _setup(context):
    path = Path(LaunchConfiguration("orbbec_config").perform(context))
    cameras = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(cameras, dict) or not cameras:
        raise ValueError(f"{path}: expected a non-empty mapping of camera instances")
    return [_camera_include(name, camera) for name, camera in cameras.items()]


def generate_launch_description() -> LaunchDescription:
    workstation_share = get_package_share_directory(
        "piper_manipulation_workstation_launch"
    )
    default_config = os.path.join(
        workstation_share, "config", "camera", "femto_bolt.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("orbbec_config", default_value=default_config),
            OpaqueFunction(function=_setup),
        ]
    )
