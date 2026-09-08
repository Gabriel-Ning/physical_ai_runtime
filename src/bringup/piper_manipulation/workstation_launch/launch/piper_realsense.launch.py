"""Launch every RealSense D435 instance listed in the camera configuration."""

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
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _camera_include(name, camera):
    parameters = dict(camera["parameters"])
    # The entry key is the node name; no second identity source is needed.
    if "camera_name" in parameters or "config_file" in parameters:
        raise ValueError(f"{name}: use the entry key as camera_name and inline parameters")
    parameters["camera_name"] = name
    arguments = {
        key: str(value).lower() if isinstance(value, bool) else str(value)
        for key, value in parameters.items()
    }
    # Keep parent/workstation settings out of the upstream parameter validator.
    return GroupAction(
        forwarding=False,
        launch_configurations=arguments,
        actions=[
            SetEnvironmentVariable("LRS_LOG_LEVEL", camera["sdk_log_level"]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
                    )
                )
            ),
        ],
    )


def _setup(context):
    path = Path(LaunchConfiguration("realsense_config").perform(context))
    cameras = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(cameras, dict) or not cameras:
        raise ValueError(f"{path}: expected a non-empty mapping of camera instances")
    actions = []
    for name, camera in cameras.items():
        delay = float(camera["startup_delay"])
        if delay < 0:
            raise ValueError(f"{name}: startup_delay must be non-negative")
        action = _camera_include(name, camera)
        actions.append(TimerAction(period=delay, actions=[action]) if delay else action)
    return actions


def generate_launch_description() -> LaunchDescription:
    workstation_share = get_package_share_directory(
        "piper_manipulation_workstation_launch"
    )
    default_config = os.path.join(
        workstation_share, "config", "camera", "realsense_d435.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("realsense_config", default_value=default_config),
            OpaqueFunction(function=_setup),
        ]
    )
