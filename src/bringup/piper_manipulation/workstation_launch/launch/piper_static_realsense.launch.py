"""Launch the static third RealSense D435i camera with RGB only."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_file = PathJoinSubstitution(
        [
            FindPackageShare("piper_manipulation_workstation_launch"),
            "config",
            "camera",
            "d435i_single.yaml",
        ]
    )
    return LaunchDescription(
        [
            SetEnvironmentVariable("LRS_LOG_LEVEL", "ERROR"),
            DeclareLaunchArgument("config_file", default_value=config_file),
            DeclareLaunchArgument("camera_name", default_value="static_d435i"),
            # Leading underscore preserves this numeric serial as a string for
            # the RealSense launch driver.
            DeclareLaunchArgument("serial_no", default_value="_310222078614"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("realsense2_camera"),
                            "launch",
                            "rs_launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "config_file": LaunchConfiguration("config_file"),
                    "camera_name": LaunchConfiguration("camera_name"),
                    "camera_namespace": "observation",
                    "serial_no": LaunchConfiguration("serial_no"),
                }.items(),
            ),
        ]
    )
