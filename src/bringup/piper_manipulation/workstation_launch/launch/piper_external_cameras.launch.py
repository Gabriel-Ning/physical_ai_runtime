"""Launch the two external D435i cameras on the NUC (192.168.1.101)."""

from __future__ import annotations

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


def _camera(camera_name: str, serial_no: str, config_file: str) -> GroupAction:
    return GroupAction(
        forwarding=False,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
                    )
                ),
                launch_arguments={
                    "config_file": config_file,
                    "camera_name": camera_name,
                    "camera_namespace": "observation",
                    "serial_no": serial_no,
                }.items(),
            )
        ],
    )


def _resolved_cameras(context, *args, **kwargs):
    del args, kwargs
    config_file = LaunchConfiguration("realsense_config_file").perform(context)
    return [
        _camera(
            "d435i1",
            LaunchConfiguration("d435i1_serial_no").perform(context),
            config_file,
        ),
        TimerAction(
            period=float(LaunchConfiguration("d435i2_delay").perform(context)),
            actions=[
                _camera(
                    "d435i2",
                    LaunchConfiguration("d435i2_serial_no").perform(context),
                    config_file,
                )
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution(
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
            DeclareLaunchArgument("realsense_config_file", default_value=config),
            DeclareLaunchArgument("d435i1_serial_no", default_value="_405622076349"),
            DeclareLaunchArgument("d435i2_serial_no", default_value="_310222078614"),
            DeclareLaunchArgument("d435i2_delay", default_value="10.0"),
            OpaqueFunction(function=_resolved_cameras),
        ]
    )
