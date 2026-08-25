"""Launch the two Piper wrist RealSense D435i cameras on the workstation.

Stream parameters come from
``piper_manipulation_workstation_launch/config/camera/d435i_dual.yaml``.
Site serials / names match the verified workstation launch defaults.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _camera_include(
    camera_name_arg: str, serial_no_arg: str, config_file=None, resolved=False
) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        launch_arguments={
            "config_file": config_file or LaunchConfiguration("realsense_config_file"),
            "camera_name": (
                camera_name_arg
                if resolved
                else LaunchConfiguration(camera_name_arg)
            ),
            "camera_namespace": "observation",
            "serial_no": serial_no_arg if resolved else LaunchConfiguration(serial_no_arg),
        }.items(),
    )


def _delayed_right_camera(context, *args, **kwargs):
    del args, kwargs
    return [
        TimerAction(
            period=float(LaunchConfiguration("right_camera_delay").perform(context)),
            actions=[
                _camera_include(
                    LaunchConfiguration("right_camera_name").perform(context),
                    LaunchConfiguration("right_serial_no").perform(context),
                    LaunchConfiguration("realsense_config_file").perform(context),
                    resolved=True,
                )
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    config_file = PathJoinSubstitution(
        [
            FindPackageShare("piper_manipulation_workstation_launch"),
            "config",
            "camera",
            "d435i_dual.yaml",
        ]
    )
    return LaunchDescription(
        [
            SetEnvironmentVariable("LRS_LOG_LEVEL", "ERROR"),
            DeclareLaunchArgument("realsense_config_file", default_value=config_file),
            DeclareLaunchArgument(
                "left_camera_name", default_value="left_hand_realsense"
            ),
            DeclareLaunchArgument(
                "right_camera_name", default_value="right_hand_realsense"
            ),
            DeclareLaunchArgument("left_serial_no", default_value="_332522075913"),
            DeclareLaunchArgument("right_serial_no", default_value="_332322073584"),
            DeclareLaunchArgument("right_camera_delay", default_value="10.0"),
            _camera_include("left_camera_name", "left_serial_no"),
            OpaqueFunction(function=_delayed_right_camera),
        ]
    )
