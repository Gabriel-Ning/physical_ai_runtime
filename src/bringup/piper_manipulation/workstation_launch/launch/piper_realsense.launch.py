"""Launch the two Piper wrist RealSense D435i cameras on the workstation.

Stream parameters come from
``piper_manipulation_workstation_launch/config/camera/d435i_dual.yaml``.
Site serials / names match the verified workstation launch defaults.
"""

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


def _camera_include(
    camera_name: str, serial_no: str, config_file: str
) -> GroupAction:
    # rs_launch.py declares many generic arguments.  Isolate the include so
    # this parent launch's left/right/site arguments are not forwarded to the
    # RealSense node as unsupported ROS parameters.
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
        TimerAction(
            period=float(LaunchConfiguration("left_camera_delay").perform(context)),
            actions=[
                _camera_include(
                    LaunchConfiguration("left_camera_name").perform(context),
                    LaunchConfiguration("left_serial_no").perform(context),
                    config_file,
                )
            ],
        ),
        TimerAction(
            period=float(LaunchConfiguration("right_camera_delay").perform(context)),
            actions=[
                _camera_include(
                    LaunchConfiguration("right_camera_name").perform(context),
                    LaunchConfiguration("right_serial_no").perform(context),
                    config_file,
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
            DeclareLaunchArgument("left_camera_delay", default_value="5.0"),
            DeclareLaunchArgument("right_camera_delay", default_value="15.0"),
            OpaqueFunction(function=_resolved_cameras),
        ]
    )
