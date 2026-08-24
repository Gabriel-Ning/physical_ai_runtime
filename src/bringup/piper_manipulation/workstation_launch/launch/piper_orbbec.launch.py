"""Launch the static Piper-cell Orbbec Femto Bolt camera.

Driver parameters come from
``piper_manipulation_workstation_launch/config/camera/femto_bolt.yaml``.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_file = PathJoinSubstitution(
        [
            FindPackageShare("piper_manipulation_workstation_launch"),
            "config",
            "camera",
            "femto_bolt.yaml",
        ]
    )
    camera = GroupAction(
        actions=[
            PushRosNamespace("observation"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("orbbec_camera"), "launch", "femto_bolt.launch.py"]
                    )
                ),
                launch_arguments={
                    "camera_name": LaunchConfiguration("camera_name"),
                    "serial_number": LaunchConfiguration("serial_number"),
                    "config_file": LaunchConfiguration("orbbec_config_file"),
                }.items(),
            ),
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("orbbec_config_file", default_value=config_file),
            DeclareLaunchArgument("camera_name", default_value="static_orbbec"),
            DeclareLaunchArgument("serial_number", default_value=""),
            camera,
        ]
    )
