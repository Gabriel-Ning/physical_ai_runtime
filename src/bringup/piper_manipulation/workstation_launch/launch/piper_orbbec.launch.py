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
    camera = GroupAction(
        actions=[
            PushRosNamespace("observation"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("orbbec_camera"),
                            "launch",
                            "femto_bolt.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "camera_name": LaunchConfiguration("camera_name"),
                    "serial_number": LaunchConfiguration("serial_number"),
                    "enable_color": "true",
                    "color_width": "1280",
                    "color_height": "720",
                    "color_fps": "30",
                    "color_format": "MJPG",
                    "enable_depth": "false",
                    "enable_ir": "false",
                    "enable_accel": "false",
                    "enable_gyro": "false",
                    "enable_sync_output_accel_gyro": "false",
                    "enable_point_cloud": "false",
                    "enable_colored_point_cloud": "false",
                    "enable_frame_sync": "false",
                    "publish_tf": "false",
                    "use_hardware_time": "false",
                }.items(),
            ),
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_name", default_value="orbbec"),
            DeclareLaunchArgument("serial_number", default_value=""),
            camera,
        ]
    )
