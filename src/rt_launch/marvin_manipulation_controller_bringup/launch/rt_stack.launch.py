"""Launch the Marvin + Pika RT-host stack: controllers and local safety guards."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory(
        "marvin_manipulation_controller_bringup"
    )
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "controller_bringup.launch.py")
        ),
        launch_arguments={
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "cpu_affinity": LaunchConfiguration("cpu_affinity"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "jtc_guard_heartbeat_timeout_s": LaunchConfiguration(
                "jtc_guard_heartbeat_timeout_s"
            ),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_fake_hardware", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("cpu_affinity", default_value="none"),
            DeclareLaunchArgument("robot_ip", default_value="10.19.0.191"),
            DeclareLaunchArgument("jtc_guard_heartbeat_timeout_s", default_value="0.5"),
            controller,
        ]
    )
