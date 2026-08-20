"""Launch the dual Piper RT-host stack: controllers and local safety guards."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("piper_manipulation_controller_bringup")
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "rt_launch", "controller_bringup.launch.py")
        ),
        launch_arguments={
            "arms": "both",
            "left_can_interface": LaunchConfiguration("left_can_interface"),
            "right_can_interface": LaunchConfiguration("right_can_interface"),
            "left_end_effector": "piper_gripper",
            "right_end_effector": "piper_gripper",
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "cpu_affinity": LaunchConfiguration("cpu_affinity"),
            "jtc_guard_heartbeat_timeout_s": LaunchConfiguration(
                "jtc_guard_heartbeat_timeout_s"
            ),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "left_can_interface",
                default_value="piper0",
                description="SocketCAN name for the left follower (default: piper0).",
            ),
            DeclareLaunchArgument(
                "right_can_interface",
                default_value="piper1",
                description="SocketCAN name for the right follower (default: piper1).",
            ),
            DeclareLaunchArgument("use_fake_hardware", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "cpu_affinity",
                default_value="",
                description=(
                    "Comma-separated CPUs for ros2_control_node. Empty uses "
                    "RT_CM_CPU_AFFINITY from the cpu RT profile. Pass none to disable."
                ),
            ),
            DeclareLaunchArgument("jtc_guard_heartbeat_timeout_s", default_value="0.5"),
            controller,
        ]
    )
