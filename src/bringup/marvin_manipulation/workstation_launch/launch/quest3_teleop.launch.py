"""Launch Quest 3 teleop from the complete Marvin-owned configuration."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_teleop(context: LaunchContext, *args, **kwargs):
    workstation_share = get_package_share_directory(
        "marvin_manipulation_workstation_launch"
    )
    teleop_mode = context.perform_substitution(LaunchConfiguration("teleop_mode")).strip()
    quest3_config = context.perform_substitution(LaunchConfiguration("quest3_config")).strip()

    if teleop_mode not in {"absolute", "relative"}:
        raise ValueError("teleop_mode must be either 'absolute' or 'relative'")

    if not quest3_config:
        quest3_config = os.path.join(
            workstation_share,
            "config",
            "teleop",
            f"quest3_bimanual_{teleop_mode}.yaml",
        )

    executable = (
        "quest3_bimanual_absolute_target"
        if teleop_mode == "absolute"
        else "quest3_bimanual_target"
    )

    # The Marvin YAML is the sole source of teleop behavior. Only values that are
    # inherently host/session-specific are supplied by the launch file.
    return [
        Node(
            package="isaacteleop_toolbox",
            executable=executable,
            name=executable,
            output="screen",
            parameters=[
                quest3_config,
                {
                    "cloudxr_install_dir": LaunchConfiguration("cloudxr_install_dir"),
                    "cloudxr_accept_eula": True,
                    "cloudxr_env_config": LaunchConfiguration("cloudxr_env_config"),
                    "cloudxr_host_client": True,
                    "mcap_replay_path": LaunchConfiguration("mcap_replay_path"),
                },
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    default_cloudxr_dir = EnvironmentVariable("CLOUDXR_DIR")
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("isaacteleop_toolbox"),
            "rviz",
            "isaacteleop_controller_replay.rviz",
        ]
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["--display-config", rviz_config],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        name="rviz2",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "teleop_mode",
                default_value="absolute",
                description="Teleop mode: 'absolute' (default) or 'relative'.",
            ),
            DeclareLaunchArgument(
                "quest3_config",
                default_value="",
                description="Optional complete Quest 3 ROS parameter file path.",
            ),
            DeclareLaunchArgument(
                "cloudxr_install_dir",
                default_value=default_cloudxr_dir,
                description="Prepared CloudXR data directory.",
            ),
            DeclareLaunchArgument(
                "cloudxr_env_config",
                default_value=[default_cloudxr_dir, "/cloudxr-env-config.env"],
                description="CloudXR environment configuration file.",
            ),
            DeclareLaunchArgument(
                "mcap_replay_path",
                default_value="",
                description="Optional DeviceIO MCAP replay path; empty uses live Quest 3.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value=EnvironmentVariable(
                    "TELEOP_WITH_RVIZ", default_value="true"
                ),
            ),
            DeclareLaunchArgument("rviz_delay_s", default_value="4.0"),
            SetEnvironmentVariable(
                "ROS_LOCALHOST_ONLY",
                EnvironmentVariable("ROS_LOCALHOST_ONLY", default_value="1"),
            ),
            SetEnvironmentVariable(
                "TELEOP_WEB_CLIENT_STATIC_DIR",
                [LaunchConfiguration("cloudxr_install_dir"), "/static-client"],
            ),
            OpaqueFunction(function=_launch_teleop),
            TimerAction(period=LaunchConfiguration("rviz_delay_s"), actions=[rviz]),
        ]
    )
