# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Marvin bimanual manipulation server for a robot/CPU host.

Starts robot_state_publisher, ros2_control, and three inactive controller
routes per arm. RMI execution services are composed by deployment launches.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro


def _continue_or_shutdown(event, next_actions, stage):
    if event.returncode == 0:
        return next_actions
    reason = f"{stage} failed with exit code {event.returncode}"
    return [LogInfo(msg=reason), EmitEvent(event=Shutdown(reason=reason))]


def _resolve_cpu_affinity(context) -> str:
    """Prefer launch arg; else RT_CM_CPU_AFFINITY from the cpu RT profile."""
    explicit = LaunchConfiguration("cpu_affinity").perform(context).strip()
    if explicit in ("none", "off", "-"):
        return ""
    if explicit:
        return explicit
    return os.environ.get("RT_CM_CPU_AFFINITY", "").strip()


def _controller_nodes(context: LaunchContext):
    bringup_share = get_package_share_directory(
        "marvin_manipulation_controller_bringup"
    )
    marvin_share = get_package_share_directory("marvin_description")
    controllers_yaml = LaunchConfiguration("controllers_yaml")
    cpu_affinity = _resolve_cpu_affinity(context)
    heartbeat_timeout_s = float(
        LaunchConfiguration("jtc_guard_heartbeat_timeout_s").perform(context)
    )
    cancel_response_timeout_s = float(
        LaunchConfiguration("jtc_guard_cancel_response_timeout_s").perform(context)
    )
    use_rviz = LaunchConfiguration("use_rviz").perform(context).strip().lower() in (
        "true",
        "1",
    )

    robot_description_xacro = os.path.join(
        bringup_share, "urdf", "marvin_manipulation.urdf.xacro"
    )
    robot_description = xacro.process_file(
        robot_description_xacro,
        mappings={
            "ros2_control": "true",
            "connected_to": context.perform_substitution(
                LaunchConfiguration("connected_to")
            ),
            "xyz": context.perform_substitution(LaunchConfiguration("xyz")),
            "rpy": context.perform_substitution(LaunchConfiguration("rpy")),
            "mounts_file": context.perform_substitution(
                LaunchConfiguration("mounts_file")
            ),
            "use_fake_hardware": context.perform_substitution(
                LaunchConfiguration("use_fake_hardware")
            ),
            "hardware_plugin": context.perform_substitution(
                LaunchConfiguration("hardware_plugin")
            ),
            "robot_ip": context.perform_substitution(LaunchConfiguration("robot_ip")),
            "stale_warn_ms": context.perform_substitution(
                LaunchConfiguration("stale_warn_ms")
            ),
            "stale_error_ms": context.perform_substitution(
                LaunchConfiguration("stale_error_ms")
            ),
            "max_joint_velocity": context.perform_substitution(
                LaunchConfiguration("max_joint_velocity")
            ),
            "left_gripper_serial_port": context.perform_substitution(
                LaunchConfiguration("left_gripper_serial_port")
            ),
            "right_gripper_serial_port": context.perform_substitution(
                LaunchConfiguration("right_gripper_serial_port")
            ),
        },
    ).toprettyxml(indent="  ")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[{"robot_description": robot_description}, controllers_yaml],
        prefix=f"taskset -c {cpu_affinity}" if cpu_affinity else None,
    )

    joint_state_broadcaster_spawner = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            (
                "ros2 run controller_manager spawner joint_state_broadcaster "
                "--controller-manager /controller_manager "
                "|| ros2 control list_controllers 2>/dev/null "
                "| grep -Eq 'joint_state_broadcaster\\s+active'"
            ),
        ],
        name="joint_state_broadcaster_spawner",
        output="screen",
    )

    route_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_tskpc",
            "left_arm_jspc",
            "left_arm_jtc",
            "right_arm_tskpc",
            "right_arm_jspc",
            "right_arm_jtc",
            "left_pika_gripper_fwd",
            "right_pika_gripper_fwd",
            "--inactive",
            "--controller-manager",
            "/controller_manager",
            "--controller-ros-args",
            (
                "--remap left_arm_jtc/follow_joint_trajectory:=/execution/left_arm/follow_joint_trajectory "
                "--remap right_arm_jtc/follow_joint_trajectory:=/execution/right_arm/follow_joint_trajectory "
                "--remap left_pika_gripper_fwd/commands:=/execution/left_gripper/joint_reference "
                "--remap right_pika_gripper_fwd/commands:=/execution/right_gripper/joint_reference"
            ),
        ],
        output="screen",
    )

    actions = []
    if cpu_affinity:
        actions.append(
            LogInfo(
                msg=(
                    f"Pinning ros2_control_node to CPUs {cpu_affinity} "
                    "(taskset; from cpu_affinity or RT_CM_CPU_AFFINITY)."
                )
            )
        )

    actions.extend(
        [
            robot_state_publisher,
            controller_manager,
            *[
                Node(
                    package="joint_trajectory_controller_guard",
                    executable="jtc_guard_node",
                    name=f"{side}_arm_jtc_guard",
                    parameters=[
                        {
                            "action_name": (
                                f"/execution/{side}_arm/follow_joint_trajectory"
                            ),
                            "heartbeat_topic": (
                                f"/execution/{side}_arm/trajectory_guard_heartbeat"
                            ),
                            "heartbeat_timeout_s": heartbeat_timeout_s,
                            "cancel_response_timeout_s": cancel_response_timeout_s,
                        }
                    ],
                    output="screen",
                )
                for side in ("left", "right")
            ],
            joint_state_broadcaster_spawner,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=lambda event, context: _continue_or_shutdown(
                        event,
                        [route_controller_spawner],
                        "joint_state_broadcaster startup",
                    ),
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=route_controller_spawner,
                    on_exit=lambda event, context: _continue_or_shutdown(
                        event,
                        [],
                        "route controller startup",
                    ),
                )
            ),
        ]
    )

    if use_rviz:
        rviz_config = os.path.join(marvin_share, "rviz", "visualize_marvin.rviz")
        rviz_args = ["-d", rviz_config] if os.path.exists(rviz_config) else []
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=rviz_args,
            output="screen",
        )
        actions.append(rviz_node)

    return actions


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory(
        "marvin_manipulation_controller_bringup"
    )
    marvin_share = get_package_share_directory("marvin_description")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_yaml",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "controller", "controllers.yaml"]
                ),
                description="controller_manager + three routes for both arms.",
            ),
            DeclareLaunchArgument(
                "connected_to",
                default_value="world",
                description="Parent frame for the Marvin base mount.",
            ),
            DeclareLaunchArgument(
                "xyz",
                default_value="0 0 0",
                description="Marvin base translation in connected_to [m].",
            ),
            DeclareLaunchArgument(
                "rpy",
                default_value="0 0 0",
                description="Marvin base rotation in connected_to [rad].",
            ),
            DeclareLaunchArgument(
                "mounts_file",
                default_value=PathJoinSubstitution(
                    [marvin_share, "config", "arm_mounts.yaml"]
                ),
                description="Calibrated left/right arm mount transforms.",
            ),
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="true",
                description=(
                    "true: mock_components/GenericSystem (default, safe). "
                    "false: real Marvin SDK bridge -- only with the robot "
                    "present, powered, and safed."
                ),
            ),
            DeclareLaunchArgument(
                "hardware_plugin",
                default_value="marvin_hardware_interface/MarvinBimanualArmHardware",
                description=(
                    "Real ros2_control hardware plugin "
                    "(used when use_fake_hardware:=false)."
                ),
            ),
            DeclareLaunchArgument(
                "left_gripper_serial_port", default_value="/dev/ttyUSB0"
            ),
            DeclareLaunchArgument(
                "right_gripper_serial_port", default_value="/dev/ttyUSB1"
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="10.19.0.191",
                description=(
                    "Marvin controller IP (used when use_fake_hardware:=false)."
                ),
            ),
            DeclareLaunchArgument(
                "stale_warn_ms",
                default_value="20.0",
                description="Warn after this many ms without a fresh Marvin frame.",
            ),
            DeclareLaunchArgument(
                "stale_error_ms",
                default_value="100.0",
                description="Return hardware ERROR after this stale duration [ms].",
            ),
            DeclareLaunchArgument(
                "max_joint_velocity",
                default_value="6.2832",
                description="Write-guard joint velocity limit [rad/s].",
            ),
            DeclareLaunchArgument(
                "cpu_affinity",
                default_value="",
                description=(
                    "Comma-separated CPUs for ros2_control_node taskset. "
                    "Empty uses RT_CM_CPU_AFFINITY from the cpu RT profile "
                    "(see docs/CPU_HOST_SETUP.md). Pass an explicit list to "
                    "override."
                ),
            ),
            DeclareLaunchArgument(
                "jtc_guard_heartbeat_timeout_s",
                default_value="0.5",
                description=(
                    "Cancel an armed JTC goal after this many seconds without "
                    "a workstation heartbeat."
                ),
            ),
            DeclareLaunchArgument(
                "jtc_guard_cancel_response_timeout_s",
                default_value="0.5",
                description="Maximum wait for a local JTC cancel response.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Whether to launch RViz2 for visualization and debugging.",
            ),
            OpaqueFunction(function=_controller_nodes),
        ]
    )
