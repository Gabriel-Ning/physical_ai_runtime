# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Physical AI Runtime controller bringup for one Franka FR3 arm.

Owns FR3 + Pika ros2_control composition:
  robot_state_publisher, ros2_control_node, joint_state_publisher,
  serialized JSB -> inactive route controllers, optional RViz, CPU pin.

Does not modify vendor ``franka_bringup/franka.launch.py``.
"""

from __future__ import annotations

import os

import xacro
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_cpu_affinity(context) -> str:
    """Prefer launch arg; else RT_CM_CPU_AFFINITY from the cpu RT profile."""
    explicit = LaunchConfiguration("cpu_affinity").perform(context).strip()
    if explicit in ("none", "off", "-"):
        return ""
    if explicit:
        return explicit
    return os.environ.get("RT_CM_CPU_AFFINITY", "").strip()


def _launch_setup(context, *args, **kwargs):
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    controllers_yaml = LaunchConfiguration("controllers_yaml").perform(context)
    gripper_serial_port = LaunchConfiguration("gripper_serial_port").perform(context)
    load_pika = LaunchConfiguration("load_pika_hardware").perform(
        context
    ).strip().lower() in ("true", "1")
    use_rviz = LaunchConfiguration("use_rviz").perform(context).strip().lower() in (
        "true",
        "1",
    )
    joint_state_rate = int(
        LaunchConfiguration("joint_state_rate").perform(context).strip() or "100"
    )
    jtc_guard_heartbeat_timeout_s = float(
        LaunchConfiguration("jtc_guard_heartbeat_timeout_s").perform(context)
    )
    jtc_guard_cancel_response_timeout_s = float(
        LaunchConfiguration("jtc_guard_cancel_response_timeout_s").perform(context)
    )
    cpu_affinity = _resolve_cpu_affinity(context)

    bringup_share = FindPackageShare("franka_manipulation_rt_launch")
    default_real_yaml = PathJoinSubstitution(
        [bringup_share, "config", "controller", "controllers.yaml"]
    ).perform(context)
    default_fake_yaml = PathJoinSubstitution(
        [bringup_share, "config", "controller", "controllers_fake.yaml"]
    ).perform(context)
    if (
        use_fake_hardware.lower() in ("true", "1")
        and controllers_yaml == default_real_yaml
    ):
        controllers_yaml = default_fake_yaml

    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare("franka_manipulation_rt_launch"),
            "urdf",
            "fr3_manipulation.urdf.xacro",
        ]
    ).perform(context)
    robot_description = xacro.process_file(
        urdf_path,
        mappings={
            "robot_type": "fr3",
            "arm_prefix": "",
            "robot_ip": robot_ip,
            "hand": "false",
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": "false",
            "gripper_serial_port": gripper_serial_port,
            "load_pika_hardware": "true" if load_pika else "false",
        },
    ).toprettyxml(indent="  ")

    route_controllers = [
        "franka_arm_tsjic",
        "franka_arm_jsic",
        "franka_arm_jtc",
    ]
    if load_pika:
        route_controllers.append("pika_gripper_fwd")

    actions = []
    if cpu_affinity:
        actions.append(
            LogInfo(
                msg=(
                    f"Pinning ros2_control_node to CPUs {cpu_affinity} "
                    "(taskset prefix; from cpu_affinity or RT_CM_CPU_AFFINITY)."
                )
            )
        )

    # JSB remapped off /joint_states; joint_state_publisher merges back.
    joint_state_sources = [
        "franka/joint_states",
        "franka_gripper/joint_states",
    ]

    cm_kwargs = {
        "package": "controller_manager",
        "executable": "ros2_control_node",
        "parameters": [
            {"robot_description": robot_description},
            controllers_yaml,
            {"robot_type": "fr3"},
            {"load_gripper": False},
            {"arm_prefix": ""},
        ],
        "output": "screen",
        "on_exit": Shutdown(),
    }
    if cpu_affinity:
        cm_kwargs["prefix"] = f"taskset -c {cpu_affinity}"

    actions.extend(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(**cm_kwargs),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                parameters=[
                    {
                        "source_list": joint_state_sources,
                        "rate": joint_state_rate,
                    }
                ],
                output="screen",
            ),
            Node(
                package="joint_trajectory_controller_guard",
                executable="jtc_guard_node",
                name="franka_arm_jtc_guard",
                parameters=[
                    {
                        "action_name": "/execution/arm/follow_joint_trajectory",
                        "heartbeat_topic": (
                            "/execution/arm/trajectory_guard_heartbeat"
                        ),
                        "heartbeat_timeout_s": jtc_guard_heartbeat_timeout_s,
                        "cancel_response_timeout_s": (
                            jtc_guard_cancel_response_timeout_s
                        ),
                    }
                ],
                output="screen",
            ),
        ]
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
            "--controller-ros-args",
            "--remap joint_states:=franka/joint_states",
        ],
        output="screen",
    )
    route_remaps = [
        "--remap franka_arm_jtc/follow_joint_trajectory:=/execution/arm/follow_joint_trajectory",
    ]
    if load_pika:
        route_remaps.append(
            "--remap pika_gripper_fwd/commands:=/execution/end_effector/joint_reference"
        )
    route_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            *route_controllers,
            "--inactive",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
            "--controller-ros-args",
            " ".join(route_remaps),
        ],
        output="screen",
    )
    actions.append(joint_state_spawner)
    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_spawner,
                on_exit=[route_controller_spawner],
            )
        )
    )

    if use_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "--display-config",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("franka_description"),
                            "rviz",
                            "visualize_franka.rviz",
                        ]
                    ),
                ],
                output="screen",
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    """Compose the FR3 + Pika control path without vendor franka.launch.py."""
    bringup_share = FindPackageShare("franka_manipulation_rt_launch")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_yaml",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "controller", "controllers.yaml"]
                ),
                description=(
                    "controller_manager YAML. Fake hardware defaults to "
                    "controllers_fake.yaml."
                ),
            ),
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="true",
                description=(
                    "Use mock_components/GenericSystem. Set false only for a "
                    "present, powered, and safed FR3."
                ),
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="192.168.2.101",
                description="FR3 hostname or IP; ignored by fake hardware.",
            ),
            DeclareLaunchArgument(
                "gripper_serial_port",
                default_value="/dev/ttyUSB0",
                description="Serial device for the attached Pika gripper.",
            ),
            DeclareLaunchArgument(
                "load_pika_hardware",
                default_value="true",
                description=(
                    "Load Pika ros2_control + pika_gripper_fwd. Set false when "
                    "the gripper is not installed; URDF/TCP stay for planning."
                ),
            ),
            DeclareLaunchArgument(
                "joint_state_rate",
                default_value="100",
                description="joint_state_publisher rate (Hz).",
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
                description="Maximum wait for the local JTC cancel response.",
            ),
            DeclareLaunchArgument(
                "cpu_affinity",
                default_value="",
                description=(
                    "Comma-separated CPUs for ros2_control_node taskset. "
                    "Empty uses RT_CM_CPU_AFFINITY from the cpu RT profile "
                    "(see docs/CPU_HOST_SETUP.md). Pass none to disable."
                ),
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Whether to launch RViz2 for visualization and debugging.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
