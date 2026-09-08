# Copyright 2026 physical_ai_runtime
# SPDX-License-Identifier: Apache-2.0
"""MuJoCo simulation bringup for Franka FR3 + Pika.

Owns the sim control path:
  task → MJCF resolution, MujocoSystemInterface URDF, ros2_control_node,
  inactive route controllers, optional mujoco_image_bridge (SHM → Image).

Real / fake hardware: ``controller_bringup.launch.py``.
"""

from __future__ import annotations

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_DEFAULT_LIBERO_TASK = (
    "pick_up_the_black_bowl_between_the_plate_and_the_ramekin_and_place_it_on_the_plate"
)


def _resolve_mujoco_model(task: str) -> str:
    """Map a task launch argument to an absolute MJCF path.

    ``task`` may be an absolute path to an existing ``.xml``, or a LIBERO task
    name looked up under ``libero_tasks`` share. RoboTwin names are rejected.
    """
    task_name = (task or "").strip() or _DEFAULT_LIBERO_TASK

    if os.path.isabs(task_name) and os.path.exists(task_name):
        return task_name

    candidates: list[str] = []
    libero_share = ""
    try:
        libero_share = get_package_share_directory("libero_tasks")
        candidates.extend(
            [
                os.path.join(libero_share, "mjcf", f"{task_name}.xml"),
                os.path.join(libero_share, "mjcf", "tasks", f"{task_name}.xml"),
            ]
        )
    except Exception:
        pass

    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate

    try:
        rt_share = get_package_share_directory("robotwin_tasks")
        rt_candidate = os.path.join(rt_share, "mjcf", f"{task_name}.xml")
        if os.path.exists(rt_candidate):
            raise RuntimeError(
                f"Task '{task_name}' belongs to the RoboTwin benchmark "
                f"(Piper dual-arm). Current runtime stack is 'rt-franka' "
                f"(Franka FR3). Please run "
                f"'pixi run rt-piper backend:=mujoco task:={task_name}' "
                f"for Piper, or select a LIBERO task for Franka."
            )
    except RuntimeError:
        raise
    except Exception:
        pass

    if libero_share:
        return os.path.join(libero_share, "mjcf", f"{_DEFAULT_LIBERO_TASK}.xml")
    return task_name


def _camera_bridge_parameters(config_path: str) -> dict | None:
    """Use exactly the explicitly enabled cameras from the plugin's YAML."""
    with open(config_path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    camera = config["/**"]["ros__parameters"]["mujoco_plugins"]["mujoco_camera_plugin"]
    output = camera.get("output", "ros")
    if output == "ros":
        return None
    if output not in ("shm", "both"):
        raise ValueError("Camera output must be ros, shm, or both")
    if camera.get("default_policy") != "disabled":
        raise ValueError(
            "SHM bringup requires default_policy: disabled so all active cameras are bridged"
        )
    names = [
        name
        for name, entry in camera.items()
        if isinstance(entry, dict) and entry.get("policy") in ("streaming", "polled")
    ]
    return {
        "camera_names": names,
        "shm_prefix": camera.get("shm_prefix", "/pai_mj_cam_"),
        "poll_hz": 120.0,
        "use_sim_time": False,
    }


def _camera_bridge_environment() -> dict[str, str]:
    """Scope image data unicast to the bridge; keep inherited NIC/peer settings.

    Cyclone merges comma-separated configuration sources in order. Large-image
    multicast dropped frames with two local readers on the deployment network.
    Other RMW implementations ignore this Cyclone-specific environment variable.
    """
    inherited = os.environ.get("CYCLONEDDS_URI", "").strip()
    override = (
        "<CycloneDDS><Domain><General><AllowMulticast>spdp</AllowMulticast>"
        "</General></Domain></CycloneDDS>"
    )
    return {
        "CYCLONEDDS_URI": os.environ.get(
            "MUJOCO_IMAGE_BRIDGE_CYCLONEDDS_URI",
            ",".join(part for part in (inherited, override) if part),
        )
    }


def _launch_setup(context, *args, **kwargs):
    bringup_share = get_package_share_directory("franka_manipulation_rt_launch")

    mujoco_model_path = LaunchConfiguration("mujoco_model").perform(context).strip()
    if not mujoco_model_path:
        mujoco_model_path = _resolve_mujoco_model(
            LaunchConfiguration("task").perform(context)
        )

    headless = LaunchConfiguration("headless").perform(context).lower().strip() in (
        "true",
        "1",
        "yes",
    )
    load_pika = LaunchConfiguration("load_pika_hardware").perform(
        context
    ).strip().lower() in ("true", "1")
    joint_state_rate = int(
        LaunchConfiguration("joint_state_rate").perform(context).strip() or "100"
    )
    jtc_guard_heartbeat_timeout_s = float(
        LaunchConfiguration("jtc_guard_heartbeat_timeout_s").perform(context)
    )
    jtc_guard_cancel_response_timeout_s = float(
        LaunchConfiguration("jtc_guard_cancel_response_timeout_s").perform(context)
    )
    controllers_yaml = LaunchConfiguration("controllers_yaml").perform(context)
    mujoco_plugins_yaml = LaunchConfiguration("mujoco_plugins_yaml").perform(context)
    bridge_parameters = _camera_bridge_parameters(mujoco_plugins_yaml)

    robot_description = xacro.process_file(
        os.path.join(bringup_share, "urdf", "fr3_manipulation.urdf.xacro"),
        mappings={
            "robot_type": "fr3",
            "arm_prefix": "",
            "robot_ip": "",
            "hand": "false",
            "use_fake_hardware": "false",
            "use_sim_mujoco": "true",
            "mujoco_model": mujoco_model_path,
            "headless": "true" if headless else "false",
            "fake_sensor_commands": "false",
            "gripper_serial_port": LaunchConfiguration("gripper_serial_port").perform(
                context
            ),
            "load_pika_hardware": "true" if load_pika else "false",
        },
    ).toprettyxml(indent="  ")

    route_controllers = [
        "franka_arm_tskpc",
        "franka_arm_jspc",
        "franka_arm_jtc",
    ]
    if load_pika:
        route_controllers.extend(["pika_gripper_fwd", "pika_gripper_action"])

    joint_state_sources = [
        "franka/joint_states",
        "franka_gripper/joint_states",
    ]

    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        ),
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            name="controller_manager",
            parameters=[
                {"robot_description": robot_description},
                controllers_yaml,
                {"robot_type": "fr3"},
                {"load_gripper": False},
                {"arm_prefix": ""},
                {"use_sim_time": True},
                {"headless": headless},
                mujoco_plugins_yaml,
            ],
            output="screen",
            on_exit=Shutdown(),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[
                {
                    "source_list": joint_state_sources,
                    "rate": joint_state_rate,
                    "use_sim_time": True,
                }
            ],
            output="screen",
        ),
        Node(
            package="joint_trajectory_controller_guard",
            executable="jtc_guard_node",
            name="franka_arm_jtc_guard",
            output="screen",
            parameters=[
                {
                    "action_name": "/execution/arm/follow_joint_trajectory",
                    "heartbeat_topic": "/execution/arm/trajectory_guard_heartbeat",
                    "heartbeat_timeout_s": jtc_guard_heartbeat_timeout_s,
                    "cancel_response_timeout_s": jtc_guard_cancel_response_timeout_s,
                    "use_sim_time": True,
                }
            ],
        ),
    ]

    if bridge_parameters is not None:
        actions.append(
            Node(
                package="mujoco_ros2_control_plugins",
                executable="mujoco_image_bridge",
                name="mujoco_image_bridge",
                output="screen",
                parameters=[bridge_parameters],
                additional_env=_camera_bridge_environment(),
            )
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
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    route_remaps = [
        "--remap franka_arm_jtc/follow_joint_trajectory:=/execution/arm/follow_joint_trajectory",
    ]
    if load_pika:
        route_remaps.extend(
            [
                "--remap pika_gripper_fwd/commands:=/execution/end_effector/joint_reference",
                "--remap pika_gripper_action/gripper_cmd:=/execution/end_effector/gripper_command",
            ]
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
        parameters=[{"use_sim_time": True}],
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
    return actions


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("franka_manipulation_rt_launch")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "task",
                default_value=_DEFAULT_LIBERO_TASK,
                description="LIBERO task name or absolute MJCF path.",
            ),
            DeclareLaunchArgument(
                "mujoco_model",
                default_value="",
                description=(
                    "Absolute MJCF path. When empty, resolved from task:=."
                ),
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run MuJoCo without a rendering window.",
            ),
            DeclareLaunchArgument(
                "mujoco_plugins_yaml",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "mujoco_plugins.yaml"]
                ),
                description="CameraPlugin and SHM bridge configuration.",
            ),
            DeclareLaunchArgument(
                "controllers_yaml",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "controller", "controllers.yaml"]
                ),
                description="Same effort controllers as real hardware.",
            ),
            DeclareLaunchArgument(
                "load_pika_hardware",
                default_value="true",
                description="Include Pika gripper joints in the MuJoCo system.",
            ),
            DeclareLaunchArgument(
                "gripper_serial_port",
                default_value="/dev/pika_left_gripper",
                description="Unused in sim; kept for xacro parity.",
            ),
            DeclareLaunchArgument(
                "joint_state_rate",
                default_value="100",
                description="joint_state_publisher rate (Hz).",
            ),
            DeclareLaunchArgument(
                "jtc_guard_heartbeat_timeout_s",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "jtc_guard_cancel_response_timeout_s",
                default_value="0.5",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
