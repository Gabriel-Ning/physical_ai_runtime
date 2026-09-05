# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Physical AI Runtime controller bringup for one Franka FR3 arm.

Owns FR3 + Pika ros2_control composition:
  robot_state_publisher, ros2_control_node (Real / Fake / MuJoCo),
  joint_state_publisher, serialized JSB -> inactive route controllers,
  optional RViz, CPU pin.

Does not modify vendor ``franka_bringup/franka.launch.py``.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
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
import xacro
import yaml


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
        raise ValueError("SHM bringup requires default_policy: disabled so all active cameras are bridged")
    names = [name for name, entry in camera.items()
             if isinstance(entry, dict) and entry.get("policy") in ("streaming", "polled")]
    return {"camera_names": names, "shm_prefix": camera.get("shm_prefix", "/pai_mj_cam_"),
            "poll_hz": 120.0, "use_sim_time": False}


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
    return {"CYCLONEDDS_URI": os.environ.get(
        "MUJOCO_IMAGE_BRIDGE_CYCLONEDDS_URI",
        ",".join(part for part in (inherited, override) if part),
    )}


def _resolve_cpu_affinity(context) -> str:
    """Prefer launch arg; else RT_CM_CPU_AFFINITY from the RT host profile."""
    explicit = LaunchConfiguration("cpu_affinity").perform(context).strip()
    if explicit in ("none", "off", "-"):
        return ""
    if explicit:
        return explicit
    return os.environ.get("RT_CM_CPU_AFFINITY", "").strip()


def _launch_setup(context, *args, **kwargs):
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    use_sim_mujoco_arg = LaunchConfiguration("use_sim_mujoco").perform(context).lower().strip()
    backend_arg = LaunchConfiguration("backend").perform(context).lower().strip()

    if backend_arg:
        if backend_arg not in ("real", "fake", "mujoco"):
            raise RuntimeError(f"'backend' must be real, fake, or mujoco, got '{backend_arg}'")
        is_mujoco = (backend_arg == "mujoco")
        fake = "true" if backend_arg == "fake" else "false"
    elif use_sim_mujoco_arg in ("true", "1", "yes"):
        is_mujoco = True
        fake = "false"
    else:
        is_mujoco = False
        fake = use_fake_hardware

    task_name = LaunchConfiguration("task").perform(context).strip()
    headless_str = LaunchConfiguration("headless").perform(context).lower().strip()
    headless = headless_str in ("true", "1", "yes")

    bringup_share = get_package_share_directory("franka_manipulation_rt_launch")

    mujoco_model_path = ""
    if is_mujoco:
        if os.path.isabs(task_name) and os.path.exists(task_name):
            mujoco_model_path = task_name
        else:
            candidates = []
            libero_share = ""
            try:
                libero_share = get_package_share_directory("libero_tasks")
                candidates.extend([
                    os.path.join(libero_share, "mjcf", f"{task_name}.xml"),
                    os.path.join(libero_share, "mjcf", "tasks", f"{task_name}.xml"),
                ])
            except Exception:
                pass

            for candidate in candidates:
                if os.path.exists(candidate):
                    mujoco_model_path = candidate
                    break

            if not mujoco_model_path:
                # Check if this task belongs to RoboTwin suite
                try:
                    rt_share = get_package_share_directory("robotwin_tasks")
                    rt_candidate = os.path.join(rt_share, "mjcf", f"{task_name}.xml")
                    if os.path.exists(rt_candidate):
                        raise RuntimeError(
                            f"Task '{task_name}' belongs to the RoboTwin benchmark (Piper dual-arm). "
                            f"Current runtime stack is 'rt-franka' (Franka FR3). "
                            f"Please run 'pixi run rt-piper backend:=mujoco task:={task_name}' for Piper, "
                            f"or select a LIBERO task for Franka."
                        )
                except RuntimeError:
                    raise
                except Exception:
                    pass

                default_model = os.path.join(
                    libero_share, "mjcf",
                    "pick_up_the_black_bowl_between_the_plate_and_the_ramekin_and_place_it_on_the_plate.xml"
                ) if libero_share else task_name
                mujoco_model_path = default_model

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

    default_real_yaml = os.path.join(bringup_share, "config", "controller", "controllers.yaml")
    default_fake_yaml = os.path.join(bringup_share, "config", "controller", "controllers_fake.yaml")
    if (
        fake.lower() in ("true", "1")
        and controllers_yaml == default_real_yaml
    ):
        controllers_yaml = default_fake_yaml

    urdf_path = os.path.join(bringup_share, "urdf", "fr3_manipulation.urdf.xacro")
    robot_description = xacro.process_file(
        urdf_path,
        mappings={
            "robot_type": "fr3",
            "arm_prefix": "",
            "robot_ip": robot_ip,
            "hand": "false",
            "use_fake_hardware": fake,
            "use_sim_mujoco": "true" if is_mujoco else "false",
            "mujoco_model": mujoco_model_path,
            "headless": "true" if headless else "false",
            "fake_sensor_commands": "false",
            "gripper_serial_port": gripper_serial_port,
            "load_pika_hardware": "true" if load_pika else "false",
        },
    ).toprettyxml(indent="  ")

    route_controllers = [
        "franka_arm_tskpc",
        "franka_arm_jspc",
        "franka_arm_jtc",
    ]
    route_remaps = [
        "--remap franka_arm_jtc/follow_joint_trajectory:=/execution/arm/follow_joint_trajectory",
    ]
    if load_pika:
        route_controllers.extend(["pika_gripper_fwd", "pika_gripper_action"])
        route_remaps.extend(
            [
                "--remap pika_gripper_fwd/commands:=/execution/end_effector/joint_reference",
                "--remap pika_gripper_action/gripper_cmd:=/execution/end_effector/gripper_command",
            ]
        )

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

    cm_params = [
        {"robot_description": robot_description},
        controllers_yaml,
        {"robot_type": "fr3"},
        {"load_gripper": False},
        {"arm_prefix": ""},
    ]
    if is_mujoco:
        cm_params.append({"use_sim_time": True})
        # Use the same controllers.yaml as real hardware. Prior soft-gain / limiter
        # overrides were a workaround for multi-consumer image DDS overload, not a
        # required sim/real controller split.
        mujoco_plugins_yaml = LaunchConfiguration("mujoco_plugins_yaml").perform(context)
        bridge_parameters = _camera_bridge_parameters(mujoco_plugins_yaml)
        cm_params.append(mujoco_plugins_yaml)

    cm_kwargs = {
        "package": "controller_manager",
        "executable": "ros2_control_node",
        "parameters": cm_params,
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
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description, "use_sim_time": is_mujoco}],
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
                        "use_sim_time": is_mujoco,
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
                        "use_sim_time": is_mujoco,
                    }
                ],
            ),
        ]
    )
    if is_mujoco and bridge_parameters is not None:
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
        parameters=[{"use_sim_time": is_mujoco}],
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
        parameters=[{"use_sim_time": is_mujoco}],
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


    if use_rviz and not is_mujoco:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
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
                "backend",
                default_value="",
                description="real, fake, or mujoco. Empty falls back to use_fake_hardware.",
            ),
            DeclareLaunchArgument(
                "use_sim_mujoco",
                default_value="false",
                description="Enable MuJoCo simulation backend.",
            ),
            DeclareLaunchArgument(
                "task",
                default_value="pick_up_the_black_bowl_between_the_plate_and_the_ramekin_and_place_it_on_the_plate",
                description="Task name for MuJoCo simulation (LIBERO task name).",
            ),
            DeclareLaunchArgument(
                "mujoco_plugins_yaml",
                default_value=PathJoinSubstitution([FindPackageShare("franka_manipulation_rt_launch"),
                                                   "config", "mujoco_plugins.yaml"]),
                description="Shared CameraPlugin and SHM bridge configuration.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run MuJoCo simulation headless without rendering window.",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="192.168.2.101",
                description="FR3 hostname or IP; ignored by fake hardware.",
            ),
            DeclareLaunchArgument(
                "gripper_serial_port",
                default_value="/dev/pika_left_gripper",
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
                    "Empty uses RT_CM_CPU_AFFINITY from the RT host profile "
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
