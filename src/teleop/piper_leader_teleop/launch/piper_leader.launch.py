from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _optional_bool(name: str, value: str) -> bool | None:
    normalized = value.strip().lower()
    if not normalized:
        return None
    if normalized not in {"true", "false"}:
        raise ValueError(f"{name} must be 'true', 'false', or empty")
    return normalized == "true"


def _nodes(context, *args, **kwargs):
    config = LaunchConfiguration("config")
    node_name = LaunchConfiguration("node_name").perform(context).strip()
    can_interface = LaunchConfiguration("can_interface").perform(context).strip()
    publish_rate_hz = LaunchConfiguration("publish_rate_hz").perform(context).strip()
    joint_names = LaunchConfiguration("joint_names").perform(context).strip()
    gripper_joint_name = (
        LaunchConfiguration("gripper_joint_name").perform(context).strip()
    )
    joint_reference_topic = (
        LaunchConfiguration("joint_reference_topic").perform(context).strip()
    )
    gripper_reference_topic = (
        LaunchConfiguration("gripper_reference_topic").perform(context).strip()
    )
    status_topic = LaunchConfiguration("status_topic").perform(context).strip()
    pendant_state_topic = (
        LaunchConfiguration("pendant_state_topic").perform(context).strip()
    )
    default_mode = LaunchConfiguration("default_mode").perform(context).strip()
    fallback_mode = LaunchConfiguration("fallback_mode").perform(context).strip()
    autostart = _optional_bool(
        "autostart", LaunchConfiguration("autostart").perform(context)
    )

    leader_model_xacro = LaunchConfiguration("leader_model_xacro")
    leader_robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            leader_model_xacro,
        ]
    )
    node_params = {
        "leader_robot_description": leader_robot_description,
    }
    if can_interface:
        node_params["can_interface"] = can_interface
    if publish_rate_hz:
        try:
            node_params["publish_rate_hz"] = float(publish_rate_hz)
        except ValueError:
            pass
    if default_mode:
        node_params["default_mode"] = default_mode
    if fallback_mode:
        node_params["fallback_mode"] = fallback_mode
    if autostart is not None:
        node_params["autostart"] = autostart
    if joint_names:
        parsed_joints = [j.strip() for j in joint_names.split(",") if j.strip()]
        node_params["joint_names"] = parsed_joints
        node_params["follower_joint_names"] = parsed_joints
    if gripper_joint_name:
        node_params["gripper_joint_name"] = gripper_joint_name
        node_params["follower_gripper_joint_name"] = gripper_joint_name
    if joint_reference_topic:
        node_params["joint_reference_topic"] = joint_reference_topic
    if gripper_reference_topic:
        node_params["gripper_reference_topic"] = gripper_reference_topic
    if status_topic:
        node_params["status_topic"] = status_topic
    if pendant_state_topic:
        node_params["pendant_state_topic"] = pendant_state_topic

    params = [
        config,
        node_params,
    ]

    node = Node(
        package="piper_leader_teleop",
        executable="piper_leader_node",
        name=node_name,
        output="screen",
        parameters=params,
    )
    return [node]


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("piper_leader_teleop"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config", default_value=str(share / "config" / "piper_leader.yaml")
            ),
            DeclareLaunchArgument(
                "node_name",
                default_value="piper_leader",
                description="Node name; use piper_leader_left / piper_leader_right for dual-arm.",
            ),
            DeclareLaunchArgument(
                "can_interface",
                default_value="",
                description="Optional SocketCAN interface override (e.g. can0, can1).",
            ),
            DeclareLaunchArgument(
                "publish_rate_hz",
                default_value="",
                description="Optional publish rate in Hz (e.g. 200.0).",
            ),
            DeclareLaunchArgument(
                "joint_names",
                default_value="",
                description="Comma-separated joint names (e.g. left_joint1,left_joint2,...).",
            ),
            DeclareLaunchArgument(
                "gripper_joint_name",
                default_value="",
                description="Gripper joint name (e.g. left_gripper_joint1).",
            ),
            DeclareLaunchArgument(
                "joint_reference_topic",
                default_value="",
                description="Output arm joint reference trajectory topic.",
            ),
            DeclareLaunchArgument(
                "gripper_reference_topic",
                default_value="",
                description="Output gripper reference trajectory topic.",
            ),
            DeclareLaunchArgument(
                "status_topic",
                default_value="",
                description="Status publisher topic.",
            ),
            DeclareLaunchArgument(
                "pendant_state_topic",
                default_value="",
                description="Teaching pendant state topic.",
            ),
            DeclareLaunchArgument(
                "default_mode",
                default_value="",
                description="Optional startup mode override: shadow | passive (empty = use config).",
            ),
            DeclareLaunchArgument(
                "fallback_mode",
                default_value="",
                description="Optional fallback release mode override: shadow | passive (empty = use config).",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="",
                description="Optional hardware activation override: true | false (empty = use config).",
            ),
            DeclareLaunchArgument(
                "leader_model_xacro",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("piper_description"),
                        "urdf",
                        "piper_with_teach.urdf.xacro",
                    ]
                ),
                description="Leader-only Piper + teaching-pendant xacro.",
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
