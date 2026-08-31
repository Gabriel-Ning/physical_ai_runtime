"""Launch gamepad driver and gamepad_teleop node."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _optional_bool(name: str, value: str) -> bool | None:
    normalized = value.strip().lower()
    if not normalized:
        return None
    if normalized not in {"true", "false"}:
        raise ValueError(f"{name} must be 'true', 'false', or empty")
    return normalized == "true"


def _launch_nodes(context, *args, **kwargs):
    config = LaunchConfiguration("config").perform(context).strip()
    joy_driver = LaunchConfiguration("joy_driver").perform(context).strip()
    device_id = LaunchConfiguration("device_id").perform(context).strip()
    joy_dev = LaunchConfiguration("joy_dev").perform(context).strip()
    deadzone = LaunchConfiguration("deadzone").perform(context).strip()
    autorepeat_rate = LaunchConfiguration("autorepeat_rate").perform(context).strip()
    publish_rate_hz = LaunchConfiguration("publish_rate_hz").perform(context).strip()
    frame_id = LaunchConfiguration("frame_id").perform(context).strip()
    gripper_joint_name = LaunchConfiguration("gripper_joint_name").perform(
        context
    ).strip()
    twist_topic = LaunchConfiguration("twist_topic").perform(context).strip()
    gripper_topic = LaunchConfiguration("gripper_topic").perform(context).strip()
    clutch_topic = LaunchConfiguration("clutch_topic").perform(context).strip()
    status_topic = LaunchConfiguration("status_topic").perform(context).strip()
    joy_topic = LaunchConfiguration("joy_topic").perform(context).strip()
    autostart = _optional_bool(
        "autostart", LaunchConfiguration("autostart").perform(context)
    )

    node_params: dict[str, object] = {}
    if publish_rate_hz:
        try:
            node_params["publish_rate_hz"] = float(publish_rate_hz)
        except ValueError:
            pass
    if frame_id:
        node_params["frame_id"] = frame_id
    if gripper_joint_name:
        node_params["gripper_joint_name"] = gripper_joint_name
    if twist_topic:
        node_params["twist_topic"] = twist_topic
    if gripper_topic:
        node_params["gripper_topic"] = gripper_topic
    if clutch_topic:
        node_params["clutch_topic"] = clutch_topic
    if status_topic:
        node_params["status_topic"] = status_topic
    if joy_topic:
        node_params["joy_topic"] = joy_topic
    if autostart is not None:
        node_params["autostart"] = autostart

    deadzone_val = float(deadzone) if deadzone else 0.05
    autorepeat_rate_val = float(autorepeat_rate) if autorepeat_rate else 50.0

    # 1. Joy Hardware Node
    if joy_driver == "game_controller":
        dev_id_int = int(device_id) if device_id.isdigit() else 0
        joy_node = Node(
            package="joy",
            executable="game_controller_node",
            name="game_controller_node",
            output="screen",
            parameters=[
                {
                    "device_id": dev_id_int,
                    "deadzone": deadzone_val,
                    "autorepeat_rate": autorepeat_rate_val,
                }
            ],
            remappings=[("/joy", joy_topic)] if joy_topic != "/joy" else [],
        )
    elif joy_driver == "joy_linux":
        joy_node = Node(
            package="joy_linux",
            executable="joy_linux_node",
            name="joy_node",
            output="screen",
            parameters=[
                {
                    "dev": joy_dev or "/dev/input/js0",
                    "deadzone": deadzone_val,
                    "autorepeat_rate": autorepeat_rate_val,
                }
            ],
            remappings=[("/joy", joy_topic)] if joy_topic != "/joy" else [],
        )
    else:  # joy
        dev_id_int = int(device_id) if device_id.isdigit() else 0
        joy_node = Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[
                {
                    "device_id": dev_id_int,
                    "deadzone": deadzone_val,
                    "autorepeat_rate": autorepeat_rate_val,
                }
            ],
            remappings=[("/joy", joy_topic)] if joy_topic != "/joy" else [],
        )

    # 2. Gamepad Teleop Node
    teleop_node = Node(
        package="gamepad_teleop",
        executable="gamepad_teleop_node",
        name="gamepad_teleop",
        output="screen",
        parameters=[config, node_params],
    )

    return [joy_node, teleop_node]


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("gamepad_teleop"))
    default_config = str(share / "config" / "ps5.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Path to gamepad_teleop YAML config.",
            ),
            DeclareLaunchArgument(
                "joy_driver",
                default_value="game_controller",
                description="Joy driver executable: game_controller (SDL), joy_linux, or joy.",
            ),
            DeclareLaunchArgument(
                "device_id",
                default_value="0",
                description="Joystick device ID (integer index for game_controller/joy).",
            ),
            DeclareLaunchArgument(
                "joy_dev",
                default_value="/dev/input/js0",
                description="Device path for joy_linux (e.g. /dev/input/js0).",
            ),
            DeclareLaunchArgument(
                "deadzone",
                default_value="0.05",
                description="Joystick deadzone threshold (0.0 to 1.0).",
            ),
            DeclareLaunchArgument(
                "autorepeat_rate",
                default_value="50.0",
                description="Autorepeat rate for unchanged joystick state (Hz).",
            ),
            DeclareLaunchArgument(
                "publish_rate_hz",
                default_value="",
                description="Optional publish rate override in Hz (e.g. 100.0).",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="",
                description="Optional frame_id override for TwistStamped.",
            ),
            DeclareLaunchArgument(
                "gripper_joint_name",
                default_value="",
                description="Optional gripper JointTrajectory joint-name override.",
            ),
            DeclareLaunchArgument(
                "twist_topic",
                default_value="",
                description="Optional override for output twist topic.",
            ),
            DeclareLaunchArgument(
                "gripper_topic",
                default_value="",
                description="Optional override for output gripper topic.",
            ),
            DeclareLaunchArgument(
                "clutch_topic",
                default_value="",
                description="Optional override for output clutch topic.",
            ),
            DeclareLaunchArgument(
                "status_topic",
                default_value="",
                description="Optional override for status topic.",
            ),
            DeclareLaunchArgument(
                "joy_topic",
                default_value="/joy",
                description="Raw Joy topic name.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="",
                description="Optional autostart boolean override ('true'/'false').",
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
