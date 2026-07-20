# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
#
# RViz interactive marker teleop source.
#
# Publishes a PoseStamped on a configurable topic. This launch does NOT
# compose a controller, robot, or RViz config — it is a pure source that
# apps include. Compose RViz and embodiment frames in the app package.
#
# Usage:
#   ros2 launch rviz_interactive_marker_teleop teleop.launch.py
#   ros2 launch rviz_interactive_marker_teleop teleop.launch.py \
#       pose_topic:=/action_sources/marker/pose_target \
#       publish_frequency:=100.0 \
#       base_frame:=base_link \
#       target_frame:=flange_link
#
# Typical integration:
#   - marker → EM pose_target → TaskSpaceKinematicPositionController
#   - marker → planner/IK → EM → JointSpacePositionController

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pose_topic = DeclareLaunchArgument(
        "pose_topic",
        default_value="/action_sources/marker/pose_target",
        description="Output PoseStamped topic (EM pose_target contract by default).",
    )
    publish_frequency = DeclareLaunchArgument(
        "publish_frequency",
        default_value="50.0",
        description="Marker pose publish rate in Hz.",
    )
    base_frame = DeclareLaunchArgument(
        "base_frame",
        default_value="base_link",
        description="TF base frame for the interactive marker.",
    )
    target_frame = DeclareLaunchArgument(
        "target_frame",
        default_value="flange_link",
        description="TF target frame used to initialise the marker pose.",
    )
    server_namespace = DeclareLaunchArgument(
        "server_namespace",
        default_value="target_marker",
        description="InteractiveMarkerServer namespace.",
    )
    marker_name = DeclareLaunchArgument(
        "marker_name",
        default_value="target_marker",
        description="Interactive marker name.",
    )

    target_marker = Node(
        package="rviz_interactive_marker_teleop",
        executable="target_marker_node.py",
        name="target_marker_node",
        output="screen",
        parameters=[
            {
                "base_frame": LaunchConfiguration("base_frame"),
                "target_frame": LaunchConfiguration("target_frame"),
                "pose_topic": LaunchConfiguration("pose_topic"),
                "publish_frequency": LaunchConfiguration("publish_frequency"),
                "server_namespace": LaunchConfiguration("server_namespace"),
                "marker_name": LaunchConfiguration("marker_name"),
            }
        ],
    )

    return LaunchDescription(
        [
            pose_topic,
            publish_frequency,
            base_frame,
            target_frame,
            server_namespace,
            marker_name,
            target_marker,
        ]
    )
