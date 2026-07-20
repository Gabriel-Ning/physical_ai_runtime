#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TargetMarkerNode(Node):
    """
    ROS 2 Node that provides an interactive marker in RViz to specify target Cartesian poses.
    Subscribes to robot transforms to initialize the marker pose at the current end-effector pose,
    and publishes the target pose to a configurable PoseStamped topic
    (default: /action_sources/marker/pose_target).
    """

    def __init__(self):
        super().__init__("target_marker_node")
        self.get_logger().info("Initializing TargetMarkerNode...")

        # Parameters
        self.publish_rate_hz = self.declare_parameter("publish_frequency", 50.0).value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.target_frame = self.declare_parameter("target_frame", "flange_link").value
        self.pose_topic = self.declare_parameter(
            "pose_topic", "/action_sources/marker/pose_target"
        ).value
        self.server_namespace = self.declare_parameter(
            "server_namespace", "target_marker").value
        self.marker_name = self.declare_parameter(
            "marker_name", "target_marker").value
        self.marker_description = self.declare_parameter(
            "marker_description", "Target Pose").value
        self.publish_before_first_feedback = bool(
            self.declare_parameter("publish_before_first_feedback", True).value
        )

        # TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State
        self.target_pose_stamped = None
        self.server = None
        self.user_has_moved_marker = False

        # Publisher
        self.pose_publisher = self.create_publisher(PoseStamped, self.pose_topic, 10)

        # Timer for publishing target pose and checking initialization
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)
        self.get_logger().info("TargetMarkerNode initialized. Waiting for TF to align marker...")

    def get_latest_ee_pose(self) -> PoseStamped:
        """Lookup latest transform from base_frame to target_frame."""
        try:
            # Look up transform
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.target_frame, rclpy.time.Time()
            )
            pose = PoseStamped()
            pose.header.frame_id = self.base_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose
        except TransformException as ex:
            self.get_logger().warn(
                f"Could not transform {self.base_frame} to {self.target_frame}: {ex}",
                throttle_duration_sec=2.0
            )
            return None

    def process_feedback(self, feedback):
        """Callback for interactive marker pose updates."""
        if self.target_pose_stamped is not None:
            self.target_pose_stamped = PoseStamped()
            self.target_pose_stamped.header = feedback.header
            self.target_pose_stamped.header.frame_id = self.base_frame
            self.target_pose_stamped.pose = feedback.pose
            if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
                self.user_has_moved_marker = True

    def setup_interactive_marker(self, initial_pose: PoseStamped):
        """Create and start the interactive marker server."""
        self.get_logger().info(
            f"Initializing Interactive Marker Server at pose: "
            f"x={initial_pose.pose.position.x:.3f}, "
            f"y={initial_pose.pose.position.y:.3f}, "
            f"z={initial_pose.pose.position.z:.3f}"
        )

        self.server = InteractiveMarkerServer(self, self.server_namespace)

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.base_frame
        int_marker.name = self.marker_name
        int_marker.description = self.marker_description
        int_marker.pose = initial_pose.pose
        int_marker.scale = 0.15

        # Create a visual representation (box + axis)
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.04
        box_marker.scale.y = 0.04
        box_marker.scale.z = 0.04
        box_marker.color.r = 0.0
        box_marker.color.g = 0.8
        box_marker.color.b = 0.8
        box_marker.color.a = 0.7  # Semi-transparent cyan

        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)
        int_marker.controls.append(box_control)

        # Free 3D drag (always visible)
        free_control = InteractiveMarkerControl()
        free_control.name = "move_3d"
        free_control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        free_control.always_visible = True
        int_marker.controls.append(free_control)

        # 6-DOF axis controls (translation + rotation along axes)

        # X Axis
        control = InteractiveMarkerControl()
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        # Y Axis
        control = InteractiveMarkerControl()
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0 / np.sqrt(2)
        control.orientation.y = 1.0 / np.sqrt(2)
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1.0 / np.sqrt(2)
        control.orientation.y = 1.0 / np.sqrt(2)
        int_marker.controls.append(control)

        # Z Axis
        control = InteractiveMarkerControl()
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0 / np.sqrt(2)
        control.orientation.z = 1.0 / np.sqrt(2)
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1.0 / np.sqrt(2)
        control.orientation.z = 1.0 / np.sqrt(2)
        int_marker.controls.append(control)

        # Insert marker and apply changes
        self.server.insert(int_marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

    def timer_callback(self):
        """Periodic timer callback to initialize or publish target pose."""
        if self.target_pose_stamped is None:
            # We haven't initialized yet. Attempt to lookup the TF.
            current_pose = self.get_latest_ee_pose()
            if current_pose is not None:
                self.setup_interactive_marker(current_pose)
                self.target_pose_stamped = current_pose
        else:
            # We are initialized. Publish the current target pose.
            if not self.publish_before_first_feedback and not self.user_has_moved_marker:
                return
            # Make sure to update the timestamp so downstream nodes know it's fresh.
            self.target_pose_stamped.header.stamp = self.get_clock().now().to_msg()
            self.pose_publisher.publish(self.target_pose_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = TargetMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
