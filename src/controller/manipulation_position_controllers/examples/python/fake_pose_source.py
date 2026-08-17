#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Fake CartesianTrajectory (1-pt) source for task-space controller smoke tests."""

import math
import sys

import rclpy
from moveit_msgs.msg import (
    CartesianPoint,
    CartesianTrajectory,
    CartesianTrajectoryPoint,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class FakePoseSource(Node):
    def __init__(self) -> None:
        super().__init__("fake_pose_source")
        self._topic = str(
            self.declare_parameter("pose_topic", "/test/pose_reference").value
        )
        self._frame_id = str(self.declare_parameter("frame_id", "base_link").value)
        self._rate_hz = float(self.declare_parameter("publish_rate_hz", 50.0).value)
        self._pos_x = float(self.declare_parameter("position_x", 0.4).value)
        self._pos_y = float(self.declare_parameter("position_y", 0.0).value)
        self._pos_z = float(self.declare_parameter("position_z", 0.3).value)
        self._qw = float(self.declare_parameter("orientation_w", 1.0).value)
        self._qx = float(self.declare_parameter("orientation_x", 0.0).value)
        self._qy = float(self.declare_parameter("orientation_y", 0.0).value)
        self._qz = float(self.declare_parameter("orientation_z", 0.0).value)
        self._circle_radius_m = float(
            self.declare_parameter("circle_radius_m", 0.0).value
        )
        self._circle_period_s = float(
            self.declare_parameter("circle_period_s", 5.0).value
        )
        self._duration_s = float(self.declare_parameter("duration_s", 0.0).value)

        if self._circle_radius_m < 0.0:
            self._circle_radius_m = 0.0

        self._pub = self.create_publisher(CartesianTrajectory, self._topic, 10)
        self._start_time = self.get_clock().now()
        self._count = 0
        self.create_timer(1.0 / self._rate_hz, self._publish)

        if self._duration_s > 0.0:
            self.create_timer(self._duration_s, self._shutdown)

        self.get_logger().info(
            f"Fake pose source: topic={self._topic} frame={self._frame_id} "
            f"rate={self._rate_hz:.1f}Hz pos=({self._pos_x},{self._pos_y},{self._pos_z}) "
            f"circle_radius={self._circle_radius_m:.3f}m duration={self._duration_s}s"
        )

    def _publish(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self._start_time).nanoseconds * 1e-9

        x = self._pos_x
        y = self._pos_y
        if self._circle_radius_m > 0.0 and self._circle_period_s > 0.0:
            angle = 2.0 * math.pi * elapsed / self._circle_period_s
            x += self._circle_radius_m * math.cos(angle)
            y += self._circle_radius_m * math.sin(angle)

        msg = CartesianTrajectory()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._frame_id
        pt = CartesianTrajectoryPoint()
        pt.point = CartesianPoint()
        pt.point.pose.position.x = x
        pt.point.pose.position.y = y
        pt.point.pose.position.z = self._pos_z
        pt.point.pose.orientation.x = self._qx
        pt.point.pose.orientation.y = self._qy
        pt.point.pose.orientation.z = self._qz
        pt.point.pose.orientation.w = self._qw
        msg.points.append(pt)
        self._pub.publish(msg)
        self._count += 1
        if self._count == 1:
            self.get_logger().info(
                f"First pose sent: ({x:.3f},{y:.3f},{self._pos_z:.3f})"
            )

    def _shutdown(self) -> None:
        self.get_logger().info(
            f"Fake pose source done: {self._count} messages over {self._duration_s:.0f}s"
        )
        self.destroy_publisher(self._pub)
        sys.exit(0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = FakePoseSource()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
