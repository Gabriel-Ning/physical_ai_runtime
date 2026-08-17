#!/usr/bin/env python3
"""Fake CartesianTrajectory source — circular trajectory chunk at configurable rate."""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    CartesianTrajectory,
    CartesianTrajectoryPoint,
    CartesianPoint,
)


class FakePoseChunkSource(Node):
    def __init__(self):
        super().__init__("fake_pose_chunk_source")
        self.declare_parameter("pose_topic", "/test/pose_chunk")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("chunk_size", 5)
        self.declare_parameter("dt_per_frame", 0.02)
        self.declare_parameter("circle_radius_m", 0.02)
        self.declare_parameter("circle_period_s", 4.0)
        self.declare_parameter("center_x", 0.40)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("center_z", 0.30)
        self.declare_parameter("duration_s", 12.0)
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("timed", True)

        topic = self.get_parameter("pose_topic").value
        rate = self.get_parameter("publish_rate_hz").value
        self._chunk_size = self.get_parameter("chunk_size").value
        self._dt_frame = self.get_parameter("dt_per_frame").value
        radius = self.get_parameter("circle_radius_m").value
        period = self.get_parameter("circle_period_s").value
        cx = self.get_parameter("center_x").value
        cy = self.get_parameter("center_y").value
        cz = self.get_parameter("center_z").value
        duration = self.get_parameter("duration_s").value
        self._frame_id = self.get_parameter("frame_id").value
        self._timed = bool(self.get_parameter("timed").value)

        self._pub = self.create_publisher(CartesianTrajectory, topic, 10)
        timer_period = 1.0 / rate
        self._timer = self.create_timer(timer_period, self._publish)
        self._start = self.get_clock().now()
        self._duration = duration
        self._radius = radius
        self._period = period
        self._cx, self._cy, self._cz = cx, cy, cz

        self.get_logger().info(
            f"CartesianTrajectory source: topic={topic} rate={rate:.0f}Hz "
            f"chunk={self._chunk_size} dt={self._dt_frame}s timed={self._timed} "
            f"circle=r={radius:.3f}m center=({cx:.2f},{cy:.2f},{cz:.2f})"
        )

    def _pose_at(self, t: float) -> Pose:
        omega = 2.0 * math.pi / self._period
        pose = Pose()
        pose.position.x = self._cx + self._radius * math.cos(omega * t)
        pose.position.y = self._cy + self._radius * math.sin(omega * t)
        pose.position.z = self._cz
        pose.orientation.w = 1.0
        return pose

    def _publish(self):
        now = self.get_clock().now()
        elapsed = (now - self._start).nanoseconds * 1e-9
        if elapsed > self._duration:
            self.get_logger().info("Duration reached; shutting down.")
            raise SystemExit

        msg = CartesianTrajectory()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._frame_id
        for i in range(self._chunk_size):
            t = elapsed + i * self._dt_frame
            pt = CartesianTrajectoryPoint()
            pt.point = CartesianPoint()
            pt.point.pose = self._pose_at(t)
            if self._timed:
                ns = int(round(i * self._dt_frame * 1e9))
                pt.time_from_start.sec = ns // 1_000_000_000
                pt.time_from_start.nanosec = ns % 1_000_000_000
            msg.points.append(pt)
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = FakePoseChunkSource()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
