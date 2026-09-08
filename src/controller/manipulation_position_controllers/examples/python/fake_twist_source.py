#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class FakeTwistSource(Node):
    def __init__(self):
        super().__init__("fake_twist_source")
        self.declare_parameter("twist_topic", "/test/twist_reference")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("duration_s", 8.0)
        self.declare_parameter("linear_axis", "lateral")
        self.declare_parameter("linear_amplitude_m_s", 0.03)
        self.declare_parameter("frequency_hz", 0.25)

        topic = self.get_parameter("twist_topic").value
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._frame_id = self.get_parameter("frame_id").value
        self._duration_s = float(self.get_parameter("duration_s").value)
        self._linear_axis = self.get_parameter("linear_axis").value
        self._linear_amp = float(self.get_parameter("linear_amplitude_m_s").value)
        self._frequency_hz = float(self.get_parameter("frequency_hz").value)
        self._done = False

        self._pub = self.create_publisher(TwistStamped, topic, 10)
        self._start = self.get_clock().now()
        self._timer = self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f"Twist source: topic={topic} frame={self._frame_id} "
            f"rate={rate:.0f}Hz duration={self._duration_s:.1f}s "
            f"axis={self._linear_axis} amp={self._linear_amp:.3f}m/s "
            f"freq={self._frequency_hz:.2f}Hz"
        )

    @property
    def done(self):
        return self._done

    def _publish(self):
        elapsed = (self.get_clock().now() - self._start).nanoseconds / 1e9
        if elapsed > self._duration_s:
            self.get_logger().info("Twist source done.")
            self._timer.cancel()
            self._done = True
            return

        value = self._linear_amp * math.sin(
            2.0 * math.pi * self._frequency_hz * elapsed
        )
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        if self._linear_axis == "x":
            msg.twist.linear.x = value
        elif self._linear_axis == "z":
            msg.twist.linear.z = value
        else:
            msg.twist.linear.y = value
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = FakeTwistSource()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
