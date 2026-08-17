#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
#
# Publishes JointTrajectory to stress-test JTC vs streaming controller.
# Configurable horizon_steps — >1 forces JTC to actually spline-interpolate.

import math
import random
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def sine_at(t, amp, freq):
    return amp * math.sin(2.0 * math.pi * freq * t)


class FakeJointTrajectorySource(Node):
    def __init__(self):
        super().__init__("fake_joint_trajectory_source")
        self.declare_parameter("trajectory_topic", "/test/joint_trajectory")
        self.declare_parameter("active_joint", "joint1")
        self.declare_parameter(
            "joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        )
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("amplitude_rad", 0.1)
        self.declare_parameter("frequency_hz", 0.5)
        self.declare_parameter("duration_s", 12.0)
        self.declare_parameter("dt_s", 0.02)  # total trajectory horizon
        self.declare_parameter("horizon_steps", 5)  # waypoints per trajectory
        self.declare_parameter("noise_std_rad", 0.0)  # IK solver noise simulation
        self.declare_parameter("exit_on_complete", False)

        topic = self.get_parameter("trajectory_topic").value
        self._active_joint = self.get_parameter("active_joint").value
        self._joint_names = self.get_parameter("joint_names").value
        rate = self.get_parameter("publish_rate_hz").value
        amp = self.get_parameter("amplitude_rad").value
        freq = self.get_parameter("frequency_hz").value
        duration = self.get_parameter("duration_s").value
        self.dt = self.get_parameter("dt_s").value
        self._horizon = self.get_parameter("horizon_steps").value
        self._noise_std = self.get_parameter("noise_std_rad").value
        self._exit_on_complete = self.get_parameter("exit_on_complete").value
        self.done = False

        self._pub = self.create_publisher(JointTrajectory, topic, 10)
        period = 1.0 / rate
        self._timer = self.create_timer(period, self._publish)
        self._start = self.get_clock().now()
        self._duration = duration
        self._amp = amp
        self._freq = freq

        self.get_logger().info(
            f"JointTrajectory source: topic={topic} "
            f"rate={rate:.0f}Hz amp={amp:.2f}rad freq={freq:.1f}Hz "
            f"duration={duration:.0f}s dt={self.dt:.3f}s horizon={self._horizon} "
            f"noise_std={self._noise_std:.4f}rad exit_on_complete={self._exit_on_complete}"
        )

    def _publish(self):
        elapsed = (self.get_clock().now() - self._start).nanoseconds / 1e9
        if elapsed > self._duration:
            self.get_logger().info("Source done.")
            self._timer.cancel()
            self.done = True
            return

        t0 = elapsed
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = [self._active_joint]

        step_dt = self.dt / max(self._horizon, 1)
        for k in range(self._horizon):
            t = t0 + k * step_dt
            pos = sine_at(t, self._amp, self._freq)
            pos += random.gauss(0.0, self._noise_std)  # simulate IK noise
            pt = JointTrajectoryPoint()
            pt.positions = [pos]
            # Single-point case: tfs = dt so bridge can prepend tfs=0
            tfs = self.dt if self._horizon == 1 else k * step_dt
            sec = int(tfs)
            nsec = int((tfs - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            traj.points.append(pt)

        self._pub.publish(traj)


def main():
    rclpy.init()
    node = FakeJointTrajectorySource()
    while rclpy.ok() and (not node.done or not node._exit_on_complete):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
