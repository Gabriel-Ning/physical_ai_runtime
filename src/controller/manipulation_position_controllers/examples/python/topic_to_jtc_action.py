#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
#
# Bridge: subscribes to JointTrajectory topic, forwards as FollowJointTrajectory
# action goal to JointTrajectoryController.
#
# Each incoming trajectory replaces any in-flight goal.  The trajectory is sent
# with a zero start time offset so JTC starts executing immediately.

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
import trajectory_msgs.msg
from builtin_interfaces.msg import Duration


class TopicToJtcAction(Node):
    def __init__(self):
        super().__init__("topic_to_jtc_action")
        self.declare_parameter("trajectory_topic", "/ik_joint_trajectory")
        self.declare_parameter(
            "action_name", "/joint_trajectory_controller/follow_joint_trajectory"
        )

        topic = self.get_parameter("trajectory_topic").value
        action = self.get_parameter("action_name").value

        self._sub = self.create_subscription(
            JointTrajectory,
            topic,
            self._traj_callback,
            rclpy.qos.qos_profile_system_default,
        )

        self._client = ActionClient(self, FollowJointTrajectory, action)
        self._goal_handle = None  # track in-flight goal
        self._goal_id = 0

        self.get_logger().info(f"Topic→JTC bridge: {topic} → {action}")

    def _traj_callback(self, msg: JointTrajectory):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = msg

        # header.stamp = zero → JTC interprets as "execute now"
        goal.trajectory.header.stamp.sec = 0
        goal.trajectory.header.stamp.nanosec = 0

        # For single-point trajectories, prepend a "start-now" point so JTC
        # has at least 2 points to interpolate over.
        if len(goal.trajectory.points) == 1:
            pt0 = trajectory_msgs.msg.JointTrajectoryPoint()
            pt0.positions = list(goal.trajectory.points[0].positions)
            pt0.time_from_start = Duration(sec=0, nanosec=0)
            goal.trajectory.points.insert(0, pt0)
        # Multi-point trajectories pass through as-is.

        if not self._client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn(
                "JTC action server not available", throttle_duration_sec=2.0
            )
            return

        self._goal_id += 1
        self._client.send_goal_async(goal)


def main():
    rclpy.init()
    rclpy.spin(TopicToJtcAction())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
