# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

import pathlib
import time
import unittest

import launch_testing
import launch_testing.actions
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray
from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint


def generate_test_description():
    test_dir = pathlib.Path(__file__).parent
    robot_description = (test_dir / "jtc_test_system.urdf").read_text()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[str(test_dir / "jtc_test_controllers.yaml")],
        output="screen",
    )
    spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "test_jtc",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
        output="screen",
    )
    guard = Node(
        package="joint_trajectory_controller_guard",
        executable="jtc_guard_node",
        name="test_jtc_guard",
        parameters=[
            {
                "action_name": "/test_jtc/follow_joint_trajectory",
                "heartbeat_topic": "/test_jtc_guard/heartbeat",
                "heartbeat_timeout_s": 0.25,
                "cancel_response_timeout_s": 1.0,
                "watchdog_rate_hz": 100.0,
                "diagnostic_period_s": 0.05,
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            control_node,
            spawner,
            guard,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestJtcDisconnect(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("jtc_guard_disconnect_test")

    def tearDown(self):
        self.node.destroy_node()

    def test_heartbeat_loss_cancels_active_standard_jtc_goal(self):
        action_client = ActionClient(
            self.node, FollowJointTrajectory, "/test_jtc/follow_joint_trajectory"
        )
        self.assertTrue(action_client.wait_for_server(timeout_sec=20.0))

        diagnostics = []
        diagnostic_sub = self.node.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            lambda message: diagnostics.extend(message.status),
            10,
        )
        heartbeat_pub = self.node.create_publisher(
            Bool, "/test_jtc_guard/heartbeat", 1
        )

        # Arm first, then send a deliberately long goal so it remains active
        # when workstation heartbeat publication stops.
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            heartbeat_pub.publish(Bool(data=True))
            rclpy.spin_once(self.node, timeout_sec=0.02)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["joint1", "joint2"]
        point = JointTrajectoryPoint()
        point.positions = [1.0, -1.0]
        point.time_from_start.sec = 10
        goal.trajectory.points = [point]

        send_future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        self.assertIsNotNone(goal_handle)
        self.assertTrue(goal_handle.accepted)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=5.0)
        self.assertTrue(result_future.done(), "guard did not cancel JTC after heartbeat loss")
        self.assertEqual(result_future.result().status, GoalStatus.STATUS_CANCELED)

        diagnostic_deadline = time.monotonic() + 3.0
        while time.monotonic() < diagnostic_deadline:
            if any(status.message == "TRAJECTORY_CANCEL_ACCEPTED" for status in diagnostics):
                break
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(
            any(status.message == "TRAJECTORY_CANCEL_ACCEPTED" for status in diagnostics),
            "guard did not publish the typed cancel diagnostic",
        )

        # Explicitly disarm the latched session before test shutdown.
        heartbeat_pub.publish(Bool(data=False))
        rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(diagnostic_sub)


@launch_testing.post_shutdown_test()
class TestProcesses(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
