# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

import pathlib
import time
import unittest

import launch_testing
import launch_testing.actions
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from launch import LaunchDescription
from launch_ros.actions import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def generate_test_description():
    test_dir = pathlib.Path(__file__).parent
    robot_description = (test_dir / "jspc_test_system.urdf").read_text()
    controllers = str(test_dir / "jspc_test_controllers.yaml")

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[controllers],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "test_jspc",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "20",
                ],
                output="screen",
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


def diagnostic_value(status, key):
    return next((entry.value for entry in status.values if entry.key == key), None)


class TestJspcFakeHardware(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("jspc_fake_hardware_test")

    def tearDown(self):
        self.node.destroy_node()

    def spin_until(self, predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return predicate()

    def test_zero_stamp_and_timeout_enter_typed_measured_hold(self):
        statuses = []
        diagnostic_sub = self.node.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            lambda message: statuses.extend(
                status
                for status in message.status
                if diagnostic_value(status, "safety_state") is not None
            ),
            10,
        )
        publisher = self.node.create_publisher(
            JointTrajectory, "/test_jspc/reference", 1
        )
        self.assertTrue(
            self.spin_until(lambda: publisher.get_subscription_count() > 0, 20.0),
            "JSPC did not activate and subscribe to its configured reference topic",
        )

        valid = JointTrajectory()
        valid.header.stamp = self.node.get_clock().now().to_msg()
        valid.joint_names = ["joint1", "joint2"]
        point = JointTrajectoryPoint()
        point.positions = [0.2, -0.2]
        valid.points = [point]
        def publish_valid_until_tracking():
            valid.header.stamp = self.node.get_clock().now().to_msg()
            publisher.publish(valid)
            return any(
                diagnostic_value(status, "safety_state") == "TRACKING"
                for status in statuses
            )

        self.assertTrue(
            self.spin_until(publish_valid_until_tracking, 3.0),
            "valid stamped reference did not put JSPC into TRACKING",
        )

        zero_stamp = JointTrajectory()
        zero_stamp.joint_names = ["joint1", "joint2"]
        zero_stamp.points = [point]
        def publish_zero_until_rejected():
            publisher.publish(zero_stamp)
            return any(
                (
                    diagnostic_value(status, "fault_code") == "ZERO_STAMP_REFERENCE"
                    or diagnostic_value(status, "last_fault_code")
                    == "ZERO_STAMP_REFERENCE"
                )
                and diagnostic_value(status, "action") == "MEASURED_STATE_HOLD"
                for status in statuses
            )

        self.assertTrue(
            self.spin_until(publish_zero_until_rejected, 3.0),
            "zero-stamped reference was not rejected with a typed measured-hold diagnostic",
        )

        statuses.clear()
        self.assertTrue(
            self.spin_until(publish_valid_until_tracking, 3.0),
            "JSPC did not recover to TRACKING after a new valid reference",
        )
        statuses.clear()
        self.assertTrue(
            self.spin_until(
                lambda: any(
                    diagnostic_value(status, "fault_code") == "REFERENCE_TIMEOUT"
                    and diagnostic_value(status, "action") == "MEASURED_STATE_HOLD"
                    for status in statuses
                ),
                4.0,
            ),
            "reference timeout did not produce a typed measured-hold diagnostic",
        )
        self.node.destroy_subscription(diagnostic_sub)


@launch_testing.post_shutdown_test()
class TestProcesses(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
