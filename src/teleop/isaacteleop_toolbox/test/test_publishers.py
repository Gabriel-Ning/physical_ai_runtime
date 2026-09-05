"""Unit tests for BimanualTargetPublisher and gripper trajectory publishing."""

import json
import unittest
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from isaacteleop_toolbox.ros_publishers import (
    BimanualTargetPublisher,
)
from trajectory_msgs.msg import JointTrajectory


class _DummyPublisher:
    def __init__(self, topic, events):
        self.topic = topic
        self.events = events
        self.published = []

    def publish(self, msg):
        self.published.append(msg)
        self.events.append((self.topic, msg))


class _DummyNode:
    def __init__(self):
        self.publishers = {}
        self.events = []

    def create_publisher(self, msg_type, topic, qos):
        pub = _DummyPublisher(topic, self.events)
        self.publishers[topic] = pub
        return pub

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=10, nanosec=20)))


class PublisherTest(unittest.TestCase):
    def test_gripper_trajectory_publishing(self):
        node = _DummyNode()
        pub = BimanualTargetPublisher(
            node,
            status_topic="/teleop/status",
            left_clutch_topic="/teleop/left_clutch",
            right_clutch_topic="/teleop/right_clutch",
            left_target_frame="teleop_left_ee_target",
            right_target_frame="teleop_right_ee_target",
            left_tracked_frame="left_pika_gripper_tcp",
            right_tracked_frame="right_pika_gripper_tcp",
            profile_name="test_profile",
            pose_source="aim",
            deadman_source="squeeze",
            left_output_topic="/teleop/left_pose",
            right_output_topic="/teleop/right_pose",
            left_gripper_topic="/teleop/left_gripper",
            right_gripper_topic="/teleop/right_gripper",
            left_gripper_joint_name="left_gripper_joint1",
            right_gripper_joint_name="right_gripper_joint1",
            left_snapshot_controller_topic="/teleop/left_snap_ctrl",
            left_snapshot_ee_topic="/teleop/left_snap_ee",
            right_snapshot_controller_topic="/teleop/right_snap_ctrl",
            right_snapshot_ee_topic="/teleop/right_snap_ee",
        )

        stamp = Time(sec=1, nanosec=500)
        pub.publish(
            active=True,
            left_active=True,
            right_active=False,
            initialized=True,
            left_pose_stamped=None,
            right_pose_stamped=None,
            snapshot_seq=1,
            snapshot_stamp=stamp,
            left_gripper_pos=0.035,
            right_gripper_pos=0.012,
        )

        # Check clutch message
        left_clutch_msgs = node.publishers["/teleop/left_clutch"].published
        right_clutch_msgs = node.publishers["/teleop/right_clutch"].published
        self.assertTrue(left_clutch_msgs[0].data)
        self.assertFalse(right_clutch_msgs[0].data)
        self.assertEqual(node.events[0][0], "/teleop/left_clutch")

        # Check left gripper message
        left_msgs = node.publishers["/teleop/left_gripper"].published
        self.assertEqual(len(left_msgs), 1)
        self.assertIsInstance(left_msgs[0], JointTrajectory)
        self.assertEqual(left_msgs[0].joint_names, ["left_gripper_joint1"])
        self.assertAlmostEqual(left_msgs[0].points[0].positions[0], 0.035)
        self.assertEqual(list(left_msgs[0].points[0].velocities), [0.0])
        self.assertGreater(left_msgs[0].points[0].time_from_start.nanosec, 0)

        # Check right gripper message
        right_msgs = node.publishers["/teleop/right_gripper"].published
        self.assertEqual(len(right_msgs), 1)
        self.assertIsInstance(right_msgs[0], JointTrajectory)
        self.assertEqual(right_msgs[0].joint_names, ["right_gripper_joint1"])
        self.assertAlmostEqual(right_msgs[0].points[0].positions[0], 0.012)
        self.assertEqual(list(right_msgs[0].points[0].velocities), [0.0])
        self.assertGreater(right_msgs[0].points[0].time_from_start.nanosec, 0)

        # Check status message contains gripper topics
        status_msgs = node.publishers["/teleop/status"].published
        self.assertEqual(len(status_msgs), 1)
        status_data = json.loads(status_msgs[0].data)
        self.assertEqual(status_data["left_gripper_topic"], "/teleop/left_gripper")
        self.assertEqual(status_data["right_gripper_topic"], "/teleop/right_gripper")


if __name__ == "__main__":
    unittest.main()
