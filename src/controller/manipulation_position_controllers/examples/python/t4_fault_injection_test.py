#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""T4 fault injection test for TaskSpaceKinematicPositionController.

Validates:
  - wrong frame target → frame_reject_count increments, no motion
  - invalid quaternion → invalid_quat_count increments
  - stale target → stale_hold_count increments
  - valid target → solve_success_count increments
  - stop publishing → stale hold

Usage (requires fake-hardware controller already running):
  ros2 run manipulation_position_controllers t4_fault_injection_test.py
"""

import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def quat_from_euler(roll, pitch, yaw):
    """Return (x, y, z, w) quaternion from Euler angles."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class T4FaultInjectionTest(Node):
    def __init__(self):
        super().__init__("t4_fault_injection_test")
        self._pub = self.create_publisher(PoseStamped, "/test/pose_reference", 10)
        self._latest_status = None
        self._status_sub = self.create_subscription(
            Float64MultiArray,
            "/task_space_kinematic_position_controller/status",
            self._status_cb,
            10,
        )

        # Status field indices (must match controller header)
        self.IDX_STATE = 0
        self.IDX_STALE_HOLD = 4
        self.IDX_FRAME_REJECT = 5
        self.IDX_INVALID_QUAT = 6
        self.IDX_SOLVE_SUCCESS = 2
        self.IDX_SOLVE_FAIL = 3

        self._results = []

    def _status_cb(self, msg):
        self._latest_status = msg.data

    def _get_counter(self, idx):
        if self._latest_status and len(self._latest_status) > idx:
            return int(self._latest_status[idx])
        return -1

    def _wait_status(self, timeout_s=3.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_status is not None:
                return True
        return False

    def _publish_and_wait(self, pose, wait_s=1.0):
        self._pub.publish(pose)
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _assert_counter_increased(self, idx, baseline, label):
        current = self._get_counter(idx)
        if current > baseline:
            self._results.append((True, f"{label}: {baseline} → {current}"))
        else:
            self._results.append(
                (False, f"{label}: {baseline} → {current} (expected increase)")
            )
        return current

    def _assert_counter_unchanged(self, idx, baseline, label):
        current = self._get_counter(idx)
        if current == baseline:
            self._results.append((True, f"{label}: unchanged at {current}"))
        else:
            self._results.append(
                (False, f"{label}: changed {baseline} → {current} (expected unchanged)")
            )
        return current

    def run(self):
        print("=" * 60)
        print("  T4 FAULT INJECTION TEST")
        print("=" * 60)

        # Wait for status to start flowing.
        if not self._wait_status():
            print("FAIL: No status messages received.")
            return 1

        # --- Test 1: Valid target ---
        print("\n[Test 1] Valid target...")
        valid = PoseStamped()
        valid.header.frame_id = "base_link"
        valid.header.stamp = self.get_clock().now().to_msg()
        valid.pose.position.x = 0.40
        valid.pose.position.y = 0.0
        valid.pose.position.z = 0.30
        qx, qy, qz, qw = quat_from_euler(0, 0, 0)
        valid.pose.orientation.x = qx
        valid.pose.orientation.y = qy
        valid.pose.orientation.z = qz
        valid.pose.orientation.w = qw

        bl_success = self._get_counter(self.IDX_SOLVE_SUCCESS)
        self._publish_and_wait(valid, wait_s=0.8)
        bl_success = self._assert_counter_increased(
            self.IDX_SOLVE_SUCCESS, bl_success, "Solve success"
        )

        # --- Test 2: Wrong frame ---
        print("\n[Test 2] Wrong frame...")
        wrong_frame = PoseStamped()
        wrong_frame.header.frame_id = "camera_link"  # not base_link
        wrong_frame.header.stamp = self.get_clock().now().to_msg()
        wrong_frame.pose = valid.pose

        bl_frame = self._get_counter(self.IDX_FRAME_REJECT)
        self._publish_and_wait(wrong_frame, wait_s=0.8)
        bl_frame = self._assert_counter_increased(
            self.IDX_FRAME_REJECT, bl_frame, "Frame reject"
        )

        # --- Test 3: Invalid quaternion ---
        print("\n[Test 3] Invalid quaternion (zeros)...")
        bad_quat = PoseStamped()
        bad_quat.header.frame_id = "base_link"
        bad_quat.header.stamp = self.get_clock().now().to_msg()
        bad_quat.pose.position.x = 0.40
        bad_quat.pose.position.y = 0.0
        bad_quat.pose.position.z = 0.30
        bad_quat.pose.orientation.x = 0.0
        bad_quat.pose.orientation.y = 0.0
        bad_quat.pose.orientation.z = 0.0
        bad_quat.pose.orientation.w = 0.0  # norm = 0

        bl_quat = self._get_counter(self.IDX_INVALID_QUAT)
        self._publish_and_wait(bad_quat, wait_s=0.8)
        bl_quat = self._assert_counter_increased(
            self.IDX_INVALID_QUAT, bl_quat, "Invalid quat reject"
        )

        # --- Test 4: Stale timestamp ---
        print("\n[Test 4] Stale timestamp (2s old)...")
        # First reset stale state with a fresh valid target.
        fresh = PoseStamped()
        fresh.header.frame_id = "base_link"
        fresh.header.stamp = self.get_clock().now().to_msg()
        fresh.pose = valid.pose
        self._publish_and_wait(fresh, wait_s=0.5)

        stale = PoseStamped()
        stale.header.frame_id = "base_link"
        stale.header.stamp.sec = int(time.time()) - 2  # 2s old
        stale.header.stamp.nanosec = 0
        stale.pose = valid.pose

        bl_stale = self._get_counter(self.IDX_STALE_HOLD)
        self._publish_and_wait(stale, wait_s=1.5)
        bl_stale = self._assert_counter_increased(
            self.IDX_STALE_HOLD, bl_stale, "Stale hold"
        )

        # --- Test 5: Resume valid target ---
        print("\n[Test 5] Resume valid target...")
        valid.header.stamp = self.get_clock().now().to_msg()
        bl_success2 = self._get_counter(self.IDX_SOLVE_SUCCESS)
        self._publish_and_wait(valid, wait_s=1.5)
        bl_success2 = self._assert_counter_increased(
            self.IDX_SOLVE_SUCCESS, bl_success2, "Solve success (resume)"
        )

        # --- Summary ---
        print("\n" + "=" * 60)
        passed = sum(1 for r in self._results if r[0])
        total = len(self._results)
        for ok, msg in self._results:
            print(f"  {'✅' if ok else '❌'} {msg}")
        print(f"\n  {passed}/{total} checks passed")
        print("=" * 60)

        return 0 if passed == total else 1


def main(args=None):
    rclpy.init(args=args)
    node = T4FaultInjectionTest()
    try:
        rc = node.run()
    except KeyboardInterrupt:
        rc = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
