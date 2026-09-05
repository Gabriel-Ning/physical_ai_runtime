"""ROS publishers for workspace teleop source contracts without backward-compatibility code."""

from __future__ import annotations

import json

from geometry_msgs.msg import PoseStamped, TransformStamped
from moveit_msgs.msg import CartesianPoint, CartesianTrajectory, CartesianTrajectoryPoint
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .retargeters import BimanualSnapshot, ControllerPose
from .runtime import OutputMetadata

STATUS_SCHEMA_VERSION = 1


def _controller_pose_to_msg(stamp, frame_id: str, pose: ControllerPose) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(pose.position[0])
    msg.pose.position.y = float(pose.position[1])
    msg.pose.position.z = float(pose.position[2])
    qx, qy, qz, qw = pose.rotation.as_quat()
    msg.pose.orientation.x = float(qx)
    msg.pose.orientation.y = float(qy)
    msg.pose.orientation.z = float(qz)
    msg.pose.orientation.w = float(qw)
    return msg


def _make_gripper_msg(
    stamp, joint_name: str, position_m: float, dt_s: float = 0.033
) -> JointTrajectory:
    msg = JointTrajectory()
    msg.header.stamp = stamp
    msg.joint_names = [joint_name]
    point = JointTrajectoryPoint()
    point.positions = [float(position_m)]
    point.velocities = [0.0]
    # Set positive duration (dt_s, ~33ms) for downstream JointTrajectoryController compliance
    point.time_from_start.sec = int(dt_s)
    point.time_from_start.nanosec = int((dt_s - int(dt_s)) * 1e9)
    msg.points = [point]
    return msg


def _make_pose_reference(
    pose: PoseStamped, tracked_frame: str, dt_s: float = 0.033
) -> CartesianTrajectory:
    """Build the size-one absolute Cartesian target consumed by EM."""
    msg = CartesianTrajectory()
    msg.header = pose.header
    msg.tracked_frame = tracked_frame
    point = CartesianTrajectoryPoint()
    point.point = CartesianPoint()
    point.point.pose = pose.pose
    point.time_from_start.sec = int(dt_s)
    point.time_from_start.nanosec = int((dt_s - int(dt_s)) * 1e9)
    msg.points = [point]
    return msg


class BimanualTargetPublisher:
    """Publish EM pose targets, status, target TFs, and clutch snapshots."""

    def __init__(
        self,
        node,
        *,
        status_topic: str,
        left_clutch_topic: str = "",
        right_clutch_topic: str = "",
        left_target_frame: str,
        right_target_frame: str,
        left_tracked_frame: str,
        right_tracked_frame: str,
        profile_name: str,
        pose_source: str,
        deadman_source: str,
        left_output_topic: str,
        right_output_topic: str,
        left_gripper_topic: str = "",
        right_gripper_topic: str = "",
        left_gripper_joint_name: str = "left_gripper_joint1",
        right_gripper_joint_name: str = "right_gripper_joint1",
        left_snapshot_controller_topic: str,
        left_snapshot_ee_topic: str,
        right_snapshot_controller_topic: str,
        right_snapshot_ee_topic: str,
    ) -> None:
        self._node = node
        self._left_target_frame = left_target_frame
        self._right_target_frame = right_target_frame
        self._left_tracked_frame = left_tracked_frame
        self._right_tracked_frame = right_tracked_frame
        self._profile_name = profile_name
        self._pose_source = pose_source
        self._deadman_source = deadman_source

        self._pub_status = node.create_publisher(String, status_topic, 10)
        self._pub_left_clutch = (
            node.create_publisher(Bool, left_clutch_topic, 10) if left_clutch_topic else None
        )
        self._pub_right_clutch = (
            node.create_publisher(Bool, right_clutch_topic, 10) if right_clutch_topic else None
        )
        self._pub_left_target = (
            node.create_publisher(CartesianTrajectory, left_output_topic, 10)
            if left_output_topic
            else None
        )
        self._pub_right_target = (
            node.create_publisher(CartesianTrajectory, right_output_topic, 10)
            if right_output_topic
            else None
        )
        self._pub_left_gripper = (
            node.create_publisher(JointTrajectory, left_gripper_topic, 10)
            if left_gripper_topic
            else None
        )
        self._pub_right_gripper = (
            node.create_publisher(JointTrajectory, right_gripper_topic, 10)
            if right_gripper_topic
            else None
        )
        self._pub_left_snapshot_controller = node.create_publisher(
            PoseStamped, left_snapshot_controller_topic, 10
        )
        self._pub_left_snapshot_ee = node.create_publisher(
            PoseStamped, left_snapshot_ee_topic, 10
        )
        self._pub_right_snapshot_controller = node.create_publisher(
            PoseStamped, right_snapshot_controller_topic, 10
        )
        self._pub_right_snapshot_ee = node.create_publisher(
            PoseStamped, right_snapshot_ee_topic, 10
        )
        self._tf_broadcaster = TransformBroadcaster(node)

        self.left_output_topic = left_output_topic
        self.right_output_topic = right_output_topic
        self.left_gripper_topic = left_gripper_topic
        self.right_gripper_topic = right_gripper_topic
        self.left_gripper_joint_name = left_gripper_joint_name
        self.right_gripper_joint_name = right_gripper_joint_name

    def publish(
        self,
        active: bool,
        left_active: bool,
        right_active: bool,
        initialized: bool,
        left_pose_stamped: PoseStamped | None,
        right_pose_stamped: PoseStamped | None,
        snapshot_seq: int,
        reason: str = "",
        snapshot: BimanualSnapshot | None = None,
        snapshot_stamp=None,
        snapshot_frame_id: str = "",
        metadata: OutputMetadata | None = None,
        left_gripper_pos: float | None = None,
        right_gripper_pos: float | None = None,
    ) -> None:
        # Activate the external EM source before publishing its first reference.
        # Publishing the target first creates a handover window in which EM can
        # discard that target while the source is still inactive, then switch
        # to TSKPC without a valid command until the next teleop cycle.
        if self._pub_left_clutch is not None:
            self._pub_left_clutch.publish(Bool(data=bool(left_active)))
        if self._pub_right_clutch is not None:
            self._pub_right_clutch.publish(Bool(data=bool(right_active)))

        # Publish size-one Cartesian trajectories while retaining target TFs for RViz.
        if left_pose_stamped is not None and self._pub_left_target is not None:
            self._pub_left_target.publish(
                _make_pose_reference(left_pose_stamped, self._left_tracked_frame)
            )
            self._tf_broadcaster.sendTransform(
                self._make_tf(
                    left_pose_stamped.header.stamp,
                    left_pose_stamped.header.frame_id,
                    self._left_target_frame,
                    left_pose_stamped.pose,
                )
            )
        if right_pose_stamped is not None and self._pub_right_target is not None:
            self._pub_right_target.publish(
                _make_pose_reference(right_pose_stamped, self._right_tracked_frame)
            )
            self._tf_broadcaster.sendTransform(
                self._make_tf(
                    right_pose_stamped.header.stamp,
                    right_pose_stamped.header.frame_id,
                    self._right_target_frame,
                    right_pose_stamped.pose,
                )
            )

        stamp = (
            snapshot_stamp
            if snapshot_stamp is not None
            else self._node.get_clock().now().to_msg()
        )
        if self._pub_left_gripper is not None and left_gripper_pos is not None:
            self._pub_left_gripper.publish(
                _make_gripper_msg(stamp, self.left_gripper_joint_name, left_gripper_pos)
            )
        if self._pub_right_gripper is not None and right_gripper_pos is not None:
            self._pub_right_gripper.publish(
                _make_gripper_msg(stamp, self.right_gripper_joint_name, right_gripper_pos)
            )

        if active and snapshot is not None and snapshot_stamp is not None:
            self._publish_snapshot(snapshot, snapshot_stamp, snapshot_frame_id)

        self._publish_status(
            active,
            initialized,
            snapshot_seq,
            reason,
            metadata=metadata,
        )

    def _publish_snapshot(
        self, snapshot: BimanualSnapshot, stamp, frame_id: str
    ) -> None:
        """Publish the clutch snapshot latched at the last deadman-press.

        Held constant (same values) every cycle for the duration of one
        deadman-press episode, so an episode recorder started anywhere within
        that window observes a fresh message on each snapshot topic.
        """
        if snapshot.left_controller is not None:
            self._pub_left_snapshot_controller.publish(
                _controller_pose_to_msg(stamp, frame_id, snapshot.left_controller)
            )
        if snapshot.left_ee is not None:
            self._pub_left_snapshot_ee.publish(
                _controller_pose_to_msg(stamp, frame_id, snapshot.left_ee)
            )
        if snapshot.right_controller is not None:
            self._pub_right_snapshot_controller.publish(
                _controller_pose_to_msg(stamp, frame_id, snapshot.right_controller)
            )
        if snapshot.right_ee is not None:
            self._pub_right_snapshot_ee.publish(
                _controller_pose_to_msg(stamp, frame_id, snapshot.right_ee)
            )

    def _make_tf(
        self, stamp, parent_frame: str, child_frame: str, pose
    ) -> TransformStamped:
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent_frame
        msg.child_frame_id = child_frame
        msg.transform.translation.x = float(pose.position.x)
        msg.transform.translation.y = float(pose.position.y)
        msg.transform.translation.z = float(pose.position.z)
        msg.transform.rotation.x = float(pose.orientation.x)
        msg.transform.rotation.y = float(pose.orientation.y)
        msg.transform.rotation.z = float(pose.orientation.z)
        msg.transform.rotation.w = float(pose.orientation.w)
        return msg

    def _publish_status(
        self,
        active: bool,
        initialized: bool,
        snapshot_seq: int,
        reason: str,
        *,
        metadata: OutputMetadata | None,
    ) -> None:
        msg = String()
        payload = {
            "schema_version": STATUS_SCHEMA_VERSION,
            "active": active,
            "initialized": initialized,
            "profile": self._profile_name,
            "mode": "relative",
            "pose_source": self._pose_source,
            "deadman_source": self._deadman_source,
            "left_output_topic": self.left_output_topic,
            "right_output_topic": self.right_output_topic,
            "left_gripper_topic": self.left_gripper_topic,
            "right_gripper_topic": self.right_gripper_topic,
            "snapshot_seq": snapshot_seq,
            "reason": reason,
        }
        if metadata is not None:
            payload.update(metadata.as_dict())
        msg.data = json.dumps(
            payload,
            separators=(",", ":"),
        )
        self._pub_status.publish(msg)
