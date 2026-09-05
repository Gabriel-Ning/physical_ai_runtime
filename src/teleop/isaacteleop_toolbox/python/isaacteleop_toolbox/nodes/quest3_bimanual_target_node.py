"""Quest3 bimanual source node built on IsaacTeleop TeleopSession and BaseRetargeter pipeline."""

from __future__ import annotations

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from isaacteleop_toolbox.node_parameters import create_node_parameters
from isaacteleop_toolbox.retargeters import (
    BimanualRelativeConfig,
    BimanualRelativeRetargeter,
    ControllerPose,
)
from isaacteleop_toolbox.ros_publishers import BimanualTargetPublisher
from isaacteleop_toolbox.runtime import OutputMetadata, run_teleop_session_loop
from isaacteleop_toolbox.session_builders import build_controllers_session_config


class Quest3BimanualTargetNode(Node):
    """Direct IsaacTeleop Quest3 source that publishes EM pose targets."""

    def __init__(self) -> None:
        super().__init__("quest3_bimanual_target")
        self._params = create_node_parameters(self)

        self._initialized = False

        # TF2 listener for dynamic alignment and frame transformation
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Retargeter configuration
        retarget_config = self._build_retarget_config()
        self.retargeter = BimanualRelativeRetargeter(
            retarget_config, on_activate_fn=self._get_robot_feedback_poses
        )
        self.publisher = BimanualTargetPublisher(
            self,
            status_topic=self._params.status_topic,
            left_clutch_topic=self._params.left_clutch_topic,
            right_clutch_topic=self._params.right_clutch_topic,
            left_target_frame=self._params.left_target_frame,
            right_target_frame=self._params.right_target_frame,
            left_tracked_frame=self._params.left_tcp_frame,
            right_tracked_frame=self._params.right_tcp_frame,
            profile_name=self._params.profile_name,
            pose_source=self._params.pose_source,
            deadman_source=self._params.deadman_source,
            left_output_topic=self._params.left_output_topic,
            right_output_topic=self._params.right_output_topic,
            left_gripper_topic=self._params.left_gripper_output_topic,
            right_gripper_topic=self._params.right_gripper_output_topic,
            left_gripper_joint_name=self._params.left_gripper_joint_name,
            right_gripper_joint_name=self._params.right_gripper_joint_name,
            left_snapshot_controller_topic=self._params.left_snapshot_controller_topic,
            left_snapshot_ee_topic=self._params.left_snapshot_ee_topic,
            right_snapshot_controller_topic=self._params.right_snapshot_controller_topic,
            right_snapshot_ee_topic=self._params.right_snapshot_ee_topic,
        )
        self.session_config = build_controllers_session_config(
            app_name="Quest3BimanualTeleopSource",
            mode=self._params.session_mode,
            mcap_config=self._params.mcap_config,
            retargeter=self.retargeter,
        )
        self.get_logger().info(
            "Quest3 bimanual source ready: "
            f"profile={self._params.profile_name} mode={self._params.session_mode.value} "
            f"left_topic={self._params.left_output_topic} right_topic={self._params.right_output_topic} "
            f"left_gripper={self._params.left_gripper_output_topic} right_gripper={self._params.right_gripper_output_topic}"
        )

    @property
    def runtime_params(self):
        # Compatibility with runtime.py run_teleop_session_loop
        return self._params

    def _build_retarget_config(self) -> BimanualRelativeConfig:
        return BimanualRelativeConfig(
            pose_source=self._params.pose_source,
            deadman_source=self._params.deadman_source,
            deadman_threshold=self._params.deadman_threshold,
            require_both_deadman=self._params.require_both_deadman,
            linear_scale=self._params.linear_scale,
            angular_scale=self._params.angular_scale,
            lowpass_alpha=self._params.lowpass_alpha,
            max_linear_step_m=self._params.max_linear_step_m,
            max_angular_step_rad=self._params.max_angular_step_rad,
            openxr_to_base_rotation_xyzw=self._params.openxr_to_base_rotation_xyzw,
            gripper_min_width=self._params.gripper_min_width,
            gripper_max_width=self._params.gripper_max_width,
            gripper_speed_mps=self._params.gripper_speed_mps,
            require_clutch_for_gripper=self._params.require_clutch_for_gripper,
        )

    def run(self) -> int:
        return run_teleop_session_loop(self, self.session_config, self._publish_step)

    def _get_robot_feedback_poses(
        self,
    ) -> tuple[ControllerPose | None, ControllerPose | None, bool, str]:
        if not self._params.left_tcp_frame or not self._params.right_tcp_frame:
            self.get_logger().warn(
                "Refusing clutch activation: both left_tcp_frame and right_tcp_frame "
                "are required for relative bimanual feedback.",
                throttle_duration_sec=1.0,
            )
            return None, None, False, "no_tcp_feedback"

        left_pose = None
        right_pose = None

        try:
            if self._params.left_tcp_frame:
                left_tf = self._tf_buffer.lookup_transform(
                    self._params.output_frame,
                    self._params.left_tcp_frame,
                    rclpy.time.Time(),
                )
                left_pose = ControllerPose(
                    position=np.array(
                        [
                            left_tf.transform.translation.x,
                            left_tf.transform.translation.y,
                            left_tf.transform.translation.z,
                        ],
                        dtype=float,
                    ),
                    rotation=Rotation.from_quat(
                        [
                            left_tf.transform.rotation.x,
                            left_tf.transform.rotation.y,
                            left_tf.transform.rotation.z,
                            left_tf.transform.rotation.w,
                        ]
                    ),
                )

            if self._params.right_tcp_frame:
                right_tf = self._tf_buffer.lookup_transform(
                    self._params.output_frame,
                    self._params.right_tcp_frame,
                    rclpy.time.Time(),
                )
                right_pose = ControllerPose(
                    position=np.array(
                        [
                            right_tf.transform.translation.x,
                            right_tf.transform.translation.y,
                            right_tf.transform.translation.z,
                        ],
                        dtype=float,
                    ),
                    rotation=Rotation.from_quat(
                        [
                            right_tf.transform.rotation.x,
                            right_tf.transform.rotation.y,
                            right_tf.transform.rotation.z,
                            right_tf.transform.rotation.w,
                        ]
                    ),
                )

            self.get_logger().info(
                f"Dynamic alignment: successfully aligned targets to feedback: "
                f"L={self._params.left_tcp_frame}, R={self._params.right_tcp_frame} (relative to {self._params.output_frame})"
            )
            return left_pose, right_pose, True, ""
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f"Refusing clutch activation: failed to lookup robot feedback poses ({e}). "
                "Check TF tree.",
                throttle_duration_sec=1.0,
            )
            return None, None, False, "no_tcp_feedback"

    def _transform_target_pose(
        self,
        pos: np.ndarray,
        rot: Rotation,
        target_base_frame: str,
        stamp,
    ) -> PoseStamped | None:
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = stamp
        pose_stamped.header.frame_id = target_base_frame

        # Identity optimization: if target_base_frame matches output_frame, no TF lookup needed!
        if target_base_frame == self._params.output_frame:
            pose_stamped.pose.position.x = float(pos[0])
            pose_stamped.pose.position.y = float(pos[1])
            pose_stamped.pose.position.z = float(pos[2])
            qx, qy, qz, qw = rot.as_quat()
            pose_stamped.pose.orientation.x = float(qx)
            pose_stamped.pose.orientation.y = float(qy)
            pose_stamped.pose.orientation.z = float(qz)
            pose_stamped.pose.orientation.w = float(qw)
            return pose_stamped

        try:
            trans = self._tf_buffer.lookup_transform(
                target_base_frame,
                self._params.output_frame,
                rclpy.time.Time(),
            )
            t = trans.transform.translation
            r = trans.transform.rotation
            T_rot = Rotation.from_quat([r.x, r.y, r.z, r.w])
            T_trans = np.array([t.x, t.y, t.z])

            new_pos = T_rot.apply(pos) + T_trans
            new_rot = T_rot * rot
            qx, qy, qz, qw = new_rot.as_quat()

            pose_stamped.pose.position.x = float(new_pos[0])
            pose_stamped.pose.position.y = float(new_pos[1])
            pose_stamped.pose.position.z = float(new_pos[2])
            pose_stamped.pose.orientation.x = float(qx)
            pose_stamped.pose.orientation.y = float(qy)
            pose_stamped.pose.orientation.z = float(qz)
            pose_stamped.pose.orientation.w = float(qw)
            return pose_stamped
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f"Failed to transform target pose to {target_base_frame}: {e}",
                throttle_duration_sec=1.0,
            )
            return None

    def _publish_step(
        self, session_result: dict, stamp, metadata: OutputMetadata
    ) -> None:
        active_tensor = session_result["active"][0]
        active_val = int(np.from_dlpack(active_tensor)[0])
        active = active_val > 0
        left_active = int(np.from_dlpack(session_result["left_active"][0])[0]) > 0
        right_active = int(np.from_dlpack(session_result["right_active"][0])[0]) > 0

        left_pose_stamped = None
        right_pose_stamped = None

        if active:
            # Extract 7D target pose tensors [x, y, z, qx, qy, qz, qw]
            left_ee = np.from_dlpack(session_result["left_ee_pose"][0])
            right_ee = np.from_dlpack(session_result["right_ee_pose"][0])

            left_has_target = np.any(left_ee != 0)
            right_has_target = np.any(right_ee != 0)

            # 1. Transform Left Arm target
            if left_has_target and self._params.left_output_topic and self._params.left_base_frame:
                left_pos = left_ee[:3]
                left_rot = Rotation.from_quat(left_ee[3:7])
                left_pose_stamped = self._transform_target_pose(
                    left_pos, left_rot, self._params.left_base_frame, stamp
                )

            # 2. Transform Right Arm target
            if right_has_target and self._params.right_output_topic and self._params.right_base_frame:
                right_pos = right_ee[:3]
                right_rot = Rotation.from_quat(right_ee[3:7])
                right_pose_stamped = self._transform_target_pose(
                    right_pos, right_rot, self._params.right_base_frame, stamp
                )

            # All-or-Nothing publication only when require_both_deadman is explicitly enabled
            if self._params.require_both_deadman:
                needs_left = bool(self._params.left_output_topic and self._params.left_base_frame)
                needs_right = bool(self._params.right_output_topic and self._params.right_base_frame)
                if (needs_left and left_pose_stamped is None) or (needs_right and right_pose_stamped is None):
                    left_pose_stamped = None
                    right_pose_stamped = None

        left_gripper_pos = None
        right_gripper_pos = None
        if session_result.get("left_gripper"):
            left_gripper_pos = float(
                np.from_dlpack(session_result["left_gripper"][0])[0]
            )
        if session_result.get("right_gripper"):
            right_gripper_pos = float(
                np.from_dlpack(session_result["right_gripper"][0])[0]
            )

        snapshot = self.retargeter.snapshot
        if snapshot.seq > 0:
            self._initialized = True

        if active:
            reason = ""
        elif self.retargeter.last_reject_reason:
            reason = self.retargeter.last_reject_reason
        elif self._initialized:
            reason = "clutch_released"
        else:
            reason = "waiting_for_clutch"

        self.publisher.publish(
            active=active,
            left_active=left_active,
            right_active=right_active,
            initialized=self._initialized,
            left_pose_stamped=left_pose_stamped,
            right_pose_stamped=right_pose_stamped,
            snapshot_seq=snapshot.seq,
            reason=reason,
            snapshot=snapshot,
            snapshot_stamp=stamp,
            snapshot_frame_id=self._params.output_frame,
            metadata=metadata,
            left_gripper_pos=left_gripper_pos,
            right_gripper_pos=right_gripper_pos,
        )


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = Quest3BimanualTargetNode()
        return node.run()
    except KeyboardInterrupt:
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
