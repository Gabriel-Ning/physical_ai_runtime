"""Quest3 bimanual absolute source node built on IsaacTeleop TeleopSession and BaseRetargeter pipeline."""

from __future__ import annotations

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from isaacteleop_toolbox.node_parameters import create_node_parameters
from isaacteleop_toolbox.retargeters import (
    BimanualAbsoluteConfig,
    BimanualAbsoluteRetargeter,
    ControllerPose,
)
from isaacteleop_toolbox.ros_publishers import BimanualTargetPublisher
from isaacteleop_toolbox.runtime import OutputMetadata, run_teleop_session_loop
from isaacteleop_toolbox.session_builders import build_controllers_session_config


class Quest3BimanualAbsoluteNode(Node):
    """Direct IsaacTeleop Quest3 source that publishes 1:1 absolute EM pose targets."""

    def __init__(self) -> None:
        super().__init__("quest3_bimanual_absolute_target")
        self._params = create_node_parameters(self, default_profile="quest3_bimanual_absolute")

        self._initialized = False

        # TF2 listener for dynamic home calibration alignment and frame transformations
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Retargeter configuration
        retarget_config = self._build_retarget_config()
        self.retargeter = BimanualAbsoluteRetargeter(
            retarget_config, on_calibrate_fn=self._get_robot_home_poses
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
            app_name="Quest3BimanualAbsoluteTeleopSource",
            mode=self._params.session_mode,
            mcap_config=self._params.mcap_config,
            retargeter=self.retargeter,
        )
        self.get_logger().info(
            "Quest3 bimanual absolute source ready: "
            f"profile={self._params.profile_name} mode={self._params.session_mode.value} "
            f"left_topic={self._params.left_output_topic} right_topic={self._params.right_output_topic} "
            f"left_gripper={self._params.left_gripper_output_topic} right_gripper={self._params.right_gripper_output_topic}"
        )

    @property
    def runtime_params(self):
        return self._params

    def _build_retarget_config(self) -> BimanualAbsoluteConfig:
        return BimanualAbsoluteConfig(
            pose_source=self._params.pose_source,
            calibrate_source=self._params.calibrate_source,
            calibrate_threshold=self._params.calibrate_threshold,
            enable_deadman=self._params.enable_deadman,
            deadman_source=self._params.deadman_source,
            deadman_threshold=self._params.deadman_threshold,
            require_both_deadman=self._params.require_both_deadman,
            linear_scale=self._params.linear_scale,
            angular_scale=self._params.angular_scale,
            enable_dynamic_scale=self._params.enable_dynamic_scale,
            scale_step=self._params.scale_step,
            min_scale=self._params.min_scale,
            max_scale=self._params.max_scale,
            filter_type=self._params.filter_type,
            one_euro_min_cutoff=self._params.one_euro_min_cutoff,
            one_euro_beta=self._params.one_euro_beta,
            one_euro_d_cutoff=self._params.one_euro_d_cutoff,
            lowpass_alpha=self._params.lowpass_alpha,
            max_linear_step_m=self._params.max_linear_step_m,
            max_angular_step_rad=self._params.max_angular_step_rad,
            openxr_to_base_rotation_xyzw=self._params.openxr_to_base_rotation_xyzw,
            left_home_xyz=self._params.left_home_xyz,
            left_home_xyzw=self._params.left_home_xyzw,
            right_home_xyz=self._params.right_home_xyz,
            right_home_xyzw=self._params.right_home_xyzw,
            use_live_ee_as_home=self._params.use_live_ee_as_home,
            gripper_min_width=self._params.gripper_min_width,
            gripper_max_width=self._params.gripper_max_width,
            gripper_speed_mps=self._params.gripper_speed_mps,
        )

    def run(self) -> int:
        return run_teleop_session_loop(self, self.session_config, self._publish_step)

    def _get_robot_home_poses(
        self,
    ) -> tuple[ControllerPose | None, ControllerPose | None, bool, str]:
        if not self._params.left_tcp_frame and not self._params.right_tcp_frame:
            return None, None, True, "static_yaml_home"

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
                f"Absolute calibration: successfully aligned home to feedback TCPs: "
                f"L={self._params.left_tcp_frame}, R={self._params.right_tcp_frame} (in {self._params.output_frame})"
            )
            return left_pose, right_pose, True, ""
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f"Dynamic home lookup failed ({e}), falling back to static config home.",
                throttle_duration_sec=2.0,
            )
            return None, None, True, "fallback_static_home"

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

        left_ee = np.from_dlpack(session_result["left_ee_pose"][0])
        right_ee = np.from_dlpack(session_result["right_ee_pose"][0])

        left_has_target = np.any(left_ee != 0)
        right_has_target = np.any(right_ee != 0)

        # Independent Left Arm target publication
        if left_has_target and self._params.left_output_topic and self._params.left_base_frame:
            left_pos = left_ee[:3]
            left_rot = Rotation.from_quat(left_ee[3:7])
            left_pose_stamped = self._transform_target_pose(
                left_pos, left_rot, self._params.left_base_frame, stamp
            )

        # Independent Right Arm target publication
        if right_has_target and self._params.right_output_topic and self._params.right_base_frame:
            right_pos = right_ee[:3]
            right_rot = Rotation.from_quat(right_ee[3:7])
            right_pose_stamped = self._transform_target_pose(
                right_pos, right_rot, self._params.right_base_frame, stamp
            )

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

        reason = self.retargeter.last_status_reason
        if not reason:
            reason = "active" if active else "inactive"

        self.publisher.publish(
            active=active,
            left_active=left_active,
            right_active=right_active,
            initialized=self._initialized,
            left_pose_stamped=left_pose_stamped,
            right_pose_stamped=right_pose_stamped,
            snapshot_seq=snapshot.seq,
            reason=reason,
            metadata=metadata,
            left_gripper_pos=left_gripper_pos,
            right_gripper_pos=right_gripper_pos,
        )


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = Quest3BimanualAbsoluteNode()
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
