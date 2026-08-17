#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

import sys
import threading
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
from transforms3d.quaternions import mat2quat


class CharucoDetectorNode(Node):
    def __init__(self):
        super().__init__("charuco_detector_node")
        self.get_logger().info("Initializing CharucoDetectorNode...")

        # Parameters
        self.declare_parameter(
            "image_topic", "/observation/hand_realsense/color/image_raw"
        )
        self.declare_parameter(
            "camera_info_topic", "/observation/hand_realsense/color/camera_info"
        )
        self.declare_parameter("inner_corners_x", 13)
        self.declare_parameter("inner_corners_y", 8)
        self.declare_parameter("square_length", 0.02)  # meters (e.g. 0.02m = 20mm)
        self.declare_parameter("marker_length", 0.015)  # meters (e.g. 0.015m = 15mm)
        self.declare_parameter("dictionary_id", cv2.aruco.DICT_5X5_250)
        self.declare_parameter("target_frame_id", "charuco_board")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "debug_image")
        self.declare_parameter("log_detection_changes", True)
        self.declare_parameter("swap_board_axes", False)

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        swap_axes = bool(self.get_parameter("swap_board_axes").value)
        inner_x = int(self.get_parameter("inner_corners_x").value)
        inner_y = int(self.get_parameter("inner_corners_y").value)
        if swap_axes:
            inner_x, inner_y = inner_y, inner_x
        self.inner_corners_x = inner_x
        self.inner_corners_y = inner_y
        self.squares_x = self.inner_corners_x + 1
        self.squares_y = self.inner_corners_y + 1
        self.square_length = float(self.get_parameter("square_length").value)
        self.marker_length = float(self.get_parameter("marker_length").value)
        self.dict_id = int(self.get_parameter("dictionary_id").value)
        self.target_frame_id = str(self.get_parameter("target_frame_id").value)
        self.pub_debug = bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.log_detection_changes = bool(
            self.get_parameter("log_detection_changes").value
        )
        self._last_status = None

        # OpenCV ArUco/Charuco setup
        self.dictionary = cv2.aruco.getPredefinedDictionary(self.dict_id)
        self.board = cv2.aruco.CharucoBoard(
            (self.squares_x, self.squares_y),
            self.square_length,
            self.marker_length,
            self.dictionary,
        )
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector_params.minMarkerPerimeterRate = 0.02
        self.detector_params.maxMarkerPerimeterRate = 4.0
        self.charuco_detector = cv2.aruco.CharucoDetector(
            self.board, cv2.aruco.CharucoParameters(), self.detector_params
        )
        self._min_charuco_corners_pnp = 6

        # ROS 2 Interfaces
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.lock = threading.Lock()

        # Subscriptions
        self.info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.info_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data
        )

        if self.pub_debug:
            self.debug_pub = self.create_publisher(
                Image,
                self.debug_image_topic,
                qos_profile_sensor_data,
            )

        self.get_logger().info(
            f"CharucoDetectorNode ready. Board: {self.squares_x}x{self.squares_y} "
            f"(square={self.square_length}m, marker={self.marker_length}m), topic={self.image_topic}"
        )

    def info_callback(self, msg: CameraInfo):
        with self.lock:
            if self.camera_matrix is None:
                self.camera_matrix = np.array(msg.k).reshape((3, 3))
                self.dist_coeffs = np.array(msg.d)
                self.get_logger().info("Camera intrinsic parameters loaded.")

    def image_callback(self, msg: Image):
        try:
            self._process_image(msg)
        except cv2.error as exc:
            self.get_logger().error(
                f"OpenCV error in detection: {exc}",
                throttle_duration_sec=2.0,
            )

    def _process_image(self, msg: Image):
        with self.lock:
            if self.camera_matrix is None:
                self.get_logger().warning(
                    "Waiting for CameraInfo...", throttle_duration_sec=2.0
                )
                return
            K = self.camera_matrix.copy()
            D = self.dist_coeffs.copy()

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge conversion failed: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        status = "NO DETECTION"
        detail = ["ArUco markers: 0"]
        color = (0, 0, 255)

        charuco_corners, charuco_ids, marker_corners, marker_ids = (
            self.charuco_detector.detectBoard(gray)
        )
        n_markers = 0 if marker_ids is None else len(marker_ids)
        n_corners = 0 if charuco_corners is None else len(charuco_corners)

        if n_markers > 0 and self.pub_debug and marker_corners is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, marker_corners, marker_ids)

        if n_markers > 0 or n_corners > 0:
            detail = [f"ArUco markers: {n_markers}", f"Charuco corners: {n_corners}"]

            if charuco_corners is not None and n_corners >= 4 and self.pub_debug:
                cv2.aruco.drawDetectedCornersCharuco(
                    cv_image, charuco_corners, charuco_ids
                )

            if (
                charuco_corners is not None
                and n_corners >= self._min_charuco_corners_pnp
            ):
                try:
                    retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                        charuco_corners,
                        charuco_ids,
                        self.board,
                        K,
                        D,
                        None,
                        None,
                    )
                except cv2.error as exc:
                    self.get_logger().warning(
                        f"estimatePoseCharucoBoard failed: {exc}",
                        throttle_duration_sec=2.0,
                    )
                    retval = False
                    rvec, tvec = None, None

                if retval and rvec is not None and tvec is not None:
                    status = "DETECTED (pose OK)"
                    color = (0, 255, 0)
                    detail.append(f"TF {self.target_frame_id} publishing")
                    if self.pub_debug:
                        cv2.drawFrameAxes(cv_image, K, D, rvec, tvec, 0.1)
                    self.publish_tf(msg.header, rvec.flatten(), tvec.flatten())
                else:
                    status = "PARTIAL (pose failed)"
                    color = (0, 200, 255)
            else:
                status = f"PARTIAL (need >= {self._min_charuco_corners_pnp} corners)"
                color = (0, 200, 255)

        if self.pub_debug:
            self._draw_status(cv_image, status, detail, color)
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            except CvBridgeError as e:
                self.get_logger().error(f"CvBridge debug conversion failed: {e}")

        if self.log_detection_changes and status != self._last_status:
            self.get_logger().info(f"Board status: {status} — {detail}")
            self._last_status = status

    def _draw_status(self, image, status, detail_lines, color_bgr):
        lines = [status] + detail_lines
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.6
        thickness = 2
        line_h = 24
        pad = 6
        max_w = 0
        for line in lines:
            (tw, _), _ = cv2.getTextSize(line, font, scale, thickness)
            max_w = max(max_w, tw)
        box_h = pad * 2 + line_h * len(lines)
        box_w = max_w + pad * 2
        cv2.rectangle(image, (0, 0), (box_w, box_h), (0, 0, 0), -1)
        cv2.rectangle(image, (0, 0), (box_w, box_h), color_bgr, 2)
        for i, line in enumerate(lines):
            y = pad + (i + 1) * line_h - 4
            cv2.putText(
                image, line, (pad, y), font, scale, color_bgr, thickness, cv2.LINE_AA
            )

    def publish_tf(self, header, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        quat = mat2quat(R)  # [w, x, y, z]

        t = TransformStamped()
        t.header.stamp = header.stamp
        t.header.frame_id = header.frame_id
        t.child_frame_id = self.target_frame_id

        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])

        t.transform.rotation.w = float(quat[0])
        t.transform.rotation.x = float(quat[1])
        t.transform.rotation.y = float(quat[2])
        t.transform.rotation.z = float(quat[3])

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args if args is not None else sys.argv)
    node = CharucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
