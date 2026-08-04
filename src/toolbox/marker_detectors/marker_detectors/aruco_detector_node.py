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


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__("aruco_detector_node")
        self.get_logger().info("Initializing ArucoDetectorNode...")

        self.declare_parameter("image_topic", "/observation/color/image_raw")
        self.declare_parameter("camera_info_topic", "/observation/color/camera_info")
        self.declare_parameter("marker_size", 0.05)     # 5cm physical size
        self.declare_parameter("marker_id", 0)           # Target ArUco marker ID
        self.declare_parameter("dictionary_id", cv2.aruco.DICT_5X5_250)
        self.declare_parameter("target_frame_id", "aruco_marker")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "debug_image")

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.marker_id = int(self.get_parameter("marker_id").value)
        self.dict_id = int(self.get_parameter("dictionary_id").value)
        self.target_frame_id = str(self.get_parameter("target_frame_id").value)
        self.pub_debug = bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)

        self.dictionary = cv2.aruco.getPredefinedDictionary(self.dict_id)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.dictionary, self.detector_params)

        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.lock = threading.Lock()

        self.info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.info_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data
        )

        if self.pub_debug:
            self.debug_pub = self.create_publisher(
                Image, self.debug_image_topic, qos_profile_sensor_data
            )

        self.get_logger().info(
            f"ArucoDetectorNode ready. Target ID: {self.marker_id}, size={self.marker_size}m, topic={self.image_topic}"
        )

    def info_callback(self, msg: CameraInfo):
        with self.lock:
            if self.camera_matrix is None:
                self.camera_matrix = np.array(msg.k).reshape((3, 3))
                self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg: Image):
        with self.lock:
            if self.camera_matrix is None:
                return
            K = self.camera_matrix.copy()
            D = self.dist_coeffs.copy()

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge failed: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray)

        if ids is not None and self.marker_id in ids.flatten():
            idx = int(np.where(ids.flatten() == self.marker_id)[0][0])
            m_corners = corners[idx]

            if self.pub_debug:
                cv2.aruco.drawDetectedMarkers(cv_image, [m_corners], np.array([[self.marker_id]]))

            # Estimate pose for target marker using solvePnP
            obj_points = np.array([
                [-self.marker_size / 2.0, self.marker_size / 2.0, 0],
                [self.marker_size / 2.0, self.marker_size / 2.0, 0],
                [self.marker_size / 2.0, -self.marker_size / 2.0, 0],
                [-self.marker_size / 2.0, -self.marker_size / 2.0, 0]
            ], dtype=np.float32)

            success, rvec, tvec = cv2.solvePnP(
                obj_points, m_corners.reshape(4, 2), K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            if success:
                if self.pub_debug:
                    cv2.drawFrameAxes(cv_image, K, D, rvec, tvec, 0.05)
                self.publish_tf(msg.header, rvec.flatten(), tvec.flatten())

        if self.pub_debug:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            except CvBridgeError:
                pass

    def publish_tf(self, header, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        quat = mat2quat(R)

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
    node = ArucoDetectorNode()
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
