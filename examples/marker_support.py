"""6-DoF RViz interactive marker helper that yields a CartesianState target."""

from __future__ import annotations

import threading
import time
from typing import Any

from geometry_msgs.msg import Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from motion_planner_core.contracts import CartesianState
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)

_INV_SQRT2 = 0.7071067811865476
_MARKER_NS = "target_marker"


def _axis_quat(*, y: float = 0.0, z: float = 0.0) -> Quaternion:
    q = Quaternion()
    q.w = _INV_SQRT2
    q.y = y
    q.z = z
    return q


def _add_axis_controls(int_marker: InteractiveMarker) -> None:
    axes = (
        ("x", Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)),
        ("y", _axis_quat(y=_INV_SQRT2)),
        ("z", _axis_quat(z=_INV_SQRT2)),
    )
    for name, orientation in axes:
        move = InteractiveMarkerControl()
        move.name = f"move_{name}"
        move.orientation = orientation
        move.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(move)
        rotate = InteractiveMarkerControl()
        rotate.name = f"rotate_{name}"
        rotate.orientation = orientation
        rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(rotate)


def _cartesian_from_pose(pose: Any) -> CartesianState:
    p = pose.position
    q = pose.orientation
    return CartesianState(
        position_xyz=(float(p.x), float(p.y), float(p.z)),
        orientation_wxyz=(
            float(q.w),
            float(q.x),
            float(q.y),
            float(q.z),
        ),
    )


class InteractivePoseTarget:
    """Attach a 6-DoF marker to an existing rclpy node."""

    def __init__(
        self,
        node: Any,
        *,
        frame_id: str,
        initial: CartesianState,
        namespace: str = _MARKER_NS,
        description: str = "Target Pose",
    ) -> None:
        self._lock = threading.Lock()
        self._pose = initial
        self._moved = False
        self._frame_id = frame_id
        self._description = description
        self._server = InteractiveMarkerServer(node, namespace)
        self._insert(initial)
        print(
            f"[marker] RViz InteractiveMarkers namespace=/{namespace} "
            f"frame={frame_id} (drag marker in RViz Interact mode)"
        )

    @property
    def user_moved(self) -> bool:
        with self._lock:
            return self._moved

    def current(self) -> CartesianState:
        with self._lock:
            return self._pose

    def _insert(self, initial: CartesianState) -> None:
        x, y, z = initial.position_xyz
        qw, qx, qy, qz = initial.orientation_wxyz
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self._frame_id
        int_marker.name = "target_pose"
        int_marker.description = self._description
        int_marker.scale = 0.15
        int_marker.pose.position.x = x
        int_marker.pose.position.y = y
        int_marker.pose.position.z = z
        int_marker.pose.orientation.w = qw
        int_marker.pose.orientation.x = qx
        int_marker.pose.orientation.y = qy
        int_marker.pose.orientation.z = qz

        box = Marker()
        box.type = Marker.CUBE
        box.scale.x = box.scale.y = box.scale.z = 0.04
        box.color.r = 0.0
        box.color.g = 0.8
        box.color.b = 0.9
        box.color.a = 0.7
        visual = InteractiveMarkerControl()
        visual.always_visible = True
        visual.markers.append(box)
        int_marker.controls.append(visual)

        free = InteractiveMarkerControl()
        free.name = "move_3d"
        free.interaction_mode = InteractiveMarkerControl.MOVE_3D
        free.always_visible = True
        int_marker.controls.append(free)
        _add_axis_controls(int_marker)

        self._server.insert(int_marker, feedback_callback=self._on_feedback)
        self._server.applyChanges()

    def close(self) -> None:
        shutdown = getattr(self._server, "shutdown", None)
        if shutdown is not None:
            try:
                shutdown()
            except Exception:
                pass

    def _on_feedback(self, feedback: Any) -> None:
        pose = _cartesian_from_pose(feedback.pose)
        with self._lock:
            self._pose = pose
            if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
                self._moved = True


def lookup_tip_pose(
    node: Any,
    *,
    base_frame: str,
    tip_frame: str,
    timeout_sec: float = 3.0,
) -> CartesianState:
    """Lookup current TF between base_frame and tip_frame."""
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    deadline = time.monotonic() + timeout_sec
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            tf = tf_buffer.lookup_transform(base_frame, tip_frame, Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            return CartesianState(
                position_xyz=(float(t.x), float(t.y), float(t.z)),
                orientation_wxyz=(float(q.w), float(q.x), float(q.y), float(q.z)),
            )
        except Exception as exc:
            last_error = exc
            time.sleep(0.05)
    # Default fallback if TF not ready
    return CartesianState(
        position_xyz=(0.45, 0.0, 0.35),
        orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
    )
