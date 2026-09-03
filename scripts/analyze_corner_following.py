#!/usr/bin/env python3
"""Measure cloth motion and gripper TCP motion from Piper episode MCAP files.

The fixed RGB camera is not currently connected to the robot ``world`` TF tree,
so cloth geometry is reported in image pixels.  Gripper TCP positions are
computed in metres with the deployed bimanual URDF and reported in millimetres.
The two units are deliberately never mixed into a metric tracking error.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from bisect import bisect_left
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from convert_episode_to_lerobot import (
    IMAGE_ORBBEC,
    IMAGE_STATIC_D435I,
    IMAGE_STATIC_REALSENSE,
    STATE_TOPIC,
    _decode_rgb_image,
    _episode_mcap,
    _open_reader,
)

STATIC_TOPICS = (IMAGE_STATIC_D435I, IMAGE_ORBBEC, IMAGE_STATIC_REALSENSE)
ARM_JOINTS = tuple(
    f"{side}_joint{index}" for side in ("left", "right") for index in range(1, 7)
)
CORNER_NAMES = ("top_left", "top_right", "bottom_right", "bottom_left")


@dataclass(frozen=True)
class ClothObservation:
    centroid_uv: tuple[float, float]
    corners_uv: np.ndarray
    area_px: float
    area_fraction: float
    angle_deg: float


@dataclass(frozen=True)
class TimedState:
    timestamp_ns: int
    positions: dict[str, float]


@dataclass(frozen=True)
class TimedCloth:
    timestamp_ns: int
    observation: ClothObservation
    rgb: np.ndarray


def _order_corners(points: np.ndarray) -> np.ndarray:
    """Return rotated-rectangle points as TL, TR, BR, BL in image coordinates."""
    points = np.asarray(points, dtype=np.float32).reshape(4, 2)
    sums = points.sum(axis=1)
    differences = np.diff(points, axis=1).reshape(-1)
    ordered = np.empty((4, 2), dtype=np.float32)
    ordered[0] = points[np.argmin(sums)]
    ordered[2] = points[np.argmax(sums)]
    ordered[1] = points[np.argmin(differences)]
    ordered[3] = points[np.argmax(differences)]
    return ordered


def detect_blue_cloth(
    rgb: np.ndarray,
    *,
    hsv_low: tuple[int, int, int] = (90, 70, 50),
    hsv_high: tuple[int, int, int] = (120, 255, 255),
    min_area_fraction: float = 0.01,
) -> ClothObservation | None:
    """Detect the largest blue cloth component in an RGB image."""
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError("rgb must be an HxWx3 uint8 image")
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, np.asarray(hsv_low), np.asarray(hsv_high))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    image_area = float(rgb.shape[0] * rgb.shape[1])
    if area < image_area * min_area_fraction:
        return None
    moments = cv2.moments(contour)
    if moments["m00"] <= 0.0:
        return None
    centroid = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])
    rectangle = cv2.minAreaRect(contour)
    corners = _order_corners(cv2.boxPoints(rectangle))
    width, height = rectangle[1]
    angle = float(rectangle[2])
    if width < height:
        angle += 90.0
    while angle >= 90.0:
        angle -= 180.0
    while angle < -90.0:
        angle += 180.0
    return ClothObservation(
        centroid_uv=(float(centroid[0]), float(centroid[1])),
        corners_uv=corners,
        area_px=area,
        area_fraction=area / image_area,
        angle_deg=angle,
    )


class PiperForwardKinematics:
    """Pinocchio FK for the two gripper TCP frames in the deployed URDF."""

    def __init__(self, xacro_path: Path | None = None) -> None:
        import pinocchio as pin
        import xacro
        from ament_index_python.packages import get_package_share_directory

        if xacro_path is None:
            xacro_path = (
                Path(get_package_share_directory("piper_description"))
                / "urdf"
                / "piper_bimanual_manipulation.urdf.xacro"
            )
        if not xacro_path.is_file():
            raise FileNotFoundError(f"Piper xacro not found: {xacro_path}")
        xml = xacro.process_file(
            str(xacro_path), mappings={"use_fake_hardware": "true"}
        ).toxml()
        self.pin = pin
        self.model = pin.buildModelFromXML(xml)
        self.data = self.model.createData()
        self.neutral = pin.neutral(self.model)
        self.q_indices = {
            name: self.model.joints[self.model.getJointId(name)].idx_q
            for name in ARM_JOINTS
        }
        self.frame_ids = {
            side: self.model.getFrameId(f"{side}_gripper_tcp")
            for side in ("left", "right")
        }

    def tcp_positions(self, positions: dict[str, float]) -> dict[str, np.ndarray]:
        missing = sorted(set(ARM_JOINTS) - positions.keys())
        if missing:
            raise ValueError("joint state is missing: " + ", ".join(missing))
        q = self.neutral.copy()
        for name, index in self.q_indices.items():
            q[index] = positions[name]
        self.pin.framesForwardKinematics(self.model, self.data, q)
        return {
            side: np.asarray(self.data.oMf[frame].translation, dtype=float).copy()
            for side, frame in self.frame_ids.items()
        }


def _select_static_topic(topic_types: dict[str, str], requested: str) -> str:
    if requested != "auto":
        if requested not in topic_types:
            raise RuntimeError(f"MCAP does not contain requested image topic: {requested}")
        return requested
    available = [topic for topic in STATIC_TOPICS if topic in topic_types]
    if len(available) != 1:
        raise RuntimeError(
            "MCAP must contain exactly one supported static RGB topic; found: "
            + (", ".join(available) if available else "none")
        )
    return available[0]


def _read_episode(
    mcap: Path,
    *,
    image_topic: str,
    image_sample_hz: float,
) -> tuple[list[TimedState], list[TimedCloth], int]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = _open_reader(rosbag2_py, mcap)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    selected_topic = _select_static_topic(topic_types, image_topic)
    if STATE_TOPIC not in topic_types:
        raise RuntimeError(f"MCAP is missing {STATE_TOPIC}")
    state_type = get_message(topic_types[STATE_TOPIC])
    image_type = get_message(topic_types[selected_topic])
    sample_period_ns = round(1e9 / image_sample_hz)
    next_image_ns = -1
    states: list[TimedState] = []
    cloth: list[TimedCloth] = []
    image_count = 0
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        timestamp_ns = int(timestamp_ns)
        if topic == STATE_TOPIC:
            message = deserialize_message(serialized, state_type)
            positions = {
                name: float(value)
                for name, value in zip(message.name, message.position, strict=False)
            }
            if all(name in positions and math.isfinite(positions[name]) for name in ARM_JOINTS):
                states.append(TimedState(timestamp_ns, positions))
        elif topic == selected_topic:
            image_count += 1
            if timestamp_ns < next_image_ns:
                continue
            next_image_ns = timestamp_ns + sample_period_ns
            message = deserialize_message(serialized, image_type)
            rgb = _decode_rgb_image(message)
            observation = detect_blue_cloth(rgb)
            if observation is not None:
                cloth.append(TimedCloth(timestamp_ns, observation, rgb))
    if not states:
        raise RuntimeError(f"no complete arm joint states found in {mcap}")
    if not cloth:
        raise RuntimeError(f"blue cloth was not detected in sampled frames from {mcap}")
    return states, cloth, image_count


def _nearest_state(states: list[TimedState], timestamp_ns: int) -> TimedState:
    timestamps = [state.timestamp_ns for state in states]
    index = bisect_left(timestamps, timestamp_ns)
    candidates = states[max(0, index - 1) : min(len(states), index + 1)]
    return min(candidates, key=lambda state: abs(state.timestamp_ns - timestamp_ns))


def _median_vector(values: Iterable[np.ndarray]) -> np.ndarray:
    return np.median(np.stack(tuple(values)), axis=0)


def _window(items: list[Any], *, first: bool, duration_s: float = 0.5) -> list[Any]:
    boundary_ns = round(duration_s * 1e9)
    if first:
        selected = [item for item in items if item.timestamp_ns <= items[0].timestamp_ns + boundary_ns]
    else:
        selected = [item for item in items if item.timestamp_ns >= items[-1].timestamp_ns - boundary_ns]
    return selected or [items[0 if first else -1]]


def _draw_overlay(item: TimedCloth, destination: Path) -> None:
    image = cv2.cvtColor(item.rgb, cv2.COLOR_RGB2BGR)
    corners = np.rint(item.observation.corners_uv).astype(np.int32)
    cv2.polylines(image, [corners], True, (0, 255, 0), 2, cv2.LINE_AA)
    for name, point in zip(CORNER_NAMES, corners, strict=True):
        cv2.circle(image, tuple(point), 5, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.putText(
            image,
            name,
            tuple(point + np.asarray((6, -6))),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )
    centroid = tuple(np.rint(item.observation.centroid_uv).astype(int))
    cv2.drawMarker(image, centroid, (255, 0, 255), cv2.MARKER_CROSS, 14, 2)
    if not cv2.imwrite(str(destination), image):
        raise RuntimeError(f"failed to write overlay: {destination}")


def analyze_episode(
    episode: Path,
    output_dir: Path,
    fk: PiperForwardKinematics,
    *,
    image_topic: str = "auto",
    image_sample_hz: float = 5.0,
) -> dict[str, Any]:
    mcap = _episode_mcap(episode)
    states, cloth, image_count = _read_episode(
        mcap, image_topic=image_topic, image_sample_hz=image_sample_hz
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    initial_states = _window(states, first=True)
    final_states = _window(states, first=False)
    initial_cloth = _window(cloth, first=True)
    final_cloth = _window(cloth, first=False)

    tcp_start = {
        side: _median_vector(fk.tcp_positions(item.positions)[side] for item in initial_states)
        for side in ("left", "right")
    }
    tcp_end = {
        side: _median_vector(fk.tcp_positions(item.positions)[side] for item in final_states)
        for side in ("left", "right")
    }
    cloth_start_centroid = _median_vector(
        np.asarray(item.observation.centroid_uv) for item in initial_cloth
    )
    cloth_end_centroid = _median_vector(
        np.asarray(item.observation.centroid_uv) for item in final_cloth
    )
    cloth_start_corners = np.median(
        np.stack([item.observation.corners_uv for item in initial_cloth]), axis=0
    )
    cloth_end_corners = np.median(
        np.stack([item.observation.corners_uv for item in final_cloth]), axis=0
    )

    timeseries_path = output_dir / "timeseries.csv"
    with timeseries_path.open("w", newline="", encoding="utf-8") as stream:
        fieldnames = [
            "time_s",
            "centroid_u_px",
            "centroid_v_px",
            "cloth_angle_deg",
            "cloth_area_fraction",
            "left_tcp_x_mm",
            "left_tcp_y_mm",
            "left_tcp_z_mm",
            "right_tcp_x_mm",
            "right_tcp_y_mm",
            "right_tcp_z_mm",
        ]
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        start_ns = min(states[0].timestamp_ns, cloth[0].timestamp_ns)
        for item in cloth:
            tcp = fk.tcp_positions(_nearest_state(states, item.timestamp_ns).positions)
            writer.writerow(
                {
                    "time_s": (item.timestamp_ns - start_ns) / 1e9,
                    "centroid_u_px": item.observation.centroid_uv[0],
                    "centroid_v_px": item.observation.centroid_uv[1],
                    "cloth_angle_deg": item.observation.angle_deg,
                    "cloth_area_fraction": item.observation.area_fraction,
                    **{
                        f"{side}_tcp_{axis}_mm": 1000.0 * tcp[side][index]
                        for side in ("left", "right")
                        for index, axis in enumerate("xyz")
                    },
                }
            )

    _draw_overlay(cloth[0], output_dir / "cloth_first.jpg")
    _draw_overlay(cloth[-1], output_dir / "cloth_last.jpg")
    result: dict[str, Any] = {
        "schema_version": "1.0",
        "episode": str(episode.resolve()),
        "mcap": str(mcap.resolve()),
        "units": {"cloth": "top_camera_pixels", "tcp": "world_millimetres"},
        "metric_tracking_error_available": False,
        "metric_tracking_error_blocker": (
            "static camera has no world extrinsic/homography; do not compare pixels to millimetres"
        ),
        "duration_s": (max(states[-1].timestamp_ns, cloth[-1].timestamp_ns) - min(states[0].timestamp_ns, cloth[0].timestamp_ns)) / 1e9,
        "joint_state_count": len(states),
        "static_image_count": image_count,
        "analysed_cloth_frame_count": len(cloth),
        "cloth": {
            "start_centroid_uv_px": cloth_start_centroid.tolist(),
            "end_centroid_uv_px": cloth_end_centroid.tolist(),
            "centroid_displacement_uv_px": (cloth_end_centroid - cloth_start_centroid).tolist(),
            "start_corners_uv_px": dict(zip(CORNER_NAMES, cloth_start_corners.tolist(), strict=True)),
            "end_corners_uv_px": dict(zip(CORNER_NAMES, cloth_end_corners.tolist(), strict=True)),
        },
        "tcp": {},
        "artifacts": {
            "timeseries_csv": str(timeseries_path.resolve()),
            "first_overlay": str((output_dir / "cloth_first.jpg").resolve()),
            "last_overlay": str((output_dir / "cloth_last.jpg").resolve()),
        },
    }
    for side in ("left", "right"):
        delta_mm = 1000.0 * (tcp_end[side] - tcp_start[side])
        result["tcp"][side] = {
            "start_xyz_mm": (1000.0 * tcp_start[side]).tolist(),
            "end_xyz_mm": (1000.0 * tcp_end[side]).tolist(),
            "displacement_xyz_mm": delta_mm.tolist(),
            "displacement_norm_mm": float(np.linalg.norm(delta_mm)),
        }
    (output_dir / "summary.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return result


def fit_pixel_to_tcp_regression(results: list[dict[str, Any]]) -> dict[str, Any]:
    """Fit an exploratory cross-episode mapping; this is not calibrated gain G."""
    if len(results) < 4:
        return {
            "available": False,
            "episode_count": len(results),
            "reason": "at least 4 layouts are required for an exploratory fit",
        }
    fits: dict[str, Any] = {}
    target_corners = {"left": "top_left", "right": "top_right"}
    for side, target_corner in target_corners.items():
        design = np.asarray(
            [
                result["cloth"]["start_corners_uv_px"][target_corner] + [1.0]
                for result in results
            ],
            dtype=float,
        )
        response = np.asarray(
            [result["tcp"][side]["end_xyz_mm"] for result in results], dtype=float
        )
        coefficients, *_ = np.linalg.lstsq(design, response, rcond=None)
        predicted = design @ coefficients
        residual = np.sum((response - predicted) ** 2, axis=0)
        total = np.sum((response - response.mean(axis=0)) ** 2, axis=0)
        r_squared = np.full(total.shape, np.nan, dtype=float)
        variable = total > 1e-12
        r_squared[variable] = 1.0 - residual[variable] / total[variable]
        fits[side] = {
            "target_corner": target_corner,
            "coefficient_rows": ["corner_u_px", "corner_v_px", "intercept"],
            "output_columns": ["tcp_x_mm", "tcp_y_mm", "tcp_z_mm"],
            "coefficients": coefficients.tolist(),
            "r_squared_xyz": r_squared.tolist(),
        }
    return {
        "available": True,
        "episode_count": len(results),
        "interpretation": (
            "exploratory pixel-to-final-TCP regression only; not calibrated G and not a metric corner error"
        ),
        "fits": fits,
    }


def _episode_paths(args: argparse.Namespace) -> list[Path]:
    paths = list(args.episode)
    if args.episodes_root is not None:
        paths.extend(sorted(path for path in args.episodes_root.glob("episode_*") if path.is_dir()))
    unique = list(dict.fromkeys(path.resolve() for path in paths))
    if not unique:
        raise SystemExit("pass --episode at least once, or pass --episodes-root")
    return unique


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Offline cloth-pixel and Piper TCP-motion measurement"
    )
    parser.add_argument("--episode", type=Path, action="append", default=[])
    parser.add_argument("--episodes-root", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--image-sample-hz", type=float, default=5.0)
    parser.add_argument(
        "--image-topic", default="auto", help="auto or an exact static RGB topic"
    )
    parser.add_argument("--urdf-xacro", type=Path)
    return parser


def main() -> None:
    args = _build_parser().parse_args()
    if args.image_sample_hz <= 0.0:
        raise SystemExit("--image-sample-hz must be positive")
    episodes = _episode_paths(args)
    fk = PiperForwardKinematics(args.urdf_xacro)
    results = []
    for episode in episodes:
        destination = args.output_dir / episode.stem
        print(f"[analyse] {episode} -> {destination}")
        result = analyze_episode(
            episode,
            destination,
            fk,
            image_topic=args.image_topic,
            image_sample_hz=args.image_sample_hz,
        )
        results.append(result)
        print(
            "  TCP displacement: "
            + ", ".join(
                f"{side}={result['tcp'][side]['displacement_norm_mm']:.1f} mm"
                for side in ("left", "right")
            )
        )
    aggregate = {
        "schema_version": "1.0",
        "episode_count": len(results),
        "episodes": [str(path) for path in episodes],
        "pixel_to_tcp_regression": fit_pixel_to_tcp_regression(results),
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "aggregate.json").write_text(
        json.dumps(aggregate, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(f"[done] {args.output_dir / 'aggregate.json'}")


if __name__ == "__main__":
    main()
