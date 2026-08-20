#!/usr/bin/env python3
"""Measure delivery frame rate and header-to-receipt delay for all cameras."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import math
import sys
import time
from typing import Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


CAMERA_TOPICS = {
    "static_orbbec": "/observation/static_orbbec/color/image_raw",
    "left_hand_realsense": "/observation/left_hand_realsense/color/image_raw",
    "right_hand_realsense": "/observation/right_hand_realsense/color/image_raw",
}


def percentile(values: Sequence[float], percent: float) -> float:
    """Return a linearly interpolated percentile for a non-empty sequence."""
    if not values:
        raise ValueError("percentile requires at least one value")
    if not 0.0 <= percent <= 100.0:
        raise ValueError("percent must be in [0, 100]")

    ordered = sorted(values)
    position = (len(ordered) - 1) * percent / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


@dataclass
class StreamStats:
    """Measurements collected for one image stream."""

    topic: str
    frames: int = 0
    first_arrival_ns: int | None = None
    last_arrival_ns: int | None = None
    header_delays_ms: list[float] = field(default_factory=list)

    def record(self, arrival_monotonic_ns: int, header_delay_ms: float | None) -> None:
        self.frames += 1
        if self.first_arrival_ns is None:
            self.first_arrival_ns = arrival_monotonic_ns
        self.last_arrival_ns = arrival_monotonic_ns
        if header_delay_ms is not None:
            self.header_delays_ms.append(header_delay_ms)

    def fps(self) -> float | None:
        if self.frames < 2 or self.first_arrival_ns is None or self.last_arrival_ns is None:
            return None
        elapsed_seconds = (self.last_arrival_ns - self.first_arrival_ns) / 1_000_000_000
        return (self.frames - 1) / elapsed_seconds if elapsed_seconds > 0 else None


class CameraStreamDiagnostics(Node):
    """One sensor-data QoS subscriber for each configured color image topic."""

    def __init__(self) -> None:
        super().__init__("camera_stream_diagnostics")
        self.stats = {
            name: StreamStats(topic=topic) for name, topic in CAMERA_TOPICS.items()
        }
        # Do not reuse Node._subscriptions: rclpy owns that internal list and
        # destroys its entries during Node.destroy_node().
        self._image_subscriptions = []
        for name, topic in CAMERA_TOPICS.items():
            self._image_subscriptions.append(
                self.create_subscription(
                    Image,
                    topic,
                    lambda message, camera_name=name: self._image_callback(
                        camera_name, message
                    ),
                    qos_profile_sensor_data,
                )
            )

    def _image_callback(self, camera_name: str, message: Image) -> None:
        arrival_monotonic_ns = time.monotonic_ns()
        now_ros_ns = self.get_clock().now().nanoseconds
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        header_delay_ms = None if stamp_ns == 0 else (now_ros_ns - stamp_ns) / 1_000_000
        self.stats[camera_name].record(arrival_monotonic_ns, header_delay_ms)

    def print_progress(self) -> None:
        fragments = []
        for name, stats in self.stats.items():
            rate = stats.fps()
            rate_text = "warming up" if rate is None else f"{rate:.2f} Hz"
            fragments.append(f"{name}: {stats.frames} frames, {rate_text}")
        self.get_logger().info(" | ".join(fragments))


def _format_delay_summary(delays_ms: Sequence[float]) -> str:
    if not delays_ms:
        return "no valid header timestamps"
    return (
        f"mean={sum(delays_ms) / len(delays_ms):.2f} ms, "
        f"p50={percentile(delays_ms, 50):.2f} ms, "
        f"p95={percentile(delays_ms, 95):.2f} ms, "
        f"max={max(delays_ms):.2f} ms"
    )


def print_summary(node: CameraStreamDiagnostics) -> bool:
    """Print a final report and return whether every stream delivered data."""
    print("\nCamera stream diagnostic summary")
    print("=" * 72)
    all_streams_received = True
    for name, stats in node.stats.items():
        if stats.frames == 0:
            all_streams_received = False
            print(f"{name}\n  topic: {stats.topic}\n  result: NO DATA")
            continue

        rate = stats.fps()
        rate_text = "n/a (only one frame)" if rate is None else f"{rate:.2f} Hz"
        print(
            f"{name}\n"
            f"  topic: {stats.topic}\n"
            f"  received: {stats.frames} frames\n"
            f"  delivery rate: {rate_text}\n"
            f"  header-to-callback delay: {_format_delay_summary(stats.header_delays_ms)}"
        )
    print("=" * 72)
    return all_streams_received


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Measure three camera color-stream frame rates and message-header delays."
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Measurement duration in seconds (default: 30).",
    )
    parser.add_argument(
        "--report-period",
        type=float,
        default=5.0,
        help="Progress-report period in seconds; 0 disables progress reports (default: 5).",
    )
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be greater than zero")
    if args.report_period < 0:
        parser.error("--report-period must be zero or greater")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = CameraStreamDiagnostics()
    started_at = time.monotonic()
    next_report_at = started_at + args.report_period if args.report_period else None
    interrupted = False

    try:
        while rclpy.ok() and time.monotonic() < started_at + args.duration:
            remaining = max(0.0, started_at + args.duration - time.monotonic())
            rclpy.spin_once(node, timeout_sec=min(0.2, remaining))
            if next_report_at is not None and time.monotonic() >= next_report_at:
                node.print_progress()
                next_report_at += args.report_period
    except KeyboardInterrupt:
        interrupted = True
    finally:
        received_all_streams = print_summary(node)
        node.destroy_node()
        rclpy.shutdown()

    if interrupted:
        return 130
    return 0 if received_all_streams else 1


if __name__ == "__main__":
    sys.exit(main())
