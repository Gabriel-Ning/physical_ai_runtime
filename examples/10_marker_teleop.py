#!/usr/bin/env python3
"""10_marker_teleop.py: Cartesian Pose & Velocity Twist Teleoperation Streaming.

Demonstrates **Cartesian Streaming (TSKPC)**:
1. Generates Cartesian Pose reference trajectories (`pose_reference`) OR velocity commands (`twist_reference`).
2. Streams references directly to the RT TaskSpaceKinematicPositionController (TSKPC).
3. The RT controller executes OSQP/Pinocchio DLS inverse kinematics with Cartesian limits.

Usage:
  # In terminal 1 (start RT fake hardware):
  ros2 launch franka_manipulation_controller_bringup controller_bringup.launch.py use_fake_hardware:=true

  # In terminal 2 (Cartesian Pose Orbit):
  python examples/10_marker_teleop.py --profile fr3_pika_single_arm.yaml --mode pose

  # Or (Cartesian Velocity Twist Stream):
  python examples/10_marker_teleop.py --profile fr3_pika_single_arm.yaml --mode twist
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import rmi
from geometry_msgs.msg import TwistStamped
from marker_support import InteractivePoseTarget, lookup_tip_pose
from moveit_msgs.msg import (
    CartesianTrajectory,
    CartesianTrajectoryPoint,
)
from support import PROFILE_TO_CUROBO


def make_cartesian_pose_chunk(
    center_pos: list[float],
    t: float,
    radius: float = 0.08,
    speed: float = 1.0,
    frame_id: str = "fr3_link0",
) -> CartesianTrajectory:
    """Create a CartesianTrajectory message with a circle waypoint."""
    traj = CartesianTrajectory()
    traj.header.frame_id = frame_id

    pt = CartesianTrajectoryPoint()
    pt.point.pose.position.x = center_pos[0] + radius * math.cos(speed * t)
    pt.point.pose.position.y = center_pos[1] + radius * math.sin(speed * t)
    pt.point.pose.position.z = center_pos[2] + 0.03 * math.sin(2.0 * speed * t)
    pt.point.pose.orientation.w = 1.0
    pt.point.pose.orientation.x = 0.0
    pt.point.pose.orientation.y = 0.0
    pt.point.pose.orientation.z = 0.0
    traj.points.append(pt)
    return traj


def make_twist_stamped(
    t: float, speed: float = 1.0, frame_id: str = "fr3_link0"
) -> TwistStamped:
    """Create a TwistStamped message with smooth spatial velocities."""
    twist = TwistStamped()
    twist.header.frame_id = frame_id
    twist.twist.linear.x = -0.05 * math.sin(speed * t)
    twist.twist.linear.y = 0.05 * math.cos(speed * t)
    twist.twist.linear.z = 0.02 * math.cos(2.0 * speed * t)
    twist.twist.angular.x = 0.0
    twist.twist.angular.y = 0.0
    twist.twist.angular.z = 0.1 * math.sin(speed * t)
    return twist


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Cartesian Pose & Twist Teleop Streamer"
    )
    parser.add_argument(
        "--profile",
        type=str,
        default="fr3_pika_single_arm.yaml",
        help="Embodiment profile name or path (e.g. fr3_pika_single_arm.yaml)",
    )
    parser.add_argument(
        "--mode",
        type=str,
        default="marker",
        choices=["marker", "pose", "twist"],
        help="Control mode: 'marker' (RViz Interactive Marker), 'pose' (orbit), or 'twist' (velocity)",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=50.0,
        help="Loop rate (Hz)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Run duration (seconds)",
    )
    args = parser.parse_args()

    profile_key = Path(args.profile).name
    if profile_key not in PROFILE_TO_CUROBO:
        print(
            f"[X] Unsupported profile {args.profile!r}. Available: {list(PROFILE_TO_CUROBO.keys())}"
        )
        sys.exit(1)
    info = PROFILE_TO_CUROBO[profile_key]
    arm_part = info["arm_part"]
    center_pos = info["default_target_pose"]["position"]

    print("\n=======================================================")
    print("  RMI Demo 10: Cartesian Teleop Streamer -> TSKPC")
    print(f"  Profile: {args.profile}")
    print(f"  Arm Part: {arm_part}")
    print(f"  Mode: {args.mode.upper()} reference stream at {args.rate_hz} Hz")
    print("=======================================================\n")

    # 1. Initialize RMI Context & Agent
    print("[1/2] Initializing RMI Context & Agent...")
    ctx = rmi.Context.from_profile(args.profile)
    ctx.wait_until_ready(timeout=5.0)
    robot = ctx.robot
    provider = "TeleopTwist" if args.mode == "twist" else "TeleopCartesian"
    agent = ctx.make_agent(provider, frequency=args.rate_hz)

    part_cfg = ctx.profile.parts[arm_part]
    base_frame = part_cfg.base_frame or "base_link"
    tip_frame = part_cfg.tcp_frame or part_cfg.flange_frame or "tool0"

    # Optional: Setup Interactive Marker in RViz
    marker: InteractivePoseTarget | None = None
    if args.mode == "marker":
        initial_pose = lookup_tip_pose(
            ctx.node, base_frame=base_frame, tip_frame=tip_frame
        )
        marker = InteractivePoseTarget(
            ctx.node,
            frame_id=base_frame,
            initial=initial_pose,
            description=f"Teleop Target ({arm_part})",
        )
        print(
            "  [✓] Interactive Marker active! In RViz, select 'Interact' tool and drag the marker."
        )

    # 2. Scoped Control Session
    print(f"[2/2] Streaming {args.mode.upper()} references...")
    step_count = 0

    try:
        with agent.run(robot) as session:
            start_time = time.time()
            while session.ok() and (time.time() - start_time) < args.duration:
                t = time.time() - start_time

                if args.mode == "marker" and marker is not None:
                    payload = marker.current()
                    command = "pose_reference"
                elif args.mode == "pose":
                    payload = make_cartesian_pose_chunk(
                        center_pos, t=t, speed=1.0, frame_id=base_frame
                    )
                    command = "pose_reference"
                else:
                    payload = make_twist_stamped(t=t, speed=1.0, frame_id=base_frame)
                    command = "twist_reference"

                # Send action directly to RT TSKPC controller
                session.act(
                    rmi.Action(
                        part=arm_part,
                        command=command,
                        value=payload,
                    )
                )

                step_count += 1
                if step_count % int(args.rate_hz) == 0:
                    if args.mode == "marker" and marker is not None:
                        xyz = marker.current().position_xyz
                        print(
                            f"  [t={t:5.1f}s] Marker pose: ({xyz[0]:+.3f}, {xyz[1]:+.3f}, {xyz[2]:+.3f}) (drag in RViz)"
                        )
                    elif args.mode == "pose":
                        p = payload.points[0].point.pose.position
                        print(
                            f"  [t={t:5.1f}s] Pose ref: pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
                        )
                    else:
                        v = payload.twist.linear
                        print(
                            f"  [t={t:5.1f}s] Twist ref: linear=({v.x:.3f}, {v.y:.3f}, {v.z:.3f}) m/s"
                        )

                session.wait()
    except KeyboardInterrupt:
        print("\n[!] Interrupted by user")
    finally:
        if marker is not None:
            marker.close()

    print(
        f"\n[✓] Cartesian {args.mode} streaming completed ({step_count} steps dispatched)."
    )
    ctx.close()


if __name__ == "__main__":
    main()
