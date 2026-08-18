#!/usr/bin/env python3
"""08_ik_resolver.py: Online Inverse Kinematics Resolution & JSPC Setpoint Dispatch.

Demonstrates the **Resolver Family**:
1. Takes continuous Cartesian Target poses.
2. Solves Inverse Kinematics `resolve(current, target) -> q*` via cuRobo IK.
3. Streams the single-point `q*` reference to the RT JointSpacePositionController (JSPC).

Usage:
  # In terminal 1 (start RT fake hardware):
  ros2 launch franka_manipulation_controller_bringup controller_bringup.launch.py use_fake_hardware:=true

  # In terminal 2:
  python examples/08_ik_resolver.py --profile fr3_pika_single_arm.yaml --mode circle
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import rmi
from marker_support import InteractivePoseTarget, lookup_tip_pose
from motion_planner_core.contracts import CartesianState
from support import PROFILE_TO_CUROBO, create_curobo_ik_resolver, make_circular_target


def main() -> None:
    parser = argparse.ArgumentParser(
        description="cuRobo IK Resolver & JSPC Setpoint Dispatch"
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
        default="circle",
        choices=["circle", "step", "marker"],
        help="Trajectory pattern: circle, step, or marker (RViz Interactive Marker)",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=30.0,
        help="Control loop rate (Hz)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Run duration in seconds",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda",
        help="Compute device for cuRobo (cuda or cpu)",
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
    print("  RMI Demo 08: Online IK Resolver (cuRobo IK -> JSPC)")
    print(f"  Profile: {args.profile}")
    print(f"  Arm Part: {arm_part}")
    print(f"  Mode: {args.mode} at {args.rate_hz} Hz for {args.duration}s")
    print("=======================================================\n")

    # 1. Initialize RMI Context & Agent
    print("[1/3] Initializing RMI Context & Agent...")
    ctx = rmi.Context.from_profile(args.profile)
    ctx.wait_until_ready(timeout=5.0)
    robot = ctx.robot
    agent = ctx.make_agent("Policy", frequency=args.rate_hz)

    # 2. Build cuRobo IK Resolver
    print(f"[2/3] Initializing cuRobo IK Resolver ({args.device})...")
    resolver_backend = create_curobo_ik_resolver(args.profile, device=args.device)
    resolver_backend.warmup()
    resolver = rmi.Resolver("curobo_ik", resolver_backend)

    # Optional: Setup Interactive Marker in RViz
    marker: InteractivePoseTarget | None = None
    if args.mode == "marker":
        part_cfg = ctx.profile.parts[arm_part]
        base_frame = part_cfg.base_frame or "base_link"
        tip_frame = part_cfg.tcp_frame or part_cfg.flange_frame or "tool0"
        initial_pose = lookup_tip_pose(
            ctx.node, base_frame=base_frame, tip_frame=tip_frame
        )
        marker = InteractivePoseTarget(
            ctx.node,
            frame_id=base_frame,
            initial=initial_pose,
            description=f"cuRobo IK Target ({arm_part})",
        )
        print(
            "  [✓] Interactive Marker active! In RViz, select 'Interact' tool and drag the marker."
        )

    # 3. Scoped Control Session
    print("[3/3] Starting IK streaming control loop...")
    dt = 1.0 / args.rate_hz
    step_count = 0

    try:
        with agent.run(robot) as session:
            start_time = time.time()
            while session.ok() and (time.time() - start_time) < args.duration:
                t = time.time() - start_time

                # Compute target pose
                if args.mode == "marker" and marker is not None:
                    target = marker.current()
                elif args.mode == "circle":
                    target = make_circular_target(
                        center_pos, radius=0.08, t=t, speed=1.2
                    )
                else:
                    # Step between two target poses
                    offset = 0.08 if int(t) % 2 == 0 else -0.08
                    target = CartesianState(
                        position_xyz=(center_pos[0] + offset, center_pos[1], center_pos[2]),
                        orientation_wxyz=(0.0, 1.0, 0.0, 0.0),
                    )

                # Resolve IK: resolve(current_state, target) -> q*
                resolve_result = resolver.resolve(robot=robot, target=target)
                if not resolve_result.valid or resolve_result.positions is None:
                    time.sleep(dt)
                    continue

                q_star = resolve_result.positions

                # Send single-point joint reference directly to RT JSPC
                session.act(
                    rmi.Action(
                        part=arm_part,
                        command="joint_reference",
                        value=q_star,
                    )
                )

                step_count += 1
                if step_count % int(args.rate_hz) == 0:
                    pos = target.position_xyz
                    print(
                        f"  [t={t:5.1f}s] Target: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) "
                        f"-> q*: {[round(x, 2) for x in q_star[:4]]}..."
                    )

                session.wait()
    except KeyboardInterrupt:
        print("\n[!] Interrupted by user")
    finally:
        if marker is not None:
            marker.close()

    print(f"\n[✓] IK Resolver stream completed successfully ({step_count} steps sent).")
    ctx.close()


if __name__ == "__main__":
    main()
