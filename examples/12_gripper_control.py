#!/usr/bin/env python3
"""12_gripper_control.py: Standalone & Streamed Gripper Control (Pika / Parallel Gripper).

Demonstrates **End-Effector / Gripper Control**:
1. Discovering gripper / end-effector parts declared in the embodiment profile.
2. Direct position setpoint dispatch: full open (e.g. 0.045 m), full close (0.0 m), or intermediate grip width.
3. Continuous trapezoidal motion loop: smooth opening and closing cycles.
4. Reading real-time gripper position and feedback from `session.observe()`.

Usage:
  # In terminal 1 (start RT fake hardware or real robot):
  # Real robot:
  #   ros2 launch franka_manipulation_controller_bringup controller_bringup.launch.py \
  #     use_fake_hardware:=false robot_ip:=192.168.2.101 load_pika_hardware:=true gripper_serial_port:=/dev/ttyUSB0
  #
  # Fake hardware:
  #   ros2 launch franka_manipulation_controller_bringup controller_bringup.launch.py use_fake_hardware:=true

  # In terminal 2 (cycle open/close):
  python examples/12_gripper_control.py --profile fr3_pika_single_arm.yaml --cycles 3

  # Or set a fixed width (e.g. 0.03 m):
  python examples/12_gripper_control.py --profile fr3_pika_single_arm.yaml --width 0.03
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

import rmi


def main() -> None:
    parser = argparse.ArgumentParser(description="RMI Gripper Control Example")
    parser.add_argument(
        "--profile",
        type=str,
        default="fr3_pika_single_arm.yaml",
        help="Embodiment profile path (e.g. fr3_pika_single_arm.yaml, marvin_bimanual.yaml)",
    )
    parser.add_argument(
        "--part",
        type=str,
        default="",
        help="Specific gripper part name (defaults to first gripper part found)",
    )
    parser.add_argument(
        "--width",
        type=float,
        default=-1.0,
        help="Fixed target width in meters (0.0 = closed, ~0.045 = open). If negative, runs open/close cycle demo.",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=3,
        help="Number of open/close cycles to run (when --width is not set)",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=20.0,
        help="Command stream loop rate (Hz)",
    )
    args = parser.parse_args()

    print(f"\n=======================================================")
    print(f"  RMI Demo 12: Gripper Control (Pika / Parallel Gripper)")
    print(f"  Profile: {args.profile}")
    print(f"=======================================================\n")

    # 1. Initialize RMI Context
    print("[1/3] Initializing Context & Resolving Gripper Part...")
    ctx = rmi.Context.from_profile(args.profile)
    ctx.wait_until_ready(timeout=5.0)
    robot = ctx.robot

    # Find gripper parts
    gripper_parts = [
        name for name, p in ctx.profile.parts.items()
        if "gripper" in p.part_type or "end_effector" in name or "gripper" in name
    ]
    if not gripper_parts:
        print(f"[X] No gripper / end_effector parts declared in profile {args.profile!r}")
        sys.exit(1)

    gripper_part = args.part if args.part else gripper_parts[0]
    if gripper_part not in ctx.profile.parts:
        print(f"[X] Part {gripper_part!r} not found. Available: {list(ctx.profile.parts.keys())}")
        sys.exit(1)

    part_cfg = ctx.profile.parts[gripper_part]
    joint_name = part_cfg.joint_names[0] if part_cfg.joint_names else "gripper_joint"
    print(f"  [✓] Target Gripper Part: {gripper_part!r} (Joint: {joint_name!r})")

    # 2. Build Policy Agent for Gripper
    print("[2/3] Initializing Agent for Gripper control...")
    agent = ctx.make_agent("Policy", frequency=args.rate_hz)

    # 3. Control Loop Execution
    print(f"[3/3] Executing Gripper Commands...")
    dt = 1.0 / args.rate_hz

    with agent.run(robot, parts=[gripper_part]) as session:
        initial_obs = session.observe()
        curr_pos = 0.0
        if joint_name in initial_obs.joint_names:
            idx = initial_obs.joint_names.index(joint_name)
            curr_pos = initial_obs.joint_positions[idx]
        print(f"  Initial gripper position: {curr_pos:.4f} m")

        if args.width >= 0.0:
            # Mode A: Move to single fixed width
            target_width = min(max(args.width, 0.0), 0.045)
            print(f"\n  -> Sending command: set {gripper_part} width = {target_width:.3f} m...")
            for _ in range(int(args.rate_hz * 1.5)):  # stream for 1.5s
                session.act(rmi.Action(
                    part=gripper_part,
                    command="joint_reference",
                    value=[target_width],
                ))
                session.wait()
            print(f"  [✓] Position command {target_width:.3f} m dispatched.")
        else:
            # Mode B: Open / Close smooth cycle sweep
            max_open = 0.040  # 40mm
            print(f"\n  -> Running {args.cycles} Open/Close Cycles (0.0 m <-> {max_open:.3f} m)...")
            for cycle in range(1, args.cycles + 1):
                # 1. Open
                print(f"  [Cycle {cycle}/{args.cycles}] Opening gripper to {max_open*1000:.0f} mm...")
                steps = int(args.rate_hz * 1.5)
                for step in range(steps):
                    frac = step / steps
                    w = frac * max_open
                    session.act(rmi.Action(
                        part=gripper_part,
                        command="joint_reference",
                        value=[w],
                    ))
                    session.wait()

                time.sleep(0.3)

                # 2. Close
                print(f"  [Cycle {cycle}/{args.cycles}] Closing gripper to 0 mm...")
                for step in range(steps):
                    frac = 1.0 - (step / steps)
                    w = frac * max_open
                    session.act(rmi.Action(
                        part=gripper_part,
                        command="joint_reference",
                        value=[w],
                    ))
                    session.wait()

                time.sleep(0.3)

    print("\n[✓] Gripper control execution completed successfully.")
    ctx.close()


if __name__ == "__main__":
    main()
