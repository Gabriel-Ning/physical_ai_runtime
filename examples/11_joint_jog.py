#!/usr/bin/env python3
"""11_joint_jog.py: Interactive Joint Jogging & Setpoint Stepping.

Demonstrates **Joint Jogging Teleop (JSPC)**:
1. Allows interactive joint jogging (+/- delta on selected joint).
2. Or runs an automated continuous multi-joint sweep.
3. Dispatches smooth joint position references to the RT JointSpacePositionController (JSPC).

Usage:
  # In terminal 1 (start RT fake hardware):
  ros2 launch franka_manipulation_controller_bringup controller_bringup.launch.py use_fake_hardware:=true

  # In terminal 2:
  python examples/11_joint_jog.py --profile fr3_pika_single_arm.yaml --joint 1 --delta 0.05
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import rmi
from support import PROFILE_TO_CUROBO


def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive Joint Jogging to JSPC")
    parser.add_argument(
        "--profile",
        type=str,
        default="fr3_pika_single_arm.yaml",
        help="Embodiment profile name or path (e.g. fr3_pika_single_arm.yaml)",
    )
    parser.add_argument(
        "--joint",
        type=int,
        default=4,
        help="1-indexed joint index to jog (e.g. 4 for elbow)",
    )
    parser.add_argument(
        "--delta",
        type=float,
        default=0.1,
        help="Target position delta in radians",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=3,
        help="Number of +/- jog oscillations to execute",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=30.0,
        help="Loop rate (Hz)",
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
    home_q = list(info["home_joint_positions"])
    num_dof = len(home_q)

    if args.joint < 1 or args.joint > num_dof:
        print(f"[X] Invalid joint index {args.joint}. Must be in [1, {num_dof}]")
        sys.exit(1)

    joint_idx = args.joint - 1

    print("\n=======================================================")
    print("  RMI Demo 11: Interactive Joint Jogger -> JSPC")
    print(f"  Profile: {args.profile}")
    print(f"  Arm Part: {arm_part}")
    print(
        f"  Target Joint: Joint {args.joint} (Delta = ±{args.delta} rad, Cycles = {args.cycles})"
    )
    print("=======================================================\n")

    # 1. Initialize RMI Context & Agent
    print("[1/2] Initializing RMI Context & Agent...")
    ctx = rmi.Context.from_profile(args.profile)
    robot = ctx.robot
    robot.wait_until_ready(timeout=5.0)
    agent = ctx.make_agent("TeleopJoint", frequency=args.rate_hz)

    # 2. Start Jogging Loop
    print(f"[2/2] Jogging Joint {args.joint}...")
    current_target_q = list(home_q)

    with agent.run(robot) as session:
        for cycle in range(1, args.cycles + 1):
            # Phase A: Jog positive delta
            print(
                f"  [Cycle {cycle}/{args.cycles}] Jogging Joint {args.joint} +{args.delta} rad..."
            )
            target_q = list(current_target_q)
            target_q[joint_idx] += args.delta

            # Hold and stream setpoint for 1.5 seconds
            steps = int(1.5 * args.rate_hz)
            for _ in range(steps):
                session.act(
                    rmi.Action(
                        part=arm_part,
                        command="joint_reference",
                        value=target_q,
                    )
                )
                session.wait()

            # Phase B: Jog negative delta
            print(
                f"  [Cycle {cycle}/{args.cycles}] Jogging Joint {args.joint} -{args.delta} rad..."
            )
            target_q[joint_idx] -= 2.0 * args.delta
            for _ in range(steps):
                session.act(
                    rmi.Action(
                        part=arm_part,
                        command="joint_reference",
                        value=target_q,
                    )
                )
                session.wait()

            # Phase C: Return to center
            print(f"  [Cycle {cycle}/{args.cycles}] Returning to home position...")
            for _ in range(steps):
                session.act(
                    rmi.Action(
                        part=arm_part,
                        command="joint_reference",
                        value=current_target_q,
                    )
                )
                session.wait()

    print("\n[✓] Joint jog completed successfully.")
    ctx.close()


if __name__ == "__main__":
    main()
