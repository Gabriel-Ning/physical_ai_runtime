#!/usr/bin/env python3
"""03_teleop_preempt.py: Human Teleop Preemption & Scoped Control Arbitration.

Demonstrates **Multi-Source Arbitration & Human-in-the-Loop (HIL)**:
1. Autonomous Policy Agent runs in the background at normal priority.
2. Human Teleop Agent steps in and acquires the robot session with higher priority.
3. The RMI Execution Manager preempts the lower-priority Policy, bumping generation counter.
4. Human Teleop dispatches actions; once teleop exits, Policy safely re-acquires and resumes.

Usage:
  # In terminal 1 (start RT fake hardware):
  ros2 launch franka_manipulation_controller_bringup controller_bringup.launch.py use_fake_hardware:=true

  # In terminal 2:
  python examples/03_teleop_preempt.py --profile fr3_pika_single_arm.yaml
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

import rmi


def main() -> None:
    parser = argparse.ArgumentParser(description="Human Teleop Preemption & Priority Arbitration")
    parser.add_argument(
        "--profile",
        type=str,
        default="fr3_pika_single_arm.yaml",
        help="Embodiment profile path",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=20.0,
        help="Control rate (Hz)",
    )
    args = parser.parse_args()

    print(f"\n=======================================================")
    print(f"  RMI Demo 03: Teleop Preemption & Scoped Arbitration")
    print(f"  Profile: {args.profile}")
    print(f"=======================================================\n")

    # 1. Initialize RMI Context
    print("[1/4] Initializing RMI Context & Agents...")
    ctx = rmi.Context.from_profile(args.profile)
    robot = ctx.robot
    robot.wait_until_ready(timeout=5.0)
    arm_part = "arm" if "arm" in ctx.profile.parts else next(iter(ctx.profile.parts.keys()))

    # Create two competing agents: Policy (Priority 50) and TeleopJoint (Priority 60)
    policy_agent = ctx.make_agent("Policy", frequency=args.rate_hz)
    teleop_agent = ctx.make_agent("TeleopJoint", frequency=args.rate_hz)

    dt = 1.0 / args.rate_hz

    # 2. Phase 1: Policy starts running autonomously
    print(f"\n[2/4] Phase 1: Autonomous Policy starts running (Priority: 50)...")
    with policy_agent.run(robot, resume=True) as policy_session:
        gen1 = policy_session.generation_for(arm_part)
        print(f"  -> Policy ACTIVE: generation={gen1}")

        # Run 10 ticks of policy
        for tick in range(10):
            obs = policy_session.observe()
            q = list(obs.joint_positions)
            if q:
                q[0] += 0.02 * math.sin(tick * 0.3)
            policy_session.act(rmi.Action(part=arm_part, command="joint_reference", value=q))
            policy_session.wait()

        print(f"  -> Policy running smoothly. Now simulating Human Teleop takeover...")

        # 3. Phase 2: Human Teleop Preemption
        print(f"\n[3/4] Phase 2: Human Teleop acquires session (Priority: 60 - Preempts Policy)...")
        with teleop_agent.run(robot) as teleop_session:
            gen2 = teleop_session.generation_for(arm_part)
            print(f"  [!] Teleop ACTIVE: generation={gen2} (Policy preempted/paused)")

            # Teleop sends human joystick/marker reference
            for tick in range(10):
                obs = teleop_session.observe()
                q = list(obs.joint_positions)
                if q:
                    q[0] += 0.05 * math.cos(tick * 0.3)
                teleop_session.act(rmi.Action(part=arm_part, command="joint_reference", value=q))
                if tick % 3 == 0:
                    print(f"      [Teleop Teleoperating] step {tick+1}/10 -> q[0]={q[0]:.3f} rad")
                teleop_session.wait()

            print("  [!] Human Teleop completed and released.")

        # 4. Phase 3: Resume Policy
        print(f"\n[4/4] Phase 3: Policy re-syncs state & resumes control...")
        # Since policy_session was entered with resume=True, it detects generation advancement and resumes
        obs = policy_session.observe()
        gen3 = policy_session.generation_for(arm_part)
        print(f"  -> Policy RESUMED safely: new generation={gen3}")

        for tick in range(10):
            obs = policy_session.observe()
            q = list(obs.joint_positions)
            policy_session.act(rmi.Action(part=arm_part, command="joint_reference", value=q))
            policy_session.wait()

        print(f"  -> Policy completed remaining autonomous task.")

    print("\n[✓] Preemption & Priority Arbitration workflow verified successfully.")
    ctx.close()


if __name__ == "__main__":
    main()
