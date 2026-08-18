#!/usr/bin/env python3
"""05_policy_recovery.py: Planner-in-the-Loop & Autonomous Trajectory Recovery.

Demonstrates **Hierarchical Execution & Policy Recovery (DAgger / Planner-in-the-Loop)**:
1. Low-level Policy Agent runs real-time online streaming control (e.g. VLA / Diffusion).
2. Upon encountering an anomaly, collision boundary, or human intervention signal, the system triggers
   a Trajectory Planner Recovery.
3. The Planner acquires the robot, computes a global collision-free trajectory back to the safe envelope,
   and executes it via JTC.
4. Once recovered, control is cleanly handed back to the Policy Agent.

Usage:
  # In terminal 1 (start RT fake hardware):
  ros2 launch franka_manipulation_controller_bringup controller_bringup.launch.py use_fake_hardware:=true

  # In terminal 2:
  python examples/05_policy_recovery.py --profile fr3_pika_single_arm.yaml
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

from motion_planner_core.contracts import CartesianState
import rmi
from support import PROFILE_TO_CUROBO, create_curobo_trajectory_planner


def main() -> None:
    parser = argparse.ArgumentParser(description="Planner Recovery & Policy Handover")
    parser.add_argument(
        "--profile",
        type=str,
        default="fr3_pika_single_arm.yaml",
        help="Embodiment profile name",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=20.0,
        help="Policy loop rate (Hz)",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda",
        help="Compute device for cuRobo",
    )
    args = parser.parse_args()

    profile_key = Path(args.profile).name
    if profile_key not in PROFILE_TO_CUROBO:
        print(f"[X] Unsupported profile {args.profile!r}. Available: {list(PROFILE_TO_CUROBO.keys())}")
        sys.exit(1)
    info = PROFILE_TO_CUROBO[profile_key]
    arm_part = info["arm_part"]
    safe_target_pos = info["default_target_pose"]["position"]

    print(f"\n=======================================================")
    print(f"  RMI Demo 05: Planner-in-the-Loop & Recovery Handover")
    print(f"  Profile: {args.profile}")
    print(f"=======================================================\n")

    # 1. Initialize RMI Context & Agents
    print("[1/4] Initializing RMI Context, Policy & Planner Agents...")
    ctx = rmi.Context.from_profile(args.profile)
    ctx.wait_until_ready(timeout=5.0)
    robot = ctx.robot
    arm_joints = ctx.profile.parts[arm_part].joint_names if arm_part in ctx.profile.parts else ()

    policy_agent = ctx.make_agent("Policy", frequency=args.rate_hz)
    planner_agent = ctx.make_agent("Planner")

    # 2. Build cuRobo Planner for Recovery
    print(f"[2/4] Initializing cuRobo Recovery Planner ({args.device})...")
    planner_backend = create_curobo_trajectory_planner(args.profile, device=args.device)
    planner = rmi.Planner("curobo", planner_backend)

    # 3. Policy Execution Phase (Pre-Anomaly)
    print(f"\n[3/4] Phase 1: Policy starts online execution...")
    with policy_agent.run(robot, resume=True) as policy_session:
        print(f"  -> Policy ACTIVE on part {arm_part!r} (gen={policy_session.generation_for(arm_part)})")

        for tick in range(1, 15):
            obs = policy_session.observe()
            q = list(obs.joint_positions[:len(arm_joints)])
            if q:
                # Simulate policy drifting toward boundary
                q[0] += 0.01 * tick
            policy_session.act(rmi.Action(part=arm_part, command="joint_reference", value=q))
            policy_session.wait()

        print(f"  [!] Anomaly detected: Robot drifted out-of-distribution! Triggering Planner Recovery...")

        # 4. Hierarchical Recovery Phase: Planner takes over
        print(f"\n[4/4] Phase 2: Planner takes over and computes recovery trajectory...")
        with planner_agent.run(robot) as planner_session:
            print(f"  -> Planner ACTIVE: Computing collision-free return path to safe pose {safe_target_pos}...")
            safe_target = CartesianState(
                position_xyz=tuple(safe_target_pos),
                orientation_wxyz=(0.0, 1.0, 0.0, 0.0),
            )
            plan = planner.plan(robot=robot, target=safe_target)
            if not plan.valid:
                print(f"[X] Recovery plan failed: {plan.reason}")
                return

            print(f"  [✓] Plan generated ({len(plan.points)} waypoints). Executing via JTC...")
            execution = planner_session.execute(arm_part, plan)
            execution.wait(timeout=10.0)
            print(f"  [✓] Recovery execution reached target: state={execution.state.name}")

        # 5. Handback to Policy
        print(f"\n  -> Recovery complete! Resuming Policy execution at safe state...")
        obs = policy_session.observe()
        print(f"  -> Policy RESUMED: new generation={policy_session.generation_for(arm_part)}")

        for tick in range(1, 10):
            obs = policy_session.observe()
            q = list(obs.joint_positions[:len(arm_joints)])
            policy_session.act(rmi.Action(part=arm_part, command="joint_reference", value=q))
            policy_session.wait()

    print("\n[✓] Planner-in-the-Loop Recovery completed successfully.")
    ctx.close()


if __name__ == "__main__":
    main()
