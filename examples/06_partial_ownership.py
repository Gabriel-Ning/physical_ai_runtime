#!/usr/bin/env python3
"""06_partial_ownership.py: Concurrent Multi-Part Partial Hardware Ownership.

Demonstrates **Multi-Part / Partial Hardware Scopes**:
1. In complex embodiments (bimanual or arm+gripper), different parts can be owned
   and driven by independent agents simultaneously.
2. For example, a Trajectory Planner can own and move the `arm` part while a Policy
   Agent concurrently commands the `gripper` / `end_effector` without resource conflict.

Usage:
  # In terminal 1 (start RT fake hardware):
  ros2 launch franka_manipulation_controller_bringup controller_bringup.launch.py use_fake_hardware:=true

  # In terminal 2:
  python examples/06_partial_ownership.py --profile fr3_pika_single_arm.yaml
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

from motion_planner_core.contracts import CartesianState
import rmi
from support import PROFILE_TO_CUROBO, create_curobo_trajectory_planner


def main() -> None:
    parser = argparse.ArgumentParser(description="Multi-Part Partial Ownership")
    parser.add_argument(
        "--profile",
        type=str,
        default="fr3_pika_single_arm.yaml",
        help="Embodiment profile name",
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
    target_pos = info["default_target_pose"]["position"]

    print(f"\n=======================================================")
    print(f"  RMI Demo 06: Concurrent Partial Hardware Ownership")
    print(f"  Profile: {args.profile}")
    print(f"=======================================================\n")

    # 1. Initialize RMI Context
    # 1. Initialize RMI Context
    print("[1/3] Initializing RMI Context & Agents...")
    ctx = rmi.Context.from_profile(args.profile)
    ctx.wait_until_ready(timeout=5.0)
    robot = ctx.robot

    # Determine arm and gripper parts
    all_parts = list(ctx.profile.parts.keys())
    gripper_parts = [
        name for name, p in ctx.profile.parts.items() if "gripper" in p.part_type or "effector" in name
    ]
    gripper_part = gripper_parts[0] if gripper_parts else None

    print(f"  Declared parts: {all_parts}")
    print(f"  Arm Part: {arm_part!r}, Gripper Part: {gripper_part!r}")

    # Build Planner and Policy Agents
    planner_agent = ctx.make_agent("Planner")
    policy_agent = ctx.make_agent("Policy", frequency=10.0)

    # 2. Build Planner for arm
    print(f"[2/3] Initializing cuRobo Planner ({args.device})...")
    planner_backend = create_curobo_trajectory_planner(args.profile, device=args.device)
    planner = rmi.Planner("curobo", planner_backend)

    # 3. Concurrent Scoped Execution
    print(f"[3/3] Executing Concurrent Partial Ownership...")

    # Policy owns gripper; Planner owns arm
    with policy_agent.run(robot, parts=[gripper_part] if gripper_part else [arm_part]) as policy_session:
        print(f"  -> Policy Agent acquired part: {gripper_part or arm_part!r}")

        with planner_agent.run(robot, parts=[arm_part]) as planner_session:
            print(f"  -> Planner Agent acquired part: {arm_part!r}")
            print(f"  -> Active Allocations: {robot.execution.get_allocations()}")

            # Plan trajectory for arm
            target = CartesianState(
                position_xyz=tuple(target_pos),
                orientation_wxyz=(0.0, 1.0, 0.0, 0.0),
            )
            plan = planner.plan(robot=robot, target=target)
            if not plan.valid:
                print(f"[X] Plan failed: {plan.reason}")
                return

            print(f"  -> Planner executing trajectory on {arm_part!r} ({len(plan.points)} points)...")
            execution = planner_session.execute(arm_part, plan)

            # While arm is executing under Planner, Policy concurrently commands the gripper
            for tick in range(1, 8):
                if gripper_part:
                    # Alternate gripper position
                    grip_pos = [0.04 if tick % 2 == 0 else 0.0]
                    policy_session.act(rmi.Action(
                        part=gripper_part,
                        command="joint_reference",
                        value=grip_pos,
                    ))
                    print(f"     [Policy Concurrent] Tick {tick}: Commanded {gripper_part} -> {grip_pos}")
                time.sleep(0.3)

            # Wait for arm trajectory to finish
            result = execution.wait(timeout=10.0)
            print(f"  [✓] Arm trajectory completed: state={execution.state.name}")

    print("\n[✓] Concurrent Partial Ownership demo completed successfully.")
    ctx.close()


if __name__ == "__main__":
    main()
