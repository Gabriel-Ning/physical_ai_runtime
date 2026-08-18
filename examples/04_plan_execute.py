#!/usr/bin/env python3
"""04_plan_execute.py: Segment Trajectory Planning with cuRobo & Guarded Execution.

Demonstrates the **Planner Family**:
1. Solves a complete time-parameterized joint trajectory from current state to a target pose.
2. Dispatches the full trajectory to the RT Host's JointTrajectoryController (JTC).
3. Monitored by the RT `joint_trajectory_controller_guard` for safety/cancellation.

Usage:
  # In terminal 1 (start RT fake hardware):
  ros2 launch franka_manipulation_controller_bringup controller_bringup.launch.py use_fake_hardware:=true

  # In terminal 2:
  python examples/04_plan_execute.py --profile fr3_pika_single_arm.yaml
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import rmi
from motion_planner_core.contracts import CartesianState
from support import PROFILE_TO_CUROBO, create_curobo_trajectory_planner


def main() -> None:
    parser = argparse.ArgumentParser(
        description="cuRobo Segment Trajectory Planning & Execution"
    )
    parser.add_argument(
        "--profile",
        type=str,
        default="fr3_pika_single_arm.yaml",
        help="Embodiment profile name or path (e.g. fr3_pika_single_arm.yaml)",
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
    default_target = info["default_target_pose"]

    print("\n=======================================================")
    print("  RMI Demo 04: Segment Trajectory Planner (cuRobo -> JTC)")
    print(f"  Profile: {args.profile}")
    print(f"  Arm Part: {arm_part}")
    print("=======================================================\n")

    # 1. Initialize pure RMI Context
    print("[1/4] Connecting to robot via RMI SDK...")
    ctx = rmi.Context.from_profile(args.profile)
    ctx.wait_until_ready(timeout=5.0)
    robot = ctx.robot
    obs = robot.get_observation()
    print(f"    Current joint positions: {[round(x, 4) for x in obs.joint_positions]}")

    # 2. Build cuRobo Trajectory Planner
    print(f"\n[2/4] Initializing cuRobo Trajectory Planner ({args.device})...")
    planner_backend = create_curobo_trajectory_planner(args.profile, device=args.device)
    planner = rmi.Planner("curobo_planner", planner_backend)

    # 3. Plan trajectory to target pose
    target = CartesianState(
        position_xyz=tuple(default_target["position"]),
        orientation_wxyz=tuple(default_target["orientation"]),
    )
    print("\n[3/4] Planning collision-free trajectory to target:")
    print(f"    Target Pos: {target.position_xyz}")
    print(f"    Target Ori: {target.orientation_wxyz}")

    plan_result = planner.plan(robot=robot, target=target)
    if not plan_result.valid or not plan_result.points:
        print(f"[X] Planning failed: {plan_result.reason}")
        sys.exit(1)

    num_points = len(plan_result.points)
    duration_s = plan_result.points[-1].time_from_start_s if num_points > 0 else 0.0
    print(
        f"    [✓] Plan succeeded! {num_points} waypoints, duration = {duration_s:.2f}s"
    )

    # 4. Execute trajectory on RT Host via JTC Action in a scoped session
    print("\n[4/4] Executing trajectory via JTC Action Client...")
    agent = ctx.make_agent("Planner")
    with agent.run(robot) as session:
        execution = session.execute(arm_part, plan_result)
        print(
            f"    Action goal submitted (correlation_id = {execution.correlation_id})"
        )

        # Wait for completion
        execution.wait(timeout=duration_s + 5.0)
        if execution.done and not execution.canceled:
            print("\n[✓] Trajectory execution completed successfully!")
        else:
            print(f"\n[!] Trajectory execution status: {execution.state.name}")

    ctx.close()


if __name__ == "__main__":
    main()
