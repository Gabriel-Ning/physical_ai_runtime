#!/usr/bin/env python3
"""01_context.py: RMI Context initialization & Embodiment Inspection.

Demonstrates the core RMI SDK fundamentals:
1. Loading an embodiment profile (YAML) via `rmi.Context.from_profile()`.
2. Inspecting robot topology, parts, joints, controller mappings, and coordinate frames.
3. Reading synchronized multi-part robot states via `session.observe()`.

Usage:
  # In terminal 1 (start RT fake hardware):
  ros2 launch franka_manipulation_controller_bringup controller_bringup.launch.py use_fake_hardware:=true

  # In terminal 2:
  python examples/01_context.py --profile fr3_pika_single_arm.yaml
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import rmi


def main() -> None:
    parser = argparse.ArgumentParser(description="RMI Context & Embodiment Introspection")
    parser.add_argument(
        "--profile",
        type=str,
        default="fr3_pika_single_arm.yaml",
        help="Embodiment profile name or path (e.g. fr3_pika_single_arm.yaml, marvin_bimanual.yaml)",
    )
    args = parser.parse_args()

    print(f"\n=======================================================")
    print(f"  RMI Demo 01: Context & Embodiment Introspection")
    print(f"  Profile: {args.profile}")
    print(f"=======================================================\n")

    # 1. Open RMI Context
    print("[1/3] Loading Embodiment Profile & Initializing RMI Context...")
    ctx = rmi.Context.from_profile(args.profile)
    profile = ctx.profile

    print(f"\nEmbodiment: {profile.name}")
    print(f"Embodiment Type: {profile.embodiment_type}")
    print(f"Vendor: {profile.vendor}")
    print(f"Parts Count: {len(profile.parts)}")

    # 2. Inspect Parts and Controllers
    print(f"\n[2/3] Inspecting Robot Parts ({len(profile.parts)} declared):")
    for part_name, part in profile.parts.items():
        print(f"  • Part: {part_name!r} (type: {part.part_type})")
        print(f"    - Base Frame: {part.base_frame}")
        print(f"    - TCP Frame:  {part.tcp_frame or part.flange_frame or 'N/A'}")
        print(f"    - Joints ({len(part.joint_names)}): {part.joint_names}")
        if part.controllers:
            print(f"    - Controllers:")
            for contract, ctrl in part.controllers.items():
                print(f"        [{contract}]: name={ctrl.name!r} (impl: {ctrl.implementation}, cmd: {ctrl.command_interface})")

    if profile.execution and "providers" in profile.execution:
        print(f"\n  Declared Execution Providers ({len(profile.execution['providers'])}):")
        for prov_name, prov_cfg in profile.execution["providers"].items():
            prio = prov_cfg.get("priority", 0)
            ctrls = prov_cfg.get("controllers", {})
            print(f"    • {prov_name:<18} (Priority: {prio:>2}) -> {ctrls}")

    # 3. Connect to Robot & Read Live State
    print(f"\n[3/3] Connecting to Robot & Reading Live State...")
    robot = ctx.robot
    robot.wait_until_ready(timeout=5.0)

    agent = ctx.make_agent("Policy", frequency=10.0)
    with agent.run(robot) as session:
        obs = session.observe()
        print("\n  Live Observation:")
        print(f"    - Source Time:  {obs.source_time_s:.4f}s")
        print(f"    - Receive Time: {obs.receive_time_s:.4f}s")
        print(f"    - Allocations:  {robot.execution.get_allocations()}")
        print(f"    - Joint Names:  {obs.joint_names}")
        print(f"    - Joint Positions (rad): {[round(x, 4) for x in obs.joint_positions]}")
        print(f"    - Joint Velocities:     {[round(x, 4) for x in obs.joint_velocities]}")

    print("\n[✓] Context & Robot state introspection completed successfully.")
    ctx.close()


if __name__ == "__main__":
    main()
