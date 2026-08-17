#!/usr/bin/env python3
"""09_joint_streamer.py: Receding Horizon Joint Streamer & VLA/Diffusion Action Chunking.

Demonstrates the **Streamer Family**:
1. Receding horizon online guidance via cuRobo MPC backend OR a Dummy VLA / Diffusion Policy.
2. Generates an N-step action chunk `[T, DoF]`.
3. Streams the multi-point `joint_reference` trajectory chunk directly to the RT Host's JSPC.

Usage:
  # In terminal 1 (start RT fake hardware):
  ros2 launch franka_manipulation_controller_bringup controller_bringup.launch.py use_fake_hardware:=true

  # In terminal 2 (using Dummy VLA Policy):
  python examples/09_joint_streamer.py --profile fr3_pika_single_arm.yaml --backend vla_dummy

  # Or using cuRobo MPC:
  python examples/09_joint_streamer.py --profile fr3_pika_single_arm.yaml --backend curobo_mpc
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Any

import rmi
from motion_planner_core.contracts import JointState
from support import (
    PROFILE_TO_CUROBO,
    create_curobo_joint_streamer,
    make_circular_target,
)


class DummyVlaPolicy:
    """Simulates a VLA (e.g. OpenVLA / Octo) or Diffusion Policy returning action chunks."""

    def __init__(self, horizon: int = 8, dt: float = 0.033, num_dof: int = 7) -> None:
        self.horizon = horizon
        self.dt = dt
        self.num_dof = num_dof

    def predict(self, observation: rmi.Observation, t: float) -> list[list[float]]:
        """Compute an action chunk [horizon, DoF] based on current observation and task."""
        current_q = observation.joint_positions
        if not current_q:
            current_q = [0.0] * self.num_dof

        # Generate a smooth receding horizon wave (simulating policy rollout chunk)
        chunk = []
        for step in range(1, self.horizon + 1):
            future_t = t + step * self.dt
            point = list(current_q)
            # Apply smooth sinusoidal perturbation to first few joints
            point[0] += 0.05 * math.sin(1.5 * future_t)
            point[1] += 0.03 * math.cos(1.5 * future_t)
            chunk.append(point)
        return chunk


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Receding Horizon Joint Streamer & Action Chunking"
    )
    parser.add_argument(
        "--profile",
        type=str,
        default="fr3_pika_single_arm.yaml",
        help="Embodiment profile name or path (e.g. fr3_pika_single_arm.yaml)",
    )
    parser.add_argument(
        "--backend",
        type=str,
        default="vla_dummy",
        choices=["vla_dummy", "curobo_mpc"],
        help="Streaming backend: vla_dummy (VLA/Diffusion Policy Action Chunker) or curobo_mpc",
    )
    parser.add_argument(
        "--horizon",
        type=int,
        default=8,
        help="Horizon size (number of points per chunk)",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=30.0,
        help="Loop rate (Hz)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Run duration (seconds)",
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
        print(
            f"[X] Unsupported profile {args.profile!r}. Available: {list(PROFILE_TO_CUROBO.keys())}"
        )
        sys.exit(1)
    info = PROFILE_TO_CUROBO[profile_key]
    arm_part = info["arm_part"]
    center_pos = info["default_target_pose"]["position"]

    print("\n=======================================================")
    print("  RMI Demo 09: Receding Horizon Joint Streamer -> JSPC")
    print(f"  Profile: {args.profile}")
    print(f"  Arm Part: {arm_part}")
    print(
        f"  Backend: {args.backend} (Horizon = {args.horizon}, Rate = {args.rate_hz} Hz)"
    )
    print("=======================================================\n")

    # 1. Initialize RMI Context & Agent
    print("[1/3] Initializing RMI Context & Agent...")
    ctx = rmi.Context.from_profile(args.profile)
    robot = ctx.robot
    robot.wait_until_ready(timeout=5.0)
    agent = ctx.make_agent("Policy", frequency=args.rate_hz)

    # 2. Setup chosen backend
    streamer_backend: Any = None
    policy: DummyVlaPolicy | None = None

    if args.backend == "curobo_mpc":
        print(f"[2/3] Initializing cuRobo MPC Streamer ({args.device})...")
        streamer_backend = create_curobo_joint_streamer(
            args.profile,
            horizon=args.horizon,
            dt=1.0 / args.rate_hz,
            device=args.device,
        )
    else:
        print("[2/3] Initializing Dummy VLA / Diffusion Policy...")
        num_dof = len(info["home_joint_positions"])
        policy = DummyVlaPolicy(
            horizon=args.horizon, dt=1.0 / args.rate_hz, num_dof=num_dof
        )

    # Optional: Setup Interactive Marker in RViz
    from marker_support import InteractivePoseTarget, lookup_tip_pose

    part_cfg = ctx.profile.parts[arm_part]
    base_frame = part_cfg.base_frame or "base_link"
    tip_frame = part_cfg.tcp_frame or part_cfg.flange_frame or "tool0"
    initial_pose = lookup_tip_pose(ctx.node, base_frame=base_frame, tip_frame=tip_frame)
    marker = InteractivePoseTarget(
        ctx.node,
        frame_id=base_frame,
        initial=initial_pose,
        description=f"Streamer Target ({arm_part})",
    )
    print(
        "  [✓] Interactive Marker active! In RViz, select 'Interact' tool and drag the marker."
    )

    # 3. Stream receding-horizon action chunks
    print("[3/3] Starting action chunk stream...")
    dt = 1.0 / args.rate_hz
    step_count = 0

    try:
        with agent.run(robot) as session:
            start_time = time.time()
            while session.ok() and (time.time() - start_time) < args.duration:
                t = time.time() - start_time

                # Acquire synchronized observation
                obs = session.observe()

                chunk: list[list[float]] = []

                if policy is not None:
                    # 1. Policy inference step
                    chunk = policy.predict(obs, t)
                elif streamer_backend is not None:
                    # 2. cuRobo MPC step with interactive marker
                    target = (
                        marker.current()
                        if marker.user_moved
                        else make_circular_target(
                            center_pos, radius=0.06, t=t, speed=1.0
                        )
                    )
                    streamer_backend.update_target(target)
                    current_js = JointState(
                        joint_names=streamer_backend.joint_names,
                        positions=obs.joint_positions[
                            : len(streamer_backend.joint_names)
                        ],
                    )
                    step_res = streamer_backend.step(current_state=current_js, dt=dt)
                    if (
                        step_res.valid
                        and getattr(step_res, "positions", None) is not None
                    ):
                        chunk = [list(step_res.positions)]
                    elif step_res.valid and getattr(step_res, "points", None):
                        chunk = [list(p.positions) for p in step_res.points]

                if not chunk:
                    time.sleep(dt)
                    continue

                # Stream the action chunk (trajectory_msgs/JointTrajectory) to JSPC
                session.act(
                    rmi.Action(
                        part=arm_part,
                        command="joint_reference",
                        value=chunk,
                    )
                )

                step_count += 1
                if step_count % int(args.rate_hz) == 0:
                    print(
                        f"  [t={t:5.1f}s] Streamed Chunk: shape=[{len(chunk)}, {len(chunk[0])}], "
                        f"lead point: {[round(x, 2) for x in chunk[0][:3]]}..."
                    )

                session.wait()
    except KeyboardInterrupt:
        print("\n[!] Interrupted by user")
    finally:
        marker.close()

    print(
        f"\n[✓] Joint streaming completed successfully ({step_count} chunks dispatched)."
    )
    ctx.close()


if __name__ == "__main__":
    main()
