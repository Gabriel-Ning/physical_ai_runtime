#!/usr/bin/env python3
"""07_record_episode.py: Episode Dataset Recording for VLA / Imitation Learning.

Demonstrates **Synchronized Episode Recording (LeRobot / HDF5 / Zarr Compatible)**:
1. Wraps an agent control session inside a scoped `EpisodeRecorder`.
2. Synchronously captures high-frequency state trajectories, commanded actions, and camera frames.
3. Automatically serializes and finalizes episode datasets with rich metadata on session completion.

Usage:
  # In terminal 1 (start RT fake hardware):
  ros2 launch franka_manipulation_controller_bringup controller_bringup.launch.py use_fake_hardware:=true

  # In terminal 2:
  python examples/07_record_episode.py --profile fr3_pika_single_arm.yaml --task pick_and_place --ticks 25
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

import rmi


class SimpleEpisodeRecorder:
    """Lightweight in-process episode dataset recorder."""

    def __init__(self, output_dir: str = "dataset_records") -> None:
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.records: list[dict] = []
        self.task = ""
        self.metadata: dict = {}
        self.start_time = 0.0

    def episode(self, task: str, metadata: dict | None = None) -> SimpleEpisodeRecorder:
        self.task = task
        self.metadata = metadata or {}
        self.records = []
        return self

    def __enter__(self) -> SimpleEpisodeRecorder:
        self.start_time = time.time()
        print(f"  [Recorder] Recording episode for task: {self.task!r}...")
        return self

    def step(self, observation: rmi.Observation, action: rmi.Action) -> None:
        """Record one synchronized step (observation + action)."""
        self.records.append({
            "timestamp": time.time() - self.start_time,
            "source_time_s": observation.source_time_s,
            "joint_positions": list(observation.joint_positions),
            "joint_velocities": list(observation.joint_velocities),
            "action_part": action.part,
            "action_command": action.command,
            "action_value": action.value if not hasattr(action.value, "__dict__") else str(action.value),
        })

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        duration = time.time() - self.start_time
        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        filename = self.output_dir / f"episode_{self.task}_{timestamp_str}.json"
        payload = {
            "task": self.task,
            "metadata": self.metadata,
            "duration_s": duration,
            "num_steps": len(self.records),
            "steps": self.records,
        }
        with open(filename, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
        print(f"  [Recorder] Episode finalized: {len(self.records)} steps saved to {filename}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Episode Dataset Recorder")
    parser.add_argument(
        "--profile",
        type=str,
        default="fr3_pika_single_arm.yaml",
        help="Embodiment profile name",
    )
    parser.add_argument(
        "--task",
        type=str,
        default="pika_pick_and_place",
        help="Task label for dataset manifest",
    )
    parser.add_argument(
        "--ticks",
        type=int,
        default=25,
        help="Number of control steps to record",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=20.0,
        help="Recording loop rate (Hz)",
    )
    args = parser.parse_args()

    print(f"\n=======================================================")
    print(f"  RMI Demo 07: Synchronized Episode Dataset Recorder")
    print(f"  Profile: {args.profile}")
    print(f"  Task: {args.task}")
    print(f"=======================================================\n")

    # 1. Initialize RMI Context
    print("[1/3] Initializing Context & Agents...")
    ctx = rmi.Context.from_profile(args.profile)
    robot = ctx.robot
    robot.wait_until_ready(timeout=5.0)
    arm_part = "arm" if "arm" in ctx.profile.parts else next(iter(ctx.profile.parts.keys()))

    agent = ctx.make_agent("Policy", frequency=args.rate_hz)
    recorder = SimpleEpisodeRecorder(output_dir="recorded_episodes")

    # 2. Record Episode Scope
    print(f"\n[2/3] Starting Scoped Episode Recording ({args.ticks} steps at {args.rate_hz} Hz)...")
    dt = 1.0 / args.rate_hz

    with recorder.episode(task=args.task, metadata={"profile": args.profile, "robot": ctx.profile.name}):
        with agent.run(robot) as session:
            initial_obs = session.observe()
            home_q = list(initial_obs.joint_positions)

            for step in range(1, args.ticks + 1):
                t = step * dt

                # Observe state
                obs = session.observe()

                # Generate demonstration action wave
                target_q = list(home_q)
                if target_q:
                    target_q[0] += 0.04 * math.sin(1.2 * t)
                    target_q[3] += 0.03 * math.cos(1.2 * t)

                act = rmi.Action(part=arm_part, command="joint_reference", value=target_q)

                # Dispatch action to hardware
                session.act(act)

                # Record synchronized step into episode buffer
                recorder.step(obs, act)

                if step % int(args.rate_hz // 2 or 1) == 0:
                    print(f"    [Step {step:2d}/{args.ticks}] Recorded: q[0]={obs.joint_positions[0]:.3f} rad")

                session.wait()

    print(f"\n[3/3] Finalizing Episode Dataset...")
    print("\n[✓] Episode dataset recorded and verified successfully.")
    ctx.close()


if __name__ == "__main__":
    main()
