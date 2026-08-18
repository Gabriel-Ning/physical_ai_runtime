#!/usr/bin/env python3
"""07_record_episode.py: High-Quality Synchronized Episode Dataset Recording.

Demonstrates **Multi-Modal Demonstration Recording (LeRobot / MCAP / DROID / RL Compatible)**:
1. Connects to the Franka FR3 + Pika Manipulation embodiment profile.
2. Supports Dual-Mode Recording:
   - ``--type mcap`` (Default): High-quality offline MCAP dataset (Auto-managed C++ engine, SHA-256 verified).
   - ``--type memory``: Online reinforcement learning experience replay buffer (crisp_gym compatible).
3. Executes a synchronized 30Hz demonstration trajectory (arm sinusoidal wave + gripper cycles).
4. Atomically finalizes the dataset episode with rich metadata on session completion.

Usage:
  # Real Robot 30Hz, 1 minute (1800 ticks, MCAP):
  python examples/07_record_episode.py --profile fr3_pika_single_arm.yaml --task pika_manipulation_demo --rate-hz 30 --duration 60 --type mcap

  # Online RL Memory Buffer Test (10 seconds, 300 ticks):
  python examples/07_record_episode.py --profile fr3_pika_single_arm.yaml --task pika_rl_demo --rate-hz 30 --duration 10 --type memory
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
RMI_SRC = WORKSPACE_ROOT / "src" / "interfaces" / "rmi"
if RMI_SRC.exists() and str(RMI_SRC) not in sys.path:
    sys.path.insert(0, str(RMI_SRC))

EPISODE_RECORDER_SRC = (
    WORKSPACE_ROOT / "src" / "recording" / "episode_recorder" / "python"
)
if EPISODE_RECORDER_SRC.exists() and str(EPISODE_RECORDER_SRC) not in sys.path:
    sys.path.insert(0, str(EPISODE_RECORDER_SRC))

PLANNER_CORE_SRC = (
    WORKSPACE_ROOT / "src" / "motion_planning" / "motion_planners" / "motion_planner_core"
)
if PLANNER_CORE_SRC.exists() and str(PLANNER_CORE_SRC) not in sys.path:
    sys.path.insert(0, str(PLANNER_CORE_SRC))

import rmi


class ProgressBar:
    """Adaptive progress bar with tqdm fallback and smooth terminal telemetry."""

    def __init__(self, total: int, desc: str = "Recording", unit: str = "step") -> None:
        self.total = total
        self.desc = desc
        self.unit = unit
        self.n = 0
        self._start_time = time.time()
        self._tqdm = None
        try:
            from tqdm import tqdm
            self._tqdm = tqdm(total=total, desc=desc, unit=unit, ncols=90, leave=True)
        except ImportError:
            self._tqdm = None

    def update(self, n: int = 1, postfix: dict[str, Any] | None = None) -> None:
        self.n += n
        if self._tqdm is not None:
            if postfix:
                self._tqdm.set_postfix(postfix)
            self._tqdm.update(n)
        else:
            pct = (self.n / self.total) * 100.0 if self.total else 0.0
            elapsed = time.time() - self._start_time
            rate = self.n / elapsed if elapsed > 0 else 0.0
            bar_len = 25
            filled = int(bar_len * (self.n / self.total)) if self.total else 0
            bar = "█" * filled + "░" * (bar_len - filled)
            extra = ""
            if postfix:
                extra = " | " + " ".join(f"{k}={v}" for k, v in postfix.items())
            sys.stdout.write(
                f"\r  {self.desc}: [{bar}] {pct:5.1f}% ({self.n}/{self.total} @ {rate:4.1f}Hz{extra})"
            )
            sys.stdout.flush()

    def close(self) -> None:
        if self._tqdm is not None:
            self._tqdm.close()
        else:
            sys.stdout.write("\n")
            sys.stdout.flush()


def main() -> None:
    parser = argparse.ArgumentParser(description="Unified High-Quality Episode Dataset Recorder")
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
        "--duration",
        type=float,
        default=60.0,
        help="Recording duration in seconds (e.g. 60.0 for 1 minute)",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=30.0,
        help="Recording loop rate (Hz)",
    )
    parser.add_argument(
        "--type",
        type=str,
        choices=["mcap", "memory"],
        default="mcap",
        help="Recorder mode: 'mcap' for offline dataset, 'memory' for online RL buffer",
    )
    args = parser.parse_args()

    total_ticks = int(args.duration * args.rate_hz)

    print(f"\n=======================================================")
    print(f"  RMI Demo 07: Synchronized Episode Dataset Recorder")
    print(f"  Profile: {args.profile}")
    print(f"  Task: {args.task}")
    print(f"  Mode: {args.type.upper()} | Rate: {args.rate_hz} Hz | Duration: {args.duration} s ({total_ticks} steps)")
    print(f"=======================================================\n")

    # 1. Initialize RMI Context & Dual-Mode Recorder
    print("[1/3] Initializing Context, Agents, and Recorder Backend...")
    ctx = rmi.Context.from_profile(args.profile)
    ctx.wait_until_ready(timeout=5.0)
    robot = ctx.robot
    arm_part = "arm" if "arm" in ctx.profile.parts else next(iter(ctx.profile.parts.keys()))
    gripper_part = "end_effector" if "end_effector" in ctx.profile.parts else None

    agent = ctx.make_agent("Policy", frequency=args.rate_hz)
    recorder = ctx.make_recorder(type=args.type, autostart=True)
    print(f"  [✓] Recorder initialized ({args.type.upper()} Mode).")

    # 2. Record Episode Scope
    print(f"\n[2/3] Starting Demonstration Recording ({total_ticks} steps at {args.rate_hz} Hz)...")
    dt = 1.0 / args.rate_hz

    metadata = {
        "profile": args.profile,
        "robot": ctx.profile.name,
        "rate_hz": args.rate_hz,
        "duration_s": args.duration,
        "mode": args.type,
    }

    try:
        with recorder.episode(task=args.task, metadata=metadata) as ep:
            with agent.run(robot) as session:
                initial_obs = session.observe()
                home_q = list(initial_obs.joint_positions)
                print(f"  [✓] Control session active. Initial Arm Q[0]={home_q[0]:.3f} rad\n")

                pbar = ProgressBar(total=total_ticks, desc="Recording Episode", unit="step")
                try:
                    for step in range(1, total_ticks + 1):
                        t = step * dt

                        # 1. Observe state
                        obs = session.observe()

                        # 2. Generate synchronized demonstration action wave
                        target_q = list(home_q)
                        if len(target_q) >= 4:
                            # Gentle arm sinusoidal oscillation (±0.04 rad base, ±0.03 rad elbow)
                            target_q[0] += 0.04 * math.sin(1.0 * t)
                            target_q[3] += 0.03 * math.cos(1.0 * t)

                        arm_action = rmi.Action(part=arm_part, command="joint_reference", value=target_q)
                        session.act(arm_action)

                        # Gentle gripper cycle (0.015m to 0.035m)
                        actions = [arm_action]
                        if gripper_part:
                            grip_width = 0.025 + 0.010 * math.sin(0.8 * t)
                            gripper_action = rmi.Action(part=gripper_part, command="joint_reference", value=[grip_width])
                            session.act(gripper_action)
                            actions.append(gripper_action)

                        # 4. If memory mode, step the in-memory replay buffer
                        if args.type == "memory" and hasattr(recorder, "step"):
                            recorder.step(obs, actions, reward=0.0, done=(step == total_ticks))

                        # 5. Progress telemetry
                        q0 = obs.joint_positions[0] if obs.joint_positions else 0.0
                        pbar.update(1, postfix={"Q[0]": f"{q0:.3f}rad"})

                        session.wait()
                finally:
                    pbar.close()

            print(f"\n[3/3] Finalizing Episode Dataset (verifying capture health & sealing)...")
    except KeyboardInterrupt:
        print("\n[!] Recording interrupted by user.")
    finally:
        ctx.close()

    if args.type == "memory":
        print(f"  [✓] Replay buffer populated with {len(recorder)} transitions.")
    else:
        print("  [✓] MCAP episode dataset sealed and persisted to disk (ready for LeRobot training).")
        print("  [💡] To index an episode for Foxglove Studio interactive inspection, run:")
        print("       python src/recording/episode_recorder/scripts/episode_index_mcap.py data/episodes/.../episode_XXXXXX")

    print("\n[✓] Demonstration recording workflow finished successfully.")


if __name__ == "__main__":
    main()

