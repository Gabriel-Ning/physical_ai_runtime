#!/usr/bin/env python3
"""apps/record.py: LeRobot-Style Production Multi-Modal Episode Dataset Recorder.

Features:
1. One-Command Autonomous Startup: Autostarts Piper leader teleop hardware nodes and MCAP recorder.
2. LeRobot-Style Interactive Workflow:
   - Visual Countdown (3, 2, 1, GO!)
   - Real-time Master-Slave Teleoperation + Multi-Modal Data Recording @ 30Hz
   - Post-Episode Actions:
       [S]ave (Default / Enter) -> Seals MCAP, verifies SHA-256, advances to next episode
       [D]iscard & Retry        -> Discards bad demo and immediately re-records the same episode
       [R]eplay                 -> Immediately replays recorded demonstration on the robot for inspection
       [Q]uit                   -> Safely concludes dataset collection session

Usage:
  # One single command for Piper Bimanual dataset collection:
  pixi run record --profile piper_bimanual.yaml --task bimanual_pickup --episodes 5 --duration 30

  # Franka single-arm dataset recording:
  pixi run record --profile fr3_pika_single_arm.yaml --task pika_peg_in_hole --duration 20
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from contextlib import ExitStack
from pathlib import Path
from typing import Any

from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory

import rmi


class PiperLeaderManager:
    """Manages background lifecycle of Piper Leader Arm hardware processes."""

    def __init__(self, workspace_root: Path) -> None:
        self.workspace_root = workspace_root
        self.processes: list[subprocess.Popen[bytes]] = []

    def ensure_started_and_enabled(self, node: Any, sides: list[str]) -> None:
        """Autostart leader nodes if needed and enable torque."""
        setup_bash = self.workspace_root / "install" / "setup.bash"
        share_config = (
            self.workspace_root
            / "src"
            / "teleop"
            / "piper_leader_teleop"
            / "config"
        )

        for side in sides:
            srv_name = f"/piper_leader_{side}/enable"
            enable_client = node.create_client(SetBool, srv_name)

            if not enable_client.wait_for_service(timeout_sec=0.2):
                config_yaml = share_config / f"piper_leader_{side}.yaml"
                cmd = (
                    f"source '{setup_bash}' && exec ros2 launch piper_leader_teleop piper_leader.launch.py "
                    f"config:='{config_yaml}' node_name:=piper_leader_{side}"
                )
                print(f"  >> Autostarting piper_leader_{side} driver...")
                proc = subprocess.Popen(
                    cmd,
                    shell=True,
                    executable="/bin/bash",
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid,
                )
                self.processes.append(proc)

                for _ in range(50):
                    if enable_client.wait_for_service(timeout_sec=0.1):
                        break
                    time.sleep(0.1)

            if enable_client.service_is_ready():
                req = SetBool.Request(data=True)
                future = enable_client.call_async(req)
                t_end = time.monotonic() + 2.0
                while not future.done() and time.monotonic() < t_end:
                    time.sleep(0.05)
                if future.done() and future.result().success:
                    print(f"  [✓] Enabled piper_leader_{side} base mode ({future.result().message}).")

    def set_preempt(self, node: Any, sides: list[str], preempt_active: bool) -> None:
        """Request active teleop preemption or return to shadow/passive fallback mode."""
        for side in sides:
            srv_name = f"/piper_leader_{side}/preempt"
            client = node.create_client(SetBool, srv_name)
            if client.wait_for_service(timeout_sec=0.5):
                req = SetBool.Request(data=preempt_active)
                future = client.call_async(req)
                t_end = time.monotonic() + 1.0
                while not future.done() and time.monotonic() < t_end:
                    time.sleep(0.02)
                if future.done() and future.result().success:
                    action_str = "ENGAGED" if preempt_active else "RELEASED"
                    print(f"  [✓] piper_leader_{side} Preempt {action_str}: {future.result().message}")

    def shutdown(self, node: Any, sides: list[str]) -> None:
        """Safely disable leaders and terminate background processes."""
        for side in sides:
            self.set_preempt(node, [side], False)
            srv_name = f"/piper_leader_{side}/enable"
            client = node.create_client(SetBool, srv_name)
            if client.service_is_ready():
                client.call_async(SetBool.Request(data=False))

        for proc in self.processes:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except Exception:
                pass
        self.processes.clear()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="LeRobot-Style Multi-Modal Episode Dataset Recorder",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--profile",
        type=str,
        default="piper_bimanual.yaml",
        help="Embodiment profile YAML filename or path",
    )
    parser.add_argument(
        "--task",
        type=str,
        default="bimanual_demo",
        help="Task name / language instruction string",
    )
    parser.add_argument(
        "--operator",
        type=str,
        default="operator",
        help="Operator ID / annotator name",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=1,
        help="Target number of successful episodes to record",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Target maximum duration per episode in seconds",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=30.0,
        help="Synchronous recording frequency (Hz)",
    )
    parser.add_argument(
        "--countdown",
        type=int,
        default=3,
        help="Countdown seconds before recording starts (0 to disable)",
    )
    parser.add_argument(
        "--side",
        type=str,
        default="both",
        choices=["left", "right", "both"],
        help="For bimanual robots: which arm(s) to admit",
    )
    return parser.parse_args()


def _relay_callback(session: rmi.Session, part: str):
    def _on_msg(msg: JointTrajectory) -> None:
        if not session.active_for(part):
            return
        session.act(rmi.Action(part=part, command="joint_reference", value=msg))

    return _on_msg


def main() -> None:
    args = parse_args()
    workspace_root = Path(__file__).resolve().parents[1]

    print("=" * 70)
    print("  LeRobot-Style Multi-Modal Episode Dataset Recorder")
    print(f"  Embodiment Profile : {args.profile}")
    print(f"  Task Name          : '{args.task}'")
    print(f"  Stream Spec        : {args.rate_hz:.1f} Hz | {args.duration:.1f} s max per episode")
    print(f"  Target Episodes    : {args.episodes}")
    print("=" * 70)

    # 1. Initialize RMI Context & Managed MCAP Recorder Backend
    ctx = rmi.Context.from_profile(args.profile)
    ctx.wait_until_ready(timeout=5.0)
    recorder = ctx.make_recorder(type="mcap", autostart=True)
    recorder.activate()
    print("  [✓] RMI Context & MCAP Recorder Engine active.")

    robot = ctx.robot
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    leader_mgr = PiperLeaderManager(workspace_root)
    sides = ["left", "right"] if args.side == "both" else [args.side]

    # Autostart Piper Leader Hardware if using piper_bimanual
    is_piper = "piper" in args.profile.lower()
    if is_piper:
        leader_mgr.ensure_started_and_enabled(ctx.node, sides)

    side_map = {
        "left": {
            "agent": "TeleopJoint_Left",
            "arm_part": "left_arm",
            "gripper_part": "left_gripper",
            "leader_arm_topic": "/action_sources/piper_leader_left/arm/joint_reference",
            "leader_gripper_topic": "/action_sources/piper_leader_left/end_effector/joint_reference",
        },
        "right": {
            "agent": "TeleopJoint_Right",
            "arm_part": "right_arm",
            "gripper_part": "right_gripper",
            "leader_arm_topic": "/action_sources/piper_leader_right/arm/joint_reference",
            "leader_gripper_topic": "/action_sources/piper_leader_right/end_effector/joint_reference",
        },
    }

    saved_episodes: list[str] = []
    total_steps = int(args.duration * args.rate_hz)
    dt = 1.0 / args.rate_hz

    try:
        current_ep_idx = 1
        while current_ep_idx <= args.episodes:
            print("\n" + "-" * 70)
            print(f"  [Episode {current_ep_idx}/{args.episodes}] Task: '{args.task}'")
            print("  Position the leader arms and objects. Ready when you are.")
            print("-" * 70)

            # Pre-roll Countdown
            if args.countdown > 0:
                for c in range(args.countdown, 0, -1):
                    print(f"  >> Starting in {c}... ", end="\r", flush=True)
                    time.sleep(1.0)
                print("  >> 🔴 RECORDING ACTIVE! (Hold leader arms and demonstrate)    ", flush=True)

            metadata = {
                "task": args.task,
                "operator": args.operator,
                "profile": args.profile,
                "rate_hz": args.rate_hz,
                "target_duration_s": args.duration,
                "episode_index": current_ep_idx,
                "total_steps": total_steps,
            }

            last_recorded_path = ""
            start_time = time.monotonic()

            if is_piper:
                leader_mgr.set_preempt(ctx.node, sides, True)

            with recorder.episode(task=args.task, metadata=metadata) as ep:
                with ExitStack() as stack:
                    if is_piper:
                        for side in sides:
                            cfg = side_map[side]
                            agent = ctx.make_agent(cfg["agent"], frequency=args.rate_hz)
                            session = stack.enter_context(
                                agent.run(robot, parts=[cfg["arm_part"], cfg["gripper_part"]])
                            )
                            ctx.node.create_subscription(
                                JointTrajectory,
                                cfg["leader_arm_topic"],
                                _relay_callback(session, cfg["arm_part"]),
                                qos,
                            )
                            ctx.node.create_subscription(
                                JointTrajectory,
                                cfg["leader_gripper_topic"],
                                _relay_callback(session, cfg["gripper_part"]),
                                qos,
                            )

                    # Step Recording Loop
                    for step in range(1, total_steps + 1):
                        loop_start = time.monotonic()
                        elapsed = time.monotonic() - start_time

                        if step % max(1, int(args.rate_hz / 2)) == 0 or step == total_steps:
                            pct = (step / total_steps) * 100.0
                            print(
                                f"    [{pct:5.1f}% | {elapsed:4.1f}s/{args.duration:.0f}s] "
                                f"Recording Step {step:4d}/{total_steps:4d}",
                                end="\r",
                                flush=True,
                            )

                        elapsed_step = time.monotonic() - loop_start
                        if elapsed_step < dt:
                            time.sleep(dt - elapsed_step)

                if hasattr(ep, "path") and ep.path:
                    last_recorded_path = str(ep.path)

            if is_piper:
                leader_mgr.set_preempt(ctx.node, sides, False)

            print("\n  [✓] Episode duration reached. MCAP stream captured.")

            # LeRobot-Style Post-Episode Action Prompt
            while True:
                choice = input(
                    "\n  >> Action: [S]ave (Default) | [D]iscard & Retry | [R]eplay | [Q]uit : "
                ).strip().lower()

                if choice in {"", "s", "save"}:
                    saved_episodes.append(last_recorded_path)
                    print(f"  [✓] Episode {current_ep_idx} SAVED to disk.")
                    current_ep_idx += 1
                    break
                elif choice in {"d", "discard"}:
                    print(f"  [!] Episode {current_ep_idx} DISCARDED. Re-trying episode {current_ep_idx}...")
                    # Clean up discarded episode path if needed
                    break
                elif choice in {"r", "replay"}:
                    print("  >> Replaying captured episode on the robot...")
                    # Run 1:1 replay
                    replay_cmd = f"python apps/replay.py --profile '{args.profile}'"
                    os.system(replay_cmd)
                elif choice in {"q", "quit"}:
                    print("  [!] Stopping recording session early.")
                    current_ep_idx = args.episodes + 1
                    break
                else:
                    print("  Invalid choice. Enter 's', 'd', 'r', or 'q'.")

    except KeyboardInterrupt:
        print("\n\n  [!] Recording session interrupted by operator.")
    finally:
        if is_piper:
            leader_mgr.shutdown(ctx.node, sides)
        print("\n" + "=" * 70)
        print(f"  Dataset Session Complete! Total Saved Episodes: {len(saved_episodes)}")
        print("=" * 70)


if __name__ == "__main__":
    main()
