#!/usr/bin/env python3
"""apps/record.py: Production Multi-Modal Dataset Recorder with Smooth Staging & Teleop.

Workflow:
1. Smooth Initial Staging (Homing):
   - Followers smoothly move to staging pose (default: [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]) via quintic spline.
   - Physical Leader arms in Shadow Tracking mode automatically mirror the motion to identical desk pose.
2. Ready Gate & Zero-Drop Start:
   - System prompts operator: Press [ENTER] to START.
   - On ENTER: Recorders prime, Leaders switch to 0-G Preempt float, capturing from t=0 with zero frame loss.
3. Live Interactive Teleop Demonstration:
   - Operator demonstrates task. Live timer & frame counter updates on console.
   - Press [ENTER] at any time to conclude the episode.
4. Parallel Post-Processing & Staging Reset:
   - Leaders instantly return to Shadow Tracking, Followers begin smooth return to Staging Pose.
   - Concurrently, operator selects [S]ave / [D]iscard / [R]eplay / [Q]uit while MCAP seals in background.
   - If [R]eplay: Robot replays demonstration 1:1 while Leaders physically shadow the replay.

Usage:
  # One-command bimanual dataset recording:
  pixi run record --profile piper_bimanual.yaml --task bimanual_pickup --episodes 10

  # Custom home pose and CAN overrides:
  pixi run record --left-can can0 --right-can can1 --homing-duration 2.5
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import threading
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

    def ensure_started_and_enabled(
        self, node: Any, sides: list[str], can_map: dict[str, str] | None = None
    ) -> None:
        """Autostart leader nodes if needed and enable torque."""
        setup_bash = self.workspace_root / "install" / "setup.bash"
        share_config = (
            self.workspace_root
            / "src"
            / "teleop"
            / "piper_leader_teleop"
            / "config"
        )
        if can_map is None:
            can_map = {"left": "can0", "right": "can1"}

        for side in sides:
            can_iface = can_map.get(side, "can0" if side == "left" else "can1")
            srv_name = f"/piper_leader_{side}/enable"
            enable_client = node.create_client(SetBool, srv_name)

            if not enable_client.wait_for_service(timeout_sec=0.2):
                config_yaml = share_config / f"piper_leader_{side}.yaml"
                cmd = (
                    f"source '{setup_bash}' && exec ros2 launch piper_leader_teleop piper_leader.launch.py "
                    f"config:='{config_yaml}' node_name:=piper_leader_{side} can_interface:='{can_iface}'"
                )
                print(f"  >> Autostarting piper_leader_{side} driver on '{can_iface}'...")
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
                    action_str = "ENGAGED (0-G Teleop)" if preempt_active else "RELEASED (Shadow Mode)"
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


def smooth_move_to_pose(
    ctx: rmi.Context,
    robot: Any,
    parts_map: dict[str, dict[str, Any]],
    duration_s: float = 3.0,
    rate_hz: float = 50.0,
) -> None:
    """Smooth quintic polynomial trajectory interpolation to target staging pose."""
    staging_agent = ctx.make_agent("Policy", frequency=rate_hz)
    parts = []
    for side, cfg in parts_map.items():
        parts.extend([cfg["arm_part"], cfg["gripper_part"]])

    steps = max(10, int(duration_s * rate_hz))
    dt = 1.0 / rate_hz

    with staging_agent.run(robot, parts=parts, resume=True) as session:
        obs = session.observe()
        name_to_pos = dict(zip(obs.joint_names, obs.joint_positions))

        start_poses = {}
        for side, cfg in parts_map.items():
            arm_joints = ctx.profile.parts[cfg["arm_part"]].joint_names
            start_poses[cfg["arm_part"]] = [name_to_pos.get(j, 0.0) for j in arm_joints]
            start_poses[cfg["gripper_part"]] = [
                name_to_pos.get(ctx.profile.parts[cfg["gripper_part"]].joint_names[0], 0.0)
            ]

        for i in range(steps + 1):
            t_start = time.monotonic()
            s = i / steps
            # Quintic polynomial factor h(s) = 10*s^3 - 15*s^4 + 6*s^5
            h = 10.0 * (s**3) - 15.0 * (s**4) + 6.0 * (s**5)

            for side, cfg in parts_map.items():
                arm_p = cfg["arm_part"]
                grip_p = cfg["gripper_part"]
                q_start = start_poses[arm_p]
                q_goal = cfg["home_arm_pose"]
                g_start = start_poses[grip_p][0]
                g_goal = cfg["home_gripper_pose"][0]

                interp_arm = [q_start[j] + h * (q_goal[j] - q_start[j]) for j in range(len(q_goal))]
                interp_grip = [g_start + h * (g_goal - g_start)]

                session.act(rmi.Action(part=arm_p, command="joint_reference", value=interp_arm))
                session.act(rmi.Action(part=grip_p, command="joint_reference", value=interp_grip))

            elapsed = time.monotonic() - t_start
            if elapsed < dt:
                time.sleep(dt - elapsed)


def _relay_callback(session: rmi.Session, part: str):
    def _on_msg(msg: JointTrajectory) -> None:
        if not session.active_for(part):
            return
        session.act(rmi.Action(part=part, command="joint_reference", value=msg))

    return _on_msg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Production Multi-Modal Episode Dataset Recorder with Staging & 0-G Teleop",
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
        default=None,
        help="Task name / language instruction description (default: from profile)",
    )
    parser.add_argument(
        "--operator",
        type=str,
        default="operator",
        help="Operator ID / annotator identifier",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=None,
        help="Target number of successful episodes to record (default: from profile)",
    )
    parser.add_argument(
        "--max-duration",
        type=float,
        default=None,
        help="Maximum timeout per episode in seconds (default: from profile)",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=None,
        help="Recording & teleoperation frequency in Hz (default: from profile)",
    )
    parser.add_argument(
        "--homing-duration",
        type=float,
        default=None,
        help="Smooth staging movement duration in seconds (default: from profile)",
    )
    parser.add_argument(
        "--side",
        type=str,
        default="both",
        choices=["left", "right", "both"],
        help="For bimanual robots: which arm(s) to record",
    )
    parser.add_argument(
        "--left-can",
        type=str,
        default="can0",
        help="SocketCAN interface for left leader",
    )
    parser.add_argument(
        "--right-can",
        type=str,
        default="can1",
        help="SocketCAN interface for right leader",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    workspace_root = Path(__file__).resolve().parents[1]
    can_map = {"left": args.left_can, "right": args.right_can}

    # 1. Initialize RMI Context & Managed MCAP Recorder Backend
    ctx = rmi.Context.from_profile(args.profile)
    print("[1/4] Waiting for follower hardware and controller readiness...")
    ctx.wait_until_ready(timeout=6.0)

    # Read defaults from profile's recorder block
    rec_cfg = ctx.profile.raw_data.get("recorder", {})
    task_name = args.task or rec_cfg.get("task", "bimanual_teleop_demo")
    target_episodes = args.episodes if args.episodes is not None else rec_cfg.get("episodes", 10)
    rate_hz = args.rate_hz if args.rate_hz is not None else rec_cfg.get("rate_hz", 50.0)
    max_duration_s = args.max_duration if args.max_duration is not None else rec_cfg.get("max_duration_s", 60.0)
    homing_duration_s = (
        args.homing_duration if args.homing_duration is not None else rec_cfg.get("homing_duration_s", 2.5)
    )

    print("=" * 72)
    print("  RMI Production Dataset Episode Recorder")
    print(f"  Embodiment Profile : {args.profile}")
    print(f"  Task Description   : '{task_name}'")
    print(f"  Target Episodes    : {target_episodes}")
    print(f"  Stream Frequency   : {rate_hz:.1f} Hz")
    print(f"  Homing Duration    : {homing_duration_s:.1f} s")
    print(f"  Left Leader CAN    : {args.left_can}")
    print(f"  Right Leader CAN   : {args.right_can}")
    print("=" * 72)

    recorder = ctx.make_recorder(type="mcap", autostart=True)
    recorder.activate()
    print("  [✓] Follower hardware, controllers, and MCAP Recorder active.")

    robot = ctx.robot
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    leader_mgr = PiperLeaderManager(workspace_root)
    sides = ["left", "right"] if args.side == "both" else [args.side]

    # Autostart Piper Leader Hardware in Shadow Tracking mode
    is_piper = "piper" in args.profile.lower()
    if is_piper:
        leader_mgr.ensure_started_and_enabled(ctx.node, sides, can_map=can_map)

    # Resolve parts and default staging home poses
    teleop_defs = ctx.profile.raw_data.get("teleoperators", {})
    parts_map = {}
    default_home_q = rec_cfg.get("home_pose", [0.0, 0.5, -0.5, 0.0, 0.0, 0.0])
    default_home_grip = [0.020]  # Single finger position 0.02m = 0.04m opening

    for side in sides:
        t_key = f"{side}_leader"
        t_cfg = teleop_defs.get(t_key, {})
        parts_map[side] = {
            "agent": t_cfg.get("target_agent", f"TeleopJoint_{side.capitalize()}"),
            "arm_part": t_cfg.get("arm_part", f"{side}_arm"),
            "gripper_part": t_cfg.get("gripper_part", f"{side}_gripper"),
            "leader_arm_topic": t_cfg.get(
                "arm_source", f"/action_sources/piper_leader_{side}/arm/joint_reference"
            ),
            "leader_gripper_topic": t_cfg.get(
                "gripper_source",
                f"/action_sources/piper_leader_{side}/end_effector/joint_reference",
            ),
            "home_arm_pose": list(default_home_q),
            "home_gripper_pose": list(default_home_grip),
        }

    saved_episodes: list[str] = []

    try:
        current_ep_idx = 1
        while current_ep_idx <= target_episodes:
            print("\n" + "=" * 72)
            print(f"  [EPISODE {current_ep_idx}/{target_episodes}] Task: '{task_name}'")
            print("=" * 72)

            # STEP 1: Smooth Staging Reset (Follower moves to home, Leader shadows to home)
            print("  >> Moving robot arm(s) smoothly to Staging Home Pose...")
            print("     (Leader arm(s) physically shadowing to the same start pose on desk)")
            smooth_move_to_pose(ctx, robot, parts_map, duration_s=homing_duration_s, rate_hz=rate_hz)
            print("  [✓] Staging Home Pose reached. Master & Slave aligned.")

            # STEP 2: Ready Gate (Wait for Operator)
            print("\n" + "-" * 72)
            input(f"  >> [READY] Press [ENTER] to START recording Episode {current_ep_idx}... ")
            print("-" * 72)

            metadata = {
                "task": task_name,
                "operator": args.operator,
                "profile": args.profile,
                "rate_hz": rate_hz,
                "episode_index": current_ep_idx,
            }

            stop_recording_event = threading.Event()

            def listen_for_stop():
                input()
                stop_recording_event.set()

            stop_thread = threading.Thread(target=listen_for_stop, daemon=True)

            last_recorded_path = ""
            start_time = time.monotonic()
            dt = 1.0 / rate_hz

            # STEP 3: Zero-Drop Start (Preempt 0-G + Multi-Modal Recording in lockstep)
            with recorder.episode(task=task_name, metadata=metadata) as ep:
                with ExitStack() as stack:
                    # Admit Teleop Agents & wire high-frequency relays
                    for side in sides:
                        cfg = parts_map[side]
                        agent = ctx.make_agent(cfg["agent"], frequency=rate_hz)
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

                    # Activate 0-G Float mode
                    if is_piper:
                        leader_mgr.set_preempt(ctx.node, sides, True)

                    print(f"\n  🔴 RECORDING ACTIVE! Steer arms to demonstrate task.")
                    print(f"  >> Press [ENTER] in this console when episode is COMPLETE.\n")
                    stop_thread.start()

                    step_count = 0
                    while not stop_recording_event.is_set():
                        t_loop = time.monotonic()
                        step_count += 1
                        elapsed = time.monotonic() - start_time

                        if elapsed >= max_duration_s:
                            print(f"\n  [!] Max duration {max_duration_s}s reached.")
                            break

                        if step_count % int(max(1, rate_hz / 5)) == 0:
                            print(
                                f"    Recording: {elapsed:5.1f}s | {step_count:5d} frames captured  (Press [ENTER] to Finish)",
                                end="\r",
                                flush=True,
                            )

                        t_spent = time.monotonic() - t_loop
                        if t_spent < dt:
                            time.sleep(dt - t_spent)

                    # STEP 4: Release Preempt back to Shadow Tracking
                    if is_piper:
                        leader_mgr.set_preempt(ctx.node, sides, False)

                if hasattr(ep, "path") and ep.path:
                    last_recorded_path = str(ep.path)

            print(f"\n  [✓] Demonstration concluded ({elapsed:.1f}s, {step_count} frames).")

            # STEP 5 & 6: Parallel Homing Reset & Quality Confirmation
            # Launch background homing so arms return while operator reviews
            homing_thread = threading.Thread(
                target=smooth_move_to_pose,
                args=(ctx, robot, parts_map, homing_duration_s, rate_hz),
                daemon=True,
            )
            homing_thread.start()

            import shutil

            is_committed = False
            while True:
                choice = input(
                    "\n  >> Episode Action: [S]ave (Default / Enter) | [D]iscard & Retry | [R]eplay | [Q]uit : "
                ).strip().lower()

                if choice in {"", "s", "save"}:
                    saved_episodes.append(last_recorded_path)
                    print(f"  [✓] Episode {current_ep_idx} COMMITTED & SAVED to: {last_recorded_path}")
                    current_ep_idx += 1
                    is_committed = True
                    homing_thread.join()
                    break
                elif choice in {"d", "discard"}:
                    print(f"  [!] Episode {current_ep_idx} DISCARDED. Removing temporary files...")
                    if last_recorded_path:
                        ep_dir = Path(last_recorded_path).parent
                        if ep_dir.is_dir() and "episode_" in ep_dir.name:
                            shutil.rmtree(ep_dir, ignore_errors=True)
                    homing_thread.join()
                    break
                elif choice in {"r", "replay"}:
                    homing_thread.join()
                    print("\n  >> Replaying demonstration on follower (Leader physically mirrors in Shadow mode)...")
                    replay_cmd = (
                        f"python apps/replay.py --profile '{args.profile}' --mcap-file '{last_recorded_path}'"
                    )
                    os.system(replay_cmd)
                    print("  [✓] Replay inspection finished.")
                    print("  >> Please now decide whether to [S]ave or [D]iscard this episode.")
                elif choice in {"q", "quit"}:
                    print("  [!] Stopping recording session.")
                    if not is_committed and last_recorded_path:
                        # Clean uncommitted last episode if aborted
                        ep_dir = Path(last_recorded_path).parent
                        if ep_dir.is_dir() and "episode_" in ep_dir.name:
                            shutil.rmtree(ep_dir, ignore_errors=True)
                    homing_thread.join()
                    current_ep_idx = target_episodes + 1
                    break
                else:
                    print("  Invalid choice. Enter 's', 'd', 'r', or 'q'.")

    except KeyboardInterrupt:
        print("\n\n  [!] Recording session interrupted by operator.")
    finally:
        if is_piper:
            leader_mgr.shutdown(ctx.node, sides)
        ctx.close()
        print("\n" + "=" * 72)
        print(f"  RECORDING SESSION COMPLETE. Total Saved Episodes: {len(saved_episodes)}")
        print("=" * 72)


if __name__ == "__main__":
    main()
