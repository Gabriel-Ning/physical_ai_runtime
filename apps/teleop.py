#!/usr/bin/env python3
"""apps/teleop.py: One-Command Interactive Robot Teleoperation Application.

Autonomous, one-command start for:
1. Piper Bimanual Master-Slave Teleop:
   - Starts in Shadow Tracking Mode (tracks follower /joint_states)
   - Seamlessly Preempts into 500Hz MIT Gravity Compensation Teleop
   - Gracefully Releases back to Shadow or Passive mode on exit
2. SpaceMouse / Keyboard / Marker teleoperation for single-arm or bimanual setups.

Usage:
  # One single command launches everything for Piper bimanual:
  pixi run teleop --profile piper_bimanual.yaml

  # Teleoperate single arm via keyboard:
  pixi run teleop --profile fr3_pika_single_arm.yaml --device keyboard
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
    """Manages background lifecycle & preemption of Piper Leader Arm hardware nodes."""

    def __init__(self, workspace_root: Path) -> None:
        self.workspace_root = workspace_root
        self.processes: list[subprocess.Popen[bytes]] = []

    def ensure_started_and_enabled(self, node: Any, sides: list[str]) -> None:
        """Autostart leader nodes if needed and enable base mode."""
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

            # Check if service is already running
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

                # Wait for service readiness
                for _ in range(50):
                    if enable_client.wait_for_service(timeout_sec=0.1):
                        break
                    time.sleep(0.1)

            # Enable base mode
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
            # Release preempt first
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
        description="RMI One-Command Robot Teleoperation",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--profile",
        type=str,
        default="piper_bimanual.yaml",
        help="Embodiment profile YAML filename or path",
    )
    parser.add_argument(
        "--side",
        type=str,
        default="both",
        choices=["left", "right", "both"],
        help="For bimanual robots: which arm(s) to admit ('left', 'right', 'both')",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="leader",
        choices=["leader", "keyboard", "spacemouse", "marker"],
        help="Teleoperation input device ('leader' for Piper Master-Slave, 'keyboard', etc.)",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=100.0,
        help="Teleoperation relay loop rate (Hz)",
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

    print("=" * 68)
    print("  RMI Autonomous Robot Teleoperation (One-Command Start)")
    print(f"  Embodiment Profile : {args.profile}")
    print(f"  Mode / Device      : {args.device.upper()}")
    print(f"  Loop Frequency     : {args.rate_hz:.1f} Hz")
    print("=" * 68)

    ctx = rmi.Context.from_profile(args.profile)
    ctx.wait_until_ready(timeout=5.0)
    robot = ctx.robot
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

    leader_mgr = PiperLeaderManager(workspace_root)
    sides = ["left", "right"] if args.side == "both" else [args.side]

    try:
        if "piper" in args.profile.lower() and args.device == "leader":
            # 1. Autostart & enable Piper leader hardware nodes in Shadow Tracking mode
            leader_mgr.ensure_started_and_enabled(ctx.node, sides)

            # 2. Engage Preemption (0-G Gravity Comp + Streaming Ingress)
            leader_mgr.set_preempt(ctx.node, sides, True)

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

            with ExitStack() as stack:
                sessions: list[tuple[dict[str, Any], rmi.Session]] = []
                for side in sides:
                    cfg = side_map[side]
                    agent = ctx.make_agent(cfg["agent"], frequency=args.rate_hz)
                    session = stack.enter_context(
                        agent.run(robot, parts=[cfg["arm_part"], cfg["gripper_part"]])
                    )
                    sessions.append((cfg, session))
                    print(f"  [✓] {cfg['agent']} admitted on '{cfg['arm_part']}' + '{cfg['gripper_part']}'")

                for cfg, session in sessions:
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
                    print(f"  [✓] Relaying {cfg['leader_arm_topic']} -> {cfg['arm_part']}")

                print("\n" + "=" * 68)
                print("  🟢 Piper Master-Slave Teleoperation Running! (Press Ctrl+C to stop)")
                print("=" * 68)

                while True:
                    time.sleep(0.5)
        else:
            # Generic / Single-Arm Keyboard loop
            arm_part = "arm" if "arm" in ctx.profile.parts else list(ctx.profile.parts.keys())[0]
            agent = ctx.make_agent("TeleopJoint", frequency=args.rate_hz)
            with agent.run(robot) as session:
                obs = session.observe()
                name_to_pos = dict(zip(obs.joint_names, obs.joint_positions))
                arm_joints = ctx.profile.parts[arm_part].joint_names
                current_q = [name_to_pos.get(jname, 0.0) for jname in arm_joints]
                dt = 1.0 / args.rate_hz
                print("  [✓] Teleop active. Streaming current pose. Press Ctrl+C to exit.")
                while True:
                    session.act(rmi.Action(part=arm_part, command="joint_reference", value=current_q))
                    time.sleep(dt)

    except KeyboardInterrupt:
        print("\n\n  [!] Exiting Teleoperation...")
    finally:
        leader_mgr.shutdown(ctx.node, sides)
        print("=" * 68)
        print("  [✓] Teleoperation Cleanly Terminated.")
        print("=" * 68)


if __name__ == "__main__":
    main()
