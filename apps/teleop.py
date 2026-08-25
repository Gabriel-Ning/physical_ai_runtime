#!/usr/bin/env python3
# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""apps/teleop.py: Production RMI Robot Teleoperation Client Application.

Connects to an active robot runtime and relays teleoperation commands
from physical input devices (e.g. Piper Master Leader arms) to the robot
controllers via RMI admission and preemption control.

Usage:
  # Teleoperate using profile defaults:
  pixi run teleop --profile piper_bimanual.yaml
"""

from __future__ import annotations

import argparse
import time
from contextlib import ExitStack
from typing import Any

from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory

import rmi


def _node_context_ok(node: Any) -> bool:
    try:
        context = getattr(node, "context", None)
        return bool(context is not None and context.ok())
    except Exception:
        return False


def set_teleop_preempt(node: Any, teleoperators: dict[str, Any], preempt_active: bool) -> None:
    """Toggle hardware preemption (0-G Float vs Fallback mode) on teleoperation devices."""
    if not teleoperators or not _node_context_ok(node):
        return
    for name, cfg in teleoperators.items():
        srv_name = cfg.get("preempt_service")
        if not srv_name:
            continue
        try:
            client = node.create_client(SetBool, srv_name)
            if not client.wait_for_service(timeout_sec=0.3):
                continue
            future = client.call_async(SetBool.Request(data=preempt_active))
            t_end = time.monotonic() + 0.5
            while not future.done() and time.monotonic() < t_end:
                time.sleep(0.01)
            if future.done() and future.result().success:
                mode_label = (
                    "ACTIVE (0-G Float)" if preempt_active else "RELEASED (Shadow/Passive)"
                )
                print(f"  [✓] {name} Preempt {mode_label}: {future.result().message}")
        except Exception as exc:
            # Best-effort during Ctrl+C / RCL teardown.
            print(f"  [!] {name} preempt skipped ({exc.__class__.__name__})")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Production RMI Robot Teleoperation Client",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--profile",
        type=str,
        default="piper_bimanual.yaml",
        help="Embodiment profile YAML filename or path",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    # 1. Connect to Runtime & Profile
    print("[1/3] Connecting to robot embodiment runtime...")
    ctx = rmi.Context.from_profile(args.profile)
    ctx.wait_until_ready(timeout=6.0)

    # 2. Read teleoperation devices configured in Profile
    teleoperators = ctx.profile.raw_data.get("teleoperators", {})
    if not teleoperators:
        print(f"[!] No teleoperators defined in {args.profile}")
        ctx.close()
        return

    print("=" * 68)
    print("  RMI Production Robot Teleoperation Client")
    print(f"  Embodiment Profile : {args.profile}")
    print(f"  Teleop Devices     : {list(teleoperators.keys())}")
    print("=" * 68)

    # 3. Check Stream Ingress
    print("\n[2/3] Verifying Teleoperation Ingress Streams...")
    for name, cfg in teleoperators.items():
        topic = cfg.get("arm_source", "")
        pubs = ctx.node.get_publishers_info_by_topic(topic) if topic else []
        status = "DETECTED (Streaming)" if pubs else "NO PUBLISHER (Start the workstation bringup)"
        icon = "[✓]" if pubs else "[!]"
        print(f"  {icon} {name:<14}: {topic} -> {status}")

    # 4. Interactive Teleoperation Session
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    is_active = False
    sessions: dict[str, rmi.Session] = {}
    active_stack: ExitStack | None = None

    def _release_teleop() -> None:
        nonlocal is_active, active_stack
        set_teleop_preempt(ctx.node, teleoperators, False)
        sessions.clear()
        if active_stack is not None:
            try:
                active_stack.close()
            except Exception:
                pass
            active_stack = None
        is_active = False

    try:
        print("\n[3/3] Teleoperation Control Standby.")
        print("  • Devices are active in SHADOW mode (Zero-Delta Hot Standby).")
        print("  • Press [ENTER] to ENGAGE teleoperation (Preempt 0-G float on).")
        print("  • Press [ENTER] again to RELEASE teleoperation (Preempt off).")
        print("  • Press [Ctrl+C] to quit safely.\n")

        def relay(name: str, part: str):
            def callback(msg: JointTrajectory) -> None:
                session = sessions.get(name)
                if session is not None and session.active_for(part):
                    session.act(rmi.Action(part=part, command="joint_reference", value=msg))

            return callback

        for name, cfg in teleoperators.items():
            ctx.node.create_subscription(
                JointTrajectory, cfg["arm_source"], relay(name, cfg["arm_part"]), qos
            )
            ctx.node.create_subscription(
                JointTrajectory,
                cfg["gripper_source"],
                relay(name, cfg["gripper_part"]),
                qos,
            )

        while True:
            prompt = (
                "  [STANDBY] Press [ENTER] to ENGAGE Teleop (0-G Preempt)... "
                if not is_active
                else "  🔴 [ACTIVE TELEOP] Press [ENTER] to RELEASE (Return to Standby)... "
            )
            try:
                input(prompt)
            except (KeyboardInterrupt, EOFError):
                print("\n\n[!] Teleoperation stopped by operator.")
                break

            is_active = not is_active
            if is_active:
                candidate = ExitStack()
                try:
                    for name, cfg in teleoperators.items():
                        agent = ctx.make_agent(
                            cfg["target_agent"],
                            frequency=cfg.get("publish_rate_hz", 200.0),
                        )
                        sessions[name] = candidate.enter_context(
                            agent.run(
                                ctx.robot,
                                parts=[cfg["arm_part"], cfg["gripper_part"]],
                                preempt=True,
                            )
                        )
                except Exception:
                    sessions.clear()
                    candidate.close()
                    is_active = False
                    raise
                active_stack = candidate
                set_teleop_preempt(ctx.node, teleoperators, True)
                print("  >> Master-Slave 1:1 servoing is ACTIVE! Move leader arms.")
            else:
                _release_teleop()
                print("  >> Teleop released. Master arms returned to Shadow/Standby mode.")
    finally:
        _release_teleop()
        ctx.close()
        print("[✓] Teleoperation session closed safely.")


if __name__ == "__main__":
    main()
