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

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory

import rmi


def set_teleop_preempt(
    node: Any, teleoperators: dict[str, Any], preempt_active: bool
) -> bool:
    """Set every leader mode and report failure instead of silently continuing."""
    pending: list[tuple[str, str, Any, Any]] = []
    failures: list[str] = []
    for name, cfg in teleoperators.items():
        srv_name = cfg.get("preempt_service")
        if not srv_name:
            failures.append(f"{name}: missing preempt_service")
            continue
        client = node.create_client(SetBool, srv_name)
        if not client.wait_for_service(timeout_sec=3.0):
            failures.append(f"{name}: {srv_name} unavailable")
            node.destroy_client(client)
            continue
        pending.append(
            (name, srv_name, client, client.call_async(SetBool.Request(data=preempt_active)))
        )

    deadline = time.monotonic() + 5.0
    for name, srv_name, client, future in pending:
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            failures.append(f"{name}: {srv_name} timed out")
            node.destroy_client(client)
            continue
        response = future.result()
        node.destroy_client(client)
        if response is None or not response.success:
            failures.append(
                f"{name}: {getattr(response, 'message', 'service returned no response')}"
            )
            continue
        mode_label = "ACTIVE (0-G Float)" if preempt_active else "RELEASED (Shadow/Passive)"
        print(f"  [✓] {name} Preempt {mode_label}: {response.message}")

    if failures:
        print("  [!] Leader mode transition failed: " + "; ".join(failures))
        return False
    return True


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
    # Keep the ROS context alive while Python handles Ctrl+C.  Session cleanup
    # must still be able to call the Leader release services before shutdown.
    if not rclpy.ok():
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    ctx = rmi.Context.from_profile(args.profile)
    ctx.wait_until_ready(timeout=6.0)

    # 2. Read teleoperation devices configured in Profile
    teleoperators = ctx.profile.raw_data.get("teleoperators", {})
    if not teleoperators:
        print(f"[!] No teleoperators defined in {args.profile}")
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

    try:
        print("\n[3/3] Teleoperation Control Standby.")
        print("  • Devices are active in SHADOW mode (Zero-Delta Hot Standby).")
        print("  • Press [ENTER] to ENGAGE teleoperation (Preempt 0-G float on).")
        print("  • Press [ENTER] again to RELEASE teleoperation (Preempt off).")
        print("  • Press [Ctrl+C] to quit safely.\n")

        sessions: dict[str, rmi.Session] = {}
        active_stack: ExitStack | None = None

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

        try:
            while True:
                prompt = (
                    "  [STANDBY] Press [ENTER] to ENGAGE Teleop (0-G Preempt)... "
                    if not is_active
                    else "  🔴 [ACTIVE TELEOP] Press [ENTER] to RELEASE (Return to Standby)... "
                )
                input(prompt)
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
                    if not set_teleop_preempt(ctx.node, teleoperators, True):
                        sessions.clear()
                        active_stack.close()
                        active_stack = None
                        is_active = False
                        set_teleop_preempt(ctx.node, teleoperators, False)
                        print("  >> Teleop was not engaged; both leaders must confirm 0-G mode.")
                        continue
                    print("  >> Master-Slave 1:1 servoing is ACTIVE! Move leader arms.")
                else:
                    set_teleop_preempt(ctx.node, teleoperators, False)
                    sessions.clear()
                    if active_stack is not None:
                        active_stack.close()
                        active_stack = None
                    print("  >> Teleop released. Master arms returned to Shadow/Standby mode.")
        finally:
            set_teleop_preempt(ctx.node, teleoperators, False)
            sessions.clear()
            if active_stack is not None:
                active_stack.close()

    except (KeyboardInterrupt, EOFError):
        print("\n\n[!] Teleoperation stopped by operator.")
    finally:
        ctx.close()
        print("[✓] Teleoperation session closed safely.")


if __name__ == "__main__":
    main()
