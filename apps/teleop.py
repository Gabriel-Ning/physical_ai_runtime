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
from typing import Any

import rmi
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory


def _node_context_ok(node: Any) -> bool:
    try:
        context = getattr(node, "context", None)
        return bool(context is not None and context.ok())
    except Exception:  # noqa: BLE001 - RCL context may already be torn down.
        return False


def set_teleop_preempt(
    node: Any, teleoperators: dict[str, Any], preempt_active: bool
) -> bool:
    """Set every leader mode, requiring all transitions to succeed."""
    if not teleoperators or not _node_context_ok(node):
        return not teleoperators
    failures: list[str] = []
    for name, cfg in teleoperators.items():
        srv_name = cfg.get("preempt_service")
        if not srv_name:
            failures.append(f"{name}: missing preempt_service")
            continue
        client = None
        try:
            client = node.create_client(SetBool, srv_name)
            if not client.wait_for_service(timeout_sec=3.0):
                failures.append(f"{name}: {srv_name} unavailable")
                continue
            future = client.call_async(SetBool.Request(data=preempt_active))
            t_end = time.monotonic() + 5.0
            while not future.done() and time.monotonic() < t_end:
                time.sleep(0.01)
            if not future.done():
                failures.append(f"{name}: {srv_name} timed out")
                continue
            response = future.result()
            if response is None or not response.success:
                failures.append(
                    f"{name}: {getattr(response, 'message', 'no response')}"
                )
                continue
            mode_label = (
                "ACTIVE (0-G Float)" if preempt_active else "RELEASED (Shadow/Passive)"
            )
            print(f"  [✓] {name} Preempt {mode_label}: {response.message}")
        except Exception as exc:  # noqa: BLE001 - service failures are aggregated.
            failures.append(f"{name}: {exc.__class__.__name__}")
        finally:
            if client is not None:
                try:
                    node.destroy_client(client)
                except Exception:  # noqa: BLE001, S110 - best-effort during shutdown.
                    pass
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
        status = (
            "DETECTED (Streaming)"
            if pubs
            else "NO PUBLISHER (Start the workstation bringup)"
        )
        icon = "[✓]" if pubs else "[!]"
        print(f"  {icon} {name:<14}: {topic} -> {status}")

    # 4. Interactive Teleoperation Session
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    is_active = False
    teleop_nodes: dict[str, rmi.Node] = {}
    for name, cfg in teleoperators.items():
        node_name = cfg.get("target_node") or cfg.get("target_agent", name)
        teleop_nodes[name] = ctx.make_node(node_name)

    def _release_teleop() -> None:
        nonlocal is_active
        set_teleop_preempt(ctx.node, teleoperators, False)
        is_active = False

    try:
        print("\n[3/3] Teleoperation Control Standby.")
        print("  • Devices are active in SHADOW mode (Zero-Delta Hot Standby).")
        print("  • Press [ENTER] to ENGAGE teleoperation (Preempt 0-G float on).")
        print("  • Press [ENTER] again to RELEASE teleoperation (Preempt off).")
        print("  • Press [Ctrl+C] to quit safely.\n")

        def relay(name: str, part: str):
            def callback(msg: JointTrajectory) -> None:
                if is_active and name in teleop_nodes:
                    teleop_nodes[name].submit(
                        rmi.Action(part=part, command="joint_reference", value=msg)
                    )

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
                if not set_teleop_preempt(ctx.node, teleoperators, True):
                    _release_teleop()
                    print(
                        "  >> Teleop was not engaged; every leader must confirm 0-G mode."
                    )
                    continue
                print("  >> Master-Slave 1:1 servoing is ACTIVE! Move leader arms.")
            else:
                _release_teleop()
                print(
                    "  >> Teleop released. Master arms returned to Shadow/Standby mode."
                )
    finally:
        _release_teleop()
        ctx.close()
        print("[✓] Teleoperation session closed safely.")


if __name__ == "__main__":
    main()
