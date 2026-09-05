#!/usr/bin/env python3
# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""apps/teleop.py: Profile-driven teleoperation client.

Prerequisite: RT launch, then workstation launch (teleoperators already running).
This app does not start the teleop stack.

- Profiles with ``teleoperators`` + ``preempt_service`` (Piper leaders): keyboard
  Enter toggles preempt and relays leader JointTrajectory streams into RMI.
- Other profiles (gamepad / Quest clutch already on workstation): wait until ready,
  print clutch instructions, optionally report TELEOP node ``is_active``.

Usage:
  pixi run teleop --profile fr3_pika_single_arm.yaml
  pixi run teleop --profile piper_bimanual.yaml
"""

from __future__ import annotations

import argparse
import select
import sys
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


def _needs_keyboard_preempt(teleoperators: dict[str, Any]) -> bool:
    return bool(teleoperators) and any(
        bool(cfg.get("preempt_service")) for cfg in teleoperators.values()
    )


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
        description="Profile-driven RMI teleoperation client",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--profile",
        type=str,
        default="piper_bimanual.yaml",
        help="Embodiment profile YAML filename or path",
    )
    return parser.parse_args()


def run_keyboard_session(ctx: rmi.Context, teleoperators: dict[str, Any]) -> None:
    """Piper-style: Enter toggles preempt + relays leader streams."""
    print("=" * 68)
    print("  RMI Teleoperation Client (keyboard preempt)")
    print(f"  Teleop Devices     : {list(teleoperators.keys())}")
    print("=" * 68)

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
                else "  [ACTIVE TELEOP] Press [ENTER] to RELEASE (Return to Standby)... "
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


def run_external_session(ctx: rmi.Context, profile_name: str) -> None:
    """Gamepad / Quest: workstation owns streams + clutch; this app only monitors."""
    teleop_names = [
        name
        for name, cfg in ctx.profile.nodes.items()
        if str(cfg.source_role).upper() == "TELEOP"
    ]
    teleop_nodes = {name: ctx.make_node(name) for name in teleop_names}

    print("=" * 68)
    print("  RMI Teleoperation Client (device clutch)")
    print(f"  Embodiment Profile : {profile_name}")
    print(f"  TELEOP nodes       : {teleop_names or '(none declared)'}")
    print("=" * 68)
    print("\n[2/2] Teleop is owned by the workstation stack.")
    print("  • Use the device clutch / activation button to preempt.")
    print("  • No keyboard preempt in this profile.")
    print("  • Press [Ctrl+C] to quit.\n")

    last_active: set[str] = set()
    try:
        while True:
            active = {name for name, node in teleop_nodes.items() if node.is_active}
            if active != last_active:
                if active:
                    print(f"  [ACTIVE] {sorted(active)}")
                else:
                    print("  [STANDBY] no TELEOP node active (hold clutch to engage)")
                last_active = active
            # Allow Ctrl+C; also drain Enter so stray keypresses do not buffer forever.
            if sys.stdin in select.select([sys.stdin], [], [], 0.5)[0]:
                sys.stdin.readline()
    except (KeyboardInterrupt, EOFError):
        print("\n\n[!] Teleoperation monitor stopped by operator.")


def main() -> None:
    args = parse_args()

    print("[1/3] Connecting to robot embodiment runtime...")
    print("  (Expect RT + workstation already launched.)")
    ctx = rmi.Context.from_profile(args.profile)
    try:
        ctx.wait_until_ready(timeout=6.0)
        teleoperators = ctx.profile.raw_data.get("teleoperators", {}) or {}
        if _needs_keyboard_preempt(teleoperators):
            run_keyboard_session(ctx, teleoperators)
        else:
            run_external_session(ctx, args.profile)
    finally:
        ctx.close()
        print("[✓] Teleoperation session closed safely.")


if __name__ == "__main__":
    main()
