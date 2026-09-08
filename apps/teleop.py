#!/usr/bin/env python3
# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""apps/teleop.py: Profile-driven teleoperation client.

Does not start RT or the workstation teleop devices.

  1. RT launch
  2. Workstation launch (gamepad / Quest / Piper leaders already running)
  3. python apps/teleop.py --profile <profile.yaml>

MuJoCo: add ``--use-sim-time`` so RMI stamps follow ``/clock``.

The app watches profile TELEOP nodes. Workstation ``config/teleop`` may
write ``preempt_service`` on a device; Enter calls those services and the
device publishes clutch. Otherwise the device clutch owns engage.
"""

from __future__ import annotations

import argparse
import select
import sys
import time
from pathlib import Path
from typing import Any

import rmi
import yaml
from std_srvs.srv import SetBool


def _profile_raw(profile: Any) -> dict[str, Any]:
    raw = getattr(profile, "raw_data", None)
    if raw is None and isinstance(profile, dict):
        raw = profile
    return raw if isinstance(raw, dict) else {}


def _workstation_teleop_dir(profile: Any) -> Path | None:
    em = _profile_raw(profile).get("execution_manager_config")
    if not isinstance(em, dict):
        return None
    package = em.get("package")
    if not package:
        return None
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory(str(package))) / "config" / "teleop"
        if installed.is_dir():
            return installed
    except Exception:  # noqa: BLE001 - fall back to the source tree.
        pass
    repo = Path(__file__).resolve().parents[1]
    for package_xml in repo.glob("src/**/package.xml"):
        if f"<name>{package}</name>" not in package_xml.read_text(encoding="utf-8"):
            continue
        candidate = package_xml.parent / "config" / "teleop"
        if candidate.is_dir():
            return candidate
    return None


def _written_preempt_service(block: Any) -> str:
    if not isinstance(block, dict):
        return ""
    service = block.get("preempt_service")
    if isinstance(service, str) and service.strip():
        return service.strip()
    params = block.get("ros__parameters")
    if isinstance(params, dict):
        service = params.get("preempt_service")
        if isinstance(service, str) and service.strip():
            return service.strip()
    return ""


def load_preempt_services(profile: Any) -> list[str]:
    """Services written on workstation ``config/teleop`` devices."""
    teleop_dir = _workstation_teleop_dir(profile)
    if teleop_dir is None:
        return []
    services: list[str] = []
    for path in sorted(teleop_dir.glob("*.yaml")):
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if not isinstance(data, dict):
            continue
        for block in data.values():
            service = _written_preempt_service(block)
            if service:
                services.append(service)
    return list(dict.fromkeys(services))


def teleop_node_names(profile: Any) -> list[str]:
    nodes = getattr(profile, "nodes", None) or {}
    names: list[str] = []
    for name, cfg in nodes.items():
        role = getattr(cfg, "source_role", None)
        if role is None and isinstance(cfg, dict):
            role = cfg.get("source_role")
        if str(role or "").upper() == "TELEOP":
            names.append(str(name))
    return names


def _node_context_ok(node: Any) -> bool:
    try:
        context = getattr(node, "context", None)
        return bool(context is not None and context.ok())
    except Exception:  # noqa: BLE001 - RCL context may already be torn down.
        return False


def verify_leader_preempt_services(
    node: Any, preempt_services: list[str], timeout_sec: float = 3.0
) -> bool:
    """Each listed ``preempt_service`` must be up before keyboard engage."""
    ok = True
    for srv_name in preempt_services:
        client = None
        try:
            client = node.create_client(SetBool, srv_name)
            ready = client.wait_for_service(timeout_sec=timeout_sec)
            print(
                f"  {'[✓]' if ready else '[!]'} {srv_name} -> "
                f"{'READY' if ready else 'UNAVAILABLE'}"
            )
            ok = ok and ready
        except Exception as exc:  # noqa: BLE001 - aggregated like set_teleop_preempt.
            print(f"  [!] {srv_name}: {exc.__class__.__name__}")
            ok = False
        finally:
            if client is not None:
                try:
                    node.destroy_client(client)
                except Exception:  # noqa: BLE001, S110 - best-effort during shutdown.
                    pass
    return ok


def set_teleop_preempt(
    node: Any, preempt_services: list[str], preempt_active: bool
) -> bool:
    """Call every listed ``preempt_service`` in parallel. Devices publish clutch."""
    if not preempt_services:
        return True
    if not _node_context_ok(node):
        return False
    clients: list[Any] = []
    futures: list[tuple[str, Any]] = []
    failures: list[str] = []
    try:
        for srv_name in preempt_services:
            client = node.create_client(SetBool, srv_name)
            clients.append(client)
            if not client.wait_for_service(timeout_sec=3.0):
                failures.append(f"{srv_name} unavailable")
                continue
            futures.append(
                (srv_name, client.call_async(SetBool.Request(data=preempt_active)))
            )
        t_end = time.monotonic() + 5.0
        pending = list(futures)
        while pending and time.monotonic() < t_end:
            pending = [(name, future) for name, future in pending if not future.done()]
            if pending:
                time.sleep(0.01)
        for srv_name, future in futures:
            if not future.done():
                failures.append(f"{srv_name} timed out")
                continue
            try:
                response = future.result()
            except Exception as exc:  # noqa: BLE001 - service failures are aggregated.
                failures.append(f"{srv_name}: {exc.__class__.__name__}")
                continue
            if response is None or not response.success:
                failures.append(
                    f"{srv_name}: {getattr(response, 'message', 'no response')}"
                )
                continue
            print(
                f"  [✓] {srv_name} -> "
                f"{'ACTIVE' if preempt_active else 'RELEASED'}: {response.message}"
            )
    finally:
        for client in clients:
            try:
                node.destroy_client(client)
            except Exception:  # noqa: BLE001, S110 - best-effort during shutdown.
                pass
    if failures:
        print("  [!] Leader mode transition failed: " + "; ".join(failures))
        return False
    return True


def _wait_teleop_active(
    teleop_nodes: dict[str, Any], expected: list[str], timeout_sec: float = 2.0
) -> set[str]:
    """Wait until every expected TELEOP node reports control, or timeout."""
    deadline = time.monotonic() + timeout_sec
    active: set[str] = set()
    wanted = set(expected)
    while time.monotonic() < deadline:
        active = {name for name, node in teleop_nodes.items() if node.has_control}
        if wanted and wanted <= active:
            return active
        time.sleep(0.02)
    return active


def _open_context(profile: str, *, use_sim_time: bool):
    """Build RMI Context; optionally pin the node clock to /clock."""
    if not use_sim_time:
        return rmi.Context.from_profile(profile)

    import rclpy
    from rclpy.parameter import Parameter

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("rmi_teleop")
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    return rmi.Context.from_profile(profile, node=node)


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
    parser.add_argument(
        "--use-sim-time",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Stamp commands with /clock (required when workstation uses sim time). "
            "Wall-clock stamps are dropped by EM as future_command."
        ),
    )
    return parser.parse_args()


def run_session(
    ctx: rmi.Context, profile_name: str, preempt_services: list[str]
) -> None:
    names = teleop_node_names(ctx.profile)
    teleop_nodes = {name: ctx.make_node(name) for name in names}

    print("=" * 68)
    print("  RMI Teleoperation Client")
    print(f"  Embodiment Profile : {profile_name}")
    print(f"  TELEOP nodes       : {names or '(none declared)'}")
    if preempt_services:
        print(f"  Preempt services   : {preempt_services}")
    print("=" * 68)

    if preempt_services:
        print("\n[2/3] Verifying preempt services...")
        verify_leader_preempt_services(ctx.node, preempt_services)
        print("\n[3/3] Keyboard clutch (no physical clutch on this profile).")
        print("  • Press [ENTER] to call preempt true (device publishes clutch true).")
        print("  • Press [ENTER] again to call preempt false.")
        print("  • Press [Ctrl+C] to quit safely.\n")
    else:
        print("\n[2/2] Device clutch owns engage.")
        print("  • Use the device clutch / activation button to preempt.")
        print("  • Press [Ctrl+C] to quit.\n")

    engaged = False
    last_active: set[str] = set()

    def _release() -> None:
        nonlocal engaged
        if preempt_services and engaged:
            set_teleop_preempt(ctx.node, preempt_services, False)
        engaged = False

    try:
        while True:
            active = {name for name, node in teleop_nodes.items() if node.has_control}
            if active != last_active:
                if active:
                    print(f"  [ACTIVE] {sorted(active)}")
                else:
                    print("  [STANDBY] no TELEOP node active")
                last_active = active

            if sys.stdin not in select.select([sys.stdin], [], [], 0.5)[0]:
                continue
            sys.stdin.readline()
            if not preempt_services:
                continue
            if not engaged:
                if set_teleop_preempt(ctx.node, preempt_services, True):
                    engaged = True
                    joint_names = [name for name in names if name.startswith("TeleopJoint_")]
                    _wait_teleop_active(teleop_nodes, joint_names)
                    print("  >> Preempt ACTIVE. Device clutch is true.")
                else:
                    _release()
                    print("  >> Teleop was not engaged; every preempt service must confirm.")
            else:
                _release()
                print("  >> Preempt released. Device clutch is false.")
    except (KeyboardInterrupt, EOFError):
        print("\n\n[!] Teleoperation stopped by operator.")
    finally:
        _release()


def main() -> None:
    args = parse_args()

    print("[1/3] Connecting to robot embodiment runtime...")
    print("  (Expect RT + workstation already launched.)")
    print(f"  use_sim_time={args.use_sim_time}")
    ctx = _open_context(args.profile, use_sim_time=args.use_sim_time)
    try:
        ctx.wait_until_ready(timeout=6.0)
        run_session(ctx, args.profile, load_preempt_services(ctx.profile))
    finally:
        ctx.close()
        print("[✓] Teleoperation session closed safely.")


if __name__ == "__main__":
    main()
