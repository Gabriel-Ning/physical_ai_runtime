#!/usr/bin/env python3
"""Read-only runtime evaluation for one embodiment profile.

This application never acquires a provider and never publishes a robot command.
It measures observation availability, age, rate, hardware diagnostics, and the
current provider-selection snapshot so deployments can be checked before motion.
"""

from __future__ import annotations

import argparse
import json
import time
from statistics import mean

import rmi


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read-only RMI runtime evaluation")
    parser.add_argument("--profile", default="piper_bimanual.yaml")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--max-state-age", type=float, default=0.25)
    parser.add_argument("--check-cameras", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.duration <= 0.0 or args.rate <= 0.0 or args.max_state_age <= 0.0:
        raise SystemExit("duration, rate, and max-state-age must be positive")

    with rmi.Context.from_profile(args.profile) as context:
        context.wait_until_ready(
            timeout=max(6.0, args.duration),
            check_cameras=args.check_cameras,
        )
        ages: list[float] = []
        samples = 0
        deadline = time.monotonic() + args.duration
        period = 1.0 / args.rate
        while time.monotonic() < deadline:
            observation = context.robot.get_observation()
            now_s = context.node.get_clock().now().nanoseconds / 1e9
            ages.append(max(0.0, now_s - observation.receive_time_s))
            samples += 1
            time.sleep(period)

        get_diagnostics = getattr(
            context.provider_selector, "get_hardware_diagnostics", None
        )
        diagnostics = list(get_diagnostics()) if callable(get_diagnostics) else []
        result = {
            "profile": context.profile.name,
            "passed": bool(samples)
            and max(ages, default=float("inf")) <= args.max_state_age
            and not diagnostics,
            "samples": samples,
            "mean_state_age_s": mean(ages) if ages else None,
            "max_state_age_s": max(ages) if ages else None,
            "state_age_limit_s": args.max_state_age,
            "hardware_diagnostics": diagnostics,
            "allocations": context.provider_selector.get_allocations(),
        }
        print(json.dumps(result, indent=2, sort_keys=True))
        if not result["passed"]:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
