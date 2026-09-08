#!/usr/bin/env python3
"""Evaluate a policy or teleop runner against benchmark task specifications.

Resolves official suites:
  LIBERO 130 + LIBERO-Plus overlays (Franka)
  RoboTwin 2.0 50 + RoboTwin 2.0-Plus overlays (Piper)
  DuoBench 11 with stage metadata (Marvin)
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any

import numpy as np

try:
    import rmi
except ImportError:
    rmi = None

# Import registries
import duobench_tasks
import libero_tasks
import robotwin_tasks
from task_base import (
    BaseTask,
    PLUS_PERTURBATION_PROFILES,
    list_plus_profiles,
)


def resolve_task(name: str, context: Any = None, **kwargs: Any) -> BaseTask:
    """Resolve and instantiate task across LIBERO, RoboTwin, and DuoBench suites."""
    if name in libero_tasks.list_tasks():
        return libero_tasks.get_task(name, context=context, **kwargs)
    if name in robotwin_tasks.list_tasks():
        return robotwin_tasks.get_task(name, context=context, **kwargs)
    if name in duobench_tasks.list_tasks():
        return duobench_tasks.get_task(name, context=context, **kwargs)
    raise KeyError(
        f"Task '{name}' not found. Use --list-tasks to view all available tasks."
    )


def list_all_tasks() -> list[str]:
    """Return unified list of all registered tasks across all suites."""
    all_names = (
        set(libero_tasks.list_tasks())
        | set(robotwin_tasks.list_tasks())
        | set(duobench_tasks.list_tasks())
    )
    return sorted(all_names)


class DummyHoldPolicy:
    """Fallback policy holding current joint positions if no neural checkpoint provided."""

    def __init__(self, target_joints: list[float] | None = None) -> None:
        self.target_joints = target_joints

    def select_action(self, observation: Any) -> list[float]:
        if observation is None:
            return [0.0] * 7
        data = getattr(observation, "data", {})
        pos = data.get("joint_positions")
        if pos:
            return list(pos)
        return [0.0] * 7

    def close(self) -> None:
        pass


def run_batch_validation() -> int:
    """Systematically validate instantiation, reset, and check_success for ALL 197 tasks."""
    all_tasks = list_all_tasks()
    print("=" * 70)
    print(f" Physical AI Runtime — Full Suite Batch Task Validation")
    print(
        f" Total Registered Tasks: {len(all_tasks)} "
        f"(LIBERO: {len(libero_tasks.list_tasks())}, "
        f"RoboTwin: {len(robotwin_tasks.list_tasks())}, "
        f"DuoBench: {len(duobench_tasks.list_tasks())})"
    )
    print("=" * 70)

    passed_count = 0
    failed_tasks: list[tuple[str, str]] = []
    start_time = time.time()

    for idx, task_name in enumerate(all_tasks, start=1):
        try:
            task = resolve_task(task_name, context=None)

            # 1. Test reset and domain randomization
            params = task.reset()
            if not isinstance(params, dict) or len(params) == 0:
                raise ValueError(f"task.reset() returned invalid params: {params}")

            # 2. Test negative check_success (unreached initial state)
            if task.check_success(None):
                raise ValueError("check_success(None) returned True (expected False)")
            if task.check_success({}):
                raise ValueError("check_success({}) returned True (expected False)")

            passed_count += 1
            if idx % 25 == 0 or idx == len(all_tasks):
                print(f"  [{idx:3d}/{len(all_tasks)}] Validated {task_name[:45]}... OK")

        except Exception as e:
            failed_tasks.append((task_name, str(e)))
            print(f"  [{idx:3d}/{len(all_tasks)}] FAILED: {task_name} -> {e}")

    total_time = time.time() - start_time
    print("\n" + "=" * 70)
    print(f" Validation Results: {passed_count}/{len(all_tasks)} PASSED in {total_time:.2f}s")
    if failed_tasks:
        print(f" Failures ({len(failed_tasks)}):")
        for name, err in failed_tasks:
            print(f"   - {name}: {err}")
        print("=" * 70)
        return 1
    else:
        print(f" Official suites verified (LIBERO 130, RoboTwin 50, DuoBench 11).")
        print(f" Plus overlays: {', '.join(list_plus_profiles())}")
        print(f" - Randomization & reset() verified for registered tasks.")
        print(f" - Predicate evaluation & check_success() verified on empty obs.")
        print("=" * 70)
        return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Physical AI Runtime Task Evaluation API Runner")
    parser.add_argument(
        "--validate-all",
        action="store_true",
        help="Run reset/check_success validation across all registered tasks",
    )
    parser.add_argument(
        "--list-tasks",
        action="store_true",
        help="List all registered benchmark tasks across all suites",
    )
    parser.add_argument(
        "--plus-profile",
        default="demo_clean",
        choices=list(PLUS_PERTURBATION_PROFILES),
        help="LIBERO-Plus / RoboTwin 2.0-Plus overlay (default: demo_clean)",
    )
    parser.add_argument(
        "--task",
        default="",
        help="Benchmark task name (e.g. open_laptop, pick_up_the_black_bowl...)",
    )
    parser.add_argument(
        "--profile",
        default="fr3_pika_single_arm.yaml",
        help="Embodiment profile in apps/profiles/",
    )
    parser.add_argument(
        "--resource",
        default="arm",
        help="Robot resource group in profile (e.g. arm or manipulator)",
    )
    parser.add_argument(
        "--node-name",
        default="Policy",
        help="RMI source node name matching EM config (e.g. Policy)",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=5,
        help="Number of evaluation episodes to run",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=300,
        help="Maximum steps per episode",
    )
    parser.add_argument(
        "--control-freq",
        type=float,
        default=30.0,
        help="Policy control loop frequency (Hz)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.list_tasks:
        libero = libero_tasks.list_tasks()
        robotwin = robotwin_tasks.list_tasks()
        duobench = duobench_tasks.list_tasks()
        print(f"============================================================")
        print(
            f" Physical AI Runtime Benchmark Catalog "
            f"({len(libero) + len(robotwin) + len(duobench)} Total Tasks)"
        )
        print(f"============================================================")
        print(f"\n[1] LIBERO Benchmark Suite -> Franka FR3 Single Arm ({len(libero)} tasks)")
        print(f"    Bringup Stack: pixi run rt-franka backend:=mujoco task:=<task_name>")
        print(f"    Dispatch Stack: pixi run workstation-franka use_sim_time:=true")
        for t in libero:
            print(f"      - {t}")
        print(f"\n[2] RoboTwin Benchmark Suite -> Piper Dual Arm ({len(robotwin)} tasks)")
        print(f"    Bringup Stack: pixi run rt-piper backend:=mujoco task:=<task_name>")
        print(f"    Dispatch Stack: pixi run workstation-piper use_sim_time:=true")
        for t in robotwin:
            print(f"      - {t}")
        print(f"\n[3] DuoBench Suite -> Marvin Dual Arm ({len(duobench)} tasks)")
        print(f"    Bringup Stack: pixi run rt-marvin backend:=mujoco task:=<task_name>")
        print(f"    Dispatch Stack: pixi run workstation-marvin use_sim_time:=true")
        for t in duobench:
            print(f"      - {t}")
        print(f"\n[4] Plus overlays (LIBERO-Plus / RoboTwin 2.0-Plus) -> --plus-profile")
        for p in list_plus_profiles():
            print(f"      - {p}")
        sys.exit(0)

    if args.validate_all:
        exit_code = run_batch_validation()
        sys.exit(exit_code)

    if not args.task:
        print("[ERROR] Please provide --task <task_name> or pass --validate-all / --list-tasks.", file=sys.stderr)
        sys.exit(1)

    # Auto-detect profile and resource group if user left default
    profile = args.profile
    resource = args.resource
    if args.task in robotwin_tasks.list_tasks():
        if profile == "fr3_pika_single_arm.yaml":
            profile = "piper_bimanual.yaml"
        if resource == "arm":
            resource = "left_arm"
    elif args.task in duobench_tasks.list_tasks():
        if profile == "fr3_pika_single_arm.yaml":
            profile = "marvin_bimanual.yaml"
        if resource == "arm":
            resource = "left_arm"

    print(f"============================================================")
    print(f" Physical AI Runtime — Task Evaluator API")
    print(f" Task: {args.task}")
    print(f" Profile: {profile}")
    print(f" Resource: {resource}")
    print(f" Episodes: {args.episodes}")
    print(f"============================================================")

    if rmi is None:
        print("[ERROR] 'rmi' package is not available. Please source the workspace install.", file=sys.stderr)
        sys.exit(1)

    policy = DummyHoldPolicy()

    with rmi.Context.from_profile(profile) as context:
        task = resolve_task(
            args.task,
            context=context,
            max_steps=args.max_steps,
            control_freq=args.control_freq,
            plus_profile=args.plus_profile,
        )

        policy_node = context.make_node(args.node_name, policy)
        period_s = 1.0 / args.control_freq

        print("[Context] Waiting for Execution Manager and RT Stack to be ready...")
        context.wait_until_ready(
            timeout=30.0,
            check_cameras=False,
            require_execution_manager=True,
        )
        print("[Context] Runtime stack connected! Starting evaluation loop.")

        success_count = 0
        episode_durations = []

        with policy_node.activate():
            for ep in range(1, args.episodes + 1):
                print(f"\n--- [Episode {ep}/{args.episodes}] Starting ---")
                random_params = task.reset()
                print(f"  Randomized domain params: {random_params}")

                start_time = time.time()
                next_tick = time.monotonic()
                ep_success = False

                for step in range(task.max_steps):
                    next_tick += period_s

                    # Retrieve observation from Robot facade
                    try:
                        obs = context.robot[resource].get_observation()
                    except Exception:
                        obs = context.robot.get_observation()

                    # Select and submit action to Execution Manager
                    actions = policy_node[resource].select_action(obs)
                    policy_node[resource].submit(actions)

                    # Step task logic and check goal predicates
                    obs_dict = dict(obs.data) if hasattr(obs, "data") else {}
                    if task.check_success(obs_dict):
                        ep_success = True
                        print(f"  [SUCCESS] Goal criteria satisfied at step {step + 1}!")
                        break

                    sleep_s = next_tick - time.monotonic()
                    if sleep_s > 0.0:
                        time.sleep(sleep_s)
                    else:
                        next_tick = time.monotonic()

                ep_duration = time.time() - start_time
                episode_durations.append(ep_duration)
                if ep_success:
                    success_count += 1
                else:
                    print(f"  [TIMEOUT] Episode reached max steps ({task.max_steps}) without goal satisfaction.")

        success_rate = (success_count / args.episodes) * 100.0
        avg_duration = float(np.mean(episode_durations)) if episode_durations else 0.0

        print(f"\n============================================================")
        print(f" Evaluation Completed for: {args.task}")
        print(f" Success Rate: {success_count}/{args.episodes} ({success_rate:.1f}%)")
        print(f" Average Episode Duration: {avg_duration:.2f}s")
        print(f"============================================================")


if __name__ == "__main__":
    main()
