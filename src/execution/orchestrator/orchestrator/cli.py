"""CLI tool for the thin BT Orchestrator runtime."""

from __future__ import annotations

import argparse
import importlib
import sys
import time
from pathlib import Path

from .loaders.xml import load_tree
from .registry import default_registry
from .runtime import BehaviorTreeRuntime, TaskPhase
from .tree import NodeContext


def _load_context_factory(spec: str):
    """Load ``module:callable`` supplied by the application deployment."""
    try:
        module_name, attribute = spec.split(":", 1)
    except ValueError as exc:
        raise ValueError("context factory must use module:callable syntax") from exc
    factory = getattr(importlib.import_module(module_name), attribute)
    if not callable(factory):
        raise TypeError(f"context factory {spec!r} is not callable")
    return factory


def main() -> None:
    parser = argparse.ArgumentParser(description="Behavior Tree Orchestrator CLI")
    parser.add_argument("--profile", type=str, help="Path to embodiment profile YAML")
    parser.add_argument("--tree", type=str, help="Path to a <BehaviorTree> XML file")
    parser.add_argument(
        "--context-factory",
        help=(
            "Application factory as module:callable. Called with the profile Path; "
            "returns NodeContext or (resource_owner, NodeContext)."
        ),
    )
    parser.add_argument("--tick-hz", type=float, default=10.0)
    parser.add_argument(
        "--list-nodes", action="store_true", help="Print the node catalog and exit"
    )
    args = parser.parse_args()

    registry = default_registry()
    if args.list_nodes:
        for entry in registry.catalog():
            print(f"{entry['tag']:20s} ({entry['child_arity']:4s}) {entry['description']}")
        return

    if not args.profile or not args.tree or not args.context_factory:
        parser.error(
            "--profile, --tree and --context-factory are required unless "
            "--list-nodes is given"
        )

    root = load_tree(args.tree, registry)
    built = _load_context_factory(args.context_factory)(Path(args.profile))
    if isinstance(built, NodeContext):
        resource_owner, node_context = None, built
    else:
        resource_owner, node_context = built
    if not isinstance(node_context, NodeContext):
        raise TypeError("context factory must return a NodeContext")
    runtime = BehaviorTreeRuntime(root, node_context, tick_hz=args.tick_hz)
    try:
        runtime.start(background=True)
        while runtime.phase is TaskPhase.RUNNING:
            time.sleep(0.2)
    finally:
        if resource_owner is not None and hasattr(resource_owner, "close"):
            resource_owner.close()

    print(f"[orchestrator] task finished: {runtime.phase.value}")
    sys.exit(0 if runtime.phase is TaskPhase.SUCCEEDED else 1)


if __name__ == "__main__":
    main()
