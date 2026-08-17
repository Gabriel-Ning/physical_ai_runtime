"""Observability: current node, tree status, and failure reason.

This is what the Web UI's node catalog + live tree status view is built
from — see docs/BEHAVIOR_TREE_ORCHESTRATOR_DESIGN.md §"Web UI 所需的 node
catalog 和 tree status".
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .tree import Node, Status


@dataclass
class NodeSnapshot:
    name: str
    type: str
    status: str
    children: list[NodeSnapshot] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "type": self.type,
            "status": self.status,
            "children": [child.to_dict() for child in self.children],
        }


def snapshot_node(node: Node) -> NodeSnapshot:
    return NodeSnapshot(
        name=node.name,
        type=type(node).__name__,
        status=node.status.value,
        children=[snapshot_node(child) for child in node.children],
    )


def running_path(snapshot: NodeSnapshot) -> list[str]:
    """Names of every node currently RUNNING, in tree order (any depth)."""
    path: list[str] = []
    if snapshot.status == Status.RUNNING.value:
        path.append(snapshot.name)
    for child in snapshot.children:
        path.extend(running_path(child))
    return path


def first_failure_reason(snapshot: NodeSnapshot) -> str | None:
    """Name of the first (depth-first) node whose last tick was FAILURE."""
    if snapshot.status == Status.FAILURE.value:
        return snapshot.name
    for child in snapshot.children:
        reason = first_failure_reason(child)
        if reason is not None:
            return reason
    return None


@dataclass
class TreeStatus:
    task_phase: str
    root: NodeSnapshot | None
    running_path: list[str]
    failure_reason: str | None
    blackboard: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return {
            "task_phase": self.task_phase,
            "root": self.root.to_dict() if self.root is not None else None,
            "running_path": self.running_path,
            "failure_reason": self.failure_reason,
            "blackboard": self.blackboard,
        }
