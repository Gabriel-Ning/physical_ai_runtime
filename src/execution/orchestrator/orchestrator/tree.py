"""Core Behavior Tree node contract: ``SUCCESS`` / ``FAILURE`` / ``RUNNING``.

This module intentionally has no RMI import. Leaf nodes under ``nodes/`` are
the only place that calls into RMI; everything here is generic tree
machinery so it can be unit tested with plain fakes.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from enum import Enum
from typing import Any

from .blackboard import Blackboard


class Status(Enum):
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    RUNNING = "RUNNING"
    INVALID = "INVALID"  # never ticked yet


@dataclass
class NodeContext:
    """RMI capabilities and shared task state injected into every node.

    ``robot`` is an RMI ``RobotFacade`` (or a test fake exposing the same
    ``control()`` / ``execute()`` / ``state`` surface). ``policies`` contain
    inference callables, ``planners`` contain pure planning capabilities, and
    ``sources`` contain RMI ``ActionSource`` identities. ``recorder`` is an RMI
    ``RecorderFacade`` (or ``None`` when recording is not configured).
    """

    robot: Any
    blackboard: Blackboard
    recorder: Any | None = None
    policies: Mapping[str, Any] = field(default_factory=dict)
    planners: Mapping[str, Any] = field(default_factory=dict)
    sources: Mapping[str, Any] = field(default_factory=dict)
    extras: Mapping[str, Any] = field(default_factory=dict)

    def policy(self, name: str) -> Any:
        try:
            return self.policies[name]
        except KeyError as exc:
            raise KeyError(f"unknown policy {name!r} in tree context") from exc

    def planner(self, name: str) -> Any:
        try:
            return self.planners[name]
        except KeyError as exc:
            raise KeyError(f"unknown planner {name!r} in tree context") from exc

    def source(self, name: str) -> Any:
        try:
            return self.sources[name]
        except KeyError as exc:
            raise KeyError(f"unknown action source {name!r} in tree context") from exc


class Node:
    """Base class for every BT node (leaf or control).

    Subclasses implement :meth:`update`. ``initialise``/``terminate`` follow
    the standard BT lifecycle: ``initialise`` runs once when a node starts a
    fresh run (i.e. it was not already ``RUNNING``), ``terminate`` runs once
    when a node leaves ``RUNNING`` (either terminal status).
    """

    children: Sequence[Node] = ()

    def __init__(self, name: str | None = None, **config: Any) -> None:
        self.name = name or type(self).__name__
        self.config = config
        self.status = Status.INVALID
        self.context: NodeContext | None = None

    # -- wiring -----------------------------------------------------------
    def bind(self, context: NodeContext) -> None:
        """Attach RMI capabilities/blackboard; recurse into children."""
        self.context = context
        self.blackboard = context.blackboard
        self.robot = context.robot
        self.recorder = context.recorder
        for child in self.children:
            child.bind(context)
        self.setup()

    def setup(self) -> None:
        """One-time hook after binding, before the first tick."""

    # -- BT lifecycle -------------------------------------------------------
    def initialise(self) -> None:
        """Run once when a fresh run starts (previous status was not RUNNING)."""

    def update(self) -> Status:
        raise NotImplementedError

    def terminate(self, status: Status) -> None:
        """Run once when leaving RUNNING for a terminal status."""

    def tick(self) -> Status:
        if self.status != Status.RUNNING:
            self.initialise()
            # Resources acquired by initialise() must be visible to reset()
            # even when update() raises before returning a status.
            self.status = Status.RUNNING
        status = self.update()
        if status is Status.RUNNING:
            self.status = status
            return status
        self.terminate(status)
        self.status = status
        return status

    def reset(self) -> None:
        """Force this node (and its subtree) back to a fresh, un-ticked state."""
        if self.status is Status.RUNNING:
            self.terminate(Status.INVALID)
        self.status = Status.INVALID
        for child in self.children:
            child.reset()

    def node_catalog_entry(self) -> dict[str, Any]:
        """Minimal description used by the Web UI node catalog."""
        return {"name": self.name, "type": type(self).__name__}
