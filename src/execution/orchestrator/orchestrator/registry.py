"""Dynamic node catalog: maps tree-file tag names to Node factories.

The registry is what lets the Web UI show "what nodes can I drop into a
tree" and lets :mod:`orchestrator.loaders.xml` build a tree generically
instead of hard-coding a tag switch.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from .tree import Node

#: How a tag's children (if any) are parsed from a tree file and passed to
#: its factory: "none" (leaf, no children), "one" (decorator, ``child=``),
#: "many" (control node, ``children=[...]``).
ChildArity = str


class NodeRegistry:
    """Registers Node factories under a tag name used by tree definitions."""

    def __init__(self) -> None:
        self._factories: dict[str, Callable[..., Node]] = {}
        self._descriptions: dict[str, str] = {}
        self._child_arity: dict[str, ChildArity] = {}

    def register(
        self,
        tag: str,
        factory: Callable[..., Node],
        *,
        description: str = "",
        child_arity: ChildArity = "none",
    ) -> None:
        if tag in self._factories:
            raise ValueError(f"node tag {tag!r} is already registered")
        if child_arity not in ("none", "one", "many"):
            raise ValueError("child_arity must be 'none', 'one', or 'many'")
        self._factories[tag] = factory
        self._descriptions[tag] = description
        self._child_arity[tag] = child_arity

    def unregister(self, tag: str) -> None:
        self._factories.pop(tag, None)
        self._descriptions.pop(tag, None)
        self._child_arity.pop(tag, None)

    def create(self, tag: str, name: str | None = None, **config: Any) -> Node:
        try:
            factory = self._factories[tag]
        except KeyError as exc:
            raise KeyError(f"unknown node tag {tag!r}; known: {sorted(self._factories)}") from exc
        return factory(name=name, **config)

    def child_arity(self, tag: str) -> ChildArity:
        try:
            return self._child_arity[tag]
        except KeyError as exc:
            raise KeyError(f"unknown node tag {tag!r}; known: {sorted(self._factories)}") from exc

    def known_tags(self) -> list[str]:
        return sorted(self._factories)

    def catalog(self) -> list[dict[str, str]]:
        """Node catalog payload consumed by the Web UI."""
        return [
            {
                "tag": tag,
                "description": self._descriptions.get(tag, ""),
                "child_arity": self._child_arity.get(tag, "none"),
            }
            for tag in self.known_tags()
        ]


def default_registry() -> NodeRegistry:
    """Registry pre-populated with the control nodes and RMI-backed leaves."""
    from .controls import Fallback, Parallel, Retry, Sequence
    from .nodes.conditions import CheckBlackboard
    from .nodes.planner import ExecuteRecoveryPlan
    from .nodes.policy import RunPolicy
    from .nodes.recorder import RecordEpisode
    from .nodes.task import Fail, SetBlackboard, Succeed, Wait
    from .nodes.teleop import PolicySkill

    registry = NodeRegistry()
    registry.register(
        "Sequence", Sequence, description="All children must SUCCEED, in order.", child_arity="many"
    )
    registry.register(
        "Fallback", Fallback, description="First child SUCCESS wins.", child_arity="many"
    )
    registry.register(
        "Parallel", Parallel, description="Ticks every child every tick.", child_arity="many"
    )
    registry.register(
        "Retry", Retry, description="Re-runs one child up to N attempts.", child_arity="one"
    )
    registry.register("RunPolicy", RunPolicy, description="Runs an RMI policy control scope.")
    registry.register(
        "ExecuteRecoveryPlan",
        ExecuteRecoveryPlan,
        description="Plans and executes one recovery motion via RMI.",
    )
    registry.register(
        "RecordEpisode",
        RecordEpisode,
        description="Wraps its child subtree in one RMI recorder.episode().",
        child_arity="one",
    )
    registry.register(
        "PolicySkill",
        PolicySkill,
        description="Observes policy/teleop arm ownership; never calls handover().",
    )
    registry.register("CheckBlackboard", CheckBlackboard, description="Blackboard-value condition.")
    registry.register("SetBlackboard", SetBlackboard, description="Write a blackboard value.")
    registry.register("Wait", Wait, description="RUNNING until a duration elapses.")
    registry.register("Succeed", Succeed, description="Always SUCCESS.")
    registry.register("Fail", Fail, description="Always FAILURE.")
    return registry
