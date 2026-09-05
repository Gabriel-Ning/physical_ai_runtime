"""Parallel: ticks every child every tick, independent of the others' status."""

from __future__ import annotations

from typing import Any

from ..tree import Node, Status


class Parallel(Node):
    """RUNNING until enough children terminate to decide SUCCESS or FAILURE.

    ``success_threshold`` (default: all children) is how many SUCCESSes are
    required overall; the node fails as soon as success becomes unreachable.
    Useful for e.g. ``PolicySkill`` observing both "policy owns arm" and
    "teleop intervened" facts without one masking the other.
    """

    def __init__(
        self,
        name: str | None = None,
        *,
        children: list[Node] = (),
        success_threshold: int | None = None,
        **config: Any,
    ) -> None:
        super().__init__(name, **config)
        self.children = list(children)
        self.success_threshold = (
            success_threshold if success_threshold is not None else len(self.children)
        )

    def update(self) -> Status:
        successes = 0
        failures = 0
        for child in self.children:
            status = (
                child.status
                if child.status in (Status.SUCCESS, Status.FAILURE)
                else child.tick()
            )
            if status is Status.SUCCESS:
                successes += 1
            elif status is Status.FAILURE:
                failures += 1
        if successes >= self.success_threshold:
            return Status.SUCCESS
        remaining = len(self.children) - failures
        if remaining < self.success_threshold:
            return Status.FAILURE
        return Status.RUNNING

    def terminate(self, status: Status) -> None:
        for child in self.children:
            child.reset()
