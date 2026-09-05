"""Fallback (selector)-with-memory: ticks children in order, first SUCCESS wins."""

from __future__ import annotations

from typing import Any

from ..tree import Node, Status


class Fallback(Node):
    """SUCCESS as soon as one child succeeds; FAILURE only if all children fail.

    This is the Human-in-the-Loop DAgger recovery shape: try the autonomous
    skill first, fall back to a recovery branch (e.g. ``ExecuteRecoveryPlan``)
    only when it fails.
    """

    def __init__(self, name: str | None = None, *, children: list[Node] = (), **config: Any) -> None:
        super().__init__(name, **config)
        self.children = list(children)
        self._index = 0

    def update(self) -> Status:
        while self._index < len(self.children):
            child = self.children[self._index]
            status = child.tick()
            if status is Status.RUNNING:
                return Status.RUNNING
            if status is Status.SUCCESS:
                return Status.SUCCESS
            self._index += 1
        return Status.FAILURE

    def terminate(self, status: Status) -> None:
        for child in self.children:
            child.reset()
        self._index = 0
