"""Sequence-with-memory: ticks children in order, all must SUCCEED."""

from __future__ import annotations

from typing import Any

from ..tree import Node, Status


class Sequence(Node):
    """SUCCESS only when every child succeeds, in order.

    A child returning ``RUNNING`` pauses the sequence there; the next tick
    resumes at the same child instead of re-ticking earlier (already
    succeeded) siblings. Any child ``FAILURE`` resets progress and fails the
    whole sequence.
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
            if status is Status.FAILURE:
                return Status.FAILURE
            self._index += 1
        return Status.SUCCESS

    def terminate(self, status: Status) -> None:
        # Reset progress and any not-yet-terminal children so re-running
        # this subtree (e.g. a retried task) starts from the first child.
        for child in self.children:
            child.reset()
        self._index = 0
