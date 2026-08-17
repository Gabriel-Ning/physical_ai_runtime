"""Retry decorator: re-runs one child up to N attempts before failing."""

from __future__ import annotations

from typing import Any

from ..tree import Node, Status


class Retry(Node):
    """Wrap a single child; on FAILURE, reset and re-run it up to ``num_attempts``."""

    def __init__(
        self,
        name: str | None = None,
        *,
        child: Node,
        num_attempts: int = 1,
        **config: Any,
    ) -> None:
        super().__init__(name, **config)
        if num_attempts < 1:
            raise ValueError("num_attempts must be >= 1")
        self.children = [child]
        self.num_attempts = num_attempts
        self._attempt = 0

    @property
    def child(self) -> Node:
        return self.children[0]

    def update(self) -> Status:
        status = self.child.tick()
        if status is Status.RUNNING:
            return Status.RUNNING
        if status is Status.SUCCESS:
            return Status.SUCCESS
        self._attempt += 1
        if self._attempt >= self.num_attempts:
            return Status.FAILURE
        self.child.reset()
        return Status.RUNNING

    def terminate(self, status: Status) -> None:
        self.child.reset()
        self._attempt = 0
