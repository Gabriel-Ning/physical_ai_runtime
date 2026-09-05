"""Small task-control utility leaves (blackboard writes, waiting, constants)."""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import Any

from ..tree import Node, Status


class SetBlackboard(Node):
    """Writes one value and immediately returns SUCCESS."""

    def __init__(self, name: str | None = None, *, key: str, value: Any, **config: Any) -> None:
        super().__init__(name, **config)
        self.key = key
        self.value = value

    def update(self) -> Status:
        self.blackboard.set(self.key, self.value)
        return Status.SUCCESS


class Wait(Node):
    """RUNNING until ``seconds`` have elapsed since this run started."""

    def __init__(
        self,
        name: str | None = None,
        *,
        seconds: float,
        clock: Callable[[], float] = time.monotonic,
        **config: Any,
    ) -> None:
        super().__init__(name, **config)
        self.seconds = seconds
        self._clock = clock
        self._deadline: float | None = None

    def initialise(self) -> None:
        self._deadline = self._clock() + self.seconds

    def update(self) -> Status:
        assert self._deadline is not None
        return Status.SUCCESS if self._clock() >= self._deadline else Status.RUNNING


class Succeed(Node):
    """Always SUCCESS; useful as a placeholder/test leaf."""

    def update(self) -> Status:
        return Status.SUCCESS


class Fail(Node):
    """Always FAILURE; useful as a placeholder/test leaf."""

    def update(self) -> Status:
        return Status.FAILURE
