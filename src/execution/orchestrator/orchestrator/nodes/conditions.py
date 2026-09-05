"""Simple blackboard condition leaf."""

from __future__ import annotations

from typing import Any

from ..tree import Node, Status


class CheckBlackboard(Node):
    """SUCCESS when ``blackboard[key]`` is set and (truthy, or == ``equals``)."""

    _UNSET = object()

    def __init__(
        self,
        name: str | None = None,
        *,
        key: str,
        equals: Any = _UNSET,
        **config: Any,
    ) -> None:
        super().__init__(name, **config)
        self.key = key
        self.equals = equals

    def update(self) -> Status:
        if not self.blackboard.has(self.key):
            return Status.FAILURE
        value = self.blackboard.get(self.key)
        if self.equals is self._UNSET:
            return Status.SUCCESS if value else Status.FAILURE
        return Status.SUCCESS if value == self.equals else Status.FAILURE
