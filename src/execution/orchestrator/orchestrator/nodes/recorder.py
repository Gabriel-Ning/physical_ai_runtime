"""``RecordEpisode``: a Recorder decorator wrapping a child subtree.

``RecordEpisode`` is deliberately a decorator, not a Sequence sibling of
start/stop actions — see docs/BEHAVIOR_TREE_ORCHESTRATOR_DESIGN.md §3.3.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from ..tree import Node, Status


class RecordEpisode(Node):
    """Wraps ``recorder.episode(task=...)`` around ticking a single child."""

    def __init__(
        self,
        name: str | None = None,
        *,
        child: Node,
        task: str,
        metadata: Mapping[str, Any] | None = None,
        stop_timeout: float = 60.0,
        **config: Any,
    ) -> None:
        super().__init__(name, **config)
        self.children = [child]
        self.task = task
        self.metadata = dict(metadata or {})
        self.stop_timeout = stop_timeout
        self._episode: Any = None

    @property
    def child(self) -> Node:
        return self.children[0]

    def initialise(self) -> None:
        if self.recorder is None:
            raise RuntimeError(
                f"{self.name}: no recorder configured in this tree's NodeContext"
            )
        self._episode = self.recorder.episode(
            task=self.task,
            metadata=self.metadata,
            stop_timeout=self.stop_timeout,
        )
        self._episode.__enter__()

    def update(self) -> Status:
        return self.child.tick()

    def terminate(self, status: Status) -> None:
        if self._episode is not None:
            self._episode.__exit__(None, None, None)
            self._episode = None
        self.child.reset()
