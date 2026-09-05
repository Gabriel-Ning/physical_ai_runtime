"""Thin HTTP/WebSocket-facing control plane: task control + tree status only.

Per docs/BEHAVIOR_TREE_ORCHESTRATOR_DESIGN.md, everything else the old
``OrchestratorServer`` exposed (sessions, capabilities preflight, episode
RPCs, motion verbs) moved to RMI or was dropped; this server only starts/
stops tasks and reports tree status + the node catalog for the Web UI.
"""

from __future__ import annotations

from typing import Any

from .registry import NodeRegistry
from .runtime import BehaviorTreeRuntime


class OrchestratorServer:
    """Handler methods for a Web UI / CLI transport adapter to wire up."""

    def __init__(self, runtime: BehaviorTreeRuntime, registry: NodeRegistry) -> None:
        self.runtime = runtime
        self.registry = registry

    def get_node_catalog(self) -> dict[str, Any]:
        """HTTP GET /nodes handler."""
        return {"nodes": self.registry.catalog()}

    def get_tree_status(self) -> dict[str, Any]:
        """HTTP GET /tree/status handler."""
        return self.runtime.status.to_dict()

    def start_task(self, *, background: bool = True) -> dict[str, Any]:
        """HTTP POST /task/start handler."""
        self.runtime.start(background=background)
        return {"task_phase": self.runtime.phase.value}

    def pause_task(self) -> dict[str, Any]:
        """HTTP POST /task/pause handler."""
        self.runtime.pause()
        return {"task_phase": self.runtime.phase.value}

    def resume_task(self) -> dict[str, Any]:
        """HTTP POST /task/resume handler."""
        self.runtime.resume()
        return {"task_phase": self.runtime.phase.value}

    def abort_task(self) -> dict[str, Any]:
        """HTTP POST /task/abort handler."""
        self.runtime.abort()
        return {"task_phase": self.runtime.phase.value}
