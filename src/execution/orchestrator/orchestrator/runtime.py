"""Thin BT runtime: fixed-frequency tick engine + task lifecycle.

This is the only class that owns a clock/thread. Everything it calls into
(the tree, the blackboard, RMI leaf nodes) is otherwise passive. Per the
architecture in docs/BEHAVIOR_TREE_ORCHESTRATOR_DESIGN.md, the runtime
knows nothing about EM, ros2_control, or ROS transports — it only ticks a
``Node`` and reports ``SUCCESS`` / ``FAILURE`` / ``RUNNING``.
"""

from __future__ import annotations

import threading
import time
from enum import Enum

from .status import TreeStatus, first_failure_reason, running_path, snapshot_node
from .tree import Node, NodeContext, Status


class TaskPhase(Enum):
    PENDING = "pending"
    RUNNING = "running"
    PAUSED = "paused"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    ABORTED = "aborted"


_TERMINAL_PHASES = (TaskPhase.SUCCEEDED, TaskPhase.FAILED, TaskPhase.ABORTED)
_RESTARTABLE_PHASES = (TaskPhase.PENDING, *_TERMINAL_PHASES)


class BehaviorTreeRuntime:
    """Loads one tree, ticks it, and exposes task start/abort/pause/resume."""

    def __init__(self, root: Node, context: NodeContext, *, tick_hz: float = 10.0) -> None:
        if tick_hz <= 0.0:
            raise ValueError("tick_hz must be positive")
        self.root = root
        self.context = context
        self.tick_hz = tick_hz
        self.phase = TaskPhase.PENDING
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self.last_error: str | None = None
        self.root.bind(context)

    # -- task lifecycle -----------------------------------------------------
    def start(self, *, background: bool = False) -> None:
        """Begin (or restart, once terminal) ticking this tree from scratch."""
        with self._lock:
            if self.phase not in _RESTARTABLE_PHASES:
                raise RuntimeError(f"cannot start task while phase is {self.phase.value}")
            self.root.reset()
            self.phase = TaskPhase.RUNNING
            self.last_error = None
            self._stop_event.clear()
        if background:
            self._thread = threading.Thread(target=self._run_loop, daemon=True)
            self._thread.start()

    def pause(self) -> None:
        with self._lock:
            if self.phase != TaskPhase.RUNNING:
                raise RuntimeError("can only pause a running task")
            self.phase = TaskPhase.PAUSED

    def resume(self) -> None:
        with self._lock:
            if self.phase != TaskPhase.PAUSED:
                raise RuntimeError("can only resume a paused task")
            self.phase = TaskPhase.RUNNING

    def abort(self) -> None:
        """Terminate the tree immediately, releasing any open control/episode scopes."""
        with self._lock:
            if self.phase in (TaskPhase.RUNNING, TaskPhase.PAUSED):
                self.root.reset()
                self.phase = TaskPhase.ABORTED
        self._stop_event.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)
            self._thread = None

    def stop_background(self) -> None:
        """Stop the tick thread without changing task phase (e.g. on shutdown)."""
        self._stop_event.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)
            self._thread = None

    # -- ticking --------------------------------------------------------------
    def tick_once(self) -> Status:
        with self._lock:
            if self.phase is TaskPhase.PAUSED:
                return Status.RUNNING
            if self.phase is not TaskPhase.RUNNING:
                raise RuntimeError(f"cannot tick while task phase is {self.phase.value}")
            try:
                status = self.root.tick()
            except Exception as exc:
                self.last_error = f"{type(exc).__name__}: {exc}"
                self.root.reset()
                self.phase = TaskPhase.FAILED
                self._stop_event.set()
                raise
            if status is Status.SUCCESS:
                self.phase = TaskPhase.SUCCEEDED
            elif status is Status.FAILURE:
                self.phase = TaskPhase.FAILED
            return status

    def tick_until_terminal(self, max_ticks: int | None = None) -> Status:
        """Synchronous convenience for tests/CLI: tick until SUCCESS/FAILURE."""
        ticks = 0
        while True:
            status = self.tick_once()
            if status is not Status.RUNNING:
                return status
            ticks += 1
            if max_ticks is not None and ticks >= max_ticks:
                raise TimeoutError(f"tree did not reach a terminal status within {max_ticks} ticks")

    def _run_loop(self) -> None:
        period = 1.0 / self.tick_hz
        while not self._stop_event.is_set():
            started = time.monotonic()
            if self.phase is TaskPhase.RUNNING:
                try:
                    status = self.tick_once()
                except Exception:  # noqa: BLE001 - tick_once records the leaf failure
                    break
                if status is not Status.RUNNING:
                    break
            elapsed = time.monotonic() - started
            self._stop_event.wait(max(0.0, period - elapsed))

    # -- observability --------------------------------------------------------
    @property
    def status(self) -> TreeStatus:
        snapshot = snapshot_node(self.root)
        return TreeStatus(
            task_phase=self.phase.value,
            root=snapshot,
            running_path=running_path(snapshot),
            failure_reason=self.last_error or first_failure_reason(snapshot),
            blackboard=self.context.blackboard.snapshot(),
        )
