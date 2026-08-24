"""Agent and synchronous plan-execution handle."""

from __future__ import annotations

import asyncio
import threading
from enum import Enum
from typing import Any

from .config import EmbodimentConfig
from .contracts import Action, PoseHorizonResult, ResolveResult
from .errors import TrajectoryCanceledError
from .provider import ActionProviderClient
from .robot import Robot
from .sensing import Sensor
from .session import Session

_SYNC_LOOP: asyncio.AbstractEventLoop | None = None
_SYNC_LOOP_THREAD: threading.Thread | None = None
_SYNC_LOOP_LOCK = threading.Lock()


def _ensure_sync_loop() -> asyncio.AbstractEventLoop:
    """Process-wide asyncio loop for sync wrappers around rclpy action clients.

    ``asyncio.run()`` creates and destroys a loop per call. Destroying the loop
    after ``send_goal_async`` returns cancels the in-flight FollowJointTrajectory
    goal (the Execution Manager then cancels the RT JTC). Keep one background loop alive.
    """
    global _SYNC_LOOP, _SYNC_LOOP_THREAD
    with _SYNC_LOOP_LOCK:
        if _SYNC_LOOP is not None and _SYNC_LOOP.is_running():
            return _SYNC_LOOP
        loop = asyncio.new_event_loop()

        def _run() -> None:
            asyncio.set_event_loop(loop)
            loop.run_forever()

        thread = threading.Thread(
            target=_run, name="rmi-sync-asyncio", daemon=True
        )
        thread.start()
        _SYNC_LOOP = loop
        _SYNC_LOOP_THREAD = thread
        return loop


def _run_sync(awaitable: Any, *, context: str = "synchronous RMI call") -> Any:
    """Run ``awaitable`` on the shared RMI asyncio loop when no loop is running."""
    try:
        asyncio.get_running_loop()
    except RuntimeError:
        loop = _ensure_sync_loop()
        future = asyncio.run_coroutine_threadsafe(awaitable, loop)
        return future.result()
    if hasattr(awaitable, "close"):
        awaitable.close()
    raise RuntimeError(f"{context} cannot run inside an asyncio loop")


class Agent:
    """One action-producing participant in the RMI execution system."""

    def __init__(
        self,
        name: str,
        client: ActionProviderClient,
        config: EmbodimentConfig | None = None,
        *,
        source_role: str = "POLICY",
        source_instance: str | None = None,
        metadata: dict[str, str] | None = None,
        resources: dict[str, str] | None = None,
        frequency: float | None = None,
        robot: Robot | None = None,
        sensors: tuple[Sensor[Any], ...] = (),
    ) -> None:
        if frequency is not None and frequency <= 0.0:
            raise ValueError("agent frequency must be positive")
        self.name = name
        self.source_role = source_role
        self.source_instance = source_instance or name
        self.metadata = dict(metadata or {})
        self.resources = dict(resources or {})
        self.frequency = frequency
        self._client = client
        self._config = config
        self._robot = robot
        self.sensors = sensors

    @property
    def parts(self) -> tuple[str, ...]:
        return tuple(self.resources)

    def run(
        self,
        robot: Robot | None = None,
        *,
        parts: list[str] | tuple[str, ...] | None = None,
        preempt: bool = False,
        acquire_timeout: float = 5.0,
        frequency: float | None = None,
    ) -> Session:
        """Create a lightweight observation/action execution scope."""
        target = robot if robot is not None else self._robot
        if target is None:
            raise ValueError("agent.run() requires a robot")
        return target.control(
            self,
            parts=parts,
            preempt=preempt,
            acquire_timeout=acquire_timeout,
            frequency=self.frequency if frequency is None else frequency,
        )

    def send(
        self, action: Action, *, _client: ActionProviderClient | None = None
    ) -> Any:
        if action.command == "joint_trajectory":
            raise NotImplementedError(
                "use robot.execute(part, plan) under an active control scope"
            )
        value = action.value
        if value is not None:
            _require_valid_result(value)
            if isinstance(value, ResolveResult) and action.command != "joint_reference":
                raise ValueError("ResolveResult requires command='joint_reference'")
            if (
                isinstance(value, PoseHorizonResult)
                and action.command != "pose_reference"
            ):
                raise ValueError("PoseHorizonResult requires command='pose_reference'")
        return (_client or self._client).send(action.part, action.command, value)

    def execute(
        self,
        part: str,
        plan: Any,
        *,
        _client: ActionProviderClient | None = None,
    ) -> PlanExecution:
        if not hasattr(plan, "points") or not isinstance(plan.points, (list, tuple)):
            raise TypeError("plan must have trajectory points")
        _require_valid_result(plan)
        if not plan.points:
            raise ValueError("cannot execute a plan without trajectory points")
        target_joint_names = self._part_joint_names(part)
        feedback = _FeedbackBuffer()
        command_client = _client or self._client
        handle = _run_sync(
            command_client.start_joint_trajectory(
                part,
                plan,
                target_joint_names,
                feedback.update,
            ),
            context="synchronous RMI execution",
        )
        return PlanExecution(
            self,
            part,
            handle,
            client=command_client,
            feedback=feedback,
            duration_s=_point_time_s(plan.points[-1]),
        )

    def _part_joint_names(self, part: str) -> list[str]:
        if self._config is None or part not in self._config.parts:
            raise ValueError(
                f"profile joint names are required to send a result for part {part!r}"
            )
        return list(self._config.parts[part].joint_names)

    def _wait_plan(self, part: str, handle: Any, timeout: float) -> Any:
        return _run_sync(
            self._client.wait_joint_trajectory(part, handle, timeout),
            context="synchronous RMI execution",
        )

    def _cancel_plan(self, part: str) -> None:
        _run_sync(
            self._client.cancel_joint_trajectory(part),
            context="synchronous RMI execution",
        )


class PlanExecutionState(str, Enum):
    ACCEPTED = "ACCEPTED"
    COMPLETED = "COMPLETED"
    CANCEL_REQUESTED = "CANCEL_REQUESTED"
    CANCELED = "CANCELED"
    FAILED = "FAILED"
    TIMED_OUT = "TIMED_OUT"


class PlanExecution:
    """Synchronous handle for one provider-routed JTC trajectory action."""

    def __init__(
        self,
        source: Agent,
        part: str,
        handle: Any,
        *,
        client: ActionProviderClient,
        feedback: _FeedbackBuffer | None = None,
        duration_s: float = 0.0,
    ) -> None:
        self._source = source
        self.part = part
        self._handle = handle
        self._client = client
        self._feedback = feedback or _FeedbackBuffer()
        self._duration_s = duration_s
        self.result: Any | None = None
        self.state = PlanExecutionState.ACCEPTED
        self.done = False
        self.canceled = False
        self._cancel_requested = False

    def wait(self, timeout: float = 10.0) -> Any:
        """Wait for a terminal ROS action result.

        ``TIMED_OUT`` means only that this local wait expired; the goal may
        still be running on the controller until canceled or completed.
        A locally requested ``cancel()`` makes a CANCELED terminal result a
        normal return; an external cancellation still raises so takeovers
        are not silently absorbed.
        """
        try:
            self.result = _run_sync(
                self._client.wait_joint_trajectory(
                    self.part, self._handle, timeout
                ),
                context="synchronous RMI execution",
            )
        except TimeoutError:
            self.state = PlanExecutionState.TIMED_OUT
            raise
        except TrajectoryCanceledError:
            self.done = True
            self.canceled = True
            self.state = PlanExecutionState.CANCELED
            if self._cancel_requested:
                return None
            raise
        except Exception:
            self.state = PlanExecutionState.FAILED
            raise
        self.done = True
        self.state = PlanExecutionState.COMPLETED
        return self.result

    def cancel(self) -> None:
        _run_sync(
            self._client.cancel_joint_trajectory(self.part),
            context="synchronous RMI execution",
        )
        self.canceled = True
        self._cancel_requested = True
        self.state = PlanExecutionState.CANCEL_REQUESTED

    @property
    def feedback(self) -> Any | None:
        return self._feedback.latest

    @property
    def progress(self) -> float | None:
        feedback = self.feedback
        desired = getattr(feedback, "desired", None)
        elapsed = _duration_seconds(getattr(desired, "time_from_start", None))
        if elapsed is None or self._duration_s <= 0.0:
            return None
        return min(1.0, max(0.0, elapsed / self._duration_s))


def _require_valid_result(result: Any) -> None:
    # Native ROS trajectory messages and plain containers do not expose a `.valid` flag.
    # Keep compatibility by treating all objects without `.valid` as valid by default.
    if not hasattr(result, "valid"):
        return
    if not bool(getattr(result, "valid")):
        reason = getattr(result, "reason", "<no reason provided>")
        raise ValueError(f"cannot send invalid planning result: {reason}")


def _point_time_s(point: Any) -> float:
    if hasattr(point, "time_from_start_s"):
        return float(point.time_from_start_s)
    if hasattr(point, "time_from_start"):
        t = point.time_from_start
        if hasattr(t, "sec") and hasattr(t, "nanosec"):
            return float(t.sec) + float(t.nanosec) * 1e-9
        if isinstance(t, (int, float)):
            return float(t)
    return 0.0


class _FeedbackBuffer:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: Any | None = None

    def update(self, message: Any) -> None:
        feedback = getattr(message, "feedback", message)
        with self._lock:
            self._latest = feedback

    @property
    def latest(self) -> Any | None:
        with self._lock:
            return self._latest


def _duration_seconds(duration: Any | None) -> float | None:
    if duration is None:
        return None
    return float(duration.sec) + float(duration.nanosec) * 1e-9
