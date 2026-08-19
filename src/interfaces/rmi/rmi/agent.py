"""Agent, Session, and application Robot control plane."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable, Mapping
from enum import Enum
from types import MappingProxyType, TracebackType
from typing import Any, Self

from sensor_msgs.msg import JointState as RosJointState

from .config import EmbodimentConfig
from .contracts import Action, ControlDiagnostics, Observation
from .controllers import TrajectoryCanceledError
from .execution_client import ExecutionManagerClient
from .provider import ActionProviderClient
from .sensing import Sensor


def _run_sync(awaitable: Any, *, context: str = "synchronous RMI call") -> Any:
    """Run ``awaitable`` when no asyncio loop is already running."""
    import asyncio

    try:
        asyncio.get_running_loop()
    except RuntimeError:
        return asyncio.run(awaitable)
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
        execution: ExecutionManagerClient | None = None,
        *,
        provider: str | None = None,
        frequency: float | None = None,
        robot: Robot | None = None,
        sensors: tuple[Sensor[Any], ...] = (),
    ) -> None:
        if frequency is not None and frequency <= 0.0:
            raise ValueError("agent frequency must be positive")
        self.name = name
        self.provider = provider if provider is not None else name
        self.frequency = frequency
        self._client = client
        self._config = config
        self._execution = execution
        self._robot = robot
        self.sensors = sensors

    @property
    def parts(self) -> tuple[str, ...]:
        return tuple(self._client.controllers)

    def run(
        self,
        robot: Robot | None = None,
        *,
        parts: list[str] | tuple[str, ...] | None = None,
        resume: bool = False,
        acquire_timeout: float = 5.0,
        frequency: float | None = None,
    ) -> Session:
        """Create a lightweight observation/action execution scope."""
        target = robot if robot is not None else self._robot
        if target is None:
            raise ValueError("agent.run() requires a robot")
        return Session(
            target,
            self,
            parts=tuple(parts) if parts is not None else self.parts,
            resume=resume,
            acquire_timeout=acquire_timeout,
            frequency=self.frequency if frequency is None else frequency,
        )

    def send(self, action: Action) -> Any:
        converted = self._send_planning_result(action)
        if converted:
            return None
        if action.command == "joint_trajectory":
            raise NotImplementedError(
                "use robot.execute(part, plan) under an active control scope"
            )
        return self._client.send(action.part, action.command, action.value)

    def _send_planning_result(self, action: Action) -> bool:
        from motion_planner_core import (
            JointHorizonResult,
            PlanResult,
            PoseHorizonResult,
            ResolveResult,
        )

        value = action.value
        if isinstance(value, ResolveResult):
            _require_valid_result(value)
            if action.command != "joint_reference":
                raise ValueError("ResolveResult requires command='joint_reference'")
            joint_names = value.joint_names or self._part_joint_names(action.part)
            self._client.send_joint_reference(
                action.part,
                joint_names,
                [list(value.positions or [])],
                [0.0],
            )
            return True
        if isinstance(value, JointHorizonResult):
            _require_valid_result(value)
            if action.command != "joint_reference":
                raise ValueError(
                    "JointHorizonResult requires command='joint_reference'"
                )
            self._client.send_joint_reference(
                action.part,
                self._part_joint_names(action.part),
                [list(point.positions) for point in value.points],
                [point.time_from_start_s for point in value.points],
            )
            return True
        if isinstance(value, PoseHorizonResult):
            _require_valid_result(value)
            if action.command != "pose_reference":
                raise ValueError("PoseHorizonResult requires command='pose_reference'")
            self._client.send_pose_reference(
                action.part,
                [list(point.position_xyz) for point in value.points],
                [
                    [
                        point.orientation_wxyz[1],
                        point.orientation_wxyz[2],
                        point.orientation_wxyz[3],
                        point.orientation_wxyz[0],
                    ]
                    for point in value.points
                ],
                [point.time_from_start_s for point in value.points],
                self._part_base_frame(action.part),
            )
            return True
        if isinstance(value, PlanResult):
            _require_valid_result(value)
            if action.command != "joint_trajectory":
                raise ValueError("PlanResult requires command='joint_trajectory'")
        return False

    def _part_joint_names(self, part: str) -> list[str]:
        if self._config is None or part not in self._config.parts:
            raise ValueError(
                f"profile joint names are required to send a result for part {part!r}"
            )
        return list(self._config.parts[part].joint_names)

    def _part_base_frame(self, part: str) -> str:
        if self._config is None or part not in self._config.parts:
            raise ValueError(
                f"profile base frame is required to send a result for part {part!r}"
            )
        frame = self._config.parts[part].base_frame
        if not frame:
            raise ValueError(f"part {part!r} does not declare base_frame")
        return frame

    def start_plan(self, part: str, plan: Any) -> PlanExecution:
        from motion_planner_core import PlanResult

        if not isinstance(plan, PlanResult):
            raise TypeError("plan must be a PlanResult")
        _require_valid_result(plan)
        if not plan.points:
            raise ValueError("cannot execute a plan without trajectory points")
        target_joint_names = self._part_joint_names(part)
        feedback = _FeedbackBuffer()
        handle = _run_sync(
            self._client.start_joint_trajectory(
                part,
                _trajectory_spec(plan, target_joint_names),
                target_joint_names,
                feedback.update,
            ),
            context="synchronous RMI execution",
        )
        return PlanExecution(
            self,
            part,
            handle,
            execution=self._execution,
            correlation_id=_goal_correlation_id(handle),
            feedback=feedback,
            duration_s=plan.points[-1].time_from_start_s,
        )

    def execute(self, part: str, plan: Any) -> PlanExecution:
        """Alias for start_plan."""
        return self.start_plan(part, plan)

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
    """Synchronous handle for one EM-routed JTC trajectory action."""

    def __init__(
        self,
        source: Agent,
        part: str,
        handle: Any,
        *,
        execution: ExecutionManagerClient | None = None,
        correlation_id: str = "",
        feedback: _FeedbackBuffer | None = None,
        duration_s: float = 0.0,
    ) -> None:
        self._source = source
        self.part = part
        self._handle = handle
        self._execution = execution
        self.correlation_id = correlation_id
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
            self.result = self._source._wait_plan(self.part, self._handle, timeout)
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
        self._source._cancel_plan(self.part)
        self.canceled = True
        self._cancel_requested = True
        self.state = PlanExecutionState.CANCEL_REQUESTED

    @property
    def events(self) -> list[dict[str, Any]]:
        if self._execution is None or not self.correlation_id:
            return []
        return self._execution.get_events(correlation_id=self.correlation_id)

    def wait_event(
        self,
        *kinds: str,
        timeout: float = 1.0,
    ) -> dict[str, Any] | None:
        if self._execution is None or not self.correlation_id:
            return None
        return self._execution.wait_for_execution_event(
            self.correlation_id,
            kinds,
            timeout_sec=timeout,
        )

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


class Session:
    """One Agent's lightweight observation/action execution scope."""

    def __init__(
        self,
        robot: Robot,
        source: Agent,
        *,
        parts: tuple[str, ...],
        resume: bool,
        acquire_timeout: float = 5.0,
        frequency: float | None = None,
    ) -> None:
        if acquire_timeout <= 0.0:
            raise ValueError("acquire_timeout must be positive")
        if frequency is not None and frequency <= 0.0:
            raise ValueError("session frequency must be positive")
        self._robot = robot
        self.source = source
        self.parts = parts
        self.resume = resume
        self.acquire_timeout = acquire_timeout
        self.frequency = frequency
        self.period = 1.0 / frequency if frequency is not None else None
        self._generation: int | None = None
        self._part_generations: dict[str, int] = {}
        self.diagnostics = ControlDiagnostics()
        self._entered = False
        self._next_deadline: float | None = None

    @property
    def agent(self) -> Agent:
        return self.source

    def observe(self) -> Observation:
        """Snapshot robot state and the Agent's configured sensor samples."""
        observation = self._robot.get_observation()
        samples = {sensor.name: sensor.latest for sensor in self.source.sensors}
        return Observation(
            data=observation.data,
            source_time_s=observation.source_time_s,
            receive_time_s=observation.receive_time_s,
            allocations=observation.allocations,
            sensors=MappingProxyType(samples),
        )

    def wait(self) -> None:
        """Wait for the next nominal Agent cycle without accumulating drift."""
        if self.period is None:
            raise RuntimeError("session has no configured frequency")
        now = time.monotonic()
        if self._next_deadline is None:
            self._next_deadline = now + self.period
        delay = self._next_deadline - now
        if delay > 0.0:
            time.sleep(delay)
        self._next_deadline += self.period

    @property
    def generation(self) -> int | None:
        """Latest generation, retained for single-part API compatibility."""
        return self._generation

    @generation.setter
    def generation(self, value: int | None) -> None:
        self._generation = value
        self._part_generations = (
            {part: value for part in self.parts} if value is not None else {}
        )

    def generation_for(self, part: str) -> int | None:
        """Return the authoritative generation tracked for one controlled part."""
        return self._part_generations.get(part)

    @property
    def active(self) -> bool:
        if not self._entered or self.generation is None:
            return False
        allocations = self._robot.execution.get_allocations()
        return all(self._refresh_part(part, allocations) for part in self.parts)

    def ok(self) -> bool:
        """Alias for self.active to support while session.ok() loops."""
        return self.active

    def active_for(self, part: str) -> bool:
        """Return whether this source currently owns one scoped Part."""
        if part not in self.parts:
            return False
        if not self._entered or self.generation is None:
            return False
        return self._refresh_part(part, self._robot.execution.get_allocations())

    def _owns_any_part(self) -> bool:
        allocations = self._robot.execution.get_allocations()
        return any(
            isinstance(allocation := allocations.get(part), Mapping)
            and allocation.get("provider") == self.source.provider
            for part in self.parts
        )

    def _refresh_part(
        self,
        part: str,
        allocations: Mapping[str, Any],
    ) -> bool:
        allocation = allocations.get(part)
        if not isinstance(allocation, Mapping):
            return False
        if allocation.get("provider") != self.source.provider:
            return False
        raw_generation = allocation.get("generation")
        if raw_generation is None:
            return False
        authoritative = int(raw_generation)
        tracked = self._part_generations.get(part)
        if authoritative == tracked:
            return True
        if not self.resume or (tracked is not None and authoritative < tracked):
            return False
        self._part_generations[part] = authoritative
        self._generation = max(self._part_generations.values())
        self.diagnostics.resumes += 1
        return True

    def __enter__(self) -> Self:
        if self._entered:
            raise RuntimeError("control session is already entered")
        self._robot.execution.prepare(self.source.provider)
        self.generation = self._robot.execution.acquire(self.source.provider)
        self._entered = True
        self._robot._push_control(self)
        deadline = time.monotonic() + self.acquire_timeout
        while not self.active:
            if time.monotonic() >= deadline:
                try:
                    self._release_if_owner(reason="allocation_confirmation_timeout")
                finally:
                    self._robot._pop_control(self)
                    self._entered = False
                raise TimeoutError(
                    f"timed out waiting for authoritative allocation of "
                    f"{self.source.name!r}"
                )
            time.sleep(0.01)
        if self.period is not None:
            self._next_deadline = time.monotonic() + self.period
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        del exc, traceback
        try:
            if self._owns_any_part():
                release_error = self._release_if_owner(reason="control_scope_exit")
            else:
                self.diagnostics.displaced_exits += 1
                release_error = None
        finally:
            self._robot._pop_control(self)
            self._entered = False
        if release_error is not None and exc_type is None:
            raise release_error

    def _release_if_owner(self, *, reason: str) -> BaseException | None:
        """Release ownership when still held; never raise (EM stays authoritative).

        Returns the release error, if any, so a non-unwinding caller can
        surface it.
        """
        if not self._owns_any_part():
            return None
        try:
            self._robot.execution.release(self.source.provider, reason=reason)
        except Exception as err:  # noqa: BLE001
            return err
        return None

    def send(self, action: Action, *, observation: Observation | None = None) -> None:
        """Publish when this ownership generation is still current.

        Local drops are intentionally silent in the normal control loop. Counts
        are exposed through ``diagnostics``; the EM remains authoritative.
        """
        if action.part not in self.parts:
            raise ValueError(
                f"control source {self.source.name!r} does not own requested "
                f"part {action.part!r} in this scope"
            )
        if not self.active_for(action.part):
            self.diagnostics.inactive_drops += 1
            return
        if observation is not None:
            observed_generation = observation.allocation_generation(
                action.part, self.source.provider
            )
            if observed_generation != self.generation_for(action.part):
                self.diagnostics.stale_observation_drops += 1
                return
        self.source.send(action)
        self.diagnostics.sent += 1

    def act(self, action: Action, *, observation: Observation | None = None) -> None:
        """Submit one action under the current ownership generation."""
        self.send(action, observation=observation)

    def execute(
        self,
        part: str,
        plan: Any,
        *,
        observation: Observation | None = None,
    ) -> PlanExecution:
        """Start a complete plan only while this EM generation is authoritative."""
        if part not in self.parts:
            raise ValueError(
                f"control source {self.source.name!r} does not own requested "
                f"part {part!r} in this scope"
            )
        if not self.active_for(part):
            raise RuntimeError("cannot execute a plan from an inactive control scope")
        if observation is not None:
            observed_generation = observation.allocation_generation(
                part, self.source.provider
            )
            if observed_generation != self.generation_for(part):
                raise RuntimeError("cannot execute a plan from a stale observation")
        execution = self.source.start_plan(part, plan)
        self.diagnostics.sent += 1
        return execution


class Robot:
    """Application-facing robot state and EM-routed command facade."""

    def __init__(
        self,
        config: EmbodimentConfig,
        execution: ExecutionManagerClient,
        *,
        clock: Callable[[], float] | None = None,
    ) -> None:
        self.name = config.name
        self.config = config
        self.execution = execution
        self._clock = clock if clock is not None else time.time
        self._state_lock = threading.Lock()
        self._latest_state: dict[str, Any] | None = None
        self._source_time_s = 0.0
        self._receive_time_s = 0.0
        self._control_stack: list[Session] = []

    @property
    def state(self) -> Observation:
        with self._state_lock:
            if self._latest_state is None:
                raise RuntimeError(
                    "robot state is unavailable; call wait_until_ready()"
                )
            data = dict(self._latest_state)
            source_time_s = self._source_time_s
            receive_time_s = self._receive_time_s
        allocations = {
            part: MappingProxyType(dict(allocation))
            for part, allocation in self.execution.get_allocations().items()
            if isinstance(allocation, Mapping)
        }
        return Observation(
            data=MappingProxyType(data),
            source_time_s=source_time_s,
            receive_time_s=receive_time_s,
            allocations=MappingProxyType(allocations),
        )

    def get_observation(self) -> Observation:
        return self.state

    def is_ready(self) -> bool:
        with self._state_lock:
            return self._latest_state is not None

    def wait_until_ready(
        self,
        timeout: float = 10.0,
        check_frequency: float = 50.0,
        check_hardware: bool = True,
    ) -> None:
        """Wait until joint states arrive and verify hardware component readiness."""
        deadline = time.monotonic() + timeout
        period = 1.0 / check_frequency
        while not self.is_ready():
            if time.monotonic() >= deadline:
                raise TimeoutError("timed out waiting for robot joint state")
            time.sleep(period)

        if check_hardware and hasattr(self, "execution"):
            try:
                if hasattr(self.execution, "ensure_hardware_active"):
                    activated = self.execution.ensure_hardware_active()
                    if activated:
                        _LOGGER.info(
                            "Robot hardware components automatically activated: %s",
                            activated,
                        )
            except Exception as e:
                _LOGGER.debug("Hardware readiness check non-fatal error: %s", e)

    def control(
        self,
        source: Agent,
        *,
        parts: list[str] | tuple[str, ...] | None = None,
        resume: bool = False,
        acquire_timeout: float = 5.0,
    ) -> Session:
        requested_parts = tuple(parts) if parts is not None else source.parts
        if not requested_parts:
            raise ValueError(f"source {source.name!r} has no configured Parts")
        unknown = set(requested_parts) - set(source.parts)
        if unknown:
            raise ValueError(
                f"source {source.name!r} is not configured for Parts {sorted(unknown)!r}"
            )
        return Session(
            self,
            source,
            parts=requested_parts,
            resume=resume,
            acquire_timeout=acquire_timeout,
        )

    def send_action(
        self, action: Action, *, observation: Observation | None = None
    ) -> None:
        self._control_for(action.part).send(action, observation=observation)

    def execute(
        self,
        part: str,
        plan: Any,
        *,
        observation: Observation | None = None,
    ) -> PlanExecution:
        return self._control_for(part).execute(
            part,
            plan,
            observation=observation,
        )

    def update_joint_state(
        self, message: RosJointState, *, receive_time_s: float | None = None
    ) -> None:
        stamp = message.header.stamp
        source_time_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        received = receive_time_s if receive_time_s is not None else self._clock()
        if source_time_s <= 0.0:
            source_time_s = received
        state = {
            "joint_names": tuple(message.name),
            "joint_positions": tuple(message.position),
            "joint_velocities": tuple(message.velocity),
            "joint_efforts": tuple(message.effort),
        }
        with self._state_lock:
            self._latest_state = state
            self._source_time_s = source_time_s
            self._receive_time_s = received

    def _control_for(self, part: str) -> Session:
        if not self._control_stack:
            raise RuntimeError(
                "send_action/execute requires an active agent.run(robot) scope"
            )
        for session in reversed(self._control_stack):
            if part in session.parts:
                return session
        raise RuntimeError(
            f"no active control scope owns part {part!r}; "
            "use an explicit Session when nested scopes do not cover it"
        )

    def _push_control(self, session: Session) -> None:
        self._control_stack.append(session)

    def _pop_control(self, session: Session) -> None:
        if not self._control_stack or self._control_stack[-1] is not session:
            raise RuntimeError("control sessions must exit in stack order")
        self._control_stack.pop()


def _require_valid_result(result: Any) -> None:
    if not result.valid:
        raise ValueError(f"cannot send invalid planning result: {result.reason}")


def _trajectory_spec(
    plan: Any, target_joint_names: list[str] | None = None
) -> dict[str, Any]:
    plan_joint_names = list(plan.joint_names or [])
    if (
        target_joint_names
        and plan_joint_names
        and set(target_joint_names).issubset(set(plan_joint_names))
    ):
        indices = [plan_joint_names.index(name) for name in target_joint_names]
        out_names = list(target_joint_names)
        points = []
        for point in plan.points:
            pos = [point.positions[i] for i in indices] if point.positions else []
            vel = [point.velocities[i] for i in indices] if point.velocities else []
            acc = [point.accelerations[i] for i in indices] if point.accelerations else []
            points.append(
                {
                    "positions": pos,
                    "velocities": vel,
                    "accelerations": acc,
                    "time_from_start_s": point.time_from_start_s,
                }
            )
        return {"joint_names": out_names, "points": points}

    return {
        "joint_names": plan_joint_names,
        "points": [
            {
                "positions": list(point.positions),
                "velocities": list(point.velocities or []),
                "accelerations": list(point.accelerations or []),
                "time_from_start_s": point.time_from_start_s,
            }
            for point in plan.points
        ],
    }


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


def _goal_correlation_id(handle: Any) -> str:
    goal_id = getattr(handle, "goal_id", None)
    values = getattr(goal_id, "uuid", None)
    if values is None:
        return ""
    return bytes(values).hex()
