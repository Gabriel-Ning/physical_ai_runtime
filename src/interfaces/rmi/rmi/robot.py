"""Application-facing robot state facade."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable, Mapping
from types import MappingProxyType
from typing import TYPE_CHECKING, Any

from sensor_msgs.msg import JointState as RosJointState

from .config import EmbodimentConfig
from .contracts import Action, Observation
from .selection import AuthorityClient
from .session import Session

if TYPE_CHECKING:
    from .agent import Agent, PlanExecution


class Robot:
    """Application-facing robot state and provider-routed command facade."""

    def __init__(
        self,
        config: EmbodimentConfig,
        selection: AuthorityClient,
        *,
        clock: Callable[[], float] | None = None,
    ) -> None:
        self.name = config.name
        self.config = config
        self.selection = selection
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
        # ExecutionManagerClient serves this from its status-topic cache. State
        # observation never performs an Execution Manager service request.
        allocations = {
            part: MappingProxyType(dict(allocation))
            for part, allocation in self.selection.get_allocations().items()
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
        check_hardware: bool = False,
    ) -> None:
        """Wait for robot state and query hardware health without changing it."""
        deadline = time.monotonic() + timeout
        period = 1.0 / check_frequency
        while not self.is_ready():
            if time.monotonic() >= deadline:
                raise TimeoutError("timed out waiting for robot joint state")
            time.sleep(period)

        if check_hardware:
            get_diagnostics = getattr(
                self.selection, "get_hardware_diagnostics", None
            )
            if get_diagnostics is None:
                raise RuntimeError(
                    "hardware diagnostics are not supported by this authority client"
                )
            diagnostics = get_diagnostics()
            if diagnostics:
                details = "; ".join(diagnostics)
                raise RuntimeError(f"robot hardware is not ready: {details}")

    def control(
        self,
        source: Agent,
        *,
        parts: list[str] | tuple[str, ...] | None = None,
        preempt: bool = False,
        acquire_timeout: float = 5.0,
        frequency: float | None = None,
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
            preempt=preempt,
            acquire_timeout=acquire_timeout,
            frequency=frequency,
        )

    def send_action(
        self, action: Action, *, observation: Observation | None = None
    ) -> None:
        self._control_for(action.part).act(action, observation=observation)

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
