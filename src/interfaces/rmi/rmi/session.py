"""One Agent's lease-bound observation/action scope."""

from __future__ import annotations

import time
from collections.abc import Mapping
from types import MappingProxyType, TracebackType
from typing import TYPE_CHECKING, Any, Self

from execution_manager_interfaces.msg import AuthorityEvent, ResourceAuthority

from .contracts import Action, ControlDiagnostics, Observation

if TYPE_CHECKING:
    from .agent import Agent, PlanExecution
    from .robot import Robot


class Session:
    """One atomic EM lease over an immutable set of robot resources."""

    def __init__(
        self,
        robot: Robot,
        source: Agent,
        *,
        parts: tuple[str, ...],
        preempt: bool,
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
        self.preempt = preempt
        self.acquire_timeout = acquire_timeout
        self.frequency = frequency
        self.period = 1.0 / frequency if frequency is not None else None
        self.lease_id: str | None = None
        self.diagnostics = ControlDiagnostics()
        self._entered = False
        self._client = (
            source._client.fork()
            if hasattr(source._client, "fork")
            else source._client
        )
        self._next_deadline: float | None = None

    @property
    def agent(self) -> Agent:
        return self.source

    def observe(self) -> Observation:
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
    def active(self) -> bool:
        if not self._entered or not self.lease_id:
            return False
        allocations = self._robot.selection.get_allocations()
        return all(self._owns(part, allocations) for part in self.parts)

    def active_for(self, part: str) -> bool:
        if part not in self.parts or not self._entered or not self.lease_id:
            return False
        return self._owns(part, self._robot.selection.get_allocations())

    def _owns(self, part: str, allocations: Mapping[str, Any]) -> bool:
        allocation = allocations.get(part)
        return (
            isinstance(allocation, Mapping)
            and allocation.get("authority_state") == ResourceAuthority.OWNED
            and allocation.get("lease_id") == self.lease_id
        )

    def _is_explicitly_displaced(self) -> bool:
        """Return true only when status identifies a different lease."""
        if not self.lease_id:
            return True
        allocations = self._robot.selection.get_allocations()
        if any(
            isinstance(allocations.get(part), Mapping)
            and bool(allocations[part].get("lease_id"))
            and allocations[part].get("lease_id") != self.lease_id
            for part in self.parts
        ):
            return True
        get_events = getattr(self._robot.selection, "get_events", None)
        if get_events is None:
            return False
        return any(
            event.type == AuthorityEvent.PREEMPTED
            for event in get_events(lease_id=self.lease_id)
        )

    def __enter__(self) -> Self:
        if self._entered:
            raise RuntimeError("control session is already entered")
        resources = {part: self.source.resources[part] for part in self.parts}
        grant = self._robot.selection.claim(
            self.source.source_role,
            self.source.source_instance,
            resources,
            preempt=self.preempt,
            metadata=self.source.metadata,
        )
        self.lease_id = grant.lease_id
        self._client.bind(grant)
        self._entered = True
        self._robot._push_control(self)
        deadline = time.monotonic() + self.acquire_timeout
        while not self.active:
            if time.monotonic() >= deadline:
                try:
                    self._release_claim()
                finally:
                    self._client.unbind()
                    self._robot._pop_control(self)
                    self._entered = False
                raise TimeoutError(
                    f"timed out waiting for lease {self.lease_id!r} authority"
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
            if self._is_explicitly_displaced():
                self.diagnostics.displaced_exits += 1
                release_error = None
            else:
                # Missing/expired status is unknown, not proof of displacement.
                # Release by exact lease_id; EM owns the idempotency semantics.
                release_error = self._release_claim()
        finally:
            self._client.unbind()
            self._robot._pop_control(self)
            self._entered = False
        if release_error is not None and exc_type is None:
            raise release_error

    def _release_claim(self) -> BaseException | None:
        """Release an exact lease, including before status confirmation."""
        if not self.lease_id:
            return None
        try:
            self._robot.selection.release(self.lease_id)
        except Exception as err:  # noqa: BLE001
            return err
        return None

    def act(self, action: Action, *, observation: Observation | None = None) -> None:
        if action.part not in self.parts:
            raise ValueError(
                f"control source {self.source.name!r} does not own requested "
                f"resource {action.part!r} in this scope"
            )
        if not self.active_for(action.part):
            self.diagnostics.inactive_drops += 1
            return
        if observation is not None and observation.allocation_lease(
            action.part, self.source.source_instance
        ) != self.lease_id:
            self.diagnostics.stale_observation_drops += 1
            return
        self.source.send(action, _client=self._client)
        self.diagnostics.sent += 1

    def execute(
        self,
        part: str,
        plan: Any,
        *,
        observation: Observation | None = None,
    ) -> PlanExecution:
        if part not in self.parts:
            raise ValueError(
                f"control source {self.source.name!r} does not own requested "
                f"resource {part!r} in this scope"
            )
        if not self.active_for(part):
            raise RuntimeError("cannot execute a plan from an inactive control scope")
        if observation is not None and observation.allocation_lease(
            part, self.source.source_instance
        ) != self.lease_id:
            raise RuntimeError("cannot execute a plan from a stale observation")
        execution = self.source.execute(part, plan, _client=self._client)
        self.diagnostics.sent += 1
        return execution
