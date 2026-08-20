"""ROS-independent provider lifecycle and admission core."""

from __future__ import annotations

import asyncio
import inspect
import logging
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from enum import Enum
from types import MappingProxyType
from typing import Any, Protocol

from .topology import RobotTopology

_LOGGER = logging.getLogger(__name__)


class ProviderState(str, Enum):
    """Lifecycle states owned by the ExecutionManager."""

    STOPPED = "STOPPED"
    STARTING = "STARTING"
    READY = "READY"
    ACTIVE = "ACTIVE"
    FAILED = "FAILED"


class ProviderLifecycle(Protocol):
    """Lifecycle hooks for a named provider registered with ExecutionManager.

    Distinct from :class:`~rmi.ActionProviderClient`, which is the workstation
    command-plane client that publishes to EM gateways. This protocol only
    covers ``start`` / ``stop`` / ``reset`` (and optional ``deactivate``)
    callbacks the EM invokes during prepare, handover, release, and stop.

    An optional ``async deactivate()`` may be implemented; the EM calls it on
    a provider being displaced by a takeover, before controllers are switched.
    Autonomously publishing providers should stop emitting from that hook.
    """

    async def start(self) -> None: ...

    async def stop(self) -> None: ...

    async def reset(self, observation: Any) -> None: ...


@dataclass(frozen=True)
class ProviderRegistration:
    """Static provider targets and requested controller contracts."""

    name: str
    provider: ProviderLifecycle
    controllers: Mapping[str, str]
    priority: int = 0


@dataclass(frozen=True)
class ActionChunk:
    """One provider command fenced by an EM generation."""

    provider: str
    generation: int
    parts: tuple[str, ...]
    sequence: int
    payload: Any
    correlation_id: str = ""


@dataclass(frozen=True)
class Allocation:
    """Current provider ownership for one Part."""

    provider: str
    controller: str
    generation: int


@dataclass(frozen=True)
class ExecutionEvent:
    """Raw runtime fact intended for telemetry and episode recording."""

    sequence: int
    timestamp_ns: int
    kind: str
    provider: str | None = None
    parts: tuple[str, ...] = ()
    controllers: tuple[str, ...] = ()
    generation: int | None = None
    reason: str = ""
    correlation_id: str = ""


class LifecycleTransitionError(RuntimeError):
    """A provider lifecycle operation was requested from an invalid state."""


class HandoverError(RuntimeError):
    """A handover transaction failed after its old generation was fenced."""


class ArbitrationRejected(RuntimeError):
    """A lower-priority provider attempted to take an allocated Part."""


@dataclass
class _ProviderRuntime:
    registration: ProviderRegistration
    state: ProviderState = ProviderState.STOPPED


class ExecutionManager:
    """Coordinate provider lifecycle, same-controller handover, and fencing.

    Thread affinity: the manager is not thread-safe. All coroutine entry
    points and the synchronous ``admit``/``reject``/``dispatch`` calls must
    run on one asyncio event loop; ROS-side adapters post work onto that
    loop instead of touching the manager from executor threads.

    Arbitration: ``handover`` is an explicit control-plane command. Only a
    strictly higher-priority active owner blocks a takeover; equal-priority
    providers may displace each other deliberately. Passive stickiness is
    the caller's policy, not the manager's.
    """

    def __init__(
        self,
        robot: RobotTopology,
        event_sink: Callable[[ExecutionEvent], None] | None = None,
        clock_ns: Callable[[], int] = time.monotonic_ns,
        clock_domain: str = "steady",
    ) -> None:
        self.robot = robot
        self._event_sinks = [event_sink] if event_sink is not None else []
        self._clock_ns = clock_ns
        self.clock_domain = clock_domain
        self._providers: dict[str, _ProviderRuntime] = {}
        self._allocations: dict[str, Allocation] = {}
        self._resume_candidates: dict[str, str] = {}
        self._generation = 0
        self._track_sequences: dict[tuple[str, int, str], int] = {}
        self._event_sequence = 0
        self._lock = asyncio.Lock()

    @property
    def provider_states(self) -> Mapping[str, ProviderState]:
        """Return an immutable snapshot of provider lifecycle states."""
        return MappingProxyType(
            {name: runtime.state for name, runtime in self._providers.items()}
        )

    @property
    def allocations(self) -> Mapping[str, Allocation]:
        """Return an immutable snapshot of current Part allocations."""
        return MappingProxyType(dict(self._allocations))

    @property
    def generation(self) -> int:
        """Return the latest generation issued by this manager."""
        return self._generation

    def add_event_sink(self, sink: Callable[[ExecutionEvent], None]) -> None:
        """Subscribe a synchronous, non-blocking raw-event sink."""
        if sink in self._event_sinks:
            raise ValueError("event sink is already registered")
        self._event_sinks.append(sink)

    def register(self, registration: ProviderRegistration) -> None:
        """Register a stopped provider after validating its Part contracts."""
        if not registration.name:
            raise ValueError("provider name must not be empty")
        if registration.name in self._providers:
            raise ValueError(f"provider {registration.name!r} is already registered")
        if not registration.controllers:
            raise ValueError("provider must target at least one Part")
        for part_name, contract in registration.controllers.items():
            if part_name not in self.robot.parts:
                raise ValueError(f"provider targets unknown Part {part_name!r}")
            if contract not in self.robot.parts[part_name].controllers:
                raise ValueError(
                    f"Part {part_name!r} has no controller contract {contract!r}"
                )
        frozen = ProviderRegistration(
            name=registration.name,
            provider=registration.provider,
            controllers=MappingProxyType(dict(registration.controllers)),
            priority=registration.priority,
        )
        self._providers[registration.name] = _ProviderRuntime(frozen)
        self._emit("provider_registered", provider=registration.name)

    async def prepare(self, provider_name: str) -> None:
        """Transition one provider from STOPPED through STARTING to READY."""
        async with self._lock:
            runtime = self._provider(provider_name)
            if runtime.state is not ProviderState.STOPPED:
                raise LifecycleTransitionError(
                    f"cannot prepare {provider_name!r} from {runtime.state.value}"
                )
            runtime.state = ProviderState.STARTING
            self._emit("provider_state", provider=provider_name, reason="STARTING")
            try:
                await runtime.registration.provider.start()
            except Exception as exc:
                runtime.state = ProviderState.FAILED
                self._emit("provider_failed", provider=provider_name, reason=str(exc))
                raise
            runtime.state = ProviderState.READY
            self._emit("provider_state", provider=provider_name, reason="READY")

    async def handover(self, provider_name: str, observation: Any = None) -> int:
        """Run a fenced same- or cross-controller handover transaction.

        On failure the targeted Parts stay fenced and unowned: controllers
        are rolled back (or deactivated) and hold, previous providers remain
        READY, and a supervisor must issue a new handover to resume motion.
        """
        async with self._lock:
            runtime = self._provider(provider_name)
            if runtime.state is not ProviderState.READY:
                raise LifecycleTransitionError(
                    f"cannot activate {provider_name!r} from {runtime.state.value}"
                )
            registration = runtime.registration
            self._emit(
                "provider_takeover_requested",
                provider=provider_name,
                parts=tuple(registration.controllers),
                controllers=tuple(registration.controllers.values()),
            )
            blockers = {
                allocation.provider
                for part_name in registration.controllers
                if (allocation := self._allocations.get(part_name)) is not None
                and allocation.provider != provider_name
                and self._providers[allocation.provider].registration.priority
                > registration.priority
            }
            if blockers:
                reason = "higher_priority_owner:" + ",".join(sorted(blockers))
                self._emit(
                    "arbitration_rejected",
                    provider=provider_name,
                    parts=tuple(registration.controllers),
                    reason=reason,
                )
                raise ArbitrationRejected(reason)
            for part_name in registration.controllers:
                await self.robot.parts[part_name].synchronize_controller_state()
            mismatched = [
                part_name
                for part_name, contract in registration.controllers.items()
                if self.robot.parts[part_name].controllers[contract]
                is not self.robot.parts[part_name].active_controller
            ]
            if mismatched and observation is None:
                raise ValueError(
                    "cross-controller handover requires a current observation"
                )
            self._generation += 1
            generation = self._generation
            target_parts = tuple(registration.controllers)
            self._track_sequences = {
                key: sequence
                for key, sequence in self._track_sequences.items()
                if key[2] not in target_parts
            }
            displaced_parts = tuple(
                part_name
                for part_name in registration.controllers
                if (allocation := self._allocations.get(part_name)) is not None
                and allocation.provider != provider_name
            )
            displaced = {
                self._allocations[part_name].provider for part_name in displaced_parts
            }
            if len(displaced) == 1:
                self._resume_candidates[provider_name] = next(iter(displaced))
            else:
                self._resume_candidates.pop(provider_name, None)
            previous_contracts = {
                part_name: self._active_contract(part_name) for part_name in mismatched
            }
            # Cancel in-flight transactional goals on every taken-over Part,
            # including same-controller takeovers, so the old provider's
            # motion stops before the new provider is admitted.
            old_clients = {
                id(self.robot.parts[part_name].active_controller): self.robot.parts[
                    part_name
                ].active_controller
                for part_name in {*mismatched, *displaced_parts}
                if self.robot.parts[part_name].active_controller is not None
            }
            for part_name in registration.controllers:
                self._allocations.pop(part_name, None)
            self._set_displaced_ready(displaced, generation)
            self._emit(
                "handover_started",
                provider=provider_name,
                parts=tuple(registration.controllers),
                controllers=tuple(registration.controllers.values()),
                generation=generation,
            )

            switched: list[str] = []
            reset_started = False
            try:
                for old_name in displaced:
                    deactivate = getattr(
                        self._providers[old_name].registration.provider,
                        "deactivate",
                        None,
                    )
                    if deactivate is not None:
                        await deactivate()
                for client in old_clients.values():
                    cancel = getattr(client, "cancel", None)
                    if cancel is not None:
                        await cancel()
                for part_name in mismatched:
                    contract = registration.controllers[part_name]
                    await self.robot.parts[part_name].switch_controller(contract)
                    switched.append(part_name)
                    self._emit(
                        "controller_switched",
                        provider=provider_name,
                        parts=(part_name,),
                        controllers=(contract,),
                        generation=generation,
                    )
                reset_started = True
                await registration.provider.reset(observation)
            except Exception as exc:
                rollback_errors = []
                for part_name in reversed(switched):
                    try:
                        previous = previous_contracts[part_name]
                        if previous is None:
                            await self.robot.parts[
                                part_name
                            ].deactivate_active_controller()
                        else:
                            await self.robot.parts[part_name].switch_controller(
                                previous
                            )
                    except Exception as rollback_exc:
                        rollback_errors.append(f"{part_name}: {rollback_exc}")
                if reset_started:
                    runtime.state = ProviderState.FAILED
                reason = str(exc)
                if rollback_errors:
                    reason += "; rollback failed: " + ", ".join(rollback_errors)
                self._emit(
                    "handover_failed",
                    provider=provider_name,
                    parts=tuple(registration.controllers),
                    controllers=tuple(registration.controllers.values()),
                    generation=generation,
                    reason=reason,
                )
                raise HandoverError(reason) from exc

            for part_name, contract in registration.controllers.items():
                self._allocations[part_name] = Allocation(
                    provider=provider_name,
                    controller=contract,
                    generation=generation,
                )
            runtime.state = ProviderState.ACTIVE
            self._emit(
                "provider_handover",
                provider=provider_name,
                parts=tuple(registration.controllers),
                controllers=tuple(registration.controllers.values()),
                generation=generation,
            )
            return generation

    async def acquire(self, provider_name: str, observation: Any = None) -> int:
        """Wire/API alias for :meth:`handover`.

        ``ProviderCommand.ACQUIRE`` and workstation clients call ``acquire``;
        the engine implementation remains ``handover`` (fenced takeover with
        optional controller switch and provider reset).
        """
        return await self.handover(provider_name, observation)

    def _active_contract(self, part_name: str) -> str | None:
        part = self.robot.parts[part_name]
        if part.active_controller is None:
            return None
        for contract, controller in part.controllers.items():
            if controller is part.active_controller:
                return contract
        raise RuntimeError(f"Part {part_name!r} active controller is not declared")

    def _set_displaced_ready(self, displaced: set[str], generation: int) -> None:
        for old_name in displaced:
            if not any(
                allocation.provider == old_name
                for allocation in self._allocations.values()
            ):
                self._providers[old_name].state = ProviderState.READY
                self._emit(
                    "provider_state",
                    provider=old_name,
                    generation=generation,
                    reason="READY",
                )

    def admit(self, chunk: ActionChunk) -> bool:
        """Atomically admit monotonic, independently sequenced Part tracks."""
        unique_parts = tuple(dict.fromkeys(chunk.parts))
        parts_valid = bool(unique_parts) and len(unique_parts) == len(chunk.parts)
        allocations_match = parts_valid and all(
            (allocation := self._allocations.get(part_name)) is not None
            and allocation.provider == chunk.provider
            and allocation.generation == chunk.generation
            for part_name in unique_parts
        )
        sequence_is_new = chunk.sequence >= 0 and all(
            chunk.sequence
            > self._track_sequences.get(
                (chunk.provider, chunk.generation, part_name), -1
            )
            for part_name in unique_parts
        )
        accepted = allocations_match and sequence_is_new
        if not accepted:
            if not parts_valid:
                reason = "invalid_parts"
            elif not allocations_match:
                reason = "stale_or_unallocated_generation"
            else:
                reason = "non_monotonic_sequence"
            self._emit(
                "chunk_rejected",
                provider=chunk.provider,
                parts=chunk.parts,
                generation=chunk.generation,
                reason=reason,
                correlation_id=chunk.correlation_id,
            )
            return False
        for part_name in unique_parts:
            self._track_sequences[(chunk.provider, chunk.generation, part_name)] = (
                chunk.sequence
            )
        return accepted

    def reject(self, chunk: ActionChunk, reason: str) -> None:
        """Record an adapter-level rejection without changing track state."""
        self._emit(
            "chunk_rejected",
            provider=chunk.provider,
            parts=chunk.parts,
            generation=chunk.generation,
            reason=reason,
            correlation_id=chunk.correlation_id,
        )

    def record_execution_event(
        self,
        kind: str,
        chunk: ActionChunk,
        *,
        reason: str = "",
    ) -> None:
        """Record a gateway-observed terminal/action fact for one admitted chunk."""
        self._emit(
            kind,
            provider=chunk.provider,
            parts=chunk.parts,
            generation=chunk.generation,
            reason=reason,
            correlation_id=chunk.correlation_id,
        )

    async def dispatch(self, chunk: ActionChunk, command: str) -> bool:
        """Admit and send one native command on one independently fenced Part."""
        if len(chunk.parts) != 1:
            raise ValueError("dispatch requires exactly one Part track")
        methods = {
            "joint_trajectory": "send",
            "joint_reference": "send",
            "gripper_command": "send",
            "pose_reference": "send_pose",
            "twist_reference": "send_twist",
        }
        try:
            method_name = methods[command]
        except KeyError as exc:
            raise ValueError(f"unsupported action command {command!r}") from exc
        if not self.admit(chunk):
            return False
        part_name = chunk.parts[0]
        allocation = self._allocations[part_name]
        client = self.robot.parts[part_name].controllers[allocation.controller]
        try:
            result = getattr(client, method_name)(chunk.payload)
            if inspect.isawaitable(result):
                await result
        except Exception as exc:
            self.reject(chunk, f"dispatch_failed:{exc}")
            raise
        return True

    async def fail(self, provider_name: str, reason: str) -> None:
        """Fence an unhealthy provider, stop its motion, and free its Parts."""
        async with self._lock:
            runtime = self._provider(provider_name)
            owned = tuple(
                (part, allocation.controller)
                for part, allocation in self._allocations.items()
                if allocation.provider == provider_name
            )
            self._release(provider_name)
            runtime.state = ProviderState.FAILED
            self._generation += 1
            cancel_errors = await self._cancel_clients(owned)
            if cancel_errors:
                reason += "; cancel failed: " + ", ".join(cancel_errors)
            self._emit(
                "provider_failed",
                provider=provider_name,
                parts=tuple(part for part, _ in owned),
                controllers=tuple(controller for _, controller in owned),
                generation=self._generation,
                reason=reason,
            )

    async def stop(self, provider_name: str) -> None:
        """Stop a provider, fencing its allocation and canceling its motion."""
        async with self._lock:
            runtime = self._provider(provider_name)
            if runtime.state is ProviderState.STOPPED:
                return
            owned = tuple(
                (part, allocation.controller)
                for part, allocation in self._allocations.items()
                if allocation.provider == provider_name
            )
            self._release(provider_name)
            self._generation += 1
            cancel_errors = await self._cancel_clients(owned)
            await runtime.registration.provider.stop()
            runtime.state = ProviderState.STOPPED
            reason = "STOPPED"
            if cancel_errors:
                reason += "; cancel failed: " + ", ".join(cancel_errors)
            self._emit(
                "provider_state",
                provider=provider_name,
                parts=tuple(part for part, _ in owned),
                controllers=tuple(controller for _, controller in owned),
                generation=self._generation,
                reason=reason,
            )

    async def _cancel_clients(self, owned: tuple[tuple[str, str], ...]) -> list[str]:
        """Cancel in-flight transactional goals so fenced Parts stop moving."""
        errors: list[str] = []
        seen: set[int] = set()
        for part, contract in owned:
            client = self.robot.parts[part].controllers[contract]
            cancel = getattr(client, "cancel", None)
            if cancel is None or id(client) in seen:
                continue
            seen.add(id(client))
            try:
                await cancel()
            except Exception as exc:
                errors.append(f"{part}: {exc}")
        return errors

    async def release(
        self,
        provider_name: str,
        next_provider: str | None = None,
        observation: Any = None,
        reason: str = "",
    ) -> int | None:
        """Stop an active provider and atomically restore an eligible provider."""
        async with self._lock:
            runtime = self._provider(provider_name)
            if runtime.state is not ProviderState.ACTIVE:
                raise LifecycleTransitionError(
                    f"cannot release {provider_name!r} from {runtime.state.value}"
                )
            owned = {
                part: allocation
                for part, allocation in self._allocations.items()
                if allocation.provider == provider_name
            }
            if not owned:
                raise LifecycleTransitionError(
                    f"active provider {provider_name!r} has no allocation"
                )

            resume_name = next_provider or self._select_resume_provider(
                provider_name, set(owned)
            )
            resume_runtime = (
                self._provider(resume_name) if resume_name is not None else None
            )
            if resume_runtime is not None:
                if resume_runtime.state not in (
                    ProviderState.READY,
                    ProviderState.ACTIVE,
                ):
                    raise LifecycleTransitionError(
                        f"cannot resume {resume_name!r} from "
                        f"{resume_runtime.state.value}"
                    )
                missing = set(owned) - set(resume_runtime.registration.controllers)
                if missing:
                    raise ValueError(
                        f"provider {resume_name!r} does not cover released Parts "
                        f"{sorted(missing)!r}"
                    )
                for part in owned:
                    await self.robot.parts[part].synchronize_controller_state()
                mismatched = [
                    part
                    for part in owned
                    if self.robot.parts[part].controllers[
                        resume_runtime.registration.controllers[part]
                    ]
                    is not self.robot.parts[part].active_controller
                ]
                if mismatched and observation is None:
                    raise ValueError(
                        "cross-controller release requires a current observation"
                    )
            else:
                mismatched = []

            self._generation += 1
            generation = self._generation
            released_parts = tuple(owned)
            self._track_sequences = {
                key: sequence
                for key, sequence in self._track_sequences.items()
                if key[2] not in owned
            }
            self._release(provider_name)
            cancel_errors = await self._cancel_clients(
                tuple(
                    (part, allocation.controller) for part, allocation in owned.items()
                )
            )
            if cancel_errors:
                reason = (reason + "; " if reason else "") + (
                    "cancel failed: " + ", ".join(cancel_errors)
                )
            self._emit(
                "provider_release_started",
                provider=provider_name,
                parts=released_parts,
                controllers=tuple(item.controller for item in owned.values()),
                generation=generation,
                reason=reason,
            )

            try:
                await runtime.registration.provider.stop()
            except Exception as exc:
                runtime.state = ProviderState.FAILED
                self._emit(
                    "provider_failed",
                    provider=provider_name,
                    parts=released_parts,
                    generation=generation,
                    reason=str(exc),
                )
                raise HandoverError(str(exc)) from exc
            runtime.state = ProviderState.STOPPED
            self._emit(
                "provider_state",
                provider=provider_name,
                parts=released_parts,
                generation=generation,
                reason="STOPPED",
            )
            self._emit(
                "provider_released",
                provider=provider_name,
                parts=released_parts,
                controllers=tuple(item.controller for item in owned.values()),
                generation=generation,
                reason=reason,
            )

            if resume_runtime is None or resume_name is None:
                return None

            registration = resume_runtime.registration
            previous_contracts = {
                part: self._active_contract(part) for part in mismatched
            }
            switched: list[str] = []
            try:
                for part in mismatched:
                    contract = registration.controllers[part]
                    await self.robot.parts[part].switch_controller(contract)
                    switched.append(part)
                    self._emit(
                        "controller_switched",
                        provider=resume_name,
                        parts=(part,),
                        controllers=(contract,),
                        generation=generation,
                    )
                await registration.provider.reset(observation)
            except Exception as exc:
                for part in reversed(switched):
                    previous = previous_contracts[part]
                    if previous is None:
                        await self.robot.parts[part].deactivate_active_controller()
                    else:
                        await self.robot.parts[part].switch_controller(previous)
                resume_runtime.state = ProviderState.FAILED
                self._emit(
                    "handover_failed",
                    provider=resume_name,
                    parts=released_parts,
                    generation=generation,
                    reason=str(exc),
                )
                raise HandoverError(str(exc)) from exc

            for part in released_parts:
                self._allocations[part] = Allocation(
                    provider=resume_name,
                    controller=registration.controllers[part],
                    generation=generation,
                )
            resume_runtime.state = ProviderState.ACTIVE
            self._resume_candidates.pop(provider_name, None)
            self._emit(
                "provider_handover",
                provider=resume_name,
                parts=released_parts,
                controllers=tuple(
                    registration.controllers[part] for part in released_parts
                ),
                generation=generation,
                reason=f"resumed_after:{provider_name}",
            )
            return generation

    def _select_resume_provider(
        self, releasing_provider: str, released_parts: set[str]
    ) -> str | None:
        previous = self._resume_candidates.get(releasing_provider)
        if previous is not None:
            runtime = self._providers.get(previous)
            if (
                runtime is not None
                and runtime.state in (ProviderState.READY, ProviderState.ACTIVE)
                and released_parts <= set(runtime.registration.controllers)
            ):
                return previous
        eligible = [
            (runtime.registration.priority, name)
            for name, runtime in self._providers.items()
            if name != releasing_provider
            and runtime.state in (ProviderState.READY, ProviderState.ACTIVE)
            and released_parts <= set(runtime.registration.controllers)
        ]
        return max(eligible)[1] if eligible else None

    def _provider(self, name: str) -> _ProviderRuntime:
        try:
            return self._providers[name]
        except KeyError as exc:
            raise KeyError(f"provider {name!r} is not registered") from exc

    def _release(self, provider_name: str) -> None:
        self._allocations = {
            part: allocation
            for part, allocation in self._allocations.items()
            if allocation.provider != provider_name
        }

    def _emit(
        self,
        kind: str,
        *,
        provider: str | None = None,
        parts: tuple[str, ...] = (),
        controllers: tuple[str, ...] = (),
        generation: int | None = None,
        reason: str = "",
        correlation_id: str = "",
    ) -> None:
        if not self._event_sinks:
            return
        self._event_sequence += 1
        event = ExecutionEvent(
            sequence=self._event_sequence,
            timestamp_ns=self._clock_ns(),
            kind=kind,
            provider=provider,
            parts=parts,
            controllers=controllers,
            generation=generation,
            reason=reason,
            correlation_id=correlation_id,
        )
        for sink in tuple(self._event_sinks):
            try:
                sink(event)
            except Exception:
                _LOGGER.exception("event sink failed while handling %r", kind)


class LocalExecutionManager:
    """Thread-safe, in-process execution manager for pure client SDK workflows."""

    def __init__(self, profile: Any, node: Any | None = None) -> None:
        import threading

        self.profile = profile
        self._node = node
        self._lock = threading.Lock()
        self._generation = 0
        self._allocations: dict[str, dict[str, Any]] = {}
        self._providers: dict[str, Any] = {}
        self._cm: Any | None = None
        if hasattr(profile, "execution") and profile.execution and "providers" in profile.execution:
            self._providers = profile.execution["providers"]
        if self._node is not None:
            from .controllers import ControllerManagerClient

            manager = "/controller_manager"
            if hasattr(profile, "parts") and profile.parts:
                manager = next(iter(profile.parts.values())).controller_manager
            self._cm = ControllerManagerClient(self._node, manager, timeout_sec=5.0)

    def prepare(self, provider: str) -> None:
        del provider

    def acquire(self, provider: str, next_provider: str = "", reason: str = "") -> int:
        del next_provider, reason
        with self._lock:
            self._generation += 1
            generation = self._generation
            prov_cfg = self._providers.get(provider, {})
            controllers_map = prov_cfg.get("controllers", {})
            parts = list(controllers_map.keys()) if controllers_map else (
                list(self.profile.parts.keys()) if hasattr(self.profile, "parts") else ["arm"]
            )
            activate: list[str] = []
            deactivate: list[str] = []
            if hasattr(self.profile, "parts"):
                for part in parts:
                    contract = controllers_map.get(part)
                    if not contract:
                        continue
                    part_cfg = self.profile.parts.get(part)
                    if not part_cfg or not hasattr(part_cfg, "controllers"):
                        continue
                    target_ctrl = part_cfg.controllers.get(contract)
                    if not target_ctrl:
                        continue
                    if target_ctrl.name not in activate:
                        activate.append(target_ctrl.name)
                    for other_ctrl in part_cfg.controllers.values():
                        if (
                            other_ctrl.name != target_ctrl.name
                            and other_ctrl.name not in deactivate
                        ):
                            deactivate.append(other_ctrl.name)

        self._switch_controllers(activate, deactivate)

        with self._lock:
            for part in parts:
                self._allocations[part] = {
                    "provider": provider,
                    "generation": generation,
                    "controller": controllers_map.get(part, "default"),
                }
            return generation

    def release(self, provider: str, reason: str = "") -> bool:
        del reason
        with self._lock:
            for part, alloc in list(self._allocations.items()):
                if alloc.get("provider") == provider:
                    del self._allocations[part]
            return True

    def stop(self, provider: str, reason: str = "") -> bool:
        return self.release(provider, reason=reason)

    def get_allocations(self) -> dict[str, dict[str, Any]]:
        with self._lock:
            return dict(self._allocations)

    def get_events(self, *, correlation_id: str | None = None) -> list[Any]:
        del correlation_id
        return []

    def wait_for_execution_event(
        self,
        correlation_id: str,
        kinds: Any,
        *,
        timeout_sec: float = 1.0,
    ) -> None:
        del correlation_id, kinds, timeout_sec
        return None

    def ensure_hardware_active(self) -> list[str]:
        """Automatically verify and activate any unconfigured/inactive hardware components."""
        if self._cm is None:
            return []
        try:
            return asyncio.run(self._cm.ensure_hardware_active())
        except Exception:
            return []

    def close(self) -> None:
        pass

    def _switch_controllers(self, activate: list[str], deactivate: list[str]) -> None:
        """STRICT-switch only controllers whose state actually needs to change."""
        if self._cm is None or not activate:
            return

        async def _run() -> None:
            candidates = tuple(dict.fromkeys([*activate, *deactivate]))
            active = set(await self._cm.active_controllers(candidates))
            needed_activate = tuple(name for name in activate if name not in active)
            needed_deactivate = tuple(name for name in deactivate if name in active)
            if not needed_activate and not needed_deactivate:
                return
            await self._cm.switch_controller(
                activate=needed_activate,
                deactivate=needed_deactivate,
                strict=False,
                timeout_sec=2.0,
            )

        try:
            asyncio.get_running_loop()
        except RuntimeError:
            asyncio.run(_run())
            return
        raise RuntimeError(
            "LocalExecutionManager.acquire cannot run inside an asyncio loop"
        )
