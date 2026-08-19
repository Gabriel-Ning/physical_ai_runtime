"""Pure motion-planning contracts and application wrappers for RMI.

Decoupled, backend-neutral interfaces for planning, inverse kinematics resolution,
and trajectory streaming in RMI applications.
"""

from __future__ import annotations

import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import Any, Protocol

from .contracts import Observation


# ------------------------------------------------------------------------------
# Data Contracts
# ------------------------------------------------------------------------------


@dataclass(frozen=True)
class JointState:
    """Joint-space state or target representation."""

    joint_names: list[str] = field(default_factory=list)
    positions: list[float] = field(default_factory=list)
    velocities: list[float] | None = None
    accelerations: list[float] | None = None
    stamp_s: float = 0.0


@dataclass(frozen=True)
class CartesianState:
    """Cartesian-space state or target representation."""

    position_xyz: tuple[float, float, float] | list[float] = (0.0, 0.0, 0.0)
    orientation_wxyz: tuple[float, float, float, float] | list[float] = (1.0, 0.0, 0.0, 0.0)
    linear_velocity_xyz: tuple[float, float, float] | list[float] | None = None
    angular_velocity_xyz: tuple[float, float, float] | list[float] | None = None
    linear_acceleration_xyz: tuple[float, float, float] | list[float] | None = None
    angular_acceleration_xyz: tuple[float, float, float] | list[float] | None = None
    stamp_s: float = 0.0


Target = CartesianState | JointState


@dataclass
class World:
    """Environment collision and obstacle representation."""

    obstacles: list[Any] = field(default_factory=list)
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class PlanPoint:
    """Single timed waypoint along a joint trajectory."""

    positions: list[float]
    velocities: list[float] | None = None
    accelerations: list[float] | None = None
    time_from_start_s: float = 0.0


@dataclass
class PlanResult:
    """Result of an offline or global motion planning query."""

    valid: bool = True
    reason: str = ""
    joint_names: list[str] | None = None
    points: list[PlanPoint] = field(default_factory=list)
    diagnostics: dict[str, Any] = field(default_factory=dict)


@dataclass
class ResolveResult:
    """Result of a single-query inverse kinematics (IK) resolution."""

    valid: bool = True
    reason: str = ""
    joint_names: list[str] | None = None
    positions: list[float] | None = None
    diagnostics: dict[str, Any] = field(default_factory=dict)


@dataclass
class JointHorizonPoint:
    """Single point in a receding joint horizon."""

    positions: list[float]
    velocities: list[float] | None = None
    time_from_start_s: float = 0.0


@dataclass
class JointHorizonResult:
    """Result of a receding-horizon joint streamer step."""

    valid: bool = True
    reason: str = ""
    points: list[JointHorizonPoint] = field(default_factory=list)
    diagnostics: dict[str, Any] = field(default_factory=dict)


@dataclass
class PoseHorizonPoint:
    """Single point in a receding Cartesian pose horizon."""

    position_xyz: list[float]
    orientation_wxyz: list[float]
    time_from_start_s: float = 0.0


@dataclass
class PoseHorizonResult:
    """Result of a receding-horizon Cartesian streamer step."""

    valid: bool = True
    reason: str = ""
    points: list[PoseHorizonPoint] = field(default_factory=list)
    diagnostics: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class PlannerSpec:
    """Specification and lazy factory for one planner backend."""

    name: str
    factory: Callable[[], Any]
    display_name: str = ""
    warmup_on_create: bool = True


# ------------------------------------------------------------------------------
# Backend Protocols
# ------------------------------------------------------------------------------


class RobotStateSource(Protocol):
    """Small structural boundary implemented by application Robot / Context."""

    @property
    def state(self) -> Observation: ...


class PlannerBackend(Protocol):
    """Interface required of motion planner backend implementations."""

    def update_world(self, world: World | None) -> None: ...

    def plan(
        self,
        start_state: JointState,
        target: Target,
        options: dict[str, Any],
    ) -> PlanResult: ...


class ResolverBackend(Protocol):
    """Interface required of IK resolver backend implementations."""

    def update_world(self, world: World | None) -> None: ...

    def resolve(
        self,
        start_state: JointState,
        target: Target,
        options: dict[str, Any],
    ) -> ResolveResult: ...


class JointStreamerBackend(Protocol):
    """Interface required of receding-horizon joint streamer backends."""

    def update_world(self, world: World | None) -> None: ...

    def update_target(self, target: Target) -> None: ...

    def reset(self, start: JointState) -> None: ...

    def step(self, current: JointState, dt: float) -> JointHorizonResult: ...


class CartesianStreamerBackend(Protocol):
    """Interface required of receding-horizon Cartesian streamer backends."""

    def update_world(self, world: World | None) -> None: ...

    def update_target(self, target: Target) -> None: ...

    def reset(self, start: CartesianState) -> None: ...

    def step(self, current: CartesianState, dt: float) -> PoseHorizonResult: ...


# ------------------------------------------------------------------------------
# Application Wrappers
# ------------------------------------------------------------------------------


class Planner:
    """Application-facing, side-effect-free wrapper around one planner backend."""

    def __init__(
        self,
        name: str,
        backend: PlannerBackend,
        *,
        clock: Callable[[], float] = time.time,
    ) -> None:
        self.name = name
        self._backend = backend
        self._clock = clock

    def update_world(self, world: World | None) -> None:
        self._backend.update_world(world)

    def plan(
        self,
        *,
        robot: RobotStateSource,
        target: Target,
        start: Observation | JointState | None = None,
        options: Mapping[str, Any] | None = None,
        max_state_age_s: float | None = None,
    ) -> PlanResult:
        """Compute a plan without obtaining control authority or executing it."""
        observed = robot.state if start is None else start
        error = _state_error(observed, max_state_age_s, self._clock)
        if error is not None:
            reason, diagnostics = error
            return PlanResult(valid=False, reason=reason, diagnostics=diagnostics)
        try:
            start_state = _to_joint_state(observed)
        except (TypeError, ValueError) as exc:
            return PlanResult(
                valid=False,
                reason=f"invalid robot state: {exc}",
                diagnostics={"validation_stage": "start_state"},
            )
        return self._backend.plan(start_state, target, dict(options or {}))


class Resolver:
    """Pure one-request IK; never sends the setpoint."""

    def __init__(
        self,
        name: str,
        backend: ResolverBackend,
        *,
        clock: Callable[[], float] = time.time,
    ) -> None:
        self.name = name
        self._backend = backend
        self._clock = clock

    def update_world(self, world: World | None) -> None:
        self._backend.update_world(world)

    def resolve(
        self,
        *,
        robot: RobotStateSource,
        target: Target,
        start: Observation | JointState | None = None,
        options: Mapping[str, Any] | None = None,
        max_state_age_s: float | None = None,
    ) -> ResolveResult:
        observed = robot.state if start is None else start
        error = _state_error(observed, max_state_age_s, self._clock)
        if error is not None:
            reason, diagnostics = error
            return ResolveResult(valid=False, reason=reason, diagnostics=diagnostics)
        try:
            start_state = _to_joint_state(observed)
        except (TypeError, ValueError) as exc:
            return ResolveResult(
                valid=False,
                reason=f"invalid robot state: {exc}",
                diagnostics={"validation_stage": "start_state"},
            )
        return self._backend.resolve(start_state, target, dict(options or {}))


class JointStreamer:
    """Pure receding-horizon joint guidance; the application owns the loop."""

    def __init__(
        self,
        name: str,
        backend: JointStreamerBackend,
        *,
        clock: Callable[[], float] = time.time,
    ) -> None:
        self.name = name
        self._backend = backend
        self._clock = clock

    def update_world(self, world: World | None) -> None:
        self._backend.update_world(world)

    def set_target(self, target: Target) -> None:
        self._backend.update_target(target)

    def reset(
        self,
        *,
        robot: RobotStateSource,
        start: Observation | JointState | None = None,
    ) -> None:
        self._backend.reset(_to_joint_state(robot.state if start is None else start))

    def step(
        self,
        *,
        robot: RobotStateSource,
        dt: float,
        current: Observation | JointState | None = None,
        max_state_age_s: float | None = None,
    ) -> JointHorizonResult:
        observed = robot.state if current is None else current
        error = _state_error(observed, max_state_age_s, self._clock)
        if error is not None:
            reason, diagnostics = error
            return JointHorizonResult(
                valid=False, reason=reason, diagnostics=diagnostics
            )
        try:
            state = _to_joint_state(observed)
        except (TypeError, ValueError) as exc:
            return JointHorizonResult(
                valid=False,
                reason=f"invalid robot state: {exc}",
                diagnostics={"validation_stage": "current_state"},
            )
        return self._backend.step(state, dt)


class CartesianStreamer:
    """Pure Cartesian horizon guidance over explicit measured EE poses."""

    def __init__(self, name: str, backend: CartesianStreamerBackend) -> None:
        self.name = name
        self._backend = backend

    def update_world(self, world: World | None) -> None:
        self._backend.update_world(world)

    def set_target(self, target: Target) -> None:
        self._backend.update_target(target)

    def reset(self, start: CartesianState) -> None:
        self._backend.reset(start)

    def step(self, current: CartesianState, *, dt: float) -> PoseHorizonResult:
        return self._backend.step(current, dt)


class PlannerCatalog:
    """Four distinct lazy backend registries owned by one application context."""

    def __init__(self) -> None:
        self._planners = _BackendCatalog()
        self._resolvers = _BackendCatalog()
        self._joint_streamers = _BackendCatalog()
        self._cartesian_streamers = _BackendCatalog()

    def register(
        self,
        name: str,
        factory: Callable[[], Any],
        *,
        display_name: str = "",
        warmup_on_create: bool = True,
    ) -> None:
        self._planners.register(name, factory, display_name, warmup_on_create)

    def make(self, name: str) -> Planner:
        backend = self._planners.get(name)
        return Planner(name.strip().lower(), backend)

    def available(self) -> list[str]:
        return self._planners.available()

    def register_resolver(
        self,
        name: str,
        factory: Callable[[], Any],
        *,
        display_name: str = "",
        warmup_on_create: bool = True,
    ) -> None:
        self._resolvers.register(name, factory, display_name, warmup_on_create)

    def make_resolver(self, name: str) -> Resolver:
        backend = self._resolvers.get(name)
        return Resolver(name.strip().lower(), backend)

    def register_joint_streamer(
        self,
        name: str,
        factory: Callable[[], Any],
        *,
        display_name: str = "",
        warmup_on_create: bool = True,
    ) -> None:
        self._joint_streamers.register(name, factory, display_name, warmup_on_create)

    def make_joint_streamer(self, name: str) -> JointStreamer:
        backend = self._joint_streamers.get(name)
        return JointStreamer(name.strip().lower(), backend)

    def register_cartesian_streamer(
        self,
        name: str,
        factory: Callable[[], Any],
        *,
        display_name: str = "",
        warmup_on_create: bool = True,
    ) -> None:
        self._cartesian_streamers.register(
            name, factory, display_name, warmup_on_create
        )

    def make_cartesian_streamer(self, name: str) -> CartesianStreamer:
        backend = self._cartesian_streamers.get(name)
        return CartesianStreamer(name.strip().lower(), backend)


class _BackendCatalog:
    def __init__(self) -> None:
        self._specs: dict[str, PlannerSpec] = {}
        self._instances: dict[str, Any] = {}

    def register(
        self,
        name: str,
        factory: Callable[[], Any],
        display_name: str = "",
        warmup_on_create: bool = True,
    ) -> None:
        canonical = name.strip().lower()
        self._specs[canonical] = PlannerSpec(
            name=canonical,
            factory=factory,
            display_name=display_name or name,
            warmup_on_create=warmup_on_create,
        )

    def get(self, name: str) -> Any:
        canonical = name.strip().lower()
        if canonical not in self._specs:
            raise KeyError(
                f"planner backend {name!r} not registered; available: {self.available()}"
            )
        if canonical not in self._instances:
            spec = self._specs[canonical]
            instance = spec.factory()
            if spec.warmup_on_create and hasattr(instance, "warmup"):
                instance.warmup()
            self._instances[canonical] = instance
        return self._instances[canonical]

    def available(self) -> list[str]:
        return list(self._specs.keys())


# ------------------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------------------


def _state_error(
    state: Observation | JointState,
    max_state_age_s: float | None,
    clock: Callable[[], float],
) -> tuple[str, dict[str, Any]] | None:
    if max_state_age_s is None or not isinstance(state, Observation):
        return None
    age_s = clock() - state.receive_time_s
    if age_s <= max_state_age_s:
        return None
    return (
        f"robot state is stale: age={age_s:.3f}s exceeds limit={max_state_age_s:.3f}s",
        {"state_age_s": age_s},
    )


def _to_joint_state(state: Observation | JointState) -> JointState:
    if isinstance(state, JointState):
        return state
    data = state.data
    joint_names = _required_sequence(data, "joint_names")
    positions = _required_sequence(data, "joint_positions")
    velocities = _optional_sequence(data, "joint_velocities")
    if len(joint_names) != len(positions):
        raise ValueError("robot observation has mismatched joint names and positions")
    if velocities is not None and len(velocities) != len(positions):
        raise ValueError("robot observation has mismatched positions and velocities")
    return JointState(
        joint_names=[str(name) for name in joint_names],
        positions=[float(value) for value in positions],
        velocities=None if velocities is None else [float(value) for value in velocities],
        stamp_s=state.source_time_s,
    )


def _required_sequence(data: Mapping[str, Any], key: str) -> list[Any]:
    value = data.get(key)
    if value is None:
        raise ValueError(f"robot observation is missing {key!r}")
    return list(value)


def _optional_sequence(data: Mapping[str, Any], key: str) -> list[Any] | None:
    value = data.get(key)
    if value is None:
        return None
    values = list(value)
    return values or None


__all__ = [
    "CartesianState",
    "CartesianStreamer",
    "CartesianStreamerBackend",
    "JointHorizonPoint",
    "JointHorizonResult",
    "JointState",
    "JointStreamer",
    "JointStreamerBackend",
    "PlanPoint",
    "PlanResult",
    "Planner",
    "PlannerBackend",
    "PlannerCatalog",
    "PlannerSpec",
    "PoseHorizonPoint",
    "PoseHorizonResult",
    "ResolveResult",
    "Resolver",
    "ResolverBackend",
    "RobotStateSource",
    "Target",
    "World",
]
