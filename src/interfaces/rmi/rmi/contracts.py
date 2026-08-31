"""Application DTOs for RMI observations and commands."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class Action:
    """One native command for one profile-declared robot Part."""

    part: str
    command: str
    value: Any


@dataclass
class PlanPoint:
    """Backend-neutral timed joint waypoint produced outside RMI."""

    positions: list[float]
    velocities: list[float] | None = None
    accelerations: list[float] | None = None
    time_from_start_s: float = 0.0


@dataclass
class PlanResult:
    """Backend-neutral trajectory result consumed by an RMI Node."""

    valid: bool = True
    reason: str = ""
    joint_names: list[str] | None = None
    points: list[PlanPoint] = field(default_factory=list)
    diagnostics: dict[str, Any] = field(default_factory=dict)


@dataclass
class ResolveResult:
    """Backend-neutral IK result produced by an external resolver."""

    valid: bool = True
    reason: str = ""
    joint_names: list[str] | None = None
    positions: list[float] | None = None
    diagnostics: dict[str, Any] = field(default_factory=dict)


@dataclass
class JointHorizonPoint:
    positions: list[float]
    velocities: list[float] | None = None
    time_from_start_s: float = 0.0


@dataclass
class JointHorizonResult:
    """Backend-neutral receding joint horizon from an external provider."""

    valid: bool = True
    reason: str = ""
    points: list[JointHorizonPoint] = field(default_factory=list)
    diagnostics: dict[str, Any] = field(default_factory=dict)


@dataclass
class PoseHorizonPoint:
    position_xyz: list[float]
    orientation_wxyz: list[float]
    time_from_start_s: float = 0.0


@dataclass
class PoseHorizonResult:
    """Backend-neutral Cartesian horizon from an external provider."""

    valid: bool = True
    reason: str = ""
    points: list[PoseHorizonPoint] = field(default_factory=list)
    diagnostics: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class Observation:
    """Timestamped application observation with an authority snapshot.

    ``allocations`` is sampled when this observation is constructed (for
    example on ``robot.state`` / ``get_observation()``), not when the joint
    state message arrived. Lease fencing therefore compares ownership at
    observation-read time against the current control session lease.
    """

    data: Mapping[str, Any]
    source_time_s: float
    receive_time_s: float
    allocations: Mapping[str, Mapping[str, Any]] = field(default_factory=dict)
    sensors: Mapping[str, Any] = field(default_factory=dict)

    @property
    def joint_names(self) -> list[str]:
        return list(self.data.get("joint_names") or [])

    @property
    def joint_positions(self) -> list[float]:
        return list(self.data.get("joint_positions") or [])

    @property
    def joint_velocities(self) -> list[float]:
        return list(self.data.get("joint_velocities") or [])

    def allocation_lease(self, part: str, source_instance: str) -> str | None:
        allocation = self.allocations.get(part)
        if not isinstance(allocation, Mapping):
            return None
        if allocation.get("source_instance") != source_instance:
            return None
        lease_id = allocation.get("lease_id")
        return str(lease_id) if lease_id else None


@dataclass
class ControlDiagnostics:
    """Local output-gate counters; authoritative outcomes remain manager events."""

    sent: int = 0
    inactive_drops: int = 0
    stale_observation_drops: int = 0
    displaced_exits: int = 0
