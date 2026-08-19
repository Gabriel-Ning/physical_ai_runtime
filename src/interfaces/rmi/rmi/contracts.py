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


@dataclass(frozen=True)
class Observation:
    """Timestamped application observation with an EM allocation snapshot.

    ``allocations`` is sampled when this observation is constructed (for
    example on ``robot.state`` / ``get_observation()``), not when the joint
    state message arrived. Generation fencing therefore compares ownership at
    observation-read time against the current control session generation.
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

    def allocation_generation(self, part: str, provider: str) -> int | None:
        allocation = self.allocations.get(part)
        if not isinstance(allocation, Mapping):
            return None
        if allocation.get("provider") != provider:
            return None
        generation = allocation.get("generation")
        return int(generation) if generation is not None else None


@dataclass
class ControlDiagnostics:
    """Local output-gate counters; authoritative outcomes remain EM events."""

    sent: int = 0
    inactive_drops: int = 0
    stale_observation_drops: int = 0
    resumes: int = 0
    displaced_exits: int = 0
