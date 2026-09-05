"""``PolicySkill``: observes RMI arm ownership; never drives handover itself.

Teleop is an independent ``ActionSource``/process and EM alone arbitrates
high-priority preemption. This node only reads the ownership fact RMI
already exposes on ``Observation.allocations`` — see
docs/BEHAVIOR_TREE_ORCHESTRATOR_DESIGN.md §3.4.
"""

from __future__ import annotations

from typing import Any

from ..tree import Node, Status


class PolicySkill(Node):
    """RUNNING while either the policy or teleop owns ``part``; FAILURE if neither does."""

    def __init__(
        self,
        name: str | None = None,
        *,
        policy: str,
        teleop: str = "Teleop",
        part: str = "arm",
        intervened_key: str = "intervened",
        **config: Any,
    ) -> None:
        super().__init__(name, **config)
        self._policy_name = policy
        self.teleop_name = teleop
        self.part = part
        self.intervened_key = intervened_key

    def update(self) -> Status:
        obs = self.robot.get_observation()
        allocation = obs.allocations.get(self.part) if obs.allocations else None
        owner = allocation.get("provider") if allocation else None

        if owner == self.teleop_name:
            self.blackboard.set(self.intervened_key, True)
            return Status.RUNNING
        if owner == self._policy_name:
            self.blackboard.set(self.intervened_key, False)
            return Status.RUNNING
        return Status.FAILURE
