"""``ExecuteRecoveryPlan``: one-shot planner-executed recovery motion."""

from __future__ import annotations

from typing import Any

from ..tree import Node, Status


class ExecuteRecoveryPlan(Node):
    """Plans to ``blackboard.<target_key>`` and executes it under a control scope.

    ``planner`` and ``planner_source`` are names resolved from the tree's
    ``NodeContext`` — the former a pure motion planner (``.plan(robot, target)``
    returning something with ``.valid``), the latter the RMI ``ActionSource``
    identity used to acquire control while the plan executes.
    """

    def __init__(
        self,
        name: str | None = None,
        *,
        planner: str,
        planner_source: str,
        part: str = "arm",
        target_key: str = "recovery_target",
        **config: Any,
    ) -> None:
        super().__init__(name, **config)
        self._planner_name = planner
        self._planner_source_name = planner_source
        self.part = part
        self.target_key = target_key
        self.planner: Any = None
        self.planner_source: Any = None

    def setup(self) -> None:
        assert self.context is not None
        self.planner = self.context.planner(self._planner_name)
        self.planner_source = self.context.source(self._planner_source_name)

    def update(self) -> Status:
        if not self.blackboard.has(self.target_key):
            return Status.FAILURE
        target = self.blackboard.get(self.target_key)

        plan = self.planner.plan(robot=self.robot, target=target)
        if not plan.valid:
            return Status.FAILURE

        with self.robot.control(self.planner_source):
            result = self.robot.execute(self.part, plan).wait()

        return Status.SUCCESS if result else Status.FAILURE
