"""``RunPolicy``: the canonical RMI-backed policy-execution leaf.

See docs/BEHAVIOR_TREE_ORCHESTRATOR_DESIGN.md §3.1 for the reference
pseudocode this mirrors.
"""

from __future__ import annotations

from typing import Any

from ..tree import Node, Status


class RunPolicy(Node):
    """Runs one RMI policy control scope until it reports ``done``/``uncertain``.

    ``policy`` resolves an inference callable. ``source`` resolves the distinct
    RMI ``ActionSource`` used by ``robot.control()``; it defaults to the policy
    name because deployments commonly use the same logical name for both.
    """

    def __init__(
        self,
        name: str | None = None,
        *,
        policy: str,
        source: str | None = None,
        resume: bool = True,
        recovery_target_key: str = "recovery_target",
        **config: Any,
    ) -> None:
        super().__init__(name, **config)
        self._policy_name = policy
        self._source_name = source or policy
        self.resume = resume
        self.recovery_target_key = recovery_target_key
        self.policy: Any = None
        self.source: Any = None
        self.control: Any = None

    def setup(self) -> None:
        assert self.context is not None
        self.policy = self.context.policy(self._policy_name)
        self.source = self.context.source(self._source_name)

    def observe(self) -> Any:
        return self.robot.get_observation()

    def initialise(self) -> None:
        self.control = self.robot.control(self.source, resume=self.resume)
        self.control.__enter__()

    def update(self) -> Status:
        obs = self.observe()
        result = self.policy(obs)

        if result.done:
            return Status.SUCCESS

        if result.uncertain:
            self.blackboard.set(self.recovery_target_key, result.recovery_target)
            return Status.FAILURE

        self.control.send(result.action, observation=obs)
        return Status.RUNNING

    def terminate(self, status: Status) -> None:
        if self.control is not None:
            self.control.__exit__(None, None, None)
            self.control = None
