"""Plain fakes standing in for RMI objects in Orchestrator unit tests.

None of these talk to ROS/rclpy; they only implement the small surface the
BT leaf nodes call, matching docs/BEHAVIOR_TREE_ORCHESTRATOR_DESIGN.md §3.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from types import TracebackType
from typing import Any, Self


@dataclass
class FakeObservation:
    allocations: dict[str, dict[str, Any]] = field(default_factory=dict)


class FakeControlSession:
    """Stands in for ``rmi.ControlSession``."""

    def __init__(self, source: Any, *, resume: bool = True) -> None:
        self.source = source
        self.resume = resume
        self.entered = False
        self.exited = False
        self.sent: list[tuple[Any, Any]] = []

    def __enter__(self) -> Self:
        self.entered = True
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.exited = True

    def send(self, action: Any, *, observation: Any = None) -> None:
        self.sent.append((action, observation))


class FakePlanExecution:
    def __init__(self, result: Any) -> None:
        self._result = result

    def wait(self, timeout: float = 10.0) -> Any:
        return self._result


class FakeRobot:
    """Stands in for ``rmi.RobotFacade``."""

    def __init__(self, observation: FakeObservation | None = None) -> None:
        self.observation = observation or FakeObservation()
        self.control_sessions: list[FakeControlSession] = []
        self.executed: list[tuple[str, Any]] = []
        self.execute_result: Any = True

    def get_observation(self) -> FakeObservation:
        return self.observation

    def control(self, source: Any, *, resume: bool = True, **_: Any) -> FakeControlSession:
        session = FakeControlSession(source, resume=resume)
        self.control_sessions.append(session)
        return session

    def execute(self, part: str, plan: Any) -> FakePlanExecution:
        self.executed.append((part, plan))
        return FakePlanExecution(self.execute_result)


@dataclass
class FakePolicyResult:
    action: Any = None
    done: bool = False
    uncertain: bool = False
    recovery_target: Any = None


class FakePolicy:
    """Stands in for a policy inference callable."""

    def __init__(self, name: str, results: list[FakePolicyResult]) -> None:
        self.name = name
        self.parts = ("arm",)
        self._results = list(results)
        self._index = 0
        self.calls = 0

    def __call__(self, observation: Any) -> FakePolicyResult:
        self.calls += 1
        result = self._results[min(self._index, len(self._results) - 1)]
        self._index += 1
        return result


@dataclass(frozen=True)
class FakeSource:
    """Stands in for a distinct RMI ``ActionSource`` identity."""

    name: str
    parts: tuple[str, ...] = ("arm",)


@dataclass
class FakePlan:
    valid: bool = True


class FakePlanner:
    def __init__(self, plan: FakePlan | None = None) -> None:
        self.plan_result = plan or FakePlan(valid=True)
        self.calls: list[tuple[Any, Any]] = []

    def plan(self, *, robot: Any, target: Any) -> FakePlan:
        self.calls.append((robot, target))
        return self.plan_result


class FakeEpisodeScope:
    def __init__(self, **kwargs: Any) -> None:
        self.kwargs = kwargs
        self.entered = False
        self.exited = False

    def __enter__(self) -> Self:
        self.entered = True
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.exited = True


class FakeRecorder:
    """Stands in for ``rmi.RecorderFacade``."""

    def __init__(self) -> None:
        self.episodes: list[FakeEpisodeScope] = []

    def episode(self, **kwargs: Any) -> FakeEpisodeScope:
        scope = FakeEpisodeScope(**kwargs)
        self.episodes.append(scope)
        return scope
