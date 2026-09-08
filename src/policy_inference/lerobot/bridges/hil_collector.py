"""HIL (Human-In-The-Loop) Transition Collector for RL and SAC training.

Collects online transition tuples (s_t, a_t, r_t, s_{t+1}, done, info)
where a_t must be the actual executed action (human demonstrator action or
autonomous policy action) and s_t is the complete observation dictionary
(cameras + robot state) matching the training policy's feature schema.

Note on Architecture:
  In the Physical AI Runtime, authoritative multi-modal recording of both
  human teleoperation and autonomous policy actions alongside Execution Manager (EM)
  lease preemption events is handled by `episode_recorder` (C++ MCAP recorder daemon).
  `HILTransitionCollector` serves as an in-process bridge to buffer and stream
  transitions directly to LeRobot's `LearnerService` for online RL / HIL-SAC.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

__all__ = ["HILTransition", "HILTransitionCollector"]


@dataclass
class HILTransition:
    """Standardized single transition for RL training."""

    state: dict[str, Any]
    action: np.ndarray
    reward: float
    next_state: dict[str, Any]
    done: bool
    intervened: bool
    info: dict[str, Any] = field(default_factory=dict)

    def to_lerobot_dict(self) -> dict[str, Any]:
        """Convert into LeRobot RL Transition dictionary format."""
        return {
            "observation": self.state,
            "action": self.action,
            "reward": float(self.reward),
            "next_observation": self.next_state,
            "done": bool(self.done),
            "intervened": bool(self.intervened),
            "info": self.info,
        }


class HILTransitionCollector:
    """Buffered transition collector for HIL-SAC workflows."""

    def __init__(self, *, max_buffer_size: int = 1000) -> None:
        self.max_buffer_size = max_buffer_size
        self._buffer: list[HILTransition] = []
        self._last_state: dict[str, Any] | None = None

    def start_episode(self, initial_state: dict[str, Any]) -> None:
        """Mark episode start with the initial observation."""
        self._last_state = initial_state

    def end_episode(self) -> None:
        """Mark episode termination or interruption without adding a step."""
        self._last_state = None

    def record_step(
        self,
        executed_action: Any,
        reward: float,
        next_state: dict[str, Any],
        done: bool,
        *,
        is_intervened: bool = False,
        info: dict[str, Any] | None = None,
    ) -> HILTransition | None:
        """Record one step after action execution."""
        if self._last_state is None:
            self._last_state = next_state
            return None

        action_vec = np.asarray(executed_action, dtype=np.float32).reshape(-1)
        transition = HILTransition(
            state=self._last_state,
            action=action_vec,
            reward=float(reward),
            next_state=next_state,
            done=bool(done),
            intervened=bool(is_intervened),
            info=info or {},
        )
        self._buffer.append(transition)
        if len(self._buffer) > self.max_buffer_size:
            self._buffer.pop(0)

        self._last_state = None if done else next_state
        return transition

    def flush(self) -> list[dict[str, Any]]:
        """Flush and return all collected transitions formatted for LeRobot."""
        transitions = [t.to_lerobot_dict() for t in self._buffer]
        self._buffer.clear()
        return transitions

    @property
    def pending_count(self) -> int:
        return len(self._buffer)
