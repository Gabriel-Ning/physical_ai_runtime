"""Unified inference backend contract for LeRobotPolicy.

Local Sync / RTC and remote GPU servers (so101-style ZMQ/gRPC) all implement
this small surface so ``select_action`` never branches on transport.
"""

from __future__ import annotations

from typing import Any, Literal, Protocol, runtime_checkable

InferenceMode = Literal["sync", "async", "remote"]


def normalize_inference_mode(mode: str) -> InferenceMode:
    key = mode.strip().lower()
    if key in {"sync"}:
        return "sync"
    if key in {"async", "rtc"}:
        return "async"
    if key in {"remote"}:
        return "remote"
    raise ValueError(
        f"inference mode must be sync, async/rtc, or remote, got {mode!r}"
    )


@runtime_checkable
class InferenceBackend(Protocol):
    """One-step action producer used by ``LeRobotPolicy.select_action``."""

    mode: InferenceMode

    def start(self) -> None: ...

    def stop(self) -> None: ...

    def reset(self) -> None: ...

    def take_pending_actions(self) -> list[Any]:
        """Transfer the postprocessed queue tail to the execution chunk buffer."""

    def observe(self, raw_observation: dict[str, Any]) -> None:
        """Feed a running async engine without advancing its action cursor."""

    def step(self, raw_observation: dict[str, Any]) -> Any | None:
        """Consume one hardware observation dict; return one action or ``None``."""

    def detect_replan(self) -> bool:
        """True when the next successful step starts a new action chunk."""

    def remaining_actions(self) -> list[Any] | None:
        """Post-step leftover chunk actions for debug (may be empty)."""

    def set_task(self, task: str) -> None:
        """Dynamically update the active task / instruction string."""

    def pause(self) -> None:
        """Temporarily pause inference (e.g. during robot reset / pause)."""

    def resume(self) -> None:
        """Resume inference after a pause."""

    def update_weights(self, state_dict: dict[str, Any]) -> None:
        """In-flight hot update of policy parameters (e.g. from HIL-SAC Learner)."""

    @property
    def failed(self) -> bool: ...

    @property
    def failure_traceback(self) -> str | None: ...

