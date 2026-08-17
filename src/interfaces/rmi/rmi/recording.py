"""Episode recorder wrapper over the episode_recorder SDK."""

from __future__ import annotations

import logging
from collections.abc import Mapping
from types import TracebackType
from typing import Any, Self

_LOGGER = logging.getLogger(__name__)


def _run_sync(awaitable: Any, *, context: str = "synchronous RMI call") -> Any:
    import asyncio

    try:
        asyncio.get_running_loop()
    except RuntimeError:
        return asyncio.run(awaitable)
    if hasattr(awaitable, "close"):
        awaitable.close()
    raise RuntimeError(f"{context} cannot run inside an asyncio loop")


class Recorder:
    """Expose recorder lifecycle without asyncio in the application loop."""

    def __init__(self, recorder: Any) -> None:
        self._recorder = recorder
        self._active = False

    def activate(self) -> None:
        if not self._active:
            _run_sync(self._recorder.activate(), context="synchronous RMI recording")
            self._active = True

    def episode(
        self,
        *,
        task: str,
        metadata: Mapping[str, Any] | None = None,
        stop_timeout: float = 60.0,
    ) -> EpisodeScope:
        return EpisodeScope(
            self,
            task=task,
            metadata=metadata or {},
            stop_timeout=stop_timeout,
        )

    @property
    def status(self) -> Any:
        self.activate()
        return _run_sync(
            self._recorder.get_status(), context="synchronous RMI recording"
        )

    def discard(self) -> Any:
        self.activate()
        return _run_sync(self._recorder.discard(), context="synchronous RMI recording")


class EpisodeScope:
    """One recording transaction finalized when its context exits.

    The episode owns recording lifecycle only.  Agent Sessions remain
    responsible for observation, action, and control-loop timing.
    """

    def __init__(
        self,
        recorder: Recorder,
        *,
        task: str,
        metadata: Mapping[str, Any],
        stop_timeout: float,
    ) -> None:
        if not task:
            raise ValueError("task must not be empty")
        if stop_timeout <= 0.0:
            raise ValueError("stop_timeout must be positive")
        self._recorder = recorder
        self.task = task
        self.metadata = dict(metadata)
        self.stop_timeout = stop_timeout
        self.started_status: Any | None = None
        self.final_status: Any | None = None
        self._entered = False
        self.discarded = False

    def discard(self) -> Any:
        """Discard this episode instead of finalizing it on context exit."""
        if not self._entered:
            raise RuntimeError("episode scope is not active")
        if self.discarded:
            return self.final_status
        self.final_status = _run_sync(
            self._recorder._recorder.discard(),
            context="synchronous RMI recording",
        )
        self.discarded = True
        return self.final_status

    def __enter__(self) -> Self:
        if self._entered:
            raise RuntimeError("episode scope is already entered")
        self._recorder.activate()
        self.started_status = _run_sync(
            self._recorder._recorder.start_recording(
                task=self.task,
                manifest_context=self.metadata,
            ),
            context="synchronous RMI recording",
        )
        self._entered = True
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        del exc, traceback
        try:
            if exc_type is not None and not self.discarded:
                # The application body failed: discard so the episode is not
                # persisted as normal data. Never mask the original exception.
                try:
                    self.final_status = _run_sync(
                        self._recorder._recorder.discard(),
                        context="synchronous RMI recording",
                    )
                    self.discarded = True
                except Exception:  # noqa: BLE001
                    _LOGGER.exception(
                        "failed to discard episode after application error"
                    )
            elif exc_type is None and not self.discarded:
                self.final_status = _run_sync(
                    self._recorder._recorder.stop_recording(
                        timeout_s=self.stop_timeout
                    ),
                    context="synchronous RMI recording",
                )
        finally:
            self._entered = False


__all__ = ["EpisodeScope", "Recorder"]
