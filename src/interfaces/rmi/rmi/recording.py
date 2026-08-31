# Copyright 2026 Gabriel-Ning
# SPDX-License-Identifier: Apache-2.0
"""Episode persistence and online RL transition buffers.

``EpisodeRecorder`` is the client for the independent MCAP episode_recorder
service. ``MemoryReplayBuffer`` stores paired observation/action transitions
for a later gym env / RL training loop. They are separate products.
"""

from __future__ import annotations

import asyncio
import logging
import sys
import threading
import time
from collections import deque
from collections.abc import Mapping
from types import TracebackType
from typing import Any, Protocol, Self

_LOGGER = logging.getLogger(__name__)


def _run_sync(awaitable: Any, *, context: str = "synchronous RMI call") -> Any:
    try:
        asyncio.get_running_loop()
    except RuntimeError:
        has_running_loop = False
    else:
        has_running_loop = True
    if not has_running_loop:
        return asyncio.run(awaitable)
    if hasattr(awaitable, "close"):
        awaitable.close()
    raise RuntimeError(f"{context} cannot run inside an asyncio loop")


# =============================================================================
# Mode 1: MCAP Transactional Recorder (Managed ROS 2 C++ Node Backend)
# =============================================================================


class RecorderServiceClient(Protocol):
    """Service-client contract implemented by the episode_recorder SDK."""

    async def activate(self) -> Any: ...

    async def prepare(self) -> Any: ...

    async def wait_ready(self, *, timeout_s: float) -> Any: ...

    async def get_status(self) -> Any: ...

    async def start_recording(
        self, *, task: str, manifest_context: Mapping[str, Any]
    ) -> Any: ...

    async def stop_recording(self, *, timeout_s: float) -> Any: ...

    async def discard(self) -> Any: ...

    async def close(self) -> Any: ...


class EpisodeRecorder:
    """Synchronous client for an active C++ MCAP episode_recorder service."""

    def __init__(
        self,
        recorder_client: RecorderServiceClient,
        *,
        autostart: bool = False,
        node_name: str = "/episode_recorder",
    ) -> None:
        self._client = recorder_client
        self._node_name = node_name
        self._active = False

    def activate(self) -> None:
        if not self._active:
            _run_sync(self._client.activate(), context="synchronous RMI recording")
            _run_sync(self._client.prepare(), context="synchronous RMI recording")
            self._active = True

    def prepare(self) -> None:
        self.activate()

    def wait_ready(self, *, timeout_s: float = 2.0) -> Any:
        self.activate()
        return _run_sync(
            self._client.wait_ready(timeout_s=timeout_s),
            context="synchronous RMI recording",
        )

    def __enter__(self) -> Self:
        self.activate()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        del exc_type, exc, traceback
        self.close()

    def episode(
        self,
        *,
        task: str,
        metadata: Mapping[str, Any] | None = None,
        stop_timeout: float = 300.0,
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
        return _run_sync(self._client.get_status(), context="synchronous RMI recording")

    def discard(self) -> Any:
        self.activate()
        return _run_sync(self._client.discard(), context="synchronous RMI recording")

    def close(self) -> None:
        if self._active:
            _run_sync(self._client.close(), context="synchronous RMI recording")
            self._active = False


class _FinalizeSpinner:
    """Lightweight animated terminal spinner for finalization wait loops."""

    def __init__(self, message: str = "Finalizing episode dataset...") -> None:
        self.message = message
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._start_time = 0.0

    def __enter__(self) -> Self:
        self._start_time = time.time()
        self._stop_event.clear()
        if sys.stdout.isatty():
            self._thread = threading.Thread(target=self._spin, daemon=True)
            self._thread.start()
        else:
            sys.stdout.write(f"\n  [...] {self.message}\n")
            sys.stdout.flush()
        return self

    def _spin(self) -> None:
        spinner_chars = ["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"]
        idx = 0
        while not self._stop_event.is_set():
            elapsed = time.time() - self._start_time
            sys.stdout.write(
                f"\r  {spinner_chars[idx % len(spinner_chars)]} {self.message} ({elapsed:.1f}s)"
            )
            sys.stdout.flush()
            idx += 1
            time.sleep(0.08)

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        del exc_type, exc, traceback
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
        if sys.stdout.isatty():
            sys.stdout.write("\r" + " " * 90 + "\r")
            sys.stdout.flush()


class EpisodeScope:
    """One recording transaction finalized when its context exits."""

    def __init__(
        self,
        recorder: EpisodeRecorder,
        *,
        task: str,
        metadata: Mapping[str, Any],
        stop_timeout: float = 300.0,
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

    @property
    def validated(self) -> bool:
        """Whether recorder finalization and episode validation completed."""
        status = self.final_status
        return bool(
            status is not None
            and not self.discarded
            and getattr(status, "state", None) == "ready"
            and getattr(status, "finalizer_complete", False)
            and getattr(status, "episode_path", "")
        )

    def discard(self) -> Any:
        if not self._entered:
            raise RuntimeError("episode scope is not active")
        if self.discarded:
            return self.final_status
        self.final_status = _run_sync(
            self._recorder._client.discard(),
            context="synchronous RMI recording",
        )
        self.discarded = True
        return self.final_status

    def __enter__(self) -> Self:
        if self._entered:
            raise RuntimeError("episode scope is already entered")
        self._recorder.activate()
        self.started_status = _run_sync(
            self._recorder._client.start_recording(
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
        del traceback
        try:
            if exc_type is not None and not self.discarded:
                is_interrupt = issubclass(exc_type, KeyboardInterrupt)
                if is_interrupt:
                    sys.stdout.write(
                        "\n\n[Interrupted] Ctrl+C received. Safely discarding active episode..."
                    )
                    sys.stdout.flush()
                else:
                    _LOGGER.warning("Exception in recording scope: %s", exc)
                try:
                    self.final_status = _run_sync(
                        self._recorder._client.discard(),
                        context="synchronous RMI recording",
                    )
                    self.discarded = True
                    if is_interrupt:
                        sys.stdout.write(" [✓] Discarded.\n")
                        sys.stdout.flush()
                except Exception:
                    _LOGGER.exception("failed to discard episode after error")
            elif exc_type is None and not self.discarded:
                with _FinalizeSpinner(
                    "Writing MCAP index, SHA-256 checksums & finalizing episode..."
                ):
                    self.final_status = _run_sync(
                        self._recorder._client.stop_recording(
                            timeout_s=self.stop_timeout
                        ),
                        context="synchronous RMI recording",
                    )
        finally:
            self._entered = False


# =============================================================================
# Mode 2: In-Memory Replay Buffer (Online RL / crisp_gym)
# =============================================================================


class MemoryReplayBuffer:
    """In-memory paired (observation, action) buffer for gym env / RL training.

    Distinct from :class:`EpisodeRecorder`, which writes high-quality MCAP
    episodes to disk. This buffer never persists a dataset.
    """

    def __init__(self, capacity: int = 100_000) -> None:
        self.capacity = capacity
        self.buffer: deque[dict[str, Any]] = deque(maxlen=capacity)
        self._current_episode: list[dict[str, Any]] = []

    def episode(
        self,
        *,
        task: str,
        metadata: Mapping[str, Any] | None = None,
        stop_timeout: float = 60.0,
    ) -> MemoryEpisodeScope:
        return MemoryEpisodeScope(self, task=task, metadata=metadata or {})

    def step(
        self,
        observation: Any,
        action: Any,
        reward: float = 0.0,
        next_observation: Any | None = None,
        done: bool = False,
        info: dict[str, Any] | None = None,
    ) -> None:
        """Append one paired observation/action transition."""
        entry = {
            "observation": observation,
            "action": action,
            "reward": reward,
            "next_observation": next_observation,
            "done": done,
            "info": info or {},
            "time": time.time(),
        }
        self.buffer.append(entry)
        self._current_episode.append(entry)

    def sample(self, batch_size: int) -> list[dict[str, Any]]:
        """Randomly sample a batch of transitions for policy updates."""
        import random

        if len(self.buffer) < batch_size:
            raise ValueError(
                f"Not enough samples in buffer ({len(self.buffer)} < {batch_size})"
            )
        return random.sample(list(self.buffer), batch_size)

    def __len__(self) -> int:
        return len(self.buffer)

    @property
    def last_episode(self) -> tuple[dict[str, Any], ...]:
        """Transitions recorded in the most recently closed episode scope."""
        return tuple(self._current_episode)

    def close(self) -> None:
        pass


class MemoryEpisodeScope:
    """Context scope for memory replay recording."""

    def __init__(
        self, buffer: MemoryReplayBuffer, *, task: str, metadata: Mapping[str, Any]
    ) -> None:
        self.buffer = buffer
        self.task = task
        self.metadata = dict(metadata)
        self.start_time = 0.0

    def __enter__(self) -> Self:
        self.start_time = time.time()
        self.buffer._current_episode = []
        return self

    def step(
        self,
        observation: Any,
        action: Any,
        reward: float = 0.0,
        next_observation: Any | None = None,
        done: bool = False,
        info: dict[str, Any] | None = None,
    ) -> None:
        self.buffer.step(
            observation=observation,
            action=action,
            reward=reward,
            next_observation=next_observation,
            done=done,
            info=info,
        )

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        del exc_type, exc, traceback


__all__ = [
    "EpisodeRecorder",
    "EpisodeScope",
    "MemoryEpisodeScope",
    "MemoryReplayBuffer",
    "RecorderServiceClient",
]
