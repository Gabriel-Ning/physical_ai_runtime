# Copyright 2026 Gabriel-Ning
# SPDX-License-Identifier: Apache-2.0
"""Episode recorder wrappers for offline MCAP persistence and online RL replay."""

from __future__ import annotations

import asyncio
import logging
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from collections.abc import Mapping
from pathlib import Path
from types import TracebackType
from typing import Any, Self

_LOGGER = logging.getLogger(__name__)


def _run_sync(awaitable: Any, *, context: str = "synchronous RMI call") -> Any:
    try:
        asyncio.get_running_loop()
    except RuntimeError:
        return asyncio.run(awaitable)
    if hasattr(awaitable, "close"):
        awaitable.close()
    raise RuntimeError(f"{context} cannot run inside an asyncio loop")


# =============================================================================
# Mode 1: MCAP Transactional Recorder (Managed ROS 2 C++ Node Backend)
# =============================================================================

class ManagedRosRecorder:
    """Manages the lifecycle of an offline C++ MCAP episode_recorder node."""

    def __init__(
        self,
        recorder_backend: Any,
        *,
        autostart: bool = True,
        stream_config_uri: str | Path | None = None,
        node_name: str = "/episode_recorder",
    ) -> None:
        self._backend = recorder_backend
        self._autostart = autostart
        self._stream_config_uri = str(stream_config_uri) if stream_config_uri else None
        self._node_name = node_name
        self._subprocess: subprocess.Popen[bytes] | None = None
        self._active = False

    def _ensure_service_running(self) -> None:
        if not self._autostart:
            return

        client = getattr(self._backend, "_get_state", None)
        if client is None and hasattr(self._backend, "_backend"):
            client = getattr(self._backend._backend, "_get_state", None)

        if client is not None and client.service_is_ready():
            return

        if self._stream_config_uri:
            workspace_root = Path(__file__).resolve().parents[4]
            setup_script = workspace_root / "install" / "setup.bash"
            episodes_dir = workspace_root / "data" / "episodes"
            episodes_dir.mkdir(parents=True, exist_ok=True)

            domain_id = os.environ.get("ROS_DOMAIN_ID", "2")
            if setup_script.is_file():
                cmd = f"source '{setup_script}' && export ROS_DOMAIN_ID={domain_id} && exec ros2 launch episode_recorder recorder.launch.py stream_config_uri:='{self._stream_config_uri}' root_dir:='{episodes_dir}' experiment_name:='fr3_pika_policy'"
            else:
                cmd = f"export ROS_DOMAIN_ID={domain_id} && exec ros2 launch episode_recorder recorder.launch.py stream_config_uri:='{self._stream_config_uri}' root_dir:='{episodes_dir}' experiment_name:='fr3_pika_policy'"

            _LOGGER.info("Autostarting episode_recorder launch backend: %s", cmd)
            try:
                self._subprocess = subprocess.Popen(
                    cmd,
                    shell=True,
                    executable="/bin/bash",
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid,
                )
                if client is not None:
                    # Wait up to 8.0s for the ROS 2 lifecycle service to become active
                    for _ in range(80):
                        if client.wait_for_service(timeout_sec=0.1):
                            time.sleep(0.5)
                            break
                        time.sleep(0.1)
            except Exception as e:
                _LOGGER.warning("Could not autostart episode_recorder process: %s", e)

    def activate(self) -> None:
        if not self._active:
            self._ensure_service_running()
            _run_sync(self._backend.activate(), context="synchronous RMI recording")
            self._active = True

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
        return _run_sync(
            self._backend.get_status(), context="synchronous RMI recording"
        )

    def discard(self) -> Any:
        self.activate()
        return _run_sync(self._backend.discard(), context="synchronous RMI recording")

    def close(self) -> None:
        if self._subprocess is not None:
            try:
                os.killpg(os.getpgid(self._subprocess.pid), signal.SIGINT)
                self._subprocess.wait(timeout=2.0)
            except Exception:
                pass
            self._subprocess = None


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
        recorder: ManagedRosRecorder,
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

    def discard(self) -> Any:
        if not self._entered:
            raise RuntimeError("episode scope is not active")
        if self.discarded:
            return self.final_status
        self.final_status = _run_sync(
            self._recorder._backend.discard(),
            context="synchronous RMI recording",
        )
        self.discarded = True
        return self.final_status

    def __enter__(self) -> Self:
        if self._entered:
            raise RuntimeError("episode scope is already entered")
        self._recorder.activate()
        self.started_status = _run_sync(
            self._recorder._backend.start_recording(
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
                    sys.stdout.write("\n\n[Interrupted] Ctrl+C received. Safely discarding active episode...")
                    sys.stdout.flush()
                else:
                    _LOGGER.warning("Exception in recording scope: %s", exc)
                try:
                    self.final_status = _run_sync(
                        self._recorder._backend.discard(),
                        context="synchronous RMI recording",
                    )
                    self.discarded = True
                    if is_interrupt:
                        sys.stdout.write(" [✓] Discarded.\n")
                        sys.stdout.flush()
                except Exception:
                    _LOGGER.exception("failed to discard episode after error")
            elif exc_type is None and not self.discarded:
                with _FinalizeSpinner("Writing MCAP index, SHA-256 checksums & finalizing episode..."):
                    self.final_status = _run_sync(
                        self._recorder._backend.stop_recording(
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
    """In-memory experience replay ring buffer for online RL training."""

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

    def add(
        self,
        observation: Any,
        action: Any,
        reward: float = 0.0,
        next_observation: Any | None = None,
        done: bool = False,
        info: dict[str, Any] | None = None,
    ) -> None:
        """Add a single transition tuple to the active replay buffer."""
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

    def step(
        self,
        observation: Any,
        action: Any,
        reward: float = 0.0,
        next_observation: Any | None = None,
        done: bool = False,
        info: dict[str, Any] | None = None,
    ) -> None:
        """Convenience alias for add() matching Gym step contract."""
        self.add(
            observation=observation,
            action=action,
            reward=reward,
            next_observation=next_observation,
            done=done,
            info=info,
        )

    def sample(self, batch_size: int) -> list[dict[str, Any]]:
        """Randomly sample a batch of transitions for policy updates."""
        import random
        if len(self.buffer) < batch_size:
            raise ValueError(f"Not enough samples in buffer ({len(self.buffer)} < {batch_size})")
        return random.sample(list(self.buffer), batch_size)

    def __len__(self) -> int:
        return len(self.buffer)

    def close(self) -> None:
        pass


class MemoryEpisodeScope:
    """Context scope for memory replay recording."""

    def __init__(self, buffer: MemoryReplayBuffer, *, task: str, metadata: Mapping[str, Any]) -> None:
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
        self.buffer.add(
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


# Backward compatible aliases
Recorder = ManagedRosRecorder

__all__ = [
    "EpisodeScope",
    "ManagedRosRecorder",
    "MemoryEpisodeScope",
    "MemoryReplayBuffer",
    "Recorder",
]
