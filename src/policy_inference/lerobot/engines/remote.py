"""Remote GPU inference backend (so101 / LeRobot async_inference style).

Architecture target::

    Robot.get_observation()
      -> LeRobotPolicy.select_action(observation)
           -> RemoteBackend.step(raw)   # transport.request_chunk / pop queue
      -> Node.submit(action | None)
      -> Execution Manager

Transport is pluggable (ZMQ / gRPC). See:
https://github.com/legalaspro/so101-ros-physical-ai/tree/main/policy_server
https://github.com/legalaspro/so101-ros-physical-ai/tree/main/so101_inference
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any, Protocol, runtime_checkable

import numpy as np

from .base import InferenceMode


def _apply_rename_map(
    mapping: dict[str, Any], rename_map: dict[str, str]
) -> dict[str, Any]:
    """Rename feature / camera keys for policy_server (runs before server rename).

    Accepts both full keys (``observation.images.pika_d405``) and bare camera
    names (``pika_d405``) produced by ``ObservationEncoder``.
    """
    if not rename_map:
        return dict(mapping)
    aliases: dict[str, str] = {}
    for src, dst in rename_map.items():
        src_bare = src.removeprefix("observation.images.")
        dst_bare = dst.removeprefix("observation.images.")
        aliases[src] = dst
        aliases[src_bare] = dst_bare
        aliases[f"observation.images.{src_bare}"] = f"observation.images.{dst_bare}"
    return {aliases.get(key, key): value for key, value in mapping.items()}


@runtime_checkable
class PolicyTransport(Protocol):
    """Minimal client↔server I/O (so101 ``PolicyTransport`` / lerobot gRPC)."""

    def connect(self) -> None: ...

    def close(self) -> None: ...

    def request_action_chunk(
        self, raw_observation: dict[str, Any]
    ) -> list[np.ndarray] | None:
        """Send one observation; return a chunk ``[(D,), ...]`` or ``None``."""


class LeRobotRemoteTransport:
    """Native LeRobot gRPC client transport connecting to official ``policy_server``.

    Communicates via ``lerobot.transport.services_pb2_grpc.AsyncInferenceStub``,
    streaming pickled ``TimedObservation`` and receiving ``Actions`` containing
    ``TimedAction`` lists.
    """

    def __init__(
        self,
        server_address: str,
        *,
        policy_type: str = "diffusion",
        pretrained_name_or_path: str = "",
        actions_per_chunk: int = 50,
        device: str = "cuda",
        lerobot_features: dict[str, Any] | None = None,
        rename_map: dict[str, str] | None = None,
        # First FastWAM load on the server can take minutes; inference ticks are slower too.
        timeout_s: float = 600.0,
    ) -> None:
        self.server_address = server_address
        self.policy_type = policy_type
        self.pretrained_name_or_path = pretrained_name_or_path
        self.actions_per_chunk = actions_per_chunk
        self.device = device
        self.rename_map = dict(rename_map or {})
        # policy_server looks up policy_image_features BEFORE its preprocessor rename,
        # so features (and later raw obs) must already use checkpoint camera names.
        self.lerobot_features = _apply_rename_map(
            lerobot_features or {}, self.rename_map
        )
        self.timeout_s = timeout_s

        self._channel = None
        self._stub = None
        self._timestep = 0
        self._connected = False

    def connect(self) -> None:
        if self._connected:
            return
        import grpc
        from lerobot.async_inference.helpers import RemotePolicyConfig
        from lerobot.transport import services_pb2, services_pb2_grpc
        from lerobot.transport.utils import grpc_channel_options

        options = grpc_channel_options()
        self._channel = grpc.insecure_channel(self.server_address, options=options)
        self._stub = services_pb2_grpc.AsyncInferenceStub(self._channel)

        # Handshake
        self._stub.Ready(services_pb2.Empty(), timeout=self.timeout_s)

        # Setup policy on server if pretrained info is specified
        if self.pretrained_name_or_path:
            import pickle  # nosec
            policy_config = RemotePolicyConfig(
                self.policy_type,
                self.pretrained_name_or_path,
                self.lerobot_features,
                self.actions_per_chunk,
                self.device,
                rename_map=self.rename_map,
            )
            policy_setup = services_pb2.PolicySetup(data=pickle.dumps(policy_config))
            self._stub.SendPolicyInstructions(policy_setup, timeout=self.timeout_s)

        self._connected = True

    def close(self) -> None:
        self._connected = False
        if self._channel is not None:
            self._channel.close()
            self._channel = None
            self._stub = None

    def request_action_chunk(
        self, raw_observation: dict[str, Any]
    ) -> list[np.ndarray] | None:
        if not self._connected or self._stub is None:
            raise RuntimeError("LeRobotRemoteTransport is not connected")

        import pickle  # nosec
        import time
        from lerobot.async_inference.helpers import TimedObservation
        from lerobot.transport import services_pb2
        from lerobot.transport.utils import send_bytes_in_chunks

        observation = _apply_rename_map(raw_observation, self.rename_map)
        self._timestep += 1
        timed_obs = TimedObservation(
            timestamp=time.time(),
            timestep=self._timestep,
            observation=observation,
            must_go=True,
        )
        obs_bytes = pickle.dumps(timed_obs)
        obs_iter = send_bytes_in_chunks(
            obs_bytes,
            services_pb2.Observation,
            silent=True,
        )
        self._stub.SendObservations(obs_iter, timeout=self.timeout_s)
        actions_msg = self._stub.GetActions(services_pb2.Empty(), timeout=self.timeout_s)

        if not actions_msg or not actions_msg.data:
            return None

        timed_actions = pickle.loads(actions_msg.data)
        chunk: list[np.ndarray] = []
        for item in timed_actions:
            act = item.get_action() if hasattr(item, "get_action") else getattr(item, "action", item)
            if hasattr(act, "detach"):
                act = act.detach()
            if hasattr(act, "cpu"):
                act = act.cpu().numpy()
            chunk.append(np.asarray(act, dtype=np.float64).reshape(-1))
        return chunk

    def set_task(self, task: str) -> None:
        pass


@dataclass
class RemoteBackendConfig:
    """When to ask the remote server for a new chunk."""

    chunk_size_threshold: float = 0.5
    actions_per_chunk: int = 50

    def __post_init__(self) -> None:
        if not 0.0 <= self.chunk_size_threshold <= 1.0:
            raise ValueError("chunk_size_threshold must be in [0, 1]")
        if self.actions_per_chunk < 1:
            raise ValueError("actions_per_chunk must be >= 1")


class RemoteBackend:
    """Client-side chunk buffer over a ``PolicyTransport`` or ``LeRobotRemoteTransport``."""

    mode: InferenceMode = "remote"

    def __init__(
        self,
        transport: PolicyTransport,
        *,
        config: RemoteBackendConfig | None = None,
    ) -> None:
        self._transport = transport
        self._config = config or RemoteBackendConfig()
        self._queue: deque[np.ndarray] = deque()
        self.last_step_replanned = False
        self._started = False
        self._paused = False
        self._failed = False
        self._failure_traceback: str | None = None

    def start(self) -> None:
        self._transport.connect()
        self._started = True
        self._paused = False

    def stop(self) -> None:
        self.last_step_replanned = False
        self._started = False
        self._paused = False
        self._queue.clear()
        self._transport.close()

    def take_pending_actions(self) -> list[Any]:
        result = list(self._queue)
        self._queue.clear()
        return result

    def observe(self, raw_observation: dict[str, Any]) -> None:
        pass  # Remote requests are synchronous and use the next step's observation.

    def reset(self) -> None:
        self.last_step_replanned = False
        self._queue.clear()
        self._failed = False
        self._failure_traceback = None
        if hasattr(self._transport, "reset") and callable(self._transport.reset):
            self._transport.reset()

    def set_task(self, task: str) -> None:
        setter = getattr(self._transport, "set_task", None)
        if callable(setter):
            setter(task)

    def pause(self) -> None:
        self._paused = True
        pause_fn = getattr(self._transport, "pause", None)
        if callable(pause_fn):
            pause_fn()

    def resume(self) -> None:
        self._paused = False
        resume_fn = getattr(self._transport, "resume", None)
        if callable(resume_fn):
            resume_fn()

    def update_weights(self, state_dict: dict[str, Any]) -> None:
        updater = getattr(self._transport, "update_weights", None)
        if callable(updater):
            updater(state_dict)

    @property
    def failed(self) -> bool:
        return self._failed

    @property
    def failure_traceback(self) -> str | None:
        return self._failure_traceback

    def detect_replan(self) -> bool:
        # Empty local queue => next successful pop is the head of a new chunk.
        return not self._queue

    def remaining_actions(self) -> list[Any] | None:
        return list(self._queue)

    def step(self, raw_observation: dict[str, Any]) -> Any | None:
        self.last_step_replanned = False
        if not self._started:
            raise RuntimeError("RemoteBackend is not started")
        if self._paused:
            return None
        try:
            self._maybe_refill(raw_observation)
        except Exception as exc:
            self._failed = True
            self._failure_traceback = str(exc)
            raise
        if not self._queue:
            return None
        return self._queue.popleft()

    def _maybe_refill(self, raw_observation: dict[str, Any]) -> None:
        capacity = max(1, self._config.actions_per_chunk)
        ratio = len(self._queue) / capacity
        if ratio > self._config.chunk_size_threshold and self._queue:
            return
        chunk = self._transport.request_action_chunk(raw_observation)
        if not chunk:
            return
        new_actions: list[np.ndarray] = []
        for action in chunk:
            values = np.asarray(action, dtype=np.float64).reshape(-1)
            if not np.isfinite(values).all():
                raise ValueError("remote action chunk contains NaN or Inf")
            new_actions.append(values)
        # Clear stale unexecuted actions so the robot immediately executes the freshly planned chunk
        self._queue.clear()
        self._queue.extend(new_actions)
        self.last_step_replanned = True

