"""LeRobot Learner gRPC client for HIL-SAC distributed training.

Communicates with ``lerobot.rl.learner.LearnerService`` running on a remote
training workstation / cloud server:
  - Streams transitions (s, a, r, s', done, intervened) to the learner replay buffer.
  - Streams human intervention and teleop events (start/stop, resets).
  - Fetches updated policy parameters (StreamParameters) for in-flight weight hot-reloading.
"""

from __future__ import annotations

import logging
from typing import Any

__all__ = ["LeRobotLearnerClient"]

logger = logging.getLogger(__name__)


class LeRobotLearnerClient:
    """Client for LeRobot's native ``LearnerService`` gRPC server."""

    def __init__(
        self,
        server_address: str,
        *,
        timeout_s: float = 10.0,
    ) -> None:
        self.server_address = server_address
        self.timeout_s = timeout_s

        self._channel = None
        self._stub = None
        self._connected = False

    def connect(self) -> None:
        if self._connected:
            return
        import grpc
        from lerobot.transport import services_pb2, services_pb2_grpc
        from lerobot.transport.utils import grpc_channel_options

        options = grpc_channel_options()
        self._channel = grpc.insecure_channel(self.server_address, options=options)
        self._stub = services_pb2_grpc.LearnerServiceStub(self._channel)

        # Verify server availability
        self._stub.Ready(services_pb2.Empty(), timeout=self.timeout_s)
        self._connected = True
        logger.info("Connected to LeRobot LearnerService at %s", self.server_address)

    def close(self) -> None:
        self._connected = False
        if self._channel is not None:
            self._channel.close()
            self._channel = None
            self._stub = None

    def fetch_policy_parameters(self) -> dict[str, Any] | None:
        """Fetch latest trained policy state_dict from LearnerService."""
        if not self._connected or self._stub is None:
            raise RuntimeError("LearnerClient is not connected")

        from lerobot.transport import services_pb2
        import threading

        import grpc
        from lerobot.transport.utils import bytes_to_state_dict, receive_bytes_in_chunks

        stream = self._stub.StreamParameters(
            services_pb2.Empty(), timeout=self.timeout_s
        )
        try:
            payload = receive_bytes_in_chunks(stream, None, threading.Event())
            return bytes_to_state_dict(payload) if payload else None
        except grpc.RpcError as exc:
            if exc.code() == grpc.StatusCode.DEADLINE_EXCEEDED:
                return None
            raise
        finally:
            # The server keeps streaming updates; fetch one complete update per call.
            stream.cancel()

    def send_transitions(self, transitions: list[Any]) -> None:
        """Stream a batch of transitions to Learner's ReplayBuffer."""
        if not self._connected or self._stub is None:
            raise RuntimeError("LearnerClient is not connected")
        if not transitions:
            return

        from lerobot.transport import services_pb2
        from lerobot.transport.utils import send_bytes_in_chunks, transitions_to_bytes

        trans_bytes = transitions_to_bytes(transitions)
        chunk_iter = send_bytes_in_chunks(
            trans_bytes,
            services_pb2.Transition,
            silent=True,
        )
        self._stub.SendTransitions(chunk_iter, timeout=self.timeout_s)

    def send_interactions(self, interaction_data: dict[str, Any]) -> None:
        """Stream human interaction / teleop / reward events to LearnerService."""
        if not self._connected or self._stub is None:
            raise RuntimeError("LearnerClient is not connected")

        from lerobot.transport import services_pb2
        from lerobot.transport.utils import python_object_to_bytes, send_bytes_in_chunks

        msg_bytes = python_object_to_bytes(interaction_data)
        chunk_iter = send_bytes_in_chunks(
            msg_bytes,
            services_pb2.InteractionMessage,
            silent=True,
        )
        self._stub.SendInteractions(chunk_iter, timeout=self.timeout_s)
