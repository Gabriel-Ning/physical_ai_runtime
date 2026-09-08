"""LeRobotPolicy: RMI producer boundary around a pluggable InferenceBackend.

```text
Robot.get_observation()
  -> LeRobotPolicy.select_action(observation)
       -> ObservationEncoder.encode    # RMI  -> LeRobot
       -> InferenceBackend.step        # local Sync/RTC or remote transport
       -> ActionDecoder.decode         # LeRobot -> RMI
  -> Node.submit(action | actions | None)
  -> Execution Manager
```

Does not own Context, Node, leases, TF, or the control loop.
"""

from __future__ import annotations

from typing import Any

import numpy as np
from rmi import Observation, PolicyLayout

from .bridges import ObservationEncoder, make_action_decoder
from .engines import (
    InferenceBackend,
    InferenceMode,
    RemoteBackendConfig,
    make_inference_backend,
    normalize_inference_mode,
)
from .telemetry import ChunkTelemetry

__all__ = ["LeRobotPolicy"]


class LeRobotPolicy:
    """Local or remote LeRobot policy with the standard ``select_action`` boundary."""

    def __init__(
        self,
        layout: PolicyLayout,
        checkpoint: str | None = None,
        *,
        task: str,
        device: str = "cuda",
        revision: str | None = None,
        rename_map: dict[str, str] | None = None,
        expected_policy_type: str | None = None,
        max_stream_skew_s: float = 0.5,
        action_space: str | None = None,
        gripper_max_width: float = 0.045,
        normalize_gripper: bool = False,
        position_scale: float = 0.05,
        orientation_scale: float = 0.5,
        inference: InferenceMode | str = "sync",
        rtc_config: Any | None = None,
        rtc_queue_threshold: int = 30,
        remote_transport: Any | None = None,
        remote_grpc_address: str | None = None,
        remote_config: RemoteBackendConfig | None = None,
        backend: InferenceBackend | None = None,
    ) -> None:
        if not task.strip():
            raise ValueError("task must not be empty")

        from .features import make_dataset_features, make_observation_features

        self.layout = layout
        self.task = task
        self.inference = normalize_inference_mode(inference)

        self.bundle = None
        self.compatibility = None
        self._encoder = ObservationEncoder(
            layout,
            max_stream_skew_s=max_stream_skew_s,
            normalize_gripper=normalize_gripper,
            gripper_max_width=gripper_max_width,
        )
        self._decoder = make_action_decoder(
            layout,
            action_space=action_space,
            gripper_max_width=gripper_max_width,
            position_scale=position_scale,
            orientation_scale=orientation_scale,
            normalize_gripper=normalize_gripper,
        )
        self._dataset_features = make_dataset_features(layout)
        self._observation_features = make_observation_features(layout)
        self._telemetry = ChunkTelemetry()
        self._closed = False

        if backend is not None:
            self._backend = backend
            self.inference = backend.mode
        elif self.inference == "remote":
            self._backend = make_inference_backend(
                inference="remote",
                layout=layout,
                task=task,
                device=device,
                dataset_features=self._dataset_features,
                observation_features=self._observation_features,
                remote_transport=remote_transport,
                remote_grpc_address=remote_grpc_address,
                remote_config=remote_config,
                expected_policy_type=expected_policy_type,
                checkpoint=checkpoint,
                rename_map=rename_map,
            )
        else:
            if not checkpoint:
                raise ValueError("checkpoint is required for local sync/async inference")
            from .loader import load_validated_policy_bundle

            self.bundle, self.compatibility = load_validated_policy_bundle(
                layout,
                checkpoint,
                device=device,
                revision=revision,
                rename_map=rename_map,
                expected_policy_type=expected_policy_type,
            )
            self._backend = make_inference_backend(
                inference=self.inference,
                layout=layout,
                task=task,
                device=device,
                dataset_features=self._dataset_features,
                observation_features=self._observation_features,
                bundle=self.bundle,
                rtc_config=rtc_config,
                rtc_queue_threshold=rtc_queue_threshold,
            )

        self._last_actions = None
        self._backend.start()

    def select_action(self, observation: Any) -> Any:
        """Return native RMI actions for one RMI observation.

        Cartesian layouts read TCP from ``observation.sensors[layout.pose_part]``
        to pack LIBERO EE state and to re-anchor relative actions at chunk
        boundaries. Returning ``None`` is a valid no-submit cycle
        (warm-up / empty remote queue).
        """
        if self._closed:
            raise RuntimeError("policy is closed")
        if self._backend.failed:
            trace = self._backend.failure_traceback or "unknown"
            raise RuntimeError(f"inference backend failed:\n{trace}")

        was_replan = self._backend.detect_replan()
        raw = self._encoder.encode(observation)
        # Remote policy_server expects language task on the raw observation (robot_client).
        if self.inference == "remote":
            task = getattr(self, "task", "")
            if task:
                raw = {**raw, "task": task}
        action = self._backend.step(raw)
        if action is None:
            self._telemetry.record_empty()
            return None

        # Remote refills can replace a nonempty queue during step().
        was_replan = getattr(self._backend, "last_step_replanned", was_replan)
        if self.layout.control_mode == "cartesian" and self._should_reanchor(
            was_replan
        ):
            self._reanchor_from_observation(observation)

        self._telemetry.record(
            np.asarray(action, dtype=np.float64).reshape(-1),
            was_replan=was_replan,
            remaining=self._backend.remaining_actions(),
        )
        self._last_actions = self._decoder.decode(action)
        return self._last_actions

    def set_task(self, task: str) -> None:
        """Dynamically update the active task / instruction string."""
        if not task.strip():
            raise ValueError("task must not be empty")
        self.task = task
        self._backend.set_task(task)

    def pause(self) -> None:
        """Temporarily pause inference."""
        self._backend.pause()

    def resume(self) -> None:
        """Resume inference after a pause."""
        self._backend.resume()

    def update_weights(self, state_dict: dict[str, Any]) -> None:
        """In-flight update of actor policy weights (e.g. from HIL-SAC Learner)."""
        if self._closed:
            raise RuntimeError("policy is closed")
        self._backend.update_weights(state_dict)

    def take_action_chunk(self) -> tuple[Any, ...]:
        """Transfer the last selected intent and native tail into an execution buffer.

        No inference, model reset or new lease is involved. Relative cartesian
        actions are decoded in sequence using the original chunk anchor, not
        re-anchored to the takeover pose. Never call this from another thread.
        """
        from copy import deepcopy

        if self._last_actions is None:
            return ()
        frames = [deepcopy(self._last_actions)]
        for action in self._backend.take_pending_actions():
            frames.append(self._decoder.decode(action))
        self._last_actions = None  # Transfer is one-shot; don't replay the old head.
        return tuple(frames)

    def observe(self, observation: Any) -> None:
        """Feed current state during buffered playback without popping model actions."""
        self._backend.observe(self._encoder.encode(observation))

    def reset(self) -> None:
        if self._closed:
            raise RuntimeError("policy is closed")
        self._last_actions = None
        self._backend.reset()
        self._telemetry.chunk_index = -1
        self._telemetry.clear()

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self._backend.stop()

    @property
    def encoder(self) -> ObservationEncoder:
        return self._encoder

    def encode_observation(self, observation: Observation) -> dict[str, Any]:
        """Encode an RMI observation into the multi-modal dictionary expected by LeRobot."""
        return self._encoder.encode(observation)

    # --- eval/debug passthroughs (see telemetry.py) -------------------------

    @property
    def last_raw_action(self) -> np.ndarray | None:
        return self._telemetry.last_raw_action

    @property
    def last_was_replan(self) -> bool:
        return self._telemetry.last_was_replan

    @property
    def last_chunk_index(self) -> int:
        return self._telemetry.last_chunk_index

    @property
    def last_queue_remaining(self) -> int:
        return self._telemetry.last_queue_remaining

    @property
    def last_chunk_raw(self) -> np.ndarray | None:
        return self._telemetry.last_chunk_raw

    @property
    def last_chunk0_raw(self) -> np.ndarray | None:
        return self._telemetry.last_chunk0_raw

    @property
    def prev_chunk0_raw(self) -> np.ndarray | None:
        return self._telemetry.prev_chunk0_raw

    # --- internals ---------------------------------------------------------

    def _should_reanchor(self, was_replan: bool) -> bool:
        """Re-anchor relative cartesian actions only at chunk boundaries."""
        return was_replan or self._telemetry.chunk_index < 0

    def _reanchor_from_observation(self, observation: Any) -> None:
        pose_part = self.layout.pose_part
        if not pose_part:
            raise RuntimeError("cartesian layout is missing pose_part")
        try:
            sample = observation.sensors[pose_part]
        except KeyError as exc:
            raise RuntimeError(
                f"RMI observation is missing TCP pose sensor {pose_part!r}"
            ) from exc
        pose = sample.value
        self._decoder.set_current_pose(pose.position_xyz, pose.orientation_wxyz)
