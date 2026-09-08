"""Local LeRobot SyncInferenceEngine / RTCInferenceEngine adapters."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from rmi import PolicyLayout

from ..loader import LeRobotPolicyBundle, supports_native_rtc
from .base import InferenceBackend, InferenceMode


@dataclass(frozen=True)
class _RmiRobotStub:
    """Stand-in for LeRobot ThreadSafeRobot (RTC only reads metadata)."""

    robot_type: str
    action_features: dict[str, type]


class _LocalBackendBase:
    mode: InferenceMode
    _native: Any
    _policy: Any | None = None

    def start(self) -> None:
        self._native.start()

    def stop(self) -> None:
        pause = getattr(self._native, "pause", None)
        if callable(pause):
            pause()
        self._native.stop()

    def reset(self) -> None:
        self._native.reset()

    def set_task(self, task: str) -> None:
        setter = getattr(self._native, "set_task", None)
        if callable(setter):
            setter(task)

    def pause(self) -> None:
        pause_fn = getattr(self._native, "pause", None)
        if callable(pause_fn):
            pause_fn()

    def resume(self) -> None:
        resume_fn = getattr(self._native, "resume", None)
        if callable(resume_fn):
            resume_fn()

    def update_weights(self, state_dict: dict[str, Any]) -> None:
        policy = self._policy or getattr(self._native, "policy", None)
        if policy is not None and hasattr(policy, "load_state_dict"):
            policy.load_state_dict(state_dict, strict=False)
        else:
            raise RuntimeError("Underlying policy does not support load_state_dict")

    @property
    def failed(self) -> bool:
        return bool(getattr(self._native, "failed", False))

    @property
    def failure_traceback(self) -> str | None:
        return getattr(self._native, "failure_traceback", None)

    @property
    def native(self) -> Any:
        return self._native


class LocalSyncBackend(_LocalBackendBase):
    """Inline SyncInferenceEngine: build frame then ``get_action``."""

    mode: InferenceMode = "sync"

    def __init__(
        self,
        native: Any,
        *,
        dataset_features: dict[str, dict],
        action_key: str,
        policy: Any,
    ) -> None:
        self._native = native
        self._dataset_features = dataset_features
        self._action_key = action_key
        self._policy = policy

    def _action_queue(self) -> Any | None:
        """Resolve the policy's action queue, or ``None`` when it keeps none.

        LeRobot has two chunk-queue conventions and declares both in
        ``PreTrainedPolicy._action_queue_attrs``: a ``populate_queues`` dict
        (diffusion, smolvla, pi0, vla_jepa) and a bare deque (act, groot, eo1).
        Follow that ClassVar so a policy that extends it stays supported.
        """
        attrs = getattr(self._policy, "_action_queue_attrs", ("_queues",))
        for attr in attrs:
            queue = getattr(self._policy, attr, None)
            if isinstance(queue, dict):
                queue = queue.get(self._action_key)
            if queue is not None:
                return queue
        return None

    def take_pending_actions(self) -> list[Any]:
        import torch
        from lerobot.policies.utils import make_robot_action

        queue = self._action_queue()
        result = []
        if queue is None:
            return result
        # Exactly the processing/order used by SyncInferenceEngine.get_action.
        # Consume, rather than copy and later replay, the native action cursor.
        with torch.inference_mode():
            while queue:
                action = self._native._postprocessor(queue.popleft()).squeeze(0).cpu()
                mapped = make_robot_action(action, self._native._dataset_features)
                result.append([mapped[key] for key in self._native._ordered_action_keys])
        return result

    def observe(self, raw_observation: dict[str, Any]) -> None:
        pass  # Inline inference receives the current observation on the next step.

    def detect_replan(self) -> bool:
        queue = self._action_queue()
        # No queue at all (ACT temporal ensembling, non-chunking policies) means
        # every tick runs the model, so every tick is the head of a new chunk.
        return queue is None or len(queue) == 0

    def remaining_actions(self) -> list[Any] | None:
        queue = self._action_queue()
        return None if queue is None else list(queue)

    def step(self, raw_observation: dict[str, Any]) -> Any | None:
        from lerobot.utils.constants import OBS_STR
        from lerobot.utils.feature_utils import build_dataset_frame

        frame = build_dataset_frame(
            self._dataset_features, raw_observation, prefix=OBS_STR
        )
        return self._native.get_action(frame)


class LocalAsyncBackend(_LocalBackendBase):
    """RTCInferenceEngine: ``notify_observation`` then ``get_action``."""

    mode: InferenceMode = "async"

    def __init__(self, native: Any, *, policy: Any | None = None) -> None:
        self._native = native
        self._policy = policy

    def start(self) -> None:
        self._native.start()
        self._native.resume()

    def take_pending_actions(self) -> list[Any]:
        queue = self._native.action_queue
        # Atomic transfer against RTC merge; keep the index monotonic so an
        # in-flight refill accounts for the tail already owned by the executor.
        with queue.lock:
            if queue.queue is None:
                return []
            result = list(queue.queue[queue.last_index:].clone())
            queue.last_index = len(queue.queue)
            return result

    def observe(self, raw_observation: dict[str, Any]) -> None:
        self._native.notify_observation(raw_observation)

    def detect_replan(self) -> bool:
        queue = getattr(self._native, "action_queue", None)
        if queue is None:
            return False
        return queue.get_action_index() == 0 and queue.qsize() > 0

    def remaining_actions(self) -> list[Any] | None:
        queue = getattr(self._native, "action_queue", None)
        if queue is None:
            return None
        leftover = queue.get_processed_left_over()
        if leftover is None:
            return []
        return list(leftover)

    def step(self, raw_observation: dict[str, Any]) -> Any | None:
        self._native.notify_observation(raw_observation)
        return self._native.get_action(None)


def make_local_backend(
    bundle: LeRobotPolicyBundle,
    layout: PolicyLayout,
    *,
    inference: InferenceMode,
    task: str,
    device: str,
    dataset_features: dict[str, dict],
    observation_features: dict[str, dict],
    rtc_config: Any | None = None,
    rtc_queue_threshold: int = 30,
) -> InferenceBackend:
    """Build a local Sync or RTC backend (weights already in ``bundle``)."""
    if inference == "sync":
        from lerobot.rollout.inference.sync import SyncInferenceEngine
        from lerobot.utils.constants import ACTION

        native = SyncInferenceEngine(
            policy=bundle.policy,
            preprocessor=bundle.preprocessor,
            postprocessor=bundle.postprocessor,
            dataset_features=dataset_features,
            ordered_action_keys=list(layout.action_feature_names),
            task=task,
            device=device,
            robot_type="rmi",
        )
        return LocalSyncBackend(
            native,
            dataset_features=dataset_features,
            action_key=ACTION,
            policy=bundle.policy,
        )

    if inference != "async":
        raise ValueError(f"make_local_backend expects sync|async, got {inference!r}")

    if not supports_native_rtc(bundle.policy):
        raise ValueError(
            "checkpoint policy does not support RTC async inference "
            "(supports_rtc / predict_action_chunk signature)"
        )

    from lerobot.policies.rtc.configuration_rtc import RTCConfig
    from lerobot.rollout.inference.rtc import RTCInferenceEngine

    robot = _RmiRobotStub(
        robot_type="rmi",
        action_features={name: float for name in layout.action_feature_names},
    )
    native = RTCInferenceEngine(
        policy=bundle.policy,
        preprocessor=bundle.preprocessor,
        postprocessor=bundle.postprocessor,
        robot_wrapper=robot,
        rtc_config=rtc_config or RTCConfig(),
        hw_features=observation_features,
        task=task,
        fps=float(layout.frequency),
        device=device,
        rtc_queue_threshold=rtc_queue_threshold,
    )
    return LocalAsyncBackend(native, policy=bundle.policy)
