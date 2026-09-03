"""Run a LeRobot ACT policy on the dual Piper RMI control plane.

The policy is evaluated on the workstation GPU.  The RT host owns the Piper
ros2_control hardware and receives the RMI-routed joint references over DDS.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import queue
import select
import shutil
import sys
import termios
import threading
import time
import tty
from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from enum import Enum
from itertools import pairwise
from pathlib import Path
from typing import Any

import numpy as np

JOINT_NAMES = (
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "left_gripper_joint1",
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
    "right_gripper_joint1",
)
PARTS = ("left_arm", "left_gripper", "right_arm", "right_gripper")
IMAGE_KEYS = ("top", "left_wrist", "right_wrist")
IMAGE_SHAPES = {
    "top": (720, 1280, 3),
    "left_wrist": (480, 640, 3),
    "right_wrist": (480, 640, 3),
}
POLICY_IMAGE_KEYS = tuple(f"observation.images.{name}" for name in IMAGE_KEYS)
ALL3_IMAGE_SHAPES = {
    "external_1": (720, 1280, 3),
    "external_2": (480, 640, 3),
    "external_3": (480, 640, 3),
    "left_wrist": (480, 640, 3),
    "right_wrist": (480, 640, 3),
}
ALL3_CAMERA_TOPICS = {
    "external_1": "/observation/orbbec/color/image_raw",
    "external_2": "/observation/d435i1/color/image_raw",
    "external_3": "/observation/d435i2/color/image_raw",
}
LOGGER = logging.getLogger("act_piper")


def _configure_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
    )


def _resolve_device(requested: str) -> str:
    if requested != "auto":
        return requested
    import torch

    return "cuda" if torch.cuda.is_available() else "cpu"


def _resolve_checkpoint(path: Path) -> Path:
    """Accept a pretrained_model directory or a LeRobot training run root."""
    candidates = (
        path,
        path / "pretrained_model",
        path / "checkpoints" / "last" / "pretrained_model",
    )
    for candidate in candidates:
        if (candidate / "config.json").is_file():
            return candidate.resolve()
    raise FileNotFoundError(
        "checkpoint not found; pass a pretrained_model directory or a training "
        f"output containing checkpoints/last: {path}"
    )


def _resolve_action_steps(
    configured_steps: int, chunk_size: int, requested_steps: int | None
) -> int:
    action_steps = configured_steps if requested_steps is None else requested_steps
    if not 0 < action_steps <= chunk_size:
        raise ValueError(
            f"invalid ACT action steps: {action_steps}; expected 1..{chunk_size}"
        )
    return action_steps


def _apply_gripper_guard(
    action: np.ndarray, *, hold_open: bool, open_position: float
) -> np.ndarray:
    """Override both gripper channels for approach-only policies."""
    if not hold_open:
        return action
    if not 0.0 <= open_position <= 0.04:
        raise ValueError("open gripper position must be within [0.0, 0.04] m")
    guarded = action.copy()
    guarded[6] = open_position
    guarded[13] = open_position
    return guarded


def _parse_layout_ids(value: str | None, episode_count: int) -> list[str]:
    if value is None:
        return [f"layout_{index:02d}" for index in range(episode_count)]
    layouts = [item.strip() for item in value.split(",") if item.strip()]
    if len(layouts) != episode_count:
        raise ValueError(
            f"--layout-ids contains {len(layouts)} names, expected {episode_count}"
        )
    return layouts


def _policy_image_shapes(camera_view: str) -> dict[str, tuple[int, int, int]]:
    if camera_view == "single":
        return dict(IMAGE_SHAPES)
    if camera_view == "all3":
        return dict(ALL3_IMAGE_SHAPES)
    raise ValueError(f"unsupported camera view: {camera_view}")


def _policy_image_topics(args: argparse.Namespace) -> dict[str, str]:
    camera_view = getattr(args, "camera_view", "single")
    wrist_topics = {
        "left_wrist": args.left_wrist_topic,
        "right_wrist": args.right_wrist_topic,
    }
    if camera_view == "single":
        return {"top": args.top_camera_topic, **wrist_topics}
    if camera_view == "all3":
        return {**ALL3_CAMERA_TOPICS, **wrist_topics}
    raise ValueError(f"unsupported camera view: {camera_view}")


class ActRunner:
    """Load ACT and convert raw ROS observations into action chunks."""

    def __init__(
        self,
        checkpoint: Path,
        device: str,
        action_steps: int | None = None,
        camera_view: str = "single",
    ) -> None:
        import torch
        from lerobot.policies import get_policy_class, make_pre_post_processors
        from lerobot.utils.constants import OBS_STR
        from lerobot.utils.feature_utils import hw_to_dataset_features

        checkpoint = _resolve_checkpoint(checkpoint)

        self.device = _resolve_device(device)
        self.checkpoint = checkpoint
        policy_class = get_policy_class("act")
        LOGGER.info("loading ACT checkpoint: %s", checkpoint)
        self.policy = policy_class.from_pretrained(str(checkpoint))
        self.policy.to(self.device)
        self.policy.eval()

        self.image_shapes = _policy_image_shapes(camera_view)
        self.image_keys = tuple(self.image_shapes)
        self.policy_image_keys = tuple(
            f"observation.images.{name}" for name in self.image_keys
        )
        image_features = self.policy.config.image_features
        expected = set(self.policy_image_keys)
        actual = set(image_features)
        if actual != expected:
            raise ValueError(
                "checkpoint image features do not match the Piper deployment: "
                f"expected={sorted(expected)}, actual={sorted(actual)}"
            )
        state_feature = self.policy.config.input_features.get("observation.state")
        action_feature = self.policy.config.output_features.get("action")
        if state_feature is None or tuple(state_feature.shape) != (len(JOINT_NAMES),):
            raise ValueError(
                "checkpoint must consume a 14-dimensional observation.state; "
                f"got {getattr(state_feature, 'shape', None)}"
            )
        if action_feature is None or tuple(action_feature.shape) != (len(JOINT_NAMES),):
            raise ValueError(
                "checkpoint must produce a 14-dimensional action; "
                f"got {getattr(action_feature, 'shape', None)}"
            )

        hardware_features: dict[str, type | tuple[int, int, int]] = {
            name: float for name in JOINT_NAMES
        }
        hardware_features.update(self.image_shapes)
        self.lerobot_features = hw_to_dataset_features(
            hardware_features, OBS_STR, use_video=False
        )
        device_override = {"device": self.device}
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            self.policy.config,
            pretrained_path=str(checkpoint),
            preprocessor_overrides={"device_processor": device_override},
            postprocessor_overrides={"device_processor": device_override},
        )
        self.chunk_size = int(getattr(self.policy.config, "chunk_size", 100))
        configured_action_steps = int(
            getattr(self.policy.config, "n_action_steps", self.chunk_size)
        )
        self.action_steps = _resolve_action_steps(
            configured_action_steps, self.chunk_size, action_steps
        )
        LOGGER.info(
            "ACT ready: device=%s, images=%s, state/action_dim=%d, chunk=%d, execute=%d",
            self.device,
            ", ".join(self.policy_image_keys),
            len(JOINT_NAMES),
            self.chunk_size,
            self.action_steps,
        )
        self._torch = torch

    def predict(self, raw_observation: dict[str, Any]) -> np.ndarray:
        """Return an unnormalized action chunk with shape ``[T, 14]``."""
        from lerobot.utils.constants import OBS_STATE
        from lerobot.utils.feature_utils import build_dataset_frame
        from torch.nn import functional

        frame = build_dataset_frame(
            self.lerobot_features, raw_observation, "observation"
        )
        observation: dict[str, Any] = {
            OBS_STATE: self._torch.as_tensor(frame[OBS_STATE]).unsqueeze(0)
        }
        for key in self.policy_image_keys:
            image = self._torch.as_tensor(frame[key]).permute(2, 0, 1).unsqueeze(0)
            target_shape = self.policy.config.image_features[key].shape
            image = functional.interpolate(
                image.float(),
                size=(target_shape[1], target_shape[2]),
                mode="bilinear",
                align_corners=False,
            )
            observation[key] = image / 255.0
        with self._torch.inference_mode():
            processed = self.preprocessor(observation)
            chunk = self.policy.predict_action_chunk(processed)
            if chunk.ndim == 2:
                chunk = chunk.unsqueeze(0)
            chunk = chunk[:, : self.action_steps, :]
            actions = []
            for index in range(chunk.shape[1]):
                actions.append(self.postprocessor(chunk[:, index, :]))
            action_tensor = self._torch.stack(actions, dim=1).squeeze(0)
        action = action_tensor.detach().cpu().numpy().astype(np.float32, copy=False)
        expected_shape = (self.action_steps, len(JOINT_NAMES))
        if action.shape != expected_shape:
            raise ValueError(f"ACT returned {action.shape}, expected {expected_shape}")
        if not np.isfinite(action).all():
            raise ValueError("ACT returned NaN or Inf")
        return action


def _raw_from_arrays(
    state: np.ndarray,
    images: dict[str, np.ndarray],
    image_keys: tuple[str, ...] = IMAGE_KEYS,
) -> dict[str, Any]:
    if state.shape != (len(JOINT_NAMES),):
        raise ValueError(
            f"state shape must be {(len(JOINT_NAMES),)}, got {state.shape}"
        )
    raw = {name: float(value) for name, value in zip(JOINT_NAMES, state, strict=True)}
    if set(images) != set(image_keys):
        raise ValueError(
            f"image keys must be {sorted(image_keys)}, got {sorted(images)}"
        )
    for name in image_keys:
        image = images[name]
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(f"{name} image must be HxWx3, got {image.shape}")
        raw[name] = image
    return raw


def _decode_rgb_image(message: Any) -> np.ndarray:
    """Decode the raw ROS Image formats emitted by the Piper cameras.

    The LeRobot environment intentionally does not carry the full ROS OpenCV
    stack, so policy deployment must not depend on cv_bridge.
    """
    encoding = str(message.encoding).lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(encoding)
    if encoding in {"mono8", "8uc1"}:
        channels = 1
    elif encoding in {"8uc3", "8sc3"}:
        channels = 3
    if channels is None:
        raise ValueError(f"unsupported image encoding: {message.encoding!r}")

    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
    required = height * step
    if raw.size < required:
        raise ValueError(f"image data is too short: {raw.size} < {required}")
    pixels = raw[:required].reshape(height, step)[:, : width * channels]
    pixels = pixels.reshape(height, width, channels)
    if channels == 1:
        return np.repeat(pixels, 3, axis=2).copy()
    if encoding in {"bgr8", "bgra8"}:
        return pixels[:, :, :3][:, :, ::-1].copy()
    return pixels[:, :, :3].copy()


class InputBuffer:
    """Thread-safe latest-value cache for ROS callbacks."""

    def __init__(self, image_keys: tuple[str, ...] = IMAGE_KEYS) -> None:
        self._image_keys = image_keys
        self._lock = threading.Lock()
        self._state: np.ndarray | None = None
        self._state_time = 0.0
        self._images: dict[str, np.ndarray] = {}
        self._image_times: dict[str, float] = {}

    def update_state(self, message: Any) -> None:
        positions = dict(zip(message.name, message.position, strict=False))
        if not all(name in positions for name in JOINT_NAMES):
            return
        state = np.asarray([positions[name] for name in JOINT_NAMES], dtype=np.float32)
        if not np.isfinite(state).all():
            return
        with self._lock:
            self._state = state
            self._state_time = time.monotonic()

    def update_image(self, name: str, image: np.ndarray) -> None:
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            return
        with self._lock:
            self._images[name] = np.ascontiguousarray(image)
            self._image_times[name] = time.monotonic()

    def snapshot(self) -> tuple[dict[str, Any] | None, float]:
        now = time.monotonic()
        with self._lock:
            if self._state is None or set(self._images) != set(self._image_keys):
                return None, now
            state = self._state.copy()
            images = {name: image.copy() for name, image in self._images.items()}
            times = [self._state_time, *self._image_times.values()]
        return _raw_from_arrays(state, images, self._image_keys), now - min(times)

    def ready(self) -> bool:
        with self._lock:
            return self._state is not None and set(self._images) == set(self._image_keys)


class ActPiperNode:
    """ROS subscriptions and the bounded policy action-chunk execution loop."""

    def __init__(self, node: Any, runner: ActRunner, args: argparse.Namespace) -> None:
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, JointState

        self.node = node
        self.runner = runner
        self.args = args
        image_keys = tuple(
            getattr(
                runner,
                "image_keys",
                _policy_image_shapes(getattr(args, "camera_view", "single")),
            )
        )
        self.inputs = InputBuffer(image_keys)
        self.session: Any | None = None
        self.actions: deque[np.ndarray] = deque()
        self._prediction_future: Future[tuple[np.ndarray, int]] | None = None
        self._prediction_context: dict[str, Any] | None = None
        self._prediction_generation = 0
        self._prediction_pool = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="act-piper-inference"
        )
        self._control_lock = threading.Lock()
        self._running = False
        self.fatal_error: BaseException | None = None
        self._last_inactive_log = 0.0
        self._last_publish_log = 0.0
        self._last_stale_log = 0.0
        self._episode_trace: dict[str, Any] | None = None
        self._timer_group = MutuallyExclusiveCallbackGroup()
        self.timer: Any | None = None

        node.create_subscription(
            JointState,
            args.state_topic,
            self.inputs.update_state,
            qos_profile_sensor_data,
        )
        image_topics = _policy_image_topics(args)
        for name, topic in image_topics.items():
            node.create_subscription(
                Image,
                topic,
                self._make_image_callback(name),
                qos_profile_sensor_data,
            )

    def start(self) -> None:
        """Start the control timer after RMI has granted the policy session."""
        with self._control_lock:
            if self.timer is not None:
                return
            self._running = True
            self.timer = self.node.create_timer(
                1.0 / self.args.rate_hz,
                self._tick,
                callback_group=self._timer_group,
            )
        LOGGER.info("asynchronous policy inference enabled; controller retains last target")

    def stop(self) -> None:
        with self._control_lock:
            self._running = False
            timer, self.timer = self.timer, None
            future = self._discard_prediction_locked()
            self.actions.clear()
        if timer is not None:
            self.node.destroy_timer(timer)
        if future is not None:
            future.cancel()
            if not future.cancelled():
                try:
                    future.result()
                except Exception as exc:  # noqa: BLE001
                    LOGGER.warning("discarding failed in-flight policy query: %s", exc)

    def close(self) -> None:
        self.stop()
        self._prediction_pool.shutdown(wait=True, cancel_futures=True)

    def _make_image_callback(self, name: str):
        def callback(message: Any) -> None:
            try:
                self.inputs.update_image(name, _decode_rgb_image(message))
            except Exception as exc:  # noqa: BLE001
                LOGGER.warning("failed to convert %s image: %s", name, exc)

        return callback

    def wait_for_inputs(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while not self.inputs.ready():
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    "timed out waiting for /joint_states and all policy RGB camera topics"
                )
            time.sleep(0.05)

    def set_session(self, session: Any) -> None:
        with self._control_lock:
            self.session = session

    def clear_session(self) -> None:
        with self._control_lock:
            self.session = None
            self.actions.clear()
            self._prediction_generation += 1
            reset_history = getattr(
                getattr(self, "runner", None), "reset_history", None
            )
            if reset_history is not None:
                reset_history()

    def start_episode_trace(self) -> None:
        """Start an in-memory trace; recording never performs I/O in the timer."""
        self._episode_trace = {
            "schema_version": "1.0",
            "start_time_ns": time.time_ns(),
            "queries": [],
            "published_actions": [],
        }

    def finish_episode_trace(self) -> dict[str, Any] | None:
        trace, self._episode_trace = self._episode_trace, None
        if trace is not None:
            trace["end_time_ns"] = time.time_ns()
            query_times = [item["duration_ms"] for item in trace["queries"]]
            query_starts = [item["start_time_ns"] for item in trace["queries"]]
            query_intervals_ms = [
                (later - earlier) / 1e6
                for earlier, later in pairwise(query_starts)
            ]
            publish_times = [
                item["time_ns"] for item in trace["published_actions"]
            ]
            publish_intervals_ms = [
                (later - earlier) / 1e6
                for earlier, later in pairwise(publish_times)
            ]
            trace["summary"] = {
                "query_count": len(query_times),
                "published_action_count": len(trace["published_actions"]),
                "policy_action_count": sum(
                    item.get("source", "policy") == "policy"
                    for item in trace["published_actions"]
                ),
                "hold_action_count": sum(
                    item.get("source") == "hold" for item in trace["published_actions"]
                ),
                "query_duration_ms_mean": (
                    float(np.mean(query_times)) if query_times else None
                ),
                "query_duration_ms_p95": (
                    float(np.percentile(query_times, 95)) if query_times else None
                ),
                "query_interval_ms_mean": (
                    float(np.mean(query_intervals_ms)) if query_intervals_ms else None
                ),
                "actual_query_rate_hz": (
                    1000.0 / float(np.mean(query_intervals_ms))
                    if query_intervals_ms
                    else None
                ),
                "publish_interval_ms_mean": (
                    float(np.mean(publish_intervals_ms))
                    if publish_intervals_ms
                    else None
                ),
                "publish_interval_ms_p95": (
                    float(np.percentile(publish_intervals_ms, 95))
                    if publish_intervals_ms
                    else None
                ),
                "publish_interval_ms_max": (
                    float(np.max(publish_intervals_ms))
                    if publish_intervals_ms
                    else None
                ),
                "actual_publish_rate_hz": (
                    1000.0 / float(np.mean(publish_intervals_ms))
                    if publish_intervals_ms
                    else None
                ),
            }
        return trace

    def _tick(self) -> None:
        raw_observation, age_s = self.inputs.snapshot()
        if raw_observation is None:
            return
        try:
            with self._control_lock:
                if (
                    not self._running
                    or self.fatal_error is not None
                    or self.session is None
                ):
                    return
                if age_s > self.args.max_input_age_s:
                    self.actions.clear()
                    self._prediction_generation += 1
                    reset_history = getattr(
                        getattr(self, "runner", None), "reset_history", None
                    )
                    if reset_history is not None:
                        reset_history()
                    now = time.monotonic()
                    if now - self._last_stale_log > 1.0:
                        LOGGER.warning(
                            "input stream is stale (%.3fs > %.3fs); holding output "
                            "until fresh inputs arrive",
                            age_s,
                            self.args.max_input_age_s,
                        )
                        self._last_stale_log = now
                    return
                if not self.session.active:
                    self.actions.clear()
                    self._prediction_generation += 1
                    reset_history = getattr(
                        getattr(self, "runner", None), "reset_history", None
                    )
                    if reset_history is not None:
                        reset_history()
                    now = time.monotonic()
                    if now - self._last_inactive_log > 1.0:
                        LOGGER.warning(
                            "Policy session is no longer authoritative; holding output"
                        )
                        self._last_inactive_log = now
                    return

                self._accept_prediction_locked()
                if self.actions:
                    observe = getattr(self.runner, "observe", None)
                    if observe is not None:
                        observe(raw_observation)
                    action = self.actions.popleft()
                    self._publish_action(action, source="policy")
                    return

                self._submit_prediction_locked(raw_observation, age_s)
        except Exception as exc:  # noqa: BLE001
            with self._control_lock:
                self.actions.clear()
            self._fail(exc)

    def _submit_prediction_locked(
        self, raw_observation: dict[str, Any], age_s: float
    ) -> None:
        if self._prediction_future is not None:
            return
        query_start_ns = time.time_ns()
        self._prediction_context = {
            "generation": self._prediction_generation,
            "start_time_ns": query_start_ns,
            "input_age_s": age_s,
            "observation_state": [float(raw_observation[name]) for name in JOINT_NAMES],
        }
        self._prediction_future = self._prediction_pool.submit(
            self._predict_timed, raw_observation
        )

    def _predict_timed(self, raw_observation: dict[str, Any]) -> tuple[np.ndarray, int]:
        return self.runner.predict(raw_observation), time.time_ns()

    def _accept_prediction_locked(self) -> None:
        future = self._prediction_future
        if future is None or not future.done():
            return
        context = self._prediction_context
        self._prediction_future = None
        self._prediction_context = None
        chunk, query_end_ns = future.result()
        if context is None or context["generation"] != self._prediction_generation:
            return
        self.actions.extend(chunk)
        if self._episode_trace is not None:
            query_start_ns = context["start_time_ns"]
            self._episode_trace["queries"].append(
                {
                    "start_time_ns": query_start_ns,
                    "end_time_ns": query_end_ns,
                    "duration_ms": (query_end_ns - query_start_ns) / 1e6,
                    "input_age_s": context["input_age_s"],
                    "observation_state": context["observation_state"],
                    "predicted_action_chunk": chunk.astype(float).tolist(),
                }
            )
        LOGGER.info("inferred action chunk: shape=%s", chunk.shape)

    def _discard_prediction_locked(self) -> Future[tuple[np.ndarray, int]] | None:
        self._prediction_generation += 1
        future, self._prediction_future = self._prediction_future, None
        self._prediction_context = None
        return future

    def _publish_action(self, action: np.ndarray, *, source: str = "policy") -> None:
        if action.shape != (len(JOINT_NAMES),) or not np.isfinite(action).all():
            raise ValueError(f"invalid action shape/value: {action.shape}")
        from rmi import Action

        action = _apply_gripper_guard(
            action,
            hold_open=self.args.hold_grippers_open,
            open_position=self.args.open_gripper_position,
        )
        offsets = {
            "left_arm": slice(0, 6),
            "left_gripper": slice(6, 7),
            "right_arm": slice(7, 13),
            "right_gripper": slice(13, 14),
        }
        for part, indices in offsets.items():
            self.session.act(
                Action(
                    part=part,
                    command="joint_reference",
                    value=action[indices].astype(float).tolist(),
                )
            )
        if self._episode_trace is not None:
            self._episode_trace["published_actions"].append(
                {
                    "time_ns": time.time_ns(),
                    "source": source,
                    "action": action.astype(float).tolist(),
                }
            )
        now = time.monotonic()
        if now - self._last_publish_log > 1.0:
            LOGGER.info("published Policy action: left/right arm + gripper")
            self._last_publish_log = now

    def _fail(self, error: BaseException) -> None:
        if self.fatal_error is None:
            self.fatal_error = error
            LOGGER.error("policy output stopped: %s", error)


class RecordingState(str, Enum):
    READY = "ready"
    RECORDING = "recording"
    REVIEW = "review"


def _recording_key_action(state: RecordingState, key: str) -> str | None:
    """Map one terminal key to a recording workflow action."""
    if state is RecordingState.READY:
        return {"enter": "start", "q": "quit"}.get(key)
    if state is RecordingState.RECORDING:
        return {"enter": "finish", "t": "toggle_teleop", "q": "abort"}.get(key)
    if state is RecordingState.REVIEW:
        return {"enter": "save", "s": "save", "d": "discard", "q": "quit"}.get(key)
    return None


class TerminalKeyReader:
    """Read single keys without competing ``input()`` calls for stdin."""

    def __init__(self) -> None:
        self._events: queue.SimpleQueue[str] = queue.SimpleQueue()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._fd: int | None = None
        self._attributes: list[Any] | None = None

    def start(self, *, required: bool) -> bool:
        if not sys.stdin.isatty():
            if required:
                raise RuntimeError("--record-episodes requires an interactive TTY")
            LOGGER.warning("stdin is not a TTY; terminal teleop hotkey is disabled")
            return False
        self._fd = sys.stdin.fileno()
        self._attributes = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        self._thread = threading.Thread(
            target=self._read,
            name="act-piper-terminal-keys",
            daemon=True,
        )
        self._thread.start()
        return True

    def _read(self) -> None:
        assert self._fd is not None
        while not self._stop.is_set():
            readable, _, _ = select.select([self._fd], [], [], 0.1)
            if not readable:
                continue
            value = os.read(self._fd, 1)
            if not value:
                return
            key = value.decode(errors="ignore").lower()
            if key in {"\r", "\n"}:
                self._events.put("enter")
            elif key in {"t", "s", "d", "q"}:
                self._events.put(key)

    def next(self, timeout_s: float) -> str | None:
        try:
            return self._events.get(timeout=timeout_s)
        except queue.Empty:
            return None

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
        if self._fd is not None and self._attributes is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._attributes)
        self._thread = None
        self._fd = None
        self._attributes = None


class PolicyControl:
    """Own the single Policy session used by inference, staging, and recovery."""

    def __init__(
        self, agent: Any, robot: Any, bridge: ActPiperNode, rate_hz: float
    ) -> None:
        self.agent = agent
        self.robot = robot
        self.bridge = bridge
        self.rate_hz = rate_hz
        self.session: Any | None = None

    def start(self, *, run_bridge: bool = True) -> None:
        if self.session is not None:
            return
        self.session = self.agent.run(
            self.robot,
            parts=PARTS,
            frequency=self.rate_hz,
        )
        self.session.__enter__()
        self.bridge.set_session(self.session)
        if run_bridge:
            self.bridge.start()
        LOGGER.info("Policy session acquired for %s", ", ".join(PARTS))

    def stop(self) -> None:
        self.bridge.stop()
        self.bridge.clear_session()
        session, self.session = self.session, None
        if session is not None:
            session.__exit__(None, None, None)

    def close(self) -> None:
        self.stop()


@dataclass
class AssistEpisode:
    """One MCAP episode plus the timestamps of manual corrections."""

    scope: Any
    checkpoint: Path
    task: str
    episode_index: int
    started_time_ns: int
    intervals: list[dict[str, int]]
    active_teleop_start_ns: int | None = None
    policy_type: str = "act"
    prediction_horizon_k: int | None = None
    executed_steps_m: int | None = None
    rate_hz: float | None = None
    hold_grippers_open: bool = False
    layout_id: str | None = None

    @classmethod
    def start(
        cls,
        recorder: Any,
        *,
        checkpoint: Path,
        task: str,
        episode_index: int,
        profile: str,
        rate_hz: float,
        stop_timeout_s: float,
        policy_type: str = "act",
        prediction_horizon_k: int | None = None,
        executed_steps_m: int | None = None,
        hold_grippers_open: bool = False,
        layout_id: str | None = None,
    ) -> AssistEpisode:
        metadata = {
            "task": task,
            "profile": profile,
            "rate_hz": rate_hz,
            "episode_index": episode_index,
            "policy_type": policy_type,
            "policy_checkpoint": str(checkpoint.resolve()),
            "operator": f"{policy_type}_policy_assist",
            "prediction_horizon_k": prediction_horizon_k,
            "executed_steps_m": executed_steps_m,
            "hold_grippers_open": hold_grippers_open,
            "layout_id": layout_id,
        }
        scope = recorder.episode(
            task=task,
            metadata=metadata,
            stop_timeout=stop_timeout_s,
        )
        scope.__enter__()
        return cls(
            scope=scope,
            checkpoint=checkpoint.resolve(),
            task=task,
            episode_index=episode_index,
            started_time_ns=time.time_ns(),
            intervals=[],
            policy_type=policy_type,
            prediction_horizon_k=prediction_horizon_k,
            executed_steps_m=executed_steps_m,
            rate_hz=rate_hz,
            hold_grippers_open=hold_grippers_open,
            layout_id=layout_id,
        )

    def teleop_started(self) -> None:
        if self.active_teleop_start_ns is None:
            self.active_teleop_start_ns = time.time_ns()

    def teleop_ended(self) -> None:
        if self.active_teleop_start_ns is None:
            return
        end_time_ns = time.time_ns()
        self.intervals.append(
            {
                "start_time_ns": self.active_teleop_start_ns,
                "end_time_ns": end_time_ns,
                "start_offset_ns": self.active_teleop_start_ns - self.started_time_ns,
                "end_offset_ns": end_time_ns - self.started_time_ns,
            }
        )
        self.active_teleop_start_ns = None

    def finish(
        self, *, discard: bool, policy_trace: dict[str, Any] | None = None
    ) -> Path | None:
        self.teleop_ended()
        if discard:
            self.scope.discard()
        self.scope.__exit__(None, None, None)
        if discard:
            return None
        status = self.scope.final_status
        episode_path = getattr(status, "episode_path", "") if status is not None else ""
        if not episode_path:
            raise RuntimeError("recorder finalized episode without an episode_path")
        path = Path(episode_path)
        episode_dir = path if path.is_dir() else path.parent
        payload = {
            "schema_version": "1.0",
            "policy": {"type": self.policy_type, "checkpoint": str(self.checkpoint)},
            "deployment": {
                "prediction_horizon_k": self.prediction_horizon_k,
                "executed_steps_m": self.executed_steps_m,
                "control_rate_hz": self.rate_hz,
                "nominal_query_rate_hz": (
                    self.rate_hz / self.executed_steps_m
                    if self.rate_hz is not None and self.executed_steps_m
                    else None
                ),
                "hold_grippers_open": self.hold_grippers_open,
            },
            "task": self.task,
            "layout_id": self.layout_id,
            "episode_index": self.episode_index,
            "start_time_ns": self.started_time_ns,
            "end_time_ns": time.time_ns(),
            "teleop_count": len(self.intervals),
            "teleop_intervals": self.intervals,
            "policy_trace": policy_trace,
        }
        destination = episode_dir / "episode_assist.json"
        temporary = destination.with_suffix(".tmp")
        temporary.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        temporary.replace(destination)
        LOGGER.info("wrote policy-assist metadata: %s", destination)
        return episode_dir


def _smooth_homing(context: Any, control: PolicyControl) -> None:
    """Return both followers to the profile staging pose while no episode is open."""
    from rmi import Action

    recorder_cfg = context.profile.raw_data.get("recorder", {})
    home_pose = list(recorder_cfg.get("home_pose", [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]))
    duration_s = float(recorder_cfg.get("homing_duration_s", 2.5))
    rate_hz = float(recorder_cfg.get("rate_hz", 50.0))
    if len(home_pose) != 6 or duration_s <= 0.0 or rate_hz <= 0.0:
        raise ValueError("invalid recorder home_pose, homing_duration_s, or rate_hz")

    observation = context.robot.get_observation()
    positions = dict(
        zip(observation.joint_names, observation.joint_positions, strict=False)
    )
    targets = {
        "left_arm": home_pose,
        "right_arm": home_pose,
        "left_gripper": [0.020],
        "right_gripper": [0.020],
    }
    starts = {
        part: [positions[name] for name in context.profile.parts[part].joint_names]
        for part in PARTS
    }
    steps = max(10, round(duration_s * rate_hz))
    control.start(run_bridge=False)
    try:
        started = time.monotonic()
        for index in range(steps + 1):
            phase = index / steps
            blend = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
            for part in PARTS:
                values = [
                    start + blend * (goal - start)
                    for start, goal in zip(starts[part], targets[part], strict=True)
                ]
                control.session.act(
                    Action(part=part, command="joint_reference", value=values)
                )
            remaining = (index + 1) / rate_hz - (time.monotonic() - started)
            if remaining > 0.0:
                time.sleep(remaining)
    finally:
        control.stop()


class TeleopTakeover:
    """Relay local leader commands only while the operator owns teleoperation."""

    def __init__(self, node: Any, context: Any, *, service_name: str) -> None:
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_srvs.srv import SetBool
        from trajectory_msgs.msg import JointTrajectory

        self.node = node
        self.context = context
        self._sessions: dict[str, Any] = {}
        self._requests: queue.SimpleQueue[bool] = queue.SimpleQueue()
        self._requested_mode = False
        self._request_lock = threading.Lock()
        self.active = False
        self._teleoperators = context.profile.raw_data.get("teleoperators", {})
        self._preempt_clients = {
            name: node.create_client(SetBool, cfg["preempt_service"])
            for name, cfg in self._teleoperators.items()
            if cfg.get("preempt_service")
        }
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        for name, cfg in self._teleoperators.items():
            node.create_subscription(
                JointTrajectory,
                cfg["arm_source"],
                self._relay(name, cfg["arm_part"]),
                qos,
            )
            node.create_subscription(
                JointTrajectory,
                cfg["gripper_source"],
                self._relay(name, cfg["gripper_part"]),
                qos,
            )
        self._mode_service = node.create_service(
            SetBool,
            service_name,
            self._handle_mode_request,
        )

    def _relay(self, leader: str, part: str):
        from rmi import Action

        def callback(message: Any) -> None:
            session = self._sessions.get(leader)
            if self.active and session is not None and session.active_for(part):
                session.act(
                    Action(
                        part=part,
                        command="joint_reference",
                        value=message,
                    )
                )

        return callback

    def _handle_mode_request(self, request: Any, response: Any) -> Any:
        self.request(bool(request.data))
        response.success = True
        response.message = "teleop mode transition requested"
        return response

    def request(self, enabled: bool) -> None:
        with self._request_lock:
            self._requested_mode = enabled
        self._requests.put(enabled)

    def reset_request(self) -> None:
        with self._request_lock:
            self._requested_mode = False

    def toggle_request(self) -> None:
        with self._request_lock:
            self._requested_mode = not self._requested_mode
            enabled = self._requested_mode
        self._requests.put(enabled)

    def next_request(self, timeout_s: float) -> bool | None:
        try:
            return self._requests.get(timeout=timeout_s)
        except queue.Empty:
            return None

    def activate(self) -> None:
        if self.active:
            return
        if not self._teleoperators:
            raise RuntimeError("profile does not define Piper teleoperators")
        for name, cfg in self._teleoperators.items():
            if not self.node.get_publishers_info_by_topic(cfg["arm_source"]):
                raise RuntimeError(
                    f"{name} arm source is not publishing; start workstation_stack with leaders"
                )
        for name, client in self._preempt_clients.items():
            if not client.wait_for_service(timeout_sec=0.5):
                raise RuntimeError(f"{name} preempt service is unavailable")

        entered: list[tuple[str, Any]] = []
        try:
            for name, cfg in self._teleoperators.items():
                agent = self.context.make_agent(
                    cfg["target_agent"],
                    frequency=cfg.get("publish_rate_hz", 200.0),
                )
                parts = (cfg["arm_part"], cfg["gripper_part"])
                session = agent.run(self.context.robot, parts=parts)
                session.__enter__()
                entered.append((name, session))
                self._sessions[name] = session
            self._set_preempt(True)
            self.active = True
            LOGGER.warning(
                "TELEOP ACTIVE: policy output is paused; local leaders own motion"
            )
        except Exception:
            self._set_preempt(False, best_effort=True)
            for name, session in reversed(entered):
                session.__exit__(None, None, None)
                self._sessions.pop(name, None)
            raise

    def deactivate(self) -> None:
        if not self.active:
            return
        self._set_preempt(False, best_effort=True)
        for name, session in reversed(tuple(self._sessions.items())):
            session.__exit__(None, None, None)
            self._sessions.pop(name, None)
        self.active = False
        LOGGER.info("teleop released; leader arms returned to shadow mode")

    def _set_preempt(self, enabled: bool, *, best_effort: bool = False) -> None:
        from std_srvs.srv import SetBool

        failures: list[str] = []
        for name, client in self._preempt_clients.items():
            future = client.call_async(SetBool.Request(data=enabled))
            deadline = time.monotonic() + 1.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            if not future.done() or not future.result().success:
                failures.append(name)
        if failures and not best_effort:
            raise RuntimeError(
                "failed to set leader preempt mode: " + ", ".join(failures)
            )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run a LeRobot ACT checkpoint on the dual Piper RMI runtime",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--profile", default="apps/profiles/piper_bimanual.yaml")
    parser.add_argument("--task", default="pick_bread")
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--state-topic", default="/joint_states")
    parser.add_argument(
        "--top-camera-topic",
        choices=("/observation/orbbec/color/image_raw",),
        default="/observation/orbbec/color/image_raw",
    )
    parser.add_argument(
        "--camera-view",
        choices=("single",),
        default="single",
        help="use the Orbbec top camera plus left/right wrist cameras",
    )
    parser.add_argument(
        "--left-wrist-topic",
        default="/observation/left_hand_realsense/color/image_raw",
    )
    parser.add_argument(
        "--right-wrist-topic",
        default="/observation/right_hand_realsense/color/image_raw",
    )
    parser.add_argument("--device", choices=("auto", "cuda", "cpu"), default="auto")
    parser.add_argument(
        "--action-steps",
        type=int,
        default=None,
        help="execute this many actions from each predicted chunk before querying again",
    )
    parser.add_argument(
        "--hold-grippers-open",
        action="store_true",
        help="hard-override both policy gripper channels during approach-only tests",
    )
    parser.add_argument(
        "--open-gripper-position",
        type=float,
        default=0.020,
        help="per-finger position used by --hold-grippers-open, in metres",
    )
    parser.add_argument("--input-timeout-s", type=float, default=10.0)
    parser.add_argument("--max-input-age-s", type=float, default=0.5)
    parser.add_argument(
        "--record-episodes",
        type=int,
        default=0,
        help="record this many ACT-assisted trial episodes; 0 keeps continuous inference",
    )
    parser.add_argument(
        "--layout-ids",
        default=None,
        help="comma-separated layout IDs, one for each recorded episode",
    )
    parser.add_argument(
        "--teleop-takeover",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="enable local Piper leader takeover through T or /act_piper/set_teleop",
    )
    parser.add_argument(
        "--teleop-hotkey",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="toggle teleop takeover with raw T in this terminal",
    )
    parser.add_argument(
        "--real",
        action="store_true",
        help="explicitly acknowledge that the connected RT stack may control real hardware",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="load the checkpoint and run one synthetic inference without ROS or motion",
    )
    return parser


def _run_dry_run(args: argparse.Namespace, runner: ActRunner) -> None:
    state = np.zeros(len(JOINT_NAMES), dtype=np.float32)
    images = {
        name: np.zeros(shape, dtype=np.uint8)
        for name, shape in runner.image_shapes.items()
    }
    action = runner.predict(_raw_from_arrays(state, images, runner.image_keys))
    LOGGER.info(
        "dry-run passed: action shape=%s, finite=%s",
        action.shape,
        np.isfinite(action).all(),
    )


def _wait_for_deployment_ready(context: Any, bridge: ActPiperNode, timeout_s: float) -> None:
    # The workstation authority is ExecutionManagerClient.  It owns RMI leases
    # but deliberately does not expose ros2_control diagnostics; requiring them
    # rejects an otherwise ready real deployment.  State, Execution Manager
    # recovery, and camera freshness are checked by these two readiness calls.
    context.wait_until_ready(timeout=timeout_s)
    bridge.wait_for_inputs(timeout_s)


def _switch_teleop(
    control: PolicyControl,
    teleop: TeleopTakeover,
    episode: AssistEpisode | None,
    enabled: bool,
) -> None:
    if enabled:
        control.stop()
        try:
            teleop.activate()
        except Exception:
            control.start()
            raise
        if episode is not None:
            episode.teleop_started()
        return
    teleop.deactivate()
    if episode is not None:
        episode.teleop_ended()
    control.start()


def _run_recording(
    args: argparse.Namespace,
    context: Any,
    bridge: ActPiperNode,
    control: PolicyControl,
    teleop: TeleopTakeover | None,
    keys: TerminalKeyReader,
) -> None:
    """Collect complete autonomous-plus-correction episodes in one control context."""
    from record import (
        _normalize_task_name,
        _task_recorder_config,
    )

    recorder_cfg = dict(context.profile.recording)
    max_duration_s = float(recorder_cfg.get("max_duration_s", 60.0))
    if max_duration_s <= 0.0:
        raise ValueError("profile recorder.max_duration_s must be positive")
    task = _normalize_task_name(args.task)
    expected_top_topic = "/observation/orbbec/color/image_raw"
    if args.top_camera_topic != expected_top_topic:
        raise RuntimeError(
            "deployment only supports the Orbbec top camera: "
            f"{args.top_camera_topic} != {expected_top_topic}"
        )
    recorder = context.make_recorder(
        config=_task_recorder_config(
            recorder_cfg, task, f"{getattr(args, 'policy_type', 'act')}_policy"
        ),
        autostart=True,
    )
    recorder.activate()
    LOGGER.info("recording camera contract: Orbbec + left/right wrist")
    saved = 0
    source_episode_index = 1
    layout_ids = _parse_layout_ids(args.layout_ids, args.record_episodes)

    while saved < args.record_episodes:
        layout_id = layout_ids[saved]
        LOGGER.info(
            "recording episode %d/%d (%s): returning to staging home pose",
            saved + 1,
            args.record_episodes,
            layout_id,
        )
        _smooth_homing(context, control)
        LOGGER.warning(
            "READY layout=%s: arrange the cloth, then press Enter to start episode %d/%d; press Q to exit",
            layout_id,
            saved + 1,
            args.record_episodes,
        )
        while True:
            key = keys.next(0.1)
            if bridge.fatal_error is not None:
                raise bridge.fatal_error
            action = _recording_key_action(RecordingState.READY, key or "")
            if action == "quit":
                return
            if action == "start":
                break

        policy_type = getattr(args, "policy_type", "act")
        episode = AssistEpisode.start(
            recorder,
            checkpoint=args.checkpoint,
            task=task,
            episode_index=source_episode_index,
            profile=args.profile,
            rate_hz=args.rate_hz,
            stop_timeout_s=max_duration_s + 30.0,
            policy_type=policy_type,
            prediction_horizon_k=bridge.runner.chunk_size,
            executed_steps_m=bridge.runner.action_steps,
            hold_grippers_open=args.hold_grippers_open,
            layout_id=layout_id,
        )
        source_episode_index += 1
        started = time.monotonic()
        LOGGER.warning(
            "RECORDING: Enter finishes; T toggles ACT/teleop; Q discards and exits"
        )
        bridge.start_episode_trace()
        control.start()
        episode_dir: Path | None = None
        aborted = False
        try:
            while True:
                key = keys.next(0.05)
                if bridge.fatal_error is not None:
                    raise bridge.fatal_error
                if time.monotonic() - started >= max_duration_s:
                    LOGGER.warning(
                        "recording max duration %.1fs reached", max_duration_s
                    )
                    key = "enter"
                action = _recording_key_action(RecordingState.RECORDING, key or "")
                requested_teleop = (
                    teleop.next_request(0.0) if teleop is not None else None
                )
                if requested_teleop is not None and requested_teleop != teleop.active:
                    action = "toggle_teleop"
                if action == "toggle_teleop":
                    if teleop is None:
                        LOGGER.warning("teleop takeover is disabled")
                    else:
                        enabled = (
                            requested_teleop
                            if requested_teleop is not None
                            else not teleop.active
                        )
                        _switch_teleop(control, teleop, episode, enabled)
                    continue
                if action == "abort":
                    aborted = True
                    break
                if action == "finish":
                    break
        except BaseException:
            if teleop is not None and teleop.active:
                teleop.deactivate()
            control.stop()
            episode.finish(
                discard=True, policy_trace=bridge.finish_episode_trace()
            )
            raise

        if teleop is not None and teleop.active:
            teleop.deactivate()
            episode.teleop_ended()
        control.stop()
        policy_trace = bridge.finish_episode_trace()
        if aborted:
            episode.finish(discard=True, policy_trace=policy_trace)
            return
        episode_dir = episode.finish(discard=False, policy_trace=policy_trace)
        LOGGER.warning("REVIEW: Enter/S saves; D deletes this episode; Q exits")
        while True:
            key = keys.next(0.1)
            action = _recording_key_action(RecordingState.REVIEW, key or "")
            if action == "save":
                saved += 1
                LOGGER.info(
                    "saved episode %d/%d: %s", saved, args.record_episodes, episode_dir
                )
                break
            if action == "discard":
                if episode_dir is not None and episode_dir.is_dir():
                    shutil.rmtree(episode_dir)
                LOGGER.warning("discarded episode: %s", episode_dir)
                break
            if action == "quit":
                return


def _run_ros(args: argparse.Namespace, runner: ActRunner) -> None:
    import rclpy
    import rmi
    from rclpy.signals import SignalHandlerOptions

    # Keep the ROS context valid while Python handles Ctrl+C so the Policy
    # lease can be released through Execution Manager before shutdown.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    policy_type = getattr(args, "policy_type", "act")
    node = rclpy.create_node(f"{policy_type}_piper_policy")
    bridge = ActPiperNode(node, runner, args)
    context = None
    teleop = None
    control = None
    keys = TerminalKeyReader()

    try:
        LOGGER.info(
            "policy=%s, task=%s, execution_mode=%s",
            policy_type,
            args.task,
            "REAL" if args.real else "FAKE",
        )
        if not args.real:
            LOGGER.warning(
                "--real not set: use_fake_hardware:=true; this process still publishes only through RMI"
            )
        context = rmi.Context.from_profile(
            args.profile,
            node=node,
            spin_node=True,
            state_topic=args.state_topic,
        )
        _wait_for_deployment_ready(context, bridge, args.input_timeout_s)
        agent = context.make_agent("Policy", frequency=args.rate_hz)
        control = PolicyControl(agent, context.robot, bridge, args.rate_hz)
        if args.teleop_takeover:
            teleop = TeleopTakeover(
                node,
                context,
                service_name=f"/{policy_type}_piper/set_teleop",
            )
        if args.record_episodes:
            keys.start(required=True)
            _run_recording(args, context, bridge, control, teleop, keys)
            return
        if args.teleop_hotkey:
            keys.start(required=False)
            LOGGER.info(
                "teleop takeover ready: press T, or call /%s_piper/set_teleop with SetBool",
                policy_type,
            )
        try:
            control.start()
            while rclpy.ok() and bridge.fatal_error is None:
                key = keys.next(0.0) if args.teleop_hotkey else None
                if key == "t" and teleop is not None:
                    teleop.toggle_request()
                requested_teleop = (
                    teleop.next_request(0.1) if teleop is not None else None
                )
                if requested_teleop is None or requested_teleop == teleop.active:
                    continue
                if requested_teleop:
                    try:
                        _switch_teleop(control, teleop, None, True)
                    except Exception as exc:  # noqa: BLE001
                        LOGGER.error("teleop takeover rejected: %s; resuming ACT", exc)
                        teleop.reset_request()
                else:
                    _switch_teleop(control, teleop, None, False)
        except KeyboardInterrupt:
            LOGGER.info("shutdown requested; releasing Policy session")
        if bridge.fatal_error is not None:
            raise bridge.fatal_error
    finally:
        keys.close()
        if teleop is not None:
            teleop.deactivate()
        if control is not None:
            control.close()
        else:
            bridge.stop()
            bridge.clear_session()
        bridge.close()
        if context is not None:
            context.close()
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    _configure_logging()
    args = _build_parser().parse_args()
    if args.rate_hz <= 0.0:
        raise SystemExit("--rate-hz must be positive")
    if args.record_episodes < 0:
        raise SystemExit("--record-episodes must be non-negative")
    runner = ActRunner(
        args.checkpoint, args.device, args.action_steps, camera_view=args.camera_view
    )
    if args.dry_run:
        _run_dry_run(args, runner)
    else:
        _run_ros(args, runner)


if __name__ == "__main__":
    main()
