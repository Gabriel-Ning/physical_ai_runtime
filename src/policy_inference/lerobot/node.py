"""Profile-driven LeRobot policy provider node with an RMI command boundary."""

from __future__ import annotations

import logging
import threading
from pathlib import Path
from typing import Any

from ..common.contract import PolicyIOContract
from .bridge import (
    LeRobotToRmiActionBridge,
    RmiToLeRobotObservationBridge,
    ros_image_to_numpy,
)
from .policy import load_validated_policy_bundle
from .utils import make_dataset_features

_LOGGER = logging.getLogger(__name__)


class LeRobotPolicyNode:
    """Own ROS subscriptions while a dedicated thread owns policy inference."""

    def __init__(
        self,
        node: Any,
        *,
        profile: str | Path,
        checkpoint: str,
        task: str,
        device: str = "cuda",
        node_name: str = "Policy",
        max_observation_age_s: float = 0.5,
        expected_policy_type: str | None = None,
    ) -> None:
        if not task.strip():
            raise ValueError("task must not be empty")
        if max_observation_age_s <= 0.0:
            raise ValueError("max_observation_age_s must be positive")

        import rmi
        from lerobot.rollout.inference.sync import SyncInferenceEngine

        self.node = node
        self.context = rmi.Context.from_profile(profile, node=node, spin_node=False)
        self.contract = PolicyIOContract.from_profile(
            self.context.profile, node_name=node_name
        )
        self.bundle, self.compatibility = load_validated_policy_bundle(
            self.contract,
            checkpoint,
            device=device,
            expected_policy_type=expected_policy_type,
        )
        self._cameras = {}
        for feature_name in self.contract.camera_shapes:
            camera_name = self.contract.camera_sources[feature_name]
            self._cameras[feature_name] = self.context.make_camera(
                camera_name, converter=ros_image_to_numpy, history_size=1
            )

        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._observation_bridge = RmiToLeRobotObservationBridge(
            self.contract,
            max_stream_skew_s=max_observation_age_s,
        )
        self._action_bridge = LeRobotToRmiActionBridge(self.contract)
        dataset_features = make_dataset_features(self.contract)
        self._engine = SyncInferenceEngine(
            policy=self.bundle.policy,
            preprocessor=self.bundle.preprocessor,
            postprocessor=self.bundle.postprocessor,
            dataset_features=dataset_features,
            ordered_action_keys=list(self.contract.action_feature_names),
            task=task,
            device=device,
            robot_type="rmi",
        )
        self._dataset_features = dataset_features
        self._policy_node = self.context.make_node(node_name, self)

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("policy node is already started")
        self._thread = threading.Thread(
            target=self._run, name="lerobot-policy-inference", daemon=False
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            if self._thread.is_alive():
                raise TimeoutError("policy inference thread did not stop")
            self._thread = None
        self._engine.stop()
        self.context.close()

    def _run(self) -> None:
        self._engine.start()
        try:
            self.context.wait_until_ready(
                timeout=30.0,
                check_cameras=True,
                require_execution_manager=True,
            )
            while not self._stop.is_set():
                observation = self.context.robot.get_observation()
                actions = self.select_action(observation)
                if actions is not None:
                    self._policy_node.submit(actions)
                self._stop.wait(1.0 / self.contract.frequency)
        except BaseException:
            if not self._stop.is_set():
                _LOGGER.exception("policy inference stopped")
                self._stop.set()

    def select_action(self, observation: Any) -> Any:
        """Run one LeRobot inference and return native RMI actions."""
        from lerobot.utils.constants import OBS_STR
        from lerobot.utils.feature_utils import build_dataset_frame

        raw = self._observation_bridge.encode(observation)
        frame = build_dataset_frame(self._dataset_features, raw, prefix=OBS_STR)
        action = self._engine.get_action(frame)
        return None if action is None else self._action_bridge.decode(action)


def main(args: list[str] | None = None) -> None:
    import argparse

    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    parser = argparse.ArgumentParser(
        description="Profile-driven LeRobot RMI policy node"
    )
    parser.add_argument("--profile", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--policy-type", default=None)
    parsed = parser.parse_args(args)

    rclpy.init()
    node = rclpy.create_node("lerobot_policy_provider")
    provider = LeRobotPolicyNode(
        node,
        profile=parsed.profile,
        checkpoint=parsed.checkpoint,
        task=parsed.task,
        device=parsed.device,
        expected_policy_type=parsed.policy_type,
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    provider.start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        provider.stop()
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
