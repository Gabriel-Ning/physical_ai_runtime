"""Profile-driven LeRobot policy provider node with an RMI command boundary."""

from __future__ import annotations

import logging
import threading
from contextlib import nullcontext
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
        agent_name: str = "Policy",
        publish_to_robot: bool = False,
        preempt: bool = False,
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
            self.context.profile, agent_name=agent_name
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

        self._publish_to_robot = publish_to_robot
        self._preempt = preempt
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
        self._agent = self.context.make_agent(
            agent_name,
            sensors=tuple(self._cameras.values()),
        )

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
        from lerobot.utils.constants import OBS_STR
        from lerobot.utils.feature_utils import build_dataset_frame

        control = (
            self._agent.run(
                self.context.robot,
                preempt=self._preempt,
                frequency=self.contract.frequency,
            )
            if self._publish_to_robot
            else nullcontext(None)
        )
        self._engine.start()
        try:
            self.context.robot.wait_until_ready(timeout=30.0)
            for camera in self._cameras.values():
                camera.wait_until_ready(timeout=30.0)
            with control as session:
                while not self._stop.is_set():
                    observation = (
                        session.observe()
                        if session is not None
                        else self._agent.observe()
                    )
                    raw = self._observation_bridge.encode(observation)
                    frame = build_dataset_frame(
                        self._dataset_features, raw, prefix=OBS_STR
                    )
                    action = self._engine.get_action(frame)
                    if action is not None:
                        actions = self._action_bridge.decode(action)
                        if session is not None:
                            for rmi_action in actions:
                                self.context.robot.send_action(
                                    rmi_action,
                                    observation=observation,
                                )
                    if session is not None:
                        session.wait()
                    else:
                        self._stop.wait(1.0 / self.contract.frequency)
        except BaseException:
            if not self._stop.is_set():
                _LOGGER.exception("policy inference stopped")
                self._stop.set()


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
    parser.add_argument("--publish-to-robot", action="store_true")
    parser.add_argument("--preempt", action="store_true")
    parsed = parser.parse_args(args)

    rclpy.init()
    node = rclpy.create_node("lerobot_policy_provider")
    provider = LeRobotPolicyNode(
        node,
        profile=parsed.profile,
        checkpoint=parsed.checkpoint,
        task=parsed.task,
        device=parsed.device,
        publish_to_robot=parsed.publish_to_robot,
        preempt=parsed.preempt,
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
